classdef TrajectoryPlanner < handle
    % TRAJECTORYPLANNER - Solo Matemáticas y Visualización de trayectorias
    % Clase encargada de:
    % -- Generar matrices de puntos (waypoints) en articulares y en el
    %    espacio de la tarea
    % -- Visualizar las trayectorias
    
    properties
        Model       % MyCobotModel
        VizFig      % Figura para debugging visual
    end
    
    methods
        function obj = TrajectoryPlanner(model)
            obj.Model = model;
            obj.VizFig = figure('Name', 'Planificador', 'Color', 'w');
            view(3); grid on;
        end
        
        function actualizarVisualizacion(obj, qConfig)
            % Método auxiliar para actualizar la figura
            figure(obj.VizFig);
            obj.Model.visualizarTrayectoria(qConfig);
            drawnow limitrate;
        end
        
        %% 1. PLANIFICACIÓN ARTICULAR
        % Devuelve: qMatrix [6 x N] (radianes)
        function qMatrix = planificarArticular(obj, startConfig, x, y, z, roll, pitch, yaw, tiempoTotal)
            
            % A. Calcular Meta
            targetTform = trvec2tform([x, y, z]) * eul2tform([yaw, pitch, roll]);
            [endConfig, info] = obj.Model.calcularIK(targetTform, startConfig);
            
            if info.Status ~= "success"
                warning('IK aproximada: Error %.4f', info.PoseErrorNorm);
            end
            
            % B. Interpolar (Generar Matriz)
            steps = 50; % Número de puntos
            t = linspace(0, tiempoTotal, steps);
            
            waypoints = [startConfig(:), endConfig(:)]; 
            [qMatrix, ~, ~] = quinticpolytraj(waypoints, [0, tiempoTotal], t);
            
            % C. Visualizar (Opcional, para debug)
            for i = 1:steps
                obj.actualizarVisualizacion(obj.Model.vectorToConfig(qMatrix(:,i)));
            end
        end
        
        %% 2. PLANIFICACIÓN CARTESIANA (Línea Recta)
        % Devuelve: qMatrix [6 x N] (radianes)
        function qMatrix = planificarCartesiano(obj, startConfig, x, y, z, roll, pitch, yaw, tiempoTotal)
            
            tformStart = getTransform(obj.Model.RobotTree, startConfig, obj.Model.EndEffector);
            tformEnd = trvec2tform([x, y, z]) * eul2tform([yaw, pitch, roll]);
            
            % A. Generar camino en 3D
            steps = 50;
            tSamples = linspace(0, tiempoTotal, steps);
            [tformsWaypoints, ~, ~] = transformtraj(tformStart, tformEnd, [0, tiempoTotal], tSamples);
            
            % B. Resolver IK para cada punto
            qMatrix = zeros(6, steps);
            lastConfig = startConfig; 
            
            for i = 1:steps
                [configSol, ~] = obj.Model.calcularIK(tformsWaypoints(:,:,i), lastConfig);
                qMatrix(:, i) = configSol(:);
                lastConfig = configSol;
            end
            
            % C. Visualizar
            for i = 1:steps
                obj.actualizarVisualizacion(obj.Model.vectorToConfig(qMatrix(:,i)));
            end
        end
    end
end