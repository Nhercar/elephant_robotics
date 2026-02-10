classdef TrajectoryPlanner < handle
    % TRAJECTORYPLANNER - Planificación inteligente
    
    properties
        Model       % MyCobotModel
        Hardware    % erobotics_interface
        VizFig      % Figura
    end
    
    methods
        function obj = TrajectoryPlanner(hardware, model)
            obj.Hardware = hardware;
            obj.Model = model;
            
            obj.VizFig = figure('Name', 'Planificador MyCobot', 'Color', 'w');
            
            % Sincronización inicial
            obj.sincronizarConHardware();
            view(3);
        end
        
        function sincronizarConHardware(obj)
            % Lee el hardware y actualiza el gráfico
            if ~isempty(obj.Hardware)
                qReal = obj.Hardware.obtenerConfiguracionActual();
                configReal = obj.Model.vectorToConfig(qReal);
                
                figure(obj.VizFig);
                obj.Model.visualizar(configReal);
                drawnow;
            end
        end
        
        function moverCartesiano(obj, x, y, z, roll, pitch, yaw, tiempoTotal)
            fprintf('Planificando ruta a: [%.2f %.2f %.2f]...\n', x, y, z);
            
            % 1. Obtener PUNTO DE PARTIDA REAL
            if ~isempty(obj.Hardware)
                qCurrent = obj.Hardware.obtenerConfiguracionActual();
                startConfig = obj.Model.vectorToConfig(qCurrent);
            else
                startConfig = obj.Model.HomeConfig;
            end
            
            % 2. Definir META
            targetTform = trvec2tform([x, y, z]) * eul2tform([yaw, pitch, roll]);
            
            % Dibujar meta
            figure(obj.VizFig);
            %plotTransforms(targetTform, 'FrameSize', 0.1);
            
            % 3. Calcular IK (Cinemática Inversa)
            [endConfig, info] = obj.Model.calcularIK(targetTform, startConfig);
            
            if info.Status ~= "success"
                warning('Solución IK aproximada (Error: %.4f)', info.PoseErrorNorm);
            end
            
            % 4. Ejecutar Interpolación
            obj.ejecutarTrayectoria(startConfig, endConfig, tiempoTotal);
        end
        
        function ejecutarTrayectoria(obj, startConfStruct, endConfStruct, tiempoTotal)
            % Convierte structs a vectores para interpolar
            qStart = startConfStruct';
            qEnd = endConfStruct';
            
            steps = 50;
            t = linspace(0, tiempoTotal, steps);
            
            % Interpolación Polinómica (Suave)
            [qMatrix, ~, ~] = quinticpolytraj([qStart; qEnd]', [0, tiempoTotal], t);
            
            % Animación previa
            figure(obj.VizFig);
            for i = 1:steps
                % Reconstruir struct para 'show'
                config = obj.Model.vectorToConfig(qMatrix(:,i));
                show(obj.Model.RobotTree, config, 'Parent', gca, 'PreservePlot', false, 'Collisions', 'off');
                drawnow limitrate;
            end
            
            % Envío al Hardware
            qFinalDeg = rad2deg(qEnd);
            if ~isempty(obj.Hardware)
                obj.Hardware.moverRobot(qFinalDeg, tiempoTotal);
            end
        end
        
        function irAHome(obj)
            % Wrapper simple para ir a casa
            if ~isempty(obj.Hardware)
                qCurrent = obj.Hardware.obtenerConfiguracionActual();
                startConf = obj.Model.vectorToConfig(qCurrent);
            else
                startConf = obj.Model.HomeConfig;
            end
            
            endConf = obj.Model.HomeConfig;
            obj.ejecutarTrayectoria(startConf, endConf, 3.0);
        end
    end
end