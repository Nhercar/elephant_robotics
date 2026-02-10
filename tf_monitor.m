classdef tf_monitor < handle
    % TFMONITOR - Visualizador 3D del Robot
    % Escucha /tf y dibuja el esqueleto del robot en una ventana gráfica.
    
    properties (Access = private)
        Node
        Tftree
        Timer
        LinkNames
        BaseName
        
        % Propiedades Gráficas
        FigHandle   % La ventana
        PlotLine    % La línea que une los puntos (esqueleto)
        TextLabels  % Etiquetas de texto sobre cada articulación
    end
    
    methods
        function obj = tf_monitor(rosNode, baseFrame, linkFrames)
            obj.Node = rosNode;
            obj.BaseName = baseFrame;
            obj.LinkNames = linkFrames;
            obj.TextLabels = {};
            
            try
                obj.Tftree = ros2tf(obj.Node);
                fprintf(' [tf_monitor] Listo para visualizar.\n');
            catch ME
                warning('[tf_monitor] Error tf: %s', ME.message);
            end
        end
        
        function iniciar(obj, periodo)
            % Refresco de 0.2s (5Hz) para movimiento fluido
            if nargin < 2, periodo = 0.2; end 
            obj.detener();
            
            % Crear la ventana
            obj.inicializarFigura();
            
            fprintf(' [tf_monitor] Ventana gráfica abierta.\n');
            obj.Timer = timer('ExecutionMode', 'fixedRate', ...
                'Period', periodo, ...
                'TimerFcn', @(~,~) obj.actualizarGrafico);
            start(obj.Timer);
        end
        
        function detener(obj)
            if ~isempty(obj.Timer) && isvalid(obj.Timer)
                stop(obj.Timer);
                delete(obj.Timer);
            end
            % No cerramos la figura automáticamente para que puedas ver la última pose
        end
        
        function coords = obtenerPosicion(obj, frameName)
            coords = [];
            try
                tfStamped = getTransform(obj.Tftree, obj.BaseName, frameName, "Timeout", 0.05);
                ts = tfStamped.transform.translation;
                coords = [ts.x, ts.y, ts.z];
            catch
                % Silencioso para no saturar si falla un frame
            end
        end
    end
    
    methods (Access = private)
        function inicializarFigura(obj)
            if isempty(obj.FigHandle) || ~isvalid(obj.FigHandle)
                obj.FigHandle = figure('Name', 'Monitor Robot ROS 2', ...
                    'NumberTitle', 'off', 'Color', 'w');
                
                % Configurar ejes 3D
                axes('Parent', obj.FigHandle);
                hold on; grid on; axis equal;
                view(3); % Vista 3D
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
                title(['Esqueleto del Robot (Ref: ' obj.BaseName ')']);
                
                % Definir límites fijos para que el robot no "baile" al moverse
                % Ajusta esto según el tamaño real de tu MyCobot
                xlim([-0.5 0.5]); ylim([-0.5 0.5]); zlim([0 0.6]);
                
                % Crear objeto de línea vacío (se actualizará en el timer)
                obj.PlotLine = plot3(0, 0, 0, '-o', ...
                    'LineWidth', 2, 'MarkerSize', 6, ...
                    'MarkerFaceColor', 'b', 'Color', '#333333');
            else
                figure(obj.FigHandle); % Traer al frente
            end
        end

        function actualizarGrafico(obj)
            if isempty(obj.FigHandle) || ~isvalid(obj.FigHandle)
                obj.detener(); return; % Si cierras la ventana, paramos el timer
            end
            
            % 1. El primer punto es la BASE (0,0,0)
            X = 0; Y = 0; Z = 0;
            
            % 2. Buscar coordenadas de los eslabones
            for i = 1:length(obj.LinkNames)
                frame = obj.LinkNames{i};
                pos = obj.obtenerPosicion(frame);
                
                if ~isempty(pos)
                    X(end+1) = pos(1); %#ok<AGROW>
                    Y(end+1) = pos(2); %#ok<AGROW>
                    Z(end+1) = pos(3); %#ok<AGROW>
                end
            end
            
            % 3. Dibujar solo si tenemos datos
            if length(X) > 1
                set(obj.PlotLine, 'XData', X, 'YData', Y, 'ZData', Z);
                
                % Actualizar etiquetas (opcional, puede ponerse lento si son muchas)
                if isempty(obj.TextLabels)
                    % Crear etiquetas la primera vez
                    obj.TextLabels = cell(1, length(obj.LinkNames));
                    for k = 1:length(obj.LinkNames)
                        obj.TextLabels{k} = text(0,0,0, obj.LinkNames{k}, ...
                            'FontSize', 8, 'Interpreter', 'none');
                    end
                end
                
                % Mover etiquetas
                for k = 1:min(length(obj.LinkNames), length(X)-1)
                    set(obj.TextLabels{k}, 'Position', [X(k+1), Y(k+1), Z(k+1)+0.02]);
                end
                
                drawnow limitrate; % Actualización eficiente
            end
        end
    end
end