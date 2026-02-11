classdef erobotics_interface < handle
    % EROBOTICS_INTERFACE - Driver Hardware ROS 2
    % Gestiona:
    % 1. Cliente de Acción (Enviar movimientos)
    % 2. Monitor TF (Leer posición Cartesiana)
    % 3. Suscriptor JointStates (Leer ángulos reales para planificación)
    
    properties
        Node
        ActionClient
        Monitor       % tf_monitor (Cartesiano)
        JointSub      % Suscriptor a /joint_states
        CurrentJoints % Última configuración leída [1x6]
    end
    
    methods
        function obj = erobotics_interface()
            fprintf('--- Iniciando Hardware MyCobot ---\n');
            
            % Configuración de Entorno
            setenv('ROS_DOMAIN_ID', robot_config.DomainID);
            setenv('RMW_IMPLEMENTATION', robot_config.RMW_Implementation);
            
            try
                obj.Node = ros2node(robot_config.NodeName);
                
                % 1. Monitor TF (Visualización Cartesiana)
                obj.Monitor = tf_monitor(obj.Node, robot_config.BaseFrame, robot_config.LinkFrames);
                
                % 2. Suscriptor de Estado (Para Planificación)
                obj.CurrentJoints = zeros(1, 6); % Valor por defecto
                obj.JointSub = ros2subscriber(obj.Node, "/joint_states", ...
                    "sensor_msgs/JointState", @obj.jointStateCallback);
                
                % 3. Cliente de Acción (Movimiento)
                obj.inicializarClienteAccion();
                
            catch ME
                fprintf('\n ERROR CRÍTICO: %s\n', ME.message);
                delete(obj);
            end
        end
        
        function jointStateCallback(obj, msg)
            % Guarda los ángulos actuales del robot real
            % Aseguramos que el orden coincida (simple mapeo directo por ahora)
            if numel(msg.position) >= 6
                % ROS envía radianes, guardamos radianes para consistencia interna
                obj.CurrentJoints = msg.position(1:6)'; 
            end
        end
        
        function config = obtenerConfiguracionActual(obj)
            % Devuelve el estado actual para que el Planificador lo use como inicio
            config = obj.CurrentJoints;
        end
        
        function moverRobot(obj, angulos_grad, tiempo_seg)
            % Envío de trayectoria al Action Server. Mismo mensaje que
            % genera Moveit2
            if length(angulos_grad) ~= 6
                error('Se requieren 6 ángulos en grados.');
            end
            
            fprintf(' Enviando trayectoria... Destino: [%s]\n', num2str(angulos_grad, '%.1f '));
            
            goalMsg = ros2message(obj.ActionClient);
            goalMsg.trajectory.joint_names = robot_config.JointNames;
            
            point = ros2message("trajectory_msgs/JointTrajectoryPoint");
            point.positions = deg2rad(angulos_grad); 
            point.velocities = zeros(1, 6);
            point.accelerations = zeros(1, 6);
            
            sec = floor(tiempo_seg);
            nsec = floor((tiempo_seg - sec) * 1e9);
            point.time_from_start.sec = int32(sec);
            point.time_from_start.nanosec = uint32(nsec);
            
            goalMsg.trajectory.points = point;
            
            try
                sendGoal(obj.ActionClient, goalMsg);
            catch ME
                fprintf(' -> ERROR MATLAB: %s\n', ME.message);
            end
        end
        
        
        function delete(obj)
            if ~isempty(obj.Monitor), delete(obj.Monitor); end
            obj.ActionClient = [];
            obj.Node = [];
            fprintf(' Hardware desconectado.\n');
        end

        function irAHome(obj)
            obj.moverRobot([0 0 0 0 0 45], 3);
        end
    end
    
    methods (Access = private)
        %% Creamos el cliente de Acciones para enviar trayectorias
        function inicializarClienteAccion(obj)
            obj.ActionClient = ros2actionclient(obj.Node, ...
                robot_config.ActionTopic, ...
                robot_config.ActionType);
            
            if waitForServer(obj.ActionClient, "Timeout", 2)
                fprintf(' ¡CONEXIÓN ESTABLECIDA con el Robot!\n');
            else
                warning(' Servidor de acción no encontrado.');
            end
        end

                %% Función no implementada en la interfaz.
        % function ejecutarTrayectoria(obj, qMatrix, tiempoTotal)
        % % EJECUTARTRAYECTORIA Recibe una matriz [6 x N] y la envía punto a punto
        % % qMatrix: Matriz de radianes [6 filas x N columnas]
        % % tiempoTotal: Tiempo en segundos que debe durar todo el movimiento
        % 
        % [~, numPasos] = size(qMatrix);
        % 
        % % Calculamos cuánto debe durar cada "mini-movimiento"
        % dt = tiempoTotal / numPasos; 
        % 
        % % Bucle de ejecución temporal
        % for i = 1:numPasos
        %     tic; % Iniciar cronómetro
        % 
        %     % 1. Extraer configuración actual (vector columna)
        %     qTargetRad = qMatrix(:, i);
        % 
        %     % 2. Convertir a grados (si tu hardware lo requiere)
        %     qTargetDeg = rad2deg(qTargetRad);
        % 
        %     % 3. ENVIAR AL ROBOT
        %     % Usamos moverRobot con el tiempo pequeño 'dt'
        %     % Esto le dice al robot: "Ve a este punto y tarda dt segundos"
        %     obj.moverRobot(qTargetDeg, dt);
        % 
        %     % 4. Esperar para mantener el ritmo (Sincronización)
        %     tLoop = toc;
        %     pauseTime = dt - tLoop;
        %     if pauseTime > 0
        %         pause(pauseTime);
        %     else
        %         % Si el bucle es lento, no pausamos (warning opcional)
        %     end
        % end
        % 
        % fprintf('Trayectoria finalizada.\n');
        % end

    end
end