classdef erobotics_interface < handle
    % EROBOTICS_INTERFACE - Controlador Principal
    % Coordina la conexión ROS 2, el cliente de trayectorias y el monitor.
    % Usa 'robot_config' para los parámetros y 'tf_monitor' para la visualización.

    properties
        Node           % Nodo de ROS 2
        ActionClient   % Cliente de MoveIt
        Monitor        % Instancia de nuestra librería tf_monitor
    end
    
    methods
        function obj = erobotics_interface()
            % 1. Cargar Configuración de Entorno
            fprintf(' Cargando configuración...\n');
            setenv('ROS_DOMAIN_ID', robot_config.DomainID);
            setenv('RMW_IMPLEMENTATION', robot_config.RMW_Implementation);
            
            try
                % 2. Crear Nodo ROS 2
                obj.Node = ros2node(robot_config.NodeName, str2double(robot_config.DomainID));
                 fprintf('obj.node iniciado \n');
                
                % 3. Inicializar Componentes Auxiliares
                % -> Monitor TF (Delegamos la lógica compleja a la otra clase)
                obj.Monitor = tf_monitor(obj.Node, robot_config.BaseFrame, robot_config.LinkFrames);
                fprintf('obj.Monitor iniciado \n');
                % -> Cliente de Acción (MoveIt)
                obj.inicializarClienteAccion();
                 fprintf('obj.Inizializer iniciado');
                
            catch ME
                fprintf(' Error CRÍTICO al iniciar: %s\n', ME.message);
                delete(obj);
            end
        end
        
        function moverRobot(obj, angulos_grad, tiempo_seg)
            % API Pública: Mover el robot
            if length(angulos_grad) ~= 6
                error(' Se requieren 6 ángulos en grados.');
            end
            
            fprintf(' Enviando trayectoria... Destino: [%s]\n', num2str(angulos_grad));
            
            % Construcción del mensaje usando constantes de robot_config
            goalMsg = ros2message(obj.ActionClient);
            goalMsg.trajectory.joint_names = robot_config.JointNames;
            
            point = ros2message("trajectory_msgs/JointTrajectoryPoint");
            point.positions = deg2rad(angulos_grad); 
            point.velocities = zeros(1, 6);
            point.accelerations = zeros(1, 6);
            
            % Cálculo de tiempo
            sec = floor(tiempo_seg);
            nsec = floor((tiempo_seg - sec) * 1e9);
            point.time_from_start.sec = int32(sec);
            point.time_from_start.nanosec = uint32(nsec);
            
            goalMsg.trajectory.points = point;
            
            % Envío Bloqueante
            try
                result = sendGoal(obj.ActionClient, goalMsg);
                if result.error_code == 0
                    fprintf(' -> Movimiento EXITOSO.\n');
                else
                    fprintf(' -> ERROR del Robot. Código: %d\n', result.error_code);
                end
            catch ME
                fprintf(' -> Fallo al enviar meta: %s\n', ME.message);
            end
        end
        
        function verDatos(obj, activar)
            % Activa o desactiva el monitor de pantalla
            if nargin < 2 || activar
                obj.Monitor.iniciar(1.0); % 1 Hz
            else
                obj.Monitor.detener();
            end
        end
        
        function delete(obj)
            % Limpieza ordenada
            if ~isempty(obj.Monitor), delete(obj.Monitor); end
            obj.ActionClient = [];
            obj.Node = [];
            fprintf(' Sistema desconectado.\n');
        end
    end
    
    methods (Access = private)
        function inicializarClienteAccion(obj)
            fprintf(' Conectando con el controlador del robot (%s)...\n', robot_config.ActionTopic);
            obj.ActionClient = ros2actionclient(obj.Node, ...
                robot_config.ActionTopic, ...
                robot_config.ActionType);
            
            if waitForServer(obj.ActionClient, "Timeout", 5)
                fprintf(' ¡Conexión establecida con el Robot!\n');
                fprintf(' Comandos disponibles:\n');
                fprintf('  >> robot.moverRobot([0,0,0,0,0,0], 2)\n');
                fprintf('  >> robot.verDatos(true)\n');
            else
                warning(' Servidor de acción no encontrado. Verifica tu Docker.');
            end
        end
    end
end