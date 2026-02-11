classdef MyCobotModel < handle
    % MYCOBOTMODEL - Modelo Matemático
    % Clase encargada de:
    %  --Resolver la cinemática inversa
    %  --Representar el movimiento según los valores de /joint_states.

    
    properties
        RobotTree       % rigidBodyTree
        IKSolver        % Inverse Kinematics
        HomeConfig      % Configuración struct
        EndEffector     % Nombre TCP
        Node            % Nodo de ROS2 para leer los joints_states
        JointSub        % Suscriptor a /joint_states par visualizar el robot
        CurrentJoints   % Para guardar el valor actual de los joints
    end
    
    methods
        function obj = MyCobotModel(filename)
            if nargin < 1
                filename = 'mycobot_280_arduino.urdf.xacro'; %Por defecto intentamos cargar este URDF
            end
            
            try
                obj.RobotTree = importrobot(filename); %Carga de cualquier robot
            catch
                % Fallback genérico si no encuentra el archivo específico
                warning('No se encontró el archivo URDF/XACRO específico. Buscando *.urdf...');
                files = dir('*.urdf');
                if ~isempty(files)
                    obj.RobotTree = importrobot(files(1).name);
                else
                    error('No se encuentra ningún modelo URDF válido.');
                end
            end

            %Configuraciones del objeto robot
            obj.RobotTree.DataFormat = 'row'; 
            obj.RobotTree.Gravity = [0 0 -9.81];
            obj.EndEffector = obj.RobotTree.BodyNames{end};

            %Configuración de la cinemática inversa
            obj.IKSolver = inverseKinematics('RigidBodyTree', obj.RobotTree);
            obj.IKSolver.SolverParameters.MaxIterations = 100;
            obj.IKSolver.SolverParameters.AllowRandomRestart = false;
            
            %Guardar la posicion inicial del urdf como posición home
            obj.HomeConfig = homeConfiguration(obj.RobotTree);

            obj.Node = ros2node(robot_config.NodeName);

            obj.JointSub = ros2subscriber(obj.Node, "/joint_states", ...
                    "sensor_msgs/JointState", @obj.jointStateCallback);
        end
        
        function [configSol, info] = calcularIK(obj, targetPose, initialGuess)
            weights = [1 1 1 1 1 1]; 
            if nargin < 3, initialGuess = obj.HomeConfig; end
            [configSol, info] = obj.IKSolver(obj.EndEffector, targetPose, weights, initialGuess);
        end
        
        function configRow = vectorToRow(jointVector)
            % Convierte cualquier entrada a vector fila [1x6]
            % DataFormat 'row' exige vectores fila, no columnas ni estructuras.
            configRow = jointVector(:)';
        end
        
        function visualizar(obj, config)
            if nargin < 2, config = obj.HomeConfig; end % Si no introducimos posción, visualizamos en home
            show(obj.RobotTree, config, 'PreservePlot', false, 'Collisions', 'off');
            grid on; axis equal;
            title('Visualización en Tiempo Real');
        end

        function visualizarTrayectoria(obj, config)
            if nargin < 2, config = obj.HomeConfig; end % Si no introducimos posción, visualizamos en home
            show(obj.RobotTree, config, 'PreservePlot', false, 'Collisions', 'off');
            grid on; axis equal;
            title('Visualización de trayectoria');
        end

        function jointStateCallback(obj, msg)
            if numel(msg.position) >= 6
                % ROS2 envía radianes, guardamos radianes para consistencia interna
                obj.CurrentJoints = msg.position(1:6)'; 
            end
            obj.visualizar(obj.CurrentJoints);
        end
    end
end