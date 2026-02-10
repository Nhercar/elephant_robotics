classdef MyCobotModel < handle
    % MYCOBOTMODEL - Modelo Matemático
    
    properties
        RobotTree       % rigidBodyTree
        IKSolver        % Inverse Kinematics
        HomeConfig      % Configuración struct
        EndEffector     % Nombre TCP
    end
    
    methods
        function obj = MyCobotModel(filename)
            if nargin < 1
                filename = 'mycobot_280_arduino.urdf.xacro'; 
            end
            
            % Carga inteligente (soporte parcial XACRO via importrobot o conversión externa)
            try
                obj.RobotTree = importrobot(filename);
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
            
            obj.RobotTree.DataFormat = 'row'; 
            obj.RobotTree.Gravity = [0 0 -9.81];
            obj.EndEffector = obj.RobotTree.BodyNames{end};
            
            obj.IKSolver = inverseKinematics('RigidBodyTree', obj.RobotTree);
            obj.IKSolver.SolverParameters.MaxIterations = 50;
            obj.IKSolver.SolverParameters.AllowRandomRestart = false;
            
            obj.HomeConfig = homeConfiguration(obj.RobotTree);
        end
        
        function [configSol, info] = calcularIK(obj, targetPose, initialGuess)
            weights = [0.1 0.1 0.1 1 1 1]; 
            if nargin < 3, initialGuess = obj.HomeConfig; end
            [configSol, info] = obj.IKSolver(obj.EndEffector, targetPose, weights, initialGuess);
        end
        
        function configStruct = vectorToConfig(obj, jointVector)
            % Convierte vector [q1..q6] a estructura de configuración de MATLAB
            %configStruct = obj.HomeConfig;
            configStruct = jointVector; % Initialize configStruct with home configuration
            % for i = 1:min(length(jointVector), length(configStruct))
            %     configStruct(i).JointPosition = jointVector(i);
            % end
        end
        
        function visualizar(obj, config)
            if nargin < 2, config = obj.HomeConfig; end
            show(obj.RobotTree, config, 'PreservePlot', false, 'Collisions', 'off');
            grid on; axis equal;
            title('Visualización en Tiempo Real');
        end
    end
end