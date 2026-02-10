classdef robot_config
    % ROBOTCONFIG - Archivo central de configuración
    % Aquí se definen todas las constantes, nombres de articulaciones
    % y parámetros de red para el proyecto.
    
    properties (Constant)
        % --- Configuración de Red ROS 2 ---
        DomainID = '0'
        RMW_Implementation = 'rmw_fastrtps_cpp'
        NodeName = '/matlab_erobotics_client'
        
        % --- Configuración del Servidor de Acción (MoveIt) ---
        ActionTopic = '/arm_controller/follow_joint_trajectory'
        ActionType = 'control_msgs/FollowJointTrajectory'
        
        % --- Definición Física del Robot (URDF) ---
        % Nombres exactos de las articulaciones (Joints)
        JointNames = { ...
            'joint2_to_joint1', ...
            'joint3_to_joint2', ...
            'joint4_to_joint3', ...
            'joint5_to_joint4', ...
            'joint6_to_joint5', ...
            'joint6output_to_joint6' ...
        }
        
        % --- Configuración de Monitorización (/tf) ---
        BaseFrame = 'joint1' % El punto 0,0,0 del mundo o del robot
        % Eslabones (Links) que queremos monitorizar en pantalla
        LinkFrames = {'joint2', 'joint3', 'joint4', 'joint6', 'joint6', 'joint6_flange'}
    end
end