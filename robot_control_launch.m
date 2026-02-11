%[text] ## Crear el robot
robot_hw = erobotics_interface(); %[output:3807baab]
fprintf('robot conectado') %[output:522ac8d4]
  %[control:button:3b9e]{"position":[1,2]}
%%
%[text] ## Llevar robot a posición inicial
robot_hw.moverRobot([0, 0, 0, 0, 0, 45], 2) %[output:8ce0f327]
  %[control:button:71aa]{"position":[1,2]}
%%
%[text] ## Definir posición deseada
J1 = 0 %[control:slider:2406]{"position":[6,7]} %[output:4d77b1c1]
J2 = 0 %[control:slider:00dc]{"position":[6,7]} %[output:7cf0b385]
J3 = 0 %[control:slider:9bd1]{"position":[6,7]} %[output:62d0a385]
J4 = 0 %[control:slider:4837]{"position":[6,7]} %[output:68e8b125]
J5 = 0 %[control:slider:9e07]{"position":[6,7]} %[output:89aef484]
J6 = 0 %[control:slider:53f5]{"position":[6,7]} %[output:7b88309e]
%%
%[text] ## Mover a posición deseada
angles = [J1, J2, J3, J4, J5, J6];
robot_hw.moverRobot(angles, 2);
  %[control:button:31f3]{"position":[1,2]}
%%
%[text] ## Borrar el robot
delete(robot_hw);

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[control:button:3b9e]
%   data: {"label":"Run","run":"Section"}
%---
%[control:button:71aa]
%   data: {"label":"Run","run":"Section"}
%---
%[control:slider:2406]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:slider:00dc]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:slider:9bd1]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:slider:4837]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:slider:9e07]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:slider:53f5]
%   data: {"defaultValue":0,"label":"Joint1","max":90,"min":-90,"run":"Section","runOn":"ValueChanging","step":5}
%---
%[control:button:31f3]
%   data: {"label":"Run","run":"Section"}
%---
%[output:3807baab]
%   data: {"dataType":"text","outputData":{"text":"--- Iniciando Hardware MyCobot ---\n [tf_monitor] Listo para visualizar.\n ¡CONEXIÓN ESTABLECIDA con el Robot!\n","truncated":false}}
%---
%[output:522ac8d4]
%   data: {"dataType":"text","outputData":{"text":"robot conectado","truncated":false}}
%---
%[output:8ce0f327]
%   data: {"dataType":"text","outputData":{"text":" Enviando trayectoria... Destino: [0.0  0.0  0.0  0.0  0.0 45.0]\n","truncated":false}}
%---
%[output:4d77b1c1]
%   data: {"dataType":"textualVariable","outputData":{"name":"J1","value":"0"}}
%---
%[output:7cf0b385]
%   data: {"dataType":"textualVariable","outputData":{"name":"J2","value":"0"}}
%---
%[output:62d0a385]
%   data: {"dataType":"textualVariable","outputData":{"name":"J3","value":"0"}}
%---
%[output:68e8b125]
%   data: {"dataType":"textualVariable","outputData":{"name":"J4","value":"0"}}
%---
%[output:89aef484]
%   data: {"dataType":"textualVariable","outputData":{"name":"J5","value":"0"}}
%---
%[output:7b88309e]
%   data: {"dataType":"textualVariable","outputData":{"name":"J6","value":"0"}}
%---
