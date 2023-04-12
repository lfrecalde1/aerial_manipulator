function  arm_Graph=robotPlot(x,y,z,psi,scaleRobot)
global Robot;
robotParameters; % Carga todos los puntos de la estructura del drone

% psi=psi-pi/2;
h = 0.12;
l1 = 0.4;
l2 = 0.26;
a = 0.13;
b = 0.39;
% Matriz de rotaci�n
Rz=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

scaleRobot=scaleRobot*(scaleRobot/1000);

%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.baseDroneVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x; %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y; %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z; %Escalar y dezplazar en el eje z

arm_Graph(1) = patch('Faces',Robot.baseDroneFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot


%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.soporteMotorVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x; %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y; %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z; %Escalar y dezplazar en el eje z

arm_Graph(2) = patch('Faces',Robot.soporteMotorFaces,'Vertices',armPatch','FaceColor','r','EdgeColor','none'); % Patch dibujar el robot


%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.helicesVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x; %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y; %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z; %Escalar y dezplazar en el eje z

arm_Graph(3) = patch('Faces',Robot.helicesFaces,'Vertices',armPatch','FaceColor','k','EdgeColor','none'); % Patch dibujar el robot


%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.baseBrazoVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x; %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y; %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z; %Escalar y dezplazar en el eje z

arm_Graph(4) = patch('Faces',Robot.baseBrazoFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.baseBrazoVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x+a*sin(psi+pi); %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y-a*cos(psi+pi); %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z; %Escalar y dezplazar en el eje z

arm_Graph(5) = patch('Faces',Robot.baseBrazoFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot



