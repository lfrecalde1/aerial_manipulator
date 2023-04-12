function  arm_Graph=brazoPlot(x,y,z,a,b,psi,q11,q21,q31,q41,scaleRobot)
global Robot;
robotParameters; % Carga todos los puntos de la estructura del drone

q11=q11-pi/2;
h = 0.12;
l1 = 0.4;
l2 = 0.26;

scaleRobot=scaleRobot*(scaleRobot/1000);

Rz=[cos(psi+q11) -sin(psi+q11) 0; sin(psi+q11) cos(psi+q11) 0; 0 0 1];

%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Robot.primerEslabonVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x+a*cos(psi)-b*sin(psi); %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y+a*sin(psi)+b*cos(psi); %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z-h; %Escalar y dezplazar en el eje z

arm_Graph(1) = patch('Faces',Robot.primerEslabonFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

Ry=[cos(q21), 0, sin(q21); 0, 1, 0;-sin(q21), 0, cos(q21)];
Rz=[cos(psi+q11+pi/2) -sin(psi+q11+pi/2) 0; sin(psi+q11+pi/2) cos(psi+q11+pi/2) 0; 0 0 1];

%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Ry*Robot.segundoEslabonVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x+a*cos(psi)-b*sin(psi); %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y+a*sin(psi)+b*cos(psi); %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z-h; %Escalar y dezplazar en el eje z

arm_Graph(2) = patch('Faces',Robot.segundoEslabonFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

Ry=[cos(q21+q31), 0, sin(q21+q31); 0, 1, 0;-sin(q21+q31), 0, cos(q21+q31)]; %Matrix Rotation y axis

%%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
armPatch = Rz*Ry*Robot.tercerEslabonVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
armPatch(1,:)=armPatch(1,:)*scaleRobot+x+a*cos(psi)-b*sin(psi)-l1*sin(psi+q11)*cos(q21); %Escalar y dezplazar en el eje x
armPatch(2,:)=armPatch(2,:)*scaleRobot+y+a*sin(psi)+b*cos(psi)+l1*cos(psi+q11)*cos(q21); %Escalar y dezplazar en el eje y
armPatch(3,:)=armPatch(3,:)*scaleRobot+z-h-l1*sin(q21); %Escalar y dezplazar en el eje z

arm_Graph(3) = patch('Faces',Robot.tercerEslabonFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

% Ry=[cos(q21+q31+q41), 0, sin(q21+q31+q41); 0, 1, 0;-sin(q21+q31+q41), 0, cos(q21+q31+q41)]; %Matrix Rotation z axis

% %%%%%%%%%%%%%%%%%% base robot %%%%%%%%%%%%%%%%%%%%
% armPatch = Rz*Ry*Robot.cuartoEslabonVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
% armPatch(1,:)=armPatch(1,:)*scaleRobot+x+a*cos(psi)-b*sin(psi)-l1*cos(q21)*sin(psi+q11)-l2*cos(q21+q31)*sin(psi+q11); %Escalar y dezplazar en el eje x
% armPatch(2,:)=armPatch(2,:)*scaleRobot+y+a*sin(psi)+b*cos(psi)+l1*cos(q21)*cos(psi+q11)+l2*cos(q21+q31)*cos(psi+q11); %Escalar y dezplazar en el eje y
% armPatch(3,:)=armPatch(3,:)*scaleRobot+z-h-l1*sin(q21)-l2*sin(q21+q31); %Escalar y dezplazar en el eje z

% arm_Graph(4) = patch('Faces',Robot.cuartoEslabonFaces,'Vertices',armPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

