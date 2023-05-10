function HexacopteroPlot= Hexacoptero(X,Y,Z,phi)

% Funcion "Hexacoptero" genera el gráfico del UAV para la animación

% 1) ARGUMENTOS DE ENTRADA
  % a) X ----> Posición X del UAV
  % b) Y ----> Posición Y del UAV
  % c) Z ----> Posición Z del UAV
  % d) phi --> Posición angular YAW del UAV
     
% 2) ARGUMENTOS DE SALIDA
  % Plot Hexacopero
  
% 3) Formas de llamar la funcion
  % Hexacoptero(1) --> configura escala;
  % Hexacoptero(1,[1 0 0]) --> configura escala y color a ROJO;
  % Hexacoptero(1,0,1,0,0,pi/2) --> configura posiciones y ángulos
      
global hexacopter;
if nargin >= 0 && nargin <=2
hexacopter.colorFrameS= [1 0 0];
hexacopter.colorFrameN= [0.1 0.1 0];

    if nargin==0
       scale1= 1;
    elseif nargin==1
       scale1=  X;
    elseif nargin==2
       scale1=  X;
       hexacopter.colorFrameS= Y;
    end
    
    scaleX=1;
    scaleY=0.5;
    scaleZ=0.5;

   
    % Rotacionales y frameworks
    for pp=1:1
    % Rotacionales
    angulo= 180;
    rotDibFy180 = [1 0 0; 0 cos(angulo*pi/180) -sin(angulo*pi/180);0 sin(angulo*pi/180) cos(angulo*pi/180)];
    angulo= 90;
    rotDibFy90 = [cos(angulo*pi/180) 0 sin(angulo*pi/180);0 1 0;-sin(angulo*pi/180) 0 cos(angulo*pi/180)];
    angulo= 60;
    rotDibFz = [cos(angulo*pi/180) -sin(angulo*pi/180) 0; sin(angulo*pi/180) cos(angulo*pi/180) 0; 0 0 1];
    
    
    % TODO EL FRAMEWORK: PATITAS Y BASE
    y1 = [13 11 -11 -13 -11 11 13]*scaleX*scale1;
    x1 = [0 1 1 0 -1 -1 0]*scaleY*scale1;
    z1 = ones(1,7)*scaleZ*scale1;
    hexacopter.patita1{1}=[x1;y1;z1];
    hexacopter.patita1{2}= rotDibFy180*hexacopter.patita1{1};
    
    y1 = [11 11 -11 -11 -11 11 11]*scaleX*scale1;
    x1 = [0 1 1 0 -1 -1 0]*scaleY*scale1;
    z1 = ones(1,7)*scaleZ*scale1;
    hexacopter.patita1{50}=[x1;y1;z1];
    hexacopter.patita1{3}= rotDibFy90*hexacopter.patita1{50};
    hexacopter.patita1{4}= rotDibFy180*hexacopter.patita1{3};
    
    hexacopter.patita2{1}= rotDibFz*hexacopter.patita1{1};
    hexacopter.patita2{2}= rotDibFz*hexacopter.patita1{2};
    hexacopter.patita2{3}= rotDibFz*hexacopter.patita1{3};
    hexacopter.patita2{4}= rotDibFz*hexacopter.patita1{4};
    
    hexacopter.patita3{1}= rotDibFz*hexacopter.patita2{1};
    hexacopter.patita3{2}= rotDibFz*hexacopter.patita2{2};
    hexacopter.patita3{3}= rotDibFz*hexacopter.patita2{3};
    hexacopter.patita3{4}= rotDibFz*hexacopter.patita2{4};
    end
    
    %% Tapas de los motores
    t = linspace(0, 2*pi, 20);
    r = 1;
    xC = r*cos(t)+0;
    yC = r*sin(t)+13;
    zC= 1*ones(1, length(xC));
    
    circulo1= [xC' yC' zC'];
    hexacopter.tapa{1}= circulo1*scale1;

    for pp=2:8
        hexacopter.tapa{pp}= hexacopter.tapa{pp-1}*rotDibFz;
    end
        
    
    %% Para las helices de afuera
    t = linspace(0, 2*pi, 20);
    r = 5;
    xC = r*cos(t)+0;
    yC = r*sin(t)+13;
    zC= ones(1, length(xC))+1;
    
    circulo1= [xC' yC' zC'];
    hexacopter.helice{1}= circulo1*scale1;
    
    for pp=2:8
        hexacopter.helice{pp}= hexacopter.helice{pp-1}*rotDibFz;
    end
    
    %% Para las helices de adentro
    t = linspace(0, 2*pi, 20);
    r = 3;
    xC = r*cos(t)+0;
    yC = r*sin(t)+13;
    zC= ones(1, length(xC))+1;
    
    circulo1= [xC' yC' zC'];
    hexacopter.heliceI{1}= circulo1*scale1;
    
    for pp=2:8
        hexacopter.heliceI{pp}= hexacopter.heliceI{pp-1}*rotDibFz;
    end
    
    
    %% Dibujo de motores
    
    
    [X,Y,Z] = cylinder(1,20);
    [TRI,v]= surf2patch(X+0,Y+13,2*Z-1);
    
    hexacopter.motor{1}= v*scale1;
    hexacopter.motor{9}= TRI;
%     
    for pp=2:8
        hexacopter.motor{pp}= hexacopter.motor{pp-1}*rotDibFz;
    end
%     for pp=1:8
%         dHexarotor= hexacopter.motor{pp};
%         dHexarotor(:,1)=dHexarotor(:,1)'; dHexarotor(:,2)=dHexarotor(:,2)'; dHexarotor(:,3)=dHexarotor(:,3)';
%         hexacopter.motor{pp}= dHexarotor;
%     end
    
    %% Tapas de la computadora
    t = linspace(0, 2*pi, 20);
    r = 2;
    yC = r*cos(t);
    xC = r*sin(t)*2;
    zC= ones(1, length(xC))+1.1;

    circulo1= [xC' yC' zC'];
     hexacopter.tapaP{1}= circulo1*scale1;
    
    zC1= ones(1, length(xC))-2.1;
    circulo2= [xC' yC' zC1'];
    hexacopter.tapaP{2}= circulo2*scale1;
    
    %% Computadora
    t = 0:pi/10:pi;
    [X,Y,Z] = cylinder(2+sin(t));
    [TRI,v]= surf2patch(X*2,Y,3*Z-1);
    
    hexacopter.computadora{1}= v*scale1;
    hexacopter.computadora{2}= TRI;
    
%% %%%%%%%%%%%%%%%%%%%%% SECCION DE PLOTEO%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
else
%     global hexacopter;
%     tam=1;
    
    colorFrameN= hexacopter.colorFrameN;
    colorFrameS= hexacopter.colorFrameS;

    x= X;
    y= Y;
    z= Z;
    phi=-phi;
    rotZ= [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
    rotZp= [cos(-phi) -sin(-phi) 0; sin(-phi) cos(-phi) 0; 0 0 1];
    
    HexacopteroPlot=[];
    %% Ploteo de los motores
    TRI= hexacopter.motor{9};
    
    for pp=[1 5 6 4]
        dHexarotor= hexacopter.motor{pp}*rotZ;
        dHexarotor(:,1)= dHexarotor(:,1) + x; dHexarotor(:,2)= dHexarotor(:,2) + y; dHexarotor(:,3)= dHexarotor(:,3) + z; 
        HexacopteroPlot(end+1)= patch('Vertices',dHexarotor,'Faces',TRI,'facecolor',colorFrameN,'LineStyle','none'); %,'facealpha',0.5
    end
    
    for pp=[2 3]
        dHexarotor= hexacopter.motor{pp}*rotZ;
        dHexarotor(:,1)= dHexarotor(:,1) + x; dHexarotor(:,2)= dHexarotor(:,2) + y; dHexarotor(:,3)= dHexarotor(:,3) + z; 
        HexacopteroPlot(end+1)= patch('Vertices',dHexarotor,'Faces',TRI,'facecolor',colorFrameS,'LineStyle','none'); %,'facealpha',0.5
    end
    
    %% Ploteo de la patitaaaas
    for pp=1:4
        rHexarotor = rotZp*hexacopter.patita1{pp};
        HexacopteroPlot(end+1) = patch(rHexarotor(1,:)+x,rHexarotor(2,:)+y,rHexarotor(3,:)+z,colorFrameN);
    end
    
    for pp=1:4
        rHexarotor = rotZp*hexacopter.patita2{pp};
        HexacopteroPlot(end+1) = patch(rHexarotor(1,:)+x,rHexarotor(2,:)+y,rHexarotor(3,:)+z,colorFrameN);
    end
    
    for pp=1:4
        rHexarotor = rotZp*hexacopter.patita3{pp};
        HexacopteroPlot(end+1) = patch(rHexarotor(1,:)+x,rHexarotor(2,:)+y,rHexarotor(3,:)+z,colorFrameN);
    end
    
    %% Ploteo de las tapas de los motores
    for pp=[1 2 3 4]
        dHexarotor= hexacopter.tapa{pp}*rotZ;
        HexacopteroPlot(end+1)=patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,[228/255,158/255,86/255]);
    end
    
    for pp=[5 6]
        dHexarotor= hexacopter.tapa{pp}*rotZ;
        HexacopteroPlot(end+1)= patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,[228/255,158/255,86/255]);
        %,'facealpha',0.5
    end
    
    %% Ploteo de las helices internas
%     for pp=[1 5 6 4]
%         dHexarotor= hexacopter.heliceI{pp}*rotZ;
%         HexacopteroPlot(end+1)= patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,colorFrameN,'EdgeColor','none'); %,'facealpha',0.5
%         alpha(HexacopteroPlot(end),0.6)
%     end
    
%     for pp=[2 3]
%         dHexarotor= hexacopter.heliceI{pp}*rotZ;
%         HexacopteroPlot(end+1)=patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,colorFrameN,'EdgeColor','none');
%         alpha(HexacopteroPlot(end),0.6)
%     end
    %% Ploteo de las helices exteriores
    for pp=[1 5 6 4]
        dHexarotor= hexacopter.helice{pp}*rotZ;
        HexacopteroPlot(end+1)= patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,colorFrameN); %,'facealpha',0.5
        alpha(HexacopteroPlot(end),0.2)
    end
    
    for pp=[2 3]
        dHexarotor= hexacopter.helice{pp}*rotZ;
        HexacopteroPlot(end+1)=patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,colorFrameN,'EdgeColor','red');
        alpha(HexacopteroPlot(end),0.2)
    end
    %% Ploteo de las tapas de la computadora

    dHexarotor= hexacopter.tapaP{1}*rotZ;
    HexacopteroPlot(end+1)= patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,[1 0 0]);
    
    dHexarotor= hexacopter.tapaP{2}*rotZ;
    HexacopteroPlot(end+1)= patch(dHexarotor(:,1)+x,dHexarotor(:,2)+y,dHexarotor(:,3)+z,[1 0 0]);
    
    %% ploteo de la computadora
    dHexarotor= hexacopter.computadora{1}*rotZ;
    dHexarotor(:,1)= dHexarotor(:,1)+x; dHexarotor(:,2)= dHexarotor(:,2)+y;  dHexarotor(:,3)= dHexarotor(:,3)+z;  
    TRI= hexacopter.computadora{2};
    HexacopteroPlot(end+1)= patch('Vertices',dHexarotor,'Faces',TRI,'facecolor',[0 0 0]);
    
end