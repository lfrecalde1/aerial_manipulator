function PLOT_3D=Manipulador3D(x,y,z,psi,a,b,c,q1,q2,q3,q4)

% Funcion "Manipulador3D" genera el gráfico del brazo robótico para la animación

% 1) ARGUMENTOS DE ENTRADA
  % a) x ----> Posición X del UAV
  % b) y ----> Posición Y del UAV
  % c) z ----> Posición Z del UAV
  % d) th ---> Posición angular YAW del UAV
  % e) q1 ---> Posición angular del eslabon 1
  % f) q2 ---> Posición angular del eslabon 2
  % g) q3 ---> Posición angular del eslabon 3
  % h) q4 ---> Posición angular del eslabon 4
     
% 2) ARGUMENTOS DE SALIDA
  % Plot Brazo Robótico
  
global R Dim
Dim.b=0;
color1=[0 0 1];
color4=[.2 .2 .2];
color5=[.1 .1 .1];
q1 = -q1;
psi = -psi;
b=-b;
c=-c;
R0=[cos(psi) -sin(psi) 0;
    sin(psi)  cos(psi) 0;
    0        0       1];

R1=[cos(q1) -sin(q1) 0;
    sin(q1)  cos(q1) 0;
    0        0       1];
R2=[cos(q2) 0 sin(q2);
    0       1 0;
    -sin(q2) 0 cos(q2)];
R3=[cos(q3) 0  sin(q3);
    0       1  0;
   -sin(q3) 0 cos(q3)];
R4=[cos(q4) 0 sin(q4);
    0       1 0;
    -sin(q4) 0 cos(q4)];

% Brazo
% Tapa1Motor4(1,:)=Tapa1Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
% Tapa1Motor4(3,:)=Tapa1Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;

Tapa1Motor1=R0*R.Tapa1Motor1; Tapa1Motor1(1,:)=Tapa1Motor1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa1Motor1(2,:)=Tapa1Motor1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa2Motor1=R0*R.Tapa2Motor1; Tapa2Motor1(1,:)=Tapa2Motor1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa2Motor1(2,:)=Tapa2Motor1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa3Motor1=R0*R.Tapa3Motor1; Tapa3Motor1(1,:)=Tapa3Motor1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa3Motor1(2,:)=Tapa3Motor1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa4Motor1=R0*R.Tapa4Motor1; Tapa4Motor1(1,:)=Tapa4Motor1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa4Motor1(2,:)=Tapa4Motor1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa5Motor1=R0*R.Tapa5Motor1; Tapa5Motor1(1,:)=Tapa5Motor1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa5Motor1(2,:)=Tapa5Motor1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);


Tapa1Motor2=R0*R1*R.Tapa1Motor2;
Tapa1Motor2(1,:)=Tapa1Motor2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);
Tapa1Motor2(2,:)=Tapa1Motor2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);

Tapa2Motor2=R0*R1*R.Tapa2Motor2;Tapa2Motor2(1,:)=Tapa2Motor2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi); Tapa2Motor2(2,:)=Tapa2Motor2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa3Motor2=R0*R1*R.Tapa3Motor2;Tapa3Motor2(1,:)=Tapa3Motor2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi); Tapa3Motor2(2,:)=Tapa3Motor2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa4Motor2=R0*R1*R.Tapa4Motor2;Tapa4Motor2(1,:)=Tapa4Motor2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi); Tapa4Motor2(2,:)=Tapa4Motor2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa5Motor2=R0*R1*R.Tapa5Motor2;Tapa5Motor2(1,:)=Tapa5Motor2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi); Tapa5Motor2(2,:)=Tapa5Motor2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);

Frame1_1=R0*R1*R2*R.Frame1_1;
Frame1_1(1,:)=Frame1_1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);
Frame1_1(2,:)=Frame1_1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Frame1_1(3,:)=-Frame1_1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;

Frame1_2=R0*R1*R2*R.Frame1_2;Frame1_2(1,:)=Frame1_2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Frame1_2(2,:)=Frame1_2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Frame1_2(3,:)=-Frame1_2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;
Frame1_3=R0*R1*R2*R.Frame1_3;Frame1_3(1,:)=Frame1_3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Frame1_3(2,:)=Frame1_3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Frame1_3(3,:)=-Frame1_3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;

Slabon1_1=R0*R1*R2*R.Slabon1_1;Slabon1_1(1,:)=Slabon1_1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Slabon1_1(2,:)=Slabon1_1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Slabon1_1(3,:)=-Slabon1_1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;
Slabon1_2=R0*R1*R2*R.Slabon1_2;Slabon1_2(1,:)=Slabon1_2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Slabon1_2(2,:)=Slabon1_2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Slabon1_2(3,:)=-Slabon1_2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;

% por revisar
Tapa1Motor3=R.Tapa1Motor2;Tapa1Motor3(1,:)=Tapa1Motor3(1,:)+1.5*Dim.lm2+Dim.sl1;Tapa1Motor3(3,:)=Tapa1Motor3(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;Tapa1Motor3=R0*R1*R2*Tapa1Motor3;
Tapa1Motor3(1,:)=Tapa1Motor3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa1Motor3(2,:)=Tapa1Motor3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Tapa1Motor3(3,:)=-Tapa1Motor3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;
%

Tapa2Motor3=R.Tapa2Motor2; Tapa2Motor3(1,:)=Tapa2Motor3(1,:)+1.5*Dim.lm2+Dim.sl1;  Tapa2Motor3(3,:)=Tapa2Motor3(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;

Tapa2Motor3=R0*R1*R2*Tapa2Motor3;
Tapa2Motor3(1,:)=Tapa2Motor3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);
Tapa2Motor3(2,:)=Tapa2Motor3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);
Tapa2Motor3(3,:)=-Tapa2Motor3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;



Tapa3Motor3=R.Tapa3Motor2;Tapa3Motor3(1,:)=Tapa3Motor3(1,:)+1.5*Dim.lm2+Dim.sl1;Tapa3Motor3(3,:)=Tapa3Motor3(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;Tapa3Motor3=R0*R1*R2*Tapa3Motor3;
Tapa3Motor3(1,:)=Tapa3Motor3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa3Motor3(2,:)=Tapa3Motor3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Tapa3Motor3(3,:)=-Tapa3Motor3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;
Tapa4Motor3=R.Tapa4Motor2;Tapa4Motor3(1,:)=Tapa4Motor3(1,:)+1.5*Dim.lm2+Dim.sl1;Tapa4Motor3(3,:)=Tapa4Motor3(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;Tapa4Motor3=R0*R1*R2*Tapa4Motor3;
Tapa4Motor3(1,:)=Tapa4Motor3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa4Motor3(2,:)=Tapa4Motor3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Tapa4Motor3(3,:)=-Tapa4Motor3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;
Tapa5Motor3=R.Tapa5Motor2;Tapa5Motor3(1,:)=Tapa5Motor3(1,:)+1.5*Dim.lm2+Dim.sl1;Tapa5Motor3(3,:)=Tapa5Motor3(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;Tapa5Motor3=R0*R1*R2*Tapa5Motor3;
Tapa5Motor3(1,:)=Tapa5Motor3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi);Tapa5Motor3(2,:)=Tapa5Motor3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi);Tapa5Motor3(3,:)=-Tapa5Motor3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2;

Frame2_1=R0*R1*R2*R3*R.Frame1_1;
Frame2_1(1,:)=Frame2_1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);Frame2_1(2,:)=Frame2_1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);Frame2_1(3,:)=-Frame2_1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);
Frame2_2=R.Frame1_2;Frame2_2(1,:)=Frame2_2(1,:);Frame2_2=R0*R1*R2*R3*Frame2_2;
Frame2_2(1,:)=Frame2_2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);Frame2_2(2,:)=Frame2_2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);Frame2_2(3,:)=-Frame2_2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);
Frame2_3=R.Frame1_3;Frame2_3(1,:)=Frame2_3(1,:);Frame2_3=R0*R1*R2*R3*Frame2_3;
Frame2_3(1,:)=Frame2_3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);Frame2_3(2,:)=Frame2_3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);Frame2_3(3,:)=-Frame2_3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);

Slabon2_1=R0*R1*R2*R3*R.Slabon2_1;
Slabon2_1(1,:)=Slabon2_1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);Slabon2_1(2,:)=Slabon2_1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);Slabon2_1(3,:)=-Slabon2_1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);
Slabon2_2=R0*R1*R2*R3*R.Slabon2_2;
Slabon2_2(1,:)=Slabon2_2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);Slabon2_2(2,:)=Slabon2_2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);Slabon2_2(3,:)=-Slabon2_2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);

Tapa1Motor4=R.Tapa1Motor2;
Tapa1Motor4(1,:)=Tapa1Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
Tapa1Motor4(3,:)=Tapa1Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;
Tapa1Motor4=R0*R1*R2*R3*Tapa1Motor4;

Tapa1Motor4(1,:)=Tapa1Motor4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi) +    (1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);
Tapa1Motor4(2,:)=Tapa1Motor4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi) +    (1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);
Tapa1Motor4(3,:)=-Tapa1Motor4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+    (1.5*Dim.lm2+Dim.sl1)*sin(q2);

Tapa2Motor4=R.Tapa2Motor2;
Tapa2Motor4(1,:)=Tapa2Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
Tapa2Motor4(3,:)=Tapa2Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;
Tapa2Motor4=R0*R1*R2*R3*Tapa2Motor4;
Tapa2Motor4(1,:)=Tapa2Motor4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+    (1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);
Tapa2Motor4(2,:)=Tapa2Motor4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+    (1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);
Tapa2Motor4(3,:)=-Tapa2Motor4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+    (1.5*Dim.lm2+Dim.sl1)*sin(q2);


Tapa3Motor4=R.Tapa3Motor2;
Tapa3Motor4(1,:)=Tapa3Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
Tapa3Motor4(3,:)=Tapa3Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;
Tapa3Motor4=R0*R1*R2*R3*Tapa3Motor4;
Tapa3Motor4(1,:)=Tapa3Motor4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+  (1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);
Tapa3Motor4(2,:)=Tapa3Motor4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+  (1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);
Tapa3Motor4(3,:)=-Tapa3Motor4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+  (1.5*Dim.lm2+Dim.sl1)*sin(q2);

Tapa4Motor4=R.Tapa4Motor2;
Tapa4Motor4(1,:)=Tapa4Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
Tapa4Motor4(3,:)=Tapa4Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;
Tapa4Motor4=R0*R1*R2*R3*Tapa4Motor4;
Tapa4Motor4(1,:)=Tapa4Motor4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);
Tapa4Motor4(2,:)=Tapa4Motor4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);
Tapa4Motor4(3,:)=-Tapa4Motor4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2);

Tapa5Motor4=R.Tapa5Motor2;
Tapa5Motor4(1,:)=Tapa5Motor4(1,:)+1.5*Dim.lm2+Dim.sl2;
Tapa5Motor4(3,:)=Tapa5Motor4(3,:)-Dim.h2-Dim.lh1-Dim.lh2-Dim.lm2/2;
Tapa5Motor4=R0*R1*R2*R3*Tapa5Motor4;
Tapa5Motor4(1,:)=Tapa5Motor4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+  (1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2);
Tapa5Motor4(2,:)=Tapa5Motor4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+  (1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2);
Tapa5Motor4(3,:)=-Tapa5Motor4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+  (1.5*Dim.lm2+Dim.sl1)*sin(q2);



Frame3_1=R0*R1*R2*R3*R4*R.Frame1_1;
Frame3_1(1,:)=Frame3_1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Frame3_1(2,:)=Frame3_1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Frame3_1(3,:)=-Frame3_1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Frame3_2=R0*R1*R2*R3*R4*R.Frame1_2;
Frame3_2(1,:)=Frame3_2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Frame3_2(2,:)=Frame3_2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Frame3_2(3,:)=-Frame3_2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Frame3_3=R0*R1*R2*R3*R4*R.Frame1_3;
Frame3_3(1,:)=Frame3_3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Frame3_3(2,:)=Frame3_3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Frame3_3(3,:)=-Frame3_3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);

Tapa1Motor5=R0*R1*R2*R3*R4*R.Tapa1Motor5;
Tapa1Motor5(1,:)=Tapa1Motor5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Tapa1Motor5(2,:)=Tapa1Motor5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Tapa1Motor5(3,:)=-Tapa1Motor5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Tapa2Motor5=R0*R1*R2*R3*R4*R.Tapa2Motor5;
Tapa2Motor5(1,:)=Tapa2Motor5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Tapa2Motor5(2,:)=Tapa2Motor5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Tapa2Motor5(3,:)=-Tapa2Motor5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Tapa3Motor5=R0*R1*R2*R3*R4*R.Tapa3Motor5;
Tapa3Motor5(1,:)=Tapa3Motor5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Tapa3Motor5(2,:)=Tapa3Motor5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Tapa3Motor5(3,:)=-Tapa3Motor5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Tapa4Motor5=R0*R1*R2*R3*R4*R.Tapa4Motor5;
Tapa4Motor5(1,:)=Tapa4Motor5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Tapa4Motor5(2,:)=Tapa4Motor5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Tapa4Motor5(3,:)=-Tapa4Motor5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);

Tapa5Motor5=R0*R1*R2*R3*R4*R.Tapa5Motor5;
Tapa5Motor5(1,:)=Tapa5Motor5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Tapa5Motor5(2,:)=Tapa5Motor5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Tapa5Motor5(3,:)=-Tapa5Motor5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);

Garra1=R0*R1*R2*R3*R4*R.Garra1;
Garra1(1,:)=Garra1(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra1(2,:)=Garra1(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra1(3,:)=-Garra1(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra2=R0*R1*R2*R3*R4*R.Garra2;
Garra2(1,:)=Garra2(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra2(2,:)=Garra2(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra2(3,:)=-Garra2(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra3=R0*R1*R2*R3*R4*R.Garra3;
Garra3(1,:)=Garra3(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra3(2,:)=Garra3(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra3(3,:)=-Garra3(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra4=R0*R1*R2*R3*R4*R.Garra4;
Garra4(1,:)=Garra4(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra4(2,:)=Garra4(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra4(3,:)=-Garra4(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra5=R0*R1*R2*R3*R4*R.Garra5;
Garra5(1,:)=Garra5(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra5(2,:)=Garra5(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra5(3,:)=-Garra5(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra6=R0*R1*R2*R3*R4*R.Garra6;
Garra6(1,:)=Garra6(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra6(2,:)=Garra6(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra6(3,:)=-Garra6(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra7=R0*R1*R2*R3*R4*R.Garra7;
Garra7(1,:)=Garra7(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra7(2,:)=Garra7(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra7(3,:)=-Garra7(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra8=R0*R1*R2*R3*R4*R.Garra8;
Garra8(1,:)=Garra8(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra8(2,:)=Garra8(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra8(3,:)=-Garra8(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra9=R0*R1*R2*R3*R4*R.Garra9;
Garra9(1,:)=Garra9(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra9(2,:)=Garra9(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra9(3,:)=-Garra9(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra10=R0*R1*R2*R3*R4*R.Garra10;
Garra10(1,:)=Garra10(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra10(2,:)=Garra10(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra10(3,:)=-Garra10(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra11=R0*R1*R2*R3*R4*R.Garra11;
Garra11(1,:)=Garra11(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra11(2,:)=Garra11(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra11(3,:)=-Garra11(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra12=R0*R1*R2*R3*R4*R.Garra12;
Garra12(1,:)=Garra12(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra12(2,:)=Garra12(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra12(3,:)=-Garra12(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra13=R0*R1*R2*R3*R4*R.Garra13;
Garra13(1,:)=Garra13(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra13(2,:)=Garra13(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra13(3,:)=-Garra13(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra14=R0*R1*R2*R3*R4*R.Garra14;
Garra14(1,:)=Garra14(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra14(2,:)=Garra14(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra14(3,:)=-Garra14(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);
Garra15=R0*R1*R2*R3*R4*R.Garra15;
Garra15(1,:)=Garra15(1,:)+Dim.a*cos(psi)-Dim.b*sin(psi)+(1.5*Dim.lm2+Dim.sl1)*cos(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*cos(psi+q1)*cos(q2+q3);
Garra15(2,:)=Garra15(2,:)+Dim.a*sin(psi)+Dim.b*cos(psi)+(1.5*Dim.lm2+Dim.sl1)*sin(psi+q1)*cos(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(psi+q1)*cos(q2+q3);
Garra15(3,:)=-Garra15(3,:)+Dim.h2+Dim.lh1+Dim.lh2+Dim.lm2/2+(1.5*Dim.lm2+Dim.sl1)*sin(q2)+(1.5*Dim.lm2+Dim.sl2)*sin(q2+q3);

PLOT_3D(1)=patch(Tapa1Motor1(1,:)+x+a*cos(psi)-b*sin(psi),Tapa1Motor1(2,:)+y+a*sin(psi)+b*cos(psi),Tapa1Motor1(3,:)+z+c,color4);
PLOT_3D(2)=patch(Tapa2Motor1(1,:)+x+a*cos(psi)-b*sin(psi),Tapa2Motor1(2,:)+y+a*sin(psi)+b*cos(psi),Tapa2Motor1(3,:)+z+c,color4);
PLOT_3D(3)=patch(Tapa3Motor1(1,:)+x+a*cos(psi)-b*sin(psi),Tapa3Motor1(2,:)+y+a*sin(psi)+b*cos(psi),Tapa3Motor1(3,:)+z+c,color4);
PLOT_3D(4)=patch(Tapa4Motor1(1,:)+x+a*cos(psi)-b*sin(psi),Tapa4Motor1(2,:)+y+a*sin(psi)+b*cos(psi),Tapa4Motor1(3,:)+z+c,color4);
PLOT_3D(5)=patch(Tapa5Motor1(1,:)+x+a*cos(psi)-b*sin(psi),Tapa5Motor1(2,:)+y+a*sin(psi)+b*cos(psi),Tapa5Motor1(3,:)+z+c,color4);

PLOT_3D(6)=patch(Tapa1Motor2(1,:)+x+a*cos(psi)-b*sin(psi),Tapa1Motor2(2,:)+y+a*sin(psi)+b*cos(psi),Tapa1Motor2(3,:)+z+c,color4);
PLOT_3D(7)=patch(Tapa2Motor2(1,:)+x+a*cos(psi)-b*sin(psi),Tapa2Motor2(2,:)+y+a*sin(psi)+b*cos(psi),Tapa2Motor2(3,:)+z+c,color4);
PLOT_3D(8)=patch(Tapa3Motor2(1,:)+x+a*cos(psi)-b*sin(psi),Tapa3Motor2(2,:)+y+a*sin(psi)+b*cos(psi),Tapa3Motor2(3,:)+z+c,color4);
PLOT_3D(9)=patch(Tapa4Motor2(1,:)+x+a*cos(psi)-b*sin(psi),Tapa4Motor2(2,:)+y+a*sin(psi)+b*cos(psi),Tapa4Motor2(3,:)+z+c,color4);
PLOT_3D(10)=patch(Tapa5Motor2(1,:)+x+a*cos(psi)-b*sin(psi),Tapa5Motor2(2,:)+y+a*sin(psi)+b*cos(psi),Tapa5Motor2(3,:)+z+c,color4);

PLOT_3D(11)=patch(Frame1_1(1,:)+x+a*cos(psi)-b*sin(psi),Frame1_1(2,:)+y+a*sin(psi)+b*cos(psi),Frame1_1(3,:)+z+c,color4);
PLOT_3D(12)=patch(Frame1_2(1,:)+x+a*cos(psi)-b*sin(psi),Frame1_2(2,:)+y+a*sin(psi)+b*cos(psi),Frame1_2(3,:)+z+c,color4);
PLOT_3D(13)=patch(Frame1_3(1,:)+x+a*cos(psi)-b*sin(psi),Frame1_3(2,:)+y+a*sin(psi)+b*cos(psi),Frame1_3(3,:)+z+c,color4);

PLOT_3D(14)=patch(Slabon1_1(1,:)+x+a*cos(psi)-b*sin(psi),Slabon1_1(2,:)+y+a*sin(psi)+b*cos(psi),Slabon1_1(3,:)+z+c,color1);
PLOT_3D(15)=patch(Slabon1_2(1,:)+x+a*cos(psi)-b*sin(psi),Slabon1_2(2,:)+y+a*sin(psi)+b*cos(psi),Slabon1_2(3,:)+z+c,color1);

PLOT_3D(16)=patch(Tapa1Motor3(1,:)+x+a*cos(psi)-b*sin(psi),Tapa1Motor3(2,:)+y+a*sin(psi)+b*cos(psi),Tapa1Motor3(3,:)+z+c,color4);
PLOT_3D(17)=patch(Tapa2Motor3(1,:)+x+a*cos(psi)-b*sin(psi),Tapa2Motor3(2,:)+y+a*sin(psi)+b*cos(psi),Tapa2Motor3(3,:)+z+c,color4);
PLOT_3D(18)=patch(Tapa3Motor3(1,:)+x+a*cos(psi)-b*sin(psi),Tapa3Motor3(2,:)+y+a*sin(psi)+b*cos(psi),Tapa3Motor3(3,:)+z+c,color4);
PLOT_3D(19)=patch(Tapa4Motor3(1,:)+x+a*cos(psi)-b*sin(psi),Tapa4Motor3(2,:)+y+a*sin(psi)+b*cos(psi),Tapa4Motor3(3,:)+z+c,color4);
PLOT_3D(20)=patch(Tapa5Motor3(1,:)+x+a*cos(psi)-b*sin(psi),Tapa5Motor3(2,:)+y+a*sin(psi)+b*cos(psi),Tapa5Motor3(3,:)+z+c,color4);

PLOT_3D(21)=patch(Frame2_1(1,:)+x+a*cos(psi)-b*sin(psi),Frame2_1(2,:)+y+a*sin(psi)+b*cos(psi),Frame2_1(3,:)+z+c,color4);
PLOT_3D(22)=patch(Frame2_2(1,:)+x+a*cos(psi)-b*sin(psi),Frame2_2(2,:)+y+a*sin(psi)+b*cos(psi),Frame2_2(3,:)+z+c,color4);
PLOT_3D(23)=patch(Frame2_3(1,:)+x+a*cos(psi)-b*sin(psi),Frame2_3(2,:)+y+a*sin(psi)+b*cos(psi),Frame2_3(3,:)+z+c,color4);

PLOT_3D(24)=patch(Slabon2_1(1,:)+x+a*cos(psi)-b*sin(psi),Slabon2_1(2,:)+y+a*sin(psi)+b*cos(psi),Slabon2_1(3,:)+z+c,color1);
PLOT_3D(25)=patch(Slabon2_2(1,:)+x+a*cos(psi)-b*sin(psi),Slabon2_2(2,:)+y+a*sin(psi)+b*cos(psi),Slabon2_2(3,:)+z+c,color1);

PLOT_3D(26)=patch(Tapa1Motor4(1,:)+x+a*cos(psi)-b*sin(psi),Tapa1Motor4(2,:)+y+a*sin(psi)+b*cos(psi),Tapa1Motor4(3,:)+z+c,color4);
PLOT_3D(27)=patch(Tapa2Motor4(1,:)+x+a*cos(psi)-b*sin(psi),Tapa2Motor4(2,:)+y+a*sin(psi)+b*cos(psi),Tapa2Motor4(3,:)+z+c,color4);
PLOT_3D(28)=patch(Tapa3Motor4(1,:)+x+a*cos(psi)-b*sin(psi),Tapa3Motor4(2,:)+y+a*sin(psi)+b*cos(psi),Tapa3Motor4(3,:)+z+c,color4);
PLOT_3D(29)=patch(Tapa4Motor4(1,:)+x+a*cos(psi)-b*sin(psi),Tapa4Motor4(2,:)+y+a*sin(psi)+b*cos(psi),Tapa4Motor4(3,:)+z+c,color4);
PLOT_3D(30)=patch(Tapa5Motor4(1,:)+x+a*cos(psi)-b*sin(psi),Tapa5Motor4(2,:)+y+a*sin(psi)+b*cos(psi),Tapa5Motor4(3,:)+z+c,color4);

PLOT_3D(31)=patch(Frame3_1(1,:)+x+a*cos(psi)-b*sin(psi),Frame3_1(2,:)+y+a*sin(psi)+b*cos(psi),Frame3_1(3,:)+z+c,color4);
PLOT_3D(32)=patch(Frame3_2(1,:)+x+a*cos(psi)-b*sin(psi),Frame3_2(2,:)+y+a*sin(psi)+b*cos(psi),Frame3_2(3,:)+z+c,color4);
PLOT_3D(33)=patch(Frame3_3(1,:)+x+a*cos(psi)-b*sin(psi),Frame3_3(2,:)+y+a*sin(psi)+b*cos(psi),Frame3_3(3,:)+z+c,color4);

PLOT_3D(34)=patch(Tapa1Motor5(1,:)+x+a*cos(psi)-b*sin(psi),Tapa1Motor5(2,:)+y+a*sin(psi)+b*cos(psi),Tapa1Motor5(3,:)+z+c,color4);
PLOT_3D(35)=patch(Tapa2Motor5(1,:)+x+a*cos(psi)-b*sin(psi),Tapa2Motor5(2,:)+y+a*sin(psi)+b*cos(psi),Tapa2Motor5(3,:)+z+c,color4);
PLOT_3D(36)=patch(Tapa3Motor5(1,:)+x+a*cos(psi)-b*sin(psi),Tapa3Motor5(2,:)+y+a*sin(psi)+b*cos(psi),Tapa3Motor5(3,:)+z+c,color4);
PLOT_3D(37)=patch(Tapa4Motor5(1,:)+x+a*cos(psi)-b*sin(psi),Tapa4Motor5(2,:)+y+a*sin(psi)+b*cos(psi),Tapa4Motor5(3,:)+z+c,color4);
PLOT_3D(38)=patch(Tapa5Motor5(1,:)+x+a*cos(psi)-b*sin(psi),Tapa5Motor5(2,:)+y+a*sin(psi)+b*cos(psi),Tapa5Motor5(3,:)+z+c,color4);

PLOT_3D(39)=patch(Garra1(1,:)+x+a*cos(psi)-b*sin(psi),Garra1(2,:)+y+a*sin(psi)+b*cos(psi),Garra1(3,:)+z+c,color5);
PLOT_3D(40)=patch(Garra2(1,:)+x+a*cos(psi)-b*sin(psi),Garra2(2,:)+y+a*sin(psi)+b*cos(psi),Garra2(3,:)+z+c,color5);
PLOT_3D(41)=patch(Garra3(1,:)+x+a*cos(psi)-b*sin(psi),Garra3(2,:)+y+a*sin(psi)+b*cos(psi),Garra3(3,:)+z+c,color5);
PLOT_3D(42)=patch(Garra4(1,:)+x+a*cos(psi)-b*sin(psi),Garra4(2,:)+y+a*sin(psi)+b*cos(psi),Garra4(3,:)+z+c,color5);
PLOT_3D(43)=patch(Garra5(1,:)+x+a*cos(psi)-b*sin(psi),Garra5(2,:)+y+a*sin(psi)+b*cos(psi),Garra5(3,:)+z+c,color5);
PLOT_3D(44)=patch(Garra6(1,:)+x+a*cos(psi)-b*sin(psi),Garra6(2,:)+y+a*sin(psi)+b*cos(psi),Garra6(3,:)+z+c,color5);
PLOT_3D(45)=patch(Garra7(1,:)+x+a*cos(psi)-b*sin(psi),Garra7(2,:)+y+a*sin(psi)+b*cos(psi),Garra7(3,:)+z+c,color5);
PLOT_3D(46)=patch(Garra8(1,:)+x+a*cos(psi)-b*sin(psi),Garra8(2,:)+y+a*sin(psi)+b*cos(psi),Garra8(3,:)+z+c,color5);
PLOT_3D(47)=patch(Garra9(1,:)+x+a*cos(psi)-b*sin(psi),Garra9(2,:)+y+a*sin(psi)+b*cos(psi),Garra9(3,:)+z+c,color5);
PLOT_3D(48)=patch(Garra10(1,:)+x+a*cos(psi)-b*sin(psi),Garra10(2,:)+y+a*sin(psi)+b*cos(psi),Garra10(3,:)+z+c,color5);
PLOT_3D(49)=patch(Garra11(1,:)+x+a*cos(psi)-b*sin(psi),Garra11(2,:)+y+a*sin(psi)+b*cos(psi),Garra11(3,:)+z+c,color5);
PLOT_3D(50)=patch(Garra12(1,:)+x+a*cos(psi)-b*sin(psi),Garra12(2,:)+y+a*sin(psi)+b*cos(psi),Garra12(3,:)+z+c,color5);
PLOT_3D(51)=patch(Garra13(1,:)+x+a*cos(psi)-b*sin(psi),Garra13(2,:)+y+a*sin(psi)+b*cos(psi),Garra13(3,:)+z+c,color5);
PLOT_3D(52)=patch(Garra14(1,:)+x+a*cos(psi)-b*sin(psi),Garra14(2,:)+y+a*sin(psi)+b*cos(psi),Garra14(3,:)+z+c,color5);
PLOT_3D(53)=patch(Garra15(1,:)+x+a*cos(psi)-b*sin(psi),Garra15(2,:)+y+a*sin(psi)+b*cos(psi),Garra15(3,:)+z+c,color5);



% axis equal;xlabel('X');ylabel('Y');zlabel('Z')