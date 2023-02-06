function SIMULACION(a,largo,ancho,hx,hy,hxd,hyd,phi,ts)

%% PARAMETROS PARA GRAFICAR EL ROBOT
Length_car=largo;
Width_car=ancho; 
Lc_1=(Length_car/3)+a;
Lc_2=Length_car-Lc_1;
Wc=Width_car/2;
base_robot_1=0.03;
scale=1;
%% paramtros dimensiones
car_1 = [-Lc_1,Lc_2,Lc_2,-Lc_1;-Wc,-Wc,Wc,Wc;0,0,0,0;1,1,1,1]*scale;
car_2 = [-Lc_1,Lc_2,Lc_2,-Lc_1;-Wc,-Wc,Wc,Wc;-0.1,-0.1,-0.1,-0.1;1,1,1,1]*scale;
car_3 = [-Lc_1,Lc_2,Lc_2,-Lc_1;-Wc,-Wc,-Wc,-Wc;0,0,-0.1,-0.1;1,1,1,1]*scale;
car_4 = [-Lc_1,Lc_2,Lc_2,-Lc_1;Wc,Wc,Wc,Wc;0,0,-0.1,-0.1;1,1,1,1]*scale;
car_5 = [-Lc_1,-Lc_1,-Lc_1,-Lc_1;-Wc,Wc,Wc,-Wc;0,0,-0.1,-0.1;1,1,1,1]*scale;
car_6 = [Lc_2,Lc_2,Lc_2,Lc_2;-Wc,Wc,Wc,-Wc;0,0,-0.1,-0.1;1,1,1,1]*scale;
car_centro_1=[-base_robot_1,base_robot_1,base_robot_1,-base_robot_1;-base_robot_1,-base_robot_1,base_robot_1,base_robot_1;0,0,0,0;1,1,1,1]*scale;

Rotz_r_1=[cos(phi(1)),-sin(phi(1)),0,0;sin(phi(1)),cos(phi(1)),0,0;0,0,1,0;0,0,0,1];

carr_1=Rotz_r_1*car_1;
carr_2=Rotz_r_1*car_2;
carr_3=Rotz_r_1*car_3;
carr_4=Rotz_r_1*car_4;
carr_5=Rotz_r_1*car_5;
carr_6=Rotz_r_1*car_6;
carr_centro_1=Rotz_r_1*car_centro_1;

%% grafica real parametros ventana

figure
axis equal;
carro_1= patch(carr_1(1,:)+hx(1), carr_1(2,:)+hy(1), carr_1(3,:),[0 1 1]);
carro_2= patch(carr_2(1,:)+hx(1), carr_2(2,:)+hy(1), carr_2(3,:),[0 1 1]);
carro_3= patch(carr_3(1,:)+hx(1), carr_3(2,:)+hy(1), carr_3(3,:),[0 1 1]);
carro_4= patch(carr_4(1,:)+hx(1), carr_4(2,:)+hy(1), carr_4(3,:),[0 1 1]);
carro_5= patch(carr_5(1,:)+hx(1), carr_5(2,:)+hy(1), carr_5(3,:),[0 1 1]);
carro_6= patch(carr_6(1,:)+hx(1), carr_6(2,:)+hy(1), carr_6(3,:),[0 1 1]);
carro_centro_1= patch(carr_centro_1(1,:)+hx(1), carr_centro_1(2,:)+hy(1), carr_centro_1(3,:),[0 0 1]);
h3=plot(hxd(1),hyd(1),'g'); hold on
h4=plot(hx(1),hy(1),'k'); grid on
title('Animacion')
xlabel('x [m]'); ylabel('y [m]'); zlabel('Z [m]');
ylim([-4 4]);
xlim([-4 4]);
grid on
for k=1:length(hxd)
    drawnow;
    delete(carro_1);
    delete(carro_2);
    delete(carro_3);
    delete(carro_4);
    delete(carro_5);
    delete(carro_6);
    delete(carro_centro_1); 
    delete(h3);
    delete(h4);
    zlim([-1 1]);
   
    
    %% parametros graficos movil uno
    Rotz_r_1=[cos(phi(k)),-sin(phi(k)),0,0;sin(phi(k)),cos(phi(k)),0,0;0,0,1,0;0,0,0,1];
    carr_1=Rotz_r_1*car_1;
    carr_2=Rotz_r_1*car_2;
    carr_3=Rotz_r_1*car_3;
    carr_4=Rotz_r_1*car_4;
    carr_5=Rotz_r_1*car_5;
    carr_6=Rotz_r_1*car_6;
    carr_centro_1=Rotz_r_1*car_centro_1;
   
    
    %% graficos mocil uno
    carro_1= patch(carr_1(1,:)+hx(k), carr_1(2,:)+hy(k), carr_1(3,:),[0.5 0.5 0.5]);
    carro_2= patch(carr_2(1,:)+hx(k), carr_2(2,:)+hy(k), carr_2(3,:),[0.5 0.5 0.5]);
    carro_3= patch(carr_3(1,:)+hx(k), carr_3(2,:)+hy(k), carr_3(3,:),[0.5 0.5 0.5]);
    carro_4= patch(carr_4(1,:)+hx(k), carr_4(2,:)+hy(k), carr_4(3,:),[0.5 0.5 0.5]);
    carro_5= patch(carr_5(1,:)+hx(k), carr_5(2,:)+hy(k), carr_5(3,:),[0.5 0.5 0.5]);
    carro_6= patch(carr_6(1,:)+hx(k), carr_6(2,:)+hy(k), carr_6(3,:),[0 0 0]);
    carro_centro_1= patch(carr_centro_1(1,:)+hx(k), carr_centro_1(2,:)+hy(k), carr_centro_1(3,:),[0 0 1]);

    h3=plot(hx(1:k),hy(1:k),"k"); hold on 
    h4=plot(hxd(1:k),hyd(1:k),"g"); hold on
    grid on
    pause(ts);
    
    
end

end

