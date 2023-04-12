clc
clear all
close all
%% Cinematica
l_1=sym('l_1','real');l_2=sym('l_2','real');l_3=sym('l_3','real'); 
q_1=sym('q_1','real');q_2=sym('q_2','real');q_3=sym('q_3','real');
x=sym('x','real');y=sym('y','real');z=sym('z','real');th=sym('th','real');
L1=[0 0 -l_1]';
L2=[l_2 0 0]';
L3=[l_3 0 0]';
R_uav=[x y z]';
R0=[cos(th) -sin(th) 0;sin(th) cos(th) 0;0 0 1];
R1=[cos(q_1) -sin(q_1) 0;sin(q_1) cos(q_1) 0;0 0 1];
R2=[cos(q_2) 0 sin(q_2);0 1 0;-sin(q_2) 0 cos(q_2)];
R3=[cos(q_3) 0 sin(q_3);0 1 0;-sin(q_3) 0 cos(q_3)];
H1=R_uav+R0*R1*L1;
H2=simplify(R0*R1*R2*L2+H1);
H3=simplify(R0*R1*R2*R3*L3+H2);

E=[x y z th q_1 q_2 q_3];
Ep=difft(E,E);
Epp=difft(Ep,Ep);

H1p=difft(H1,E);
H2p=difft(H2,E);
H3p=difft(H3,E);

J=jacobian(H3,[x y z th q_1 q_2 q_3]);
%% E. Cinetica
xp=sym('xp','real');yp=sym('yp','real');zp=sym('zp','real');
thp=sym('thp','real');q_1p=sym('q_1p','real');
q_2p=sym('q_2p','real');q_3p=sym('q_3p','real');
m_uav=sym('m_uav','real');J_uav=sym('J_uav','real');
m_1=sym('m_1','real');m_2=sym('m_2','real');m_3=sym('m_3','real');
R_uavp=difft(R_uav,[x y z]);
% Rp=[H1p H2p H3p];
K0=1/2*[R_uavp;thp]'*[m_uav*eye(3) zeros(3,1);zeros(1,3) J_uav]*[R_uavp;thp];
K1=1/2*H1p'*m_1*eye(3)*H1p;
K2=1/2*H2p'*m_2*eye(3)*H2p;
K3=1/2*H3p'*m_3*eye(3)*H3p;
K=K0+K1+K2+K3;
%% E. Potencial
g=sym('g','real');
ez=[0 0 1];
V=g*(m_uav*ez*R_uav+m_1*ez*H1+m_2*ez*H2+m_3*ez*H3);

%% Lagrange
L=K-V;
txt_L=mathtype(L);
for i=1:length(E)
    dLE=diffp(L,E(i));
    dLEp=diffp(L,Ep(i));
    dLEpt=difft(dLEp,[E Ep]);
    T(i)=simplify(dLEpt-dLE);
end

%% Matricial
for i=1:length(T)
    for j=1:length(Epp)
   [c,t] = coeffs(T(i), Epp(j));if t==1; M(i,j)=sym(0);else, M(i,j)=c(1);end
    end
end

for i=1:length(T)
    for j=1:length(Ep)
        [c,t] = coeffs(T(i), Ep(j));
        if t==1
            C(i,j)=sym(0);
        elseif length(t)==3
            C(i,j)=c(1)*Ep(j);
            C(i,j)=c(2)/2+C(i,j);
        elseif isequaln(t(1),Ep(j)^2)
            C(i,j)=c(1)*Ep(j);
        else
            C(i,j)=c(1)/2;
        end
    end
end

for i=1:length(E)
    [c,t] = coeffs(T(i),[Ep Epp]);
    if t(length(t))~=1;
        G(i)=sym(0);
    else
        G(i)=c(length(t));
    end
end
(M(1,7)-M(7,1))
size(M)
M_p=difft(M,[E]);
Y1=simplify((C+C')-M_p);
Y2=simplify(T'-M*Epp'-C*Ep'-G');
Y3=simplify(M-M');
%% El otro lado
K_1=sym('K_1','real');K_2=sym('K_2','real');K_3=sym('K_3','real');K_4=sym('K_4','real');
b_1=sym('b_1','real');b_2=sym('b_2','real');b_3=sym('b_3','real');b_4=sym('b_4','real');
K_ap=sym('K_ap','real');R_ap=sym('R_ap','real');
K_a=sym('K_a','real');R_a=sym('R_a','real');
K_bp=sym('K_bp','real');K_b=sym('K_b','real');
a_1=sym('a_1','real');a_2=sym('a_2','real');
a_3=sym('a_3','real');a_4=sym('a_4','real');
K_p=sym('K_p','real');K_p1=sym('K_p1','real');
K_p2=sym('K_p2','real');K_p3=sym('K_p3','real');
K_p4=sym('K_p4','real');
K_d=sym('K_d','real');K_d1=sym('K_d1','real');
K_d2=sym('K_d2','real');K_d3=sym('K_d3','real');
K_d4=sym('K_d4','real');
R_u=[R0 zeros(3,4);zeros(4,3) eye(4)];
Ap=[-K_1 -K_1 K_1 K_1;
    -K_1 K_1 K_1 -K_1;
    K_1 K_1 K_1 K_1;
    b_1 -b_1 b_1 -b_1];
A=[Ap zeros(4,3);zeros(3,4),eye(3)];


Bp=K_ap/R_ap*eye(4);
Ba=K_a/R_a*eye(3);
B=[Bp zeros(4,3);zeros(3,4) Ba];

Yp=K_ap*K_bp/R_ap*eye(4);
Ya=K_a*K_b/R_a*eye(3);
Y=[Yp zeros(4,3);zeros(3,4) Ya];

Dp=1/4*[-1 -1 1 1;-1 1 1 -1;1 1 1 1;1 -1 1 -1];
D=[Dp zeros(4,3);zeros(3,4) eye(3)];

Zp=[a_1*[-1 -1 1 1];a_1*[-1 1 1 -1];
    a_3*[1 1 1 1];a_4*[1 -1 1 -1]];
Z=[Zp zeros(4,3);zeros(3,4) eye(3)];

Fp=diag([K_p1 K_p2 K_p3 K_p4]);
F=[Fp zeros(4,3);zeros(3,4) K_p*eye(3)];
H=diag([K_d1 K_d2 K_d3 K_d4 K_d K_d K_d]);

J2=R_u;
J2p=difft(J2,E);

I=simplify(R_u*A*B*inv(D)*F);
L=simplify(I-R_u*A*Y*inv(Z));
N=simplify(R_u*A*H);

O=simplify(M*J2);
P=simplify(M*J2p+C*J2);

MM=simplify(inv(I)*(O+N));
CC=simplify(inv(I)*(P+L));
GG=simplify(inv(I)*G');

%%  sustituciones
x_1=sym('x_1','real'); x_2=sym('x_2','real');
x_3=sym('x_3','real'); x_4=sym('x_4','real');
x_5=sym('x_5','real');x_6=sym('x_6','real');
x_7=sym('x_7','real');x_8=sym('x_8','real');
x_9=sym('x_9','real');x_10=sym('x_10','real');
x_11=sym('x_11','real');x_12=sym('x_12','real');
x_13=sym('x_13','real');x_14=sym('x_14','real');
x_15=sym('x_15','real');x_16=sym('x_16','real');
x_17=sym('x_17','real');x_18=sym('x_18','real');
x_19=sym('x_19','real');x_20=sym('x_20','real');
x_21=sym('x_21','real');x_22=sym('x_22','real');
x_23=sym('x_23','real');x_24=sym('x_24','real');
x_25=sym('x_25','real');x_26=sym('x_26','real');
x_27=sym('x_27','real');x_28=sym('x_28','real');
x_29=sym('x_29','real');x_30=sym('x_30','real');
x_31=sym('x_31','real');x_32=sym('x_32','real');
x_33=sym('x_33','real');x_34=sym('x_34','real');

SubProgMM;
SubProgCC;
SubProgGG;
MM3
CC3
GG3
