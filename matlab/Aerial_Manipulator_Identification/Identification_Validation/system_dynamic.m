function [VP] = system_dynamic(chi, q, qref)

%% NEW REFERENCE SIGNAL
VREF = [qref];

%% CONSTANT VALUES
l_2= 0.092;
l_3= 0.196;
g= 9.81;
%% GET VALUES

u1 = q(1);
u2 = q(2);
uz = q(3);
thp = q(4);
q_1p = q(5);
q_2p = q(6);
q_3p = q(7);
q_1 = q(8);
q_2 = q(9);
q_3 = q(10);

V = [u1; u2; uz; thp; q_1p; q_2p; q_3p; q_1; q_2; q_3];

%% AUXILIAR VARIABLES
C_1=cos(q_1);
S_1=sin(q_1);
C_2=cos(q_2);
S_2=sin(q_2);
C_3=cos(q_3);
S_3=sin(q_3);
C_22=cos(2*q_2);
C_223=cos(2*q_2+q_3);
C_2223=cos(2*q_2+2*q_3);
S_22=sin(2*q_2);
S_223=sin(2*q_2+q_3);
S_2223=sin(2*q_2+2*q_3);
C_23=cos(q_2+q_3);
S_23=sin(q_2+q_3);

%% INERTIA MATRIX
M11=chi(1);
M12=-chi(2);
M13=chi(3);
M14=-chi(4)*(C_2*S_1*chi(5)-chi(8)+C_2*S_1*chi(6)+C_2*C_3*S_1*chi(7)-S_1*S_2*S_3*chi(7));
M15=-S_1*chi(4)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M16=-C_1*chi(4)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
M17=-C_1*S_23*chi(4)*chi(7);

M21=-chi(9);
M22=chi(10);
M23=chi(11);
M24=chi(12)*(C_1*C_2*chi(5)-chi(8)+C_1*C_2*chi(6)+C_1*C_2*C_3*chi(7)-C_1*S_2*S_3*chi(7));
M25=C_1*chi(12)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M26=-S_1*chi(12)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
M27=-S_1*S_23*chi(7)*chi(12);

M31=chi(13);
M32=chi(14);
M33=chi(15);
M34=chi(16);
M35=0;
M36=-chi(17)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M37=-C_23*chi(7)*chi(17);

M41=chi(18)*(chi(19)-S_1*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7)));
M42=-chi(18)*(chi(20)-C_1*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7)));
M43=chi(21);
M44=(chi(18)*(chi(22)+chi(23)+2*C_3*l_3*chi(6)+C_22*l_2*chi(5)+C_22*l_2*chi(6)+2*C_223*l_3*chi(6)+C_2223*l_3*chi(7)))/2;
M45=(chi(18)*(chi(23)+2*C_3*l_3*chi(6)+C_22*l_2*chi(5)+C_22*l_2*chi(6)+2*C_223*l_3*chi(6)+C_2223*l_3*chi(7)))/2;
M46=0;
M47=0;

M51=-S_1*chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M52=C_1*chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M53=0;
M54=(chi(24)*(chi(23)+2*C_3*l_3*chi(6)+C_22*l_2*chi(5)+C_22*l_2*chi(6)+2*C_223*l_3*chi(6)+C_2223*l_3*chi(7)))/2;
M55=(chi(24)*(chi(23)+2*chi(25)+2*C_3*l_3*chi(6)+C_22*l_2*chi(5)+C_22*l_2*chi(6)+2*C_223*l_3*chi(6)+C_2223*l_3*chi(7)))/2;
M56=0;
M57=0;

M61=-C_1*chi(24)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
M62=-S_1*chi(24)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
M63=-chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
M64=0;
M65=0;
M66=chi(24)*(chi(23)+chi(25)+2*C_3*l_3*chi(6));
M67=chi(7)*chi(24)*(l_3+C_3*l_2);

M71=-C_1*S_23*chi(7)*chi(24);
M72=-S_1*S_23*chi(7)*chi(24);
M73=-C_23*chi(7)*chi(24);
M74=0;
M75=0;
M76=chi(7)*chi(24)*(l_3+C_3*l_2);
M77=chi(24)*(chi(25)+l_3*chi(7));

M=[[M11,M12,M13,M14,M15,M16,M17]
    [M21,M22,M23,M24,M25,M26,M27]
    [M31,M32,M33,M34,M35,M36,M37]
    [M41,M42,M43,M44,M45,M46,M47]
    [M51,M52,M53,M54,M55,M56,M57]
    [M61,M62,M63,M64,M65,M66,M67]
    [M71,M72,M73,M74,M75,M76,M77]];

%% CENTRIFUGAL MATRIX
C11=chi(26);
C12=-thp*chi(4)*chi(27);
C13=0;
C14=chi(4)*(S_1*S_2*q_2p*chi(5)-C_1*C_2*q_1p*chi(6)-C_1*C_2*thp*chi(5)-C_1*C_2*thp*chi(6)-C_1*C_2*q_1p*chi(5)+S_1*S_2*q_2p*chi(6)-C_1*C_2*C_3*q_1p*chi(7)-C_1*C_2*C_3*thp*chi(7)+C_1*S_2*S_3*q_1p*chi(7)+C_2*S_1*S_3*q_2p*chi(7)+C_3*S_1*S_2*q_2p*chi(7)+C_2*S_1*S_3*q_3p*chi(7)+C_3*S_1*S_2*q_3p*chi(7)+C_1*S_2*S_3*thp*chi(7));
C15=chi(4)*(S_1*S_2*q_2p*chi(5)-C_1*C_2*q_1p*chi(6)-C_1*C_2*thp*chi(5)-C_1*C_2*thp*chi(6)-C_1*C_2*q_1p*chi(5)+S_1*S_2*q_2p*chi(6)-C_1*C_2*C_3*q_1p*chi(7)-C_1*C_2*C_3*thp*chi(7)+C_1*S_2*S_3*q_1p*chi(7)+C_2*S_1*S_3*q_2p*chi(7)+C_3*S_1*S_2*q_2p*chi(7)+C_2*S_1*S_3*q_3p*chi(7)+C_3*S_1*S_2*q_3p*chi(7)+C_1*S_2*S_3*thp*chi(7));
C16=chi(4)*(S_1*S_2*q_1p*chi(5)-C_1*C_2*q_2p*chi(6)-C_1*C_2*q_2p*chi(5)+S_1*S_2*q_1p*chi(6)+S_1*S_2*thp*chi(5)+S_1*S_2*thp*chi(6)-C_1*C_2*C_3*q_2p*chi(7)-C_1*C_2*C_3*q_3p*chi(7)+C_2*S_1*S_3*q_1p*chi(7)+C_3*S_1*S_2*q_1p*chi(7)+C_1*S_2*S_3*q_2p*chi(7)+C_1*S_2*S_3*q_3p*chi(7)+C_2*S_1*S_3*thp*chi(7)+C_3*S_1*S_2*thp*chi(7));
C17=chi(4)*chi(7)*(C_2*S_1*S_3*q_1p-C_1*C_2*C_3*q_3p-C_1*C_2*C_3*q_2p+C_3*S_1*S_2*q_1p+C_1*S_2*S_3*q_2p+C_1*S_2*S_3*q_3p+C_2*S_1*S_3*thp+C_3*S_1*S_2*thp);

C21=thp*chi(12)*chi(27);
C22=-chi(28);
C23=0;
C24=-chi(12)*(C_2*S_1*q_1p*chi(5)+C_1*S_2*q_2p*chi(5)+C_2*S_1*q_1p*chi(6)+C_1*S_2*q_2p*chi(6)+C_2*S_1*thp*chi(5)+C_2*S_1*thp*chi(6)+C_2*C_3*S_1*q_1p*chi(7)+C_1*C_2*S_3*q_2p*chi(7)+C_1*C_3*S_2*q_2p*chi(7)+C_1*C_2*S_3*q_3p*chi(7)+C_1*C_3*S_2*q_3p*chi(7)+C_2*C_3*S_1*thp*chi(7)-S_1*S_2*S_3*q_1p*chi(7)-S_1*S_2*S_3*thp*chi(7));
C25=-chi(12)*(C_2*S_1*q_1p*chi(5)+C_1*S_2*q_2p*chi(5)+C_2*S_1*q_1p*chi(6)+C_1*S_2*q_2p*chi(6)+C_2*S_1*thp*chi(5)+C_2*S_1*thp*chi(6)+C_2*C_3*S_1*q_1p*chi(7)+C_1*C_2*S_3*q_2p*chi(7)+C_1*C_3*S_2*q_2p*chi(7)+C_1*C_2*S_3*q_3p*chi(7)+C_1*C_3*S_2*q_3p*chi(7)+C_2*C_3*S_1*thp*chi(7)-S_1*S_2*S_3*q_1p*chi(7)-S_1*S_2*S_3*thp*chi(7));
C26=-chi(12)*(C_1*S_2*q_1p*chi(5)+C_1*S_2*q_1p*chi(6)+C_2*S_1*q_2p*chi(5)+C_2*S_1*q_2p*chi(6)+C_1*S_2*thp*chi(5)+C_1*S_2*thp*chi(6)+C_1*C_2*S_3*q_1p*chi(7)+C_1*C_3*S_2*q_1p*chi(7)+C_2*C_3*S_1*q_2p*chi(7)+C_2*C_3*S_1*q_3p*chi(7)+C_1*C_2*S_3*thp*chi(7)+C_1*C_3*S_2*thp*chi(7)-S_1*S_2*S_3*q_2p*chi(7)-S_1*S_2*S_3*q_3p*chi(7));
C27=-chi(7)*chi(12)*(C_1*C_2*S_3*q_1p+C_1*C_3*S_2*q_1p+C_2*C_3*S_1*q_2p+C_2*C_3*S_1*q_3p+C_1*C_2*S_3*thp+C_1*C_3*S_2*thp-S_1*S_2*S_3*q_2p-S_1*S_2*S_3*q_3p);

C31=0;
C32=0;
C33=-chi(29);
C34=0;
C35=0;
C36=chi(17)*(q_2p*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7))+S_23*q_3p*chi(7));
C37=S_23*chi(7)*chi(17)*(q_2p+q_3p);

C41=C_1*thp*chi(18)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
C42=S_1*thp*chi(18)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
C43=0;
C44=-chi(18)*(chi(30)+(S_3*l_3*q_3p*chi(6))/2+(S_22*l_2*q_2p*chi(5))/2+(S_22*l_2*q_2p*chi(6))/2+S_223*l_3*q_2p*chi(6)+(S_223*l_3*q_3p*chi(6))/2+(S_2223*l_3*q_2p*chi(7))/2+(S_2223*l_3*q_3p*chi(7))/2);
C45=-(chi(18)*(S_3*l_3*q_3p*chi(6)+S_22*l_2*q_2p*chi(5)+S_22*l_2*q_2p*chi(6)+2*S_223*l_3*q_2p*chi(6)+S_223*l_3*q_3p*chi(6)+S_2223*l_3*q_2p*chi(7)+S_2223*l_3*q_3p*chi(7)))/2;
C46=-(chi(18)*(q_1p+thp)*(S_22*l_2*chi(5)+S_22*l_2*chi(6)+2*S_223*l_3*chi(6)+S_2223*l_3*chi(7)))/2;
C47=-(chi(7)*chi(18)*(q_1p+thp)*(S_3*l_2+S_223*l_2+S_2223*l_3))/2;

C51=C_1*thp*chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
C52=S_1*thp*chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
C53=0;
C54=-(chi(24)*(S_3*l_3*q_3p*chi(6)+S_22*l_2*q_2p*chi(5)+S_22*l_2*q_2p*chi(6)+2*S_223*l_3*q_2p*chi(6)+S_223*l_3*q_3p*chi(6)+S_2223*l_3*q_2p*chi(7)+S_2223*l_3*q_3p*chi(7)))/2;
C55=-chi(24)*(chi(31)+(S_3*l_3*q_3p*chi(6))/2+(S_22*l_2*q_2p*chi(5))/2+(S_22*l_2*q_2p*chi(6))/2+S_223*l_3*q_2p*chi(6)+(S_223*l_3*q_3p*chi(6))/2+(S_2223*l_3*q_2p*chi(7))/2+(S_2223*l_3*q_3p*chi(7))/2);
C56=-(chi(24)*(q_1p+thp)*(S_22*l_2*chi(5)+S_22*l_2*chi(6)+2*S_223*l_3*chi(6)+S_2223*l_3*chi(7)))/2;
C57=-(chi(7)*chi(24)*(q_1p+thp)*(S_3*l_2+S_223*l_2+S_2223*l_3))/2;

C61=-S_1*thp*chi(24)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
C62=C_1*thp*chi(24)*(S_2*chi(5)+S_2*chi(6)+S_23*chi(7));
C63=0;
C64=(chi(24)*(q_1p+thp)*(S_22*l_2*chi(5)+S_22*l_2*chi(6)+2*S_223*l_3*chi(6)+S_2223*l_3*chi(7)))/2;
C65=(chi(24)*(q_1p+thp)*(S_22*l_2*chi(5)+S_22*l_2*chi(6)+2*S_223*l_3*chi(6)+S_2223*l_3*chi(7)))/2;
C66=-chi(24)*(chi(31)+S_3*l_3*q_3p*chi(6));
C67=-S_3*l_3*chi(6)*chi(24)*(q_2p+q_3p);

C71=-S_1*S_23*thp*chi(7)*chi(24);
C72=C_1*S_23*thp*chi(7)*chi(24);
C73=0;
C74=(chi(7)*chi(24)*(q_1p+thp)*(S_3*l_2+S_223*l_2+S_2223*l_3))/2;
C75=(chi(7)*chi(24)*(q_1p+thp)*(S_3*l_2+S_223*l_2+S_2223*l_3))/2;
C76=S_3*l_3*q_2p*chi(6)*chi(24);
C77=-chi(32);

C=[[C11,C12,C13,C14,C15,C16,C17]
    [C21,C22,C23,C24,C25,C26,C27]
    [C31,C32,C33,C34,C35,C36,C37]
    [C41,C42,C43,C44,C45,C46,C47]
    [C51,C52,C53,C54,C55,C56,C57]
    [C61,C62,C63,C64,C65,C66,C67]
    [C71,C72,C73,C74,C75,C76,C77]];

%% GRAVITATIONAL VECTOR
G=[0;0;g*chi(17)*chi(27);0;0;
    -g*chi(24)*(C_2*chi(5)+C_2*chi(6)+C_23*chi(7));
    -C_23*g*chi(7)*chi(24)];

A = [-inv(M)*C, zeros(7,3);...
     zeros(3, 4), eye(3,3), zeros(3,3)];
B = [inv(M);...
     zeros(3, 7)];
 
aux = [-inv(M)*G;...
       zeros(3, 1)];
   
VP = A*V  + B*VREF + aux;
end

