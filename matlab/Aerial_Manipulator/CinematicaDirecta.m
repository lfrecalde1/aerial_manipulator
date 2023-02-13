function CineDirecta = CinematicaDirecta(x,y,z,a,b,psi,l1,l2,l3,q1,q2,q3,q4,h)
%% Punto de interes en x
    hx = x + a*cos(psi) - b*sin(psi) + sin(q1+psi)*(-l1*cos(q2)-l2*cos(q2+q3)-...
        l3*cos(q2+q3+q4));
%% Punto de interes en y
    hy = y + a*sin(psi) + b*cos(psi) + cos(q1+psi)*(l1*cos(q2)+l2*cos(q2+q3)+...
        l3*cos(q2+q3+q4));
%% Punto de interes en z
    hz = z - h - l1*sin(q2) - l2*sin(q2+q3) - l3*sin(q2+q3+q4);
%% Retorna hx hy hz
    CineDirecta = [hx,hy,hz];
return