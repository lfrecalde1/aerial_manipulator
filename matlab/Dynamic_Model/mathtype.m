function txt=mathtype(eq)

txt=strrep(char(eq),'*','');
txt=strrep(txt,' ','');
txt=strrep(txt,'J_uav','J_1');
txt=strrep(txt,'m_uav','m_{uav}');
txt=strrep(txt,'m_z','m_1');
txt=strrep(txt,'m2','m_2');
txt=strrep(txt,'m3', 'm_3' );
txt=strrep(txt,'m4', 'm_4' );
txt=strrep(txt,'l2', 'l_2' );
txt=strrep(txt,'l3', 'l_3' );
txt=strrep(txt,'l4', 'l_4' );
txt=strrep(txt,'u1', 'u_1' );
txt=strrep(txt,'u2', 'u_2' );
txt=strrep(txt,'th', '\theta' );
txt=strrep(txt,'q1', 'q_1' );
txt=strrep(txt,'q2', 'q_2' );
txt=strrep(txt,'q3', 'q_3' );
txt=strrep(txt,'q4', 'q_4' );
txt=strrep(txt,'u_1_p','\dot{u}_1');
txt=strrep(txt,'u_2_p','\dot{u}_2');
txt=strrep(txt,'xpp','\ddot{x}');
txt=strrep(txt,'xp','\dot{x}');
txt=strrep(txt,'ypp','\ddot{y}');
txt=strrep(txt,'yp','\dot{y}');
txt=strrep(txt,'zpp','\ddot{z}');
txt=strrep(txt,'zp','\dot{z}');

txt=strrep(txt,'\thetapp','\ddot{\theta}');
txt=strrep(txt,'q_1pp','\ddot{q}_1');
txt=strrep(txt,'q_2pp','\ddot{q}_2');
txt=strrep(txt,'q_3pp','\ddot{q}_3');
txt=strrep(txt,'q_4pp','\ddot{q}_4');
txt=strrep(txt,'\thetap','\dot{\theta}');
txt=strrep(txt,'q_1p','\dot{q}_1');
txt=strrep(txt,'q_2p','\dot{q}_2');
txt=strrep(txt,'q_3p','\dot{q}_3');
txt=strrep(txt,'q_4p','\dot{q}_4');
txt=strrep(txt,'K_ap','K_{ap}');
txt=strrep(txt,'R_ap','R_{ap}');
txt=strrep(txt,'K_bp','K_{bp}');
txt=strrep(txt,'K_p1','K_{p1}');
txt=strrep(txt,'K_p2','K_{p2}');
txt=strrep(txt,'K_p3','K_{p3}');
txt=strrep(txt,'K_p4','K_{p4}');
txt=strrep(txt,'K_d1','K_{d1}');
txt=strrep(txt,'K_d2','K_{d2}');
txt=strrep(txt,'K_d3','K_{d3}');
txt=strrep(txt,'K_d4','K_{d4}');


for xx=32:-1:1
    txt=strrep(txt,strcat('x_',num2str(xx)),...
        strcat('\zeta_{',num2str(xx),'}'));
end