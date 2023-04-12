CC2=subs(CC, (R_ap*(m_1 + m_2 + m_3 + m_uav - K_1*K_d1))/(4*K_1*K_ap*K_p1),x_1);
CC2=subs(CC2,-(K_d2*R_ap)/(4*K_ap*K_p1),-x_2);
CC2=subs(CC2,(K_d3*R_ap)/(4*K_ap*K_p1),x_3);
CC2=subs(CC2,-R_ap/(4*K_1*K_ap*K_p1),-x_4);
CC2=subs(CC2,R_ap/(4*K_1*K_ap*K_p1),x_4);
CC2=subs(CC2,l_2*m_2,x_5);
CC2=subs(CC2,l_2*m_3,x_6);
CC2=subs(CC2,l_3*m_3,x_7);
CC2=subs(CC2,- K_1*K_d4,-x_8);

CC2=subs(CC2,-(K_d1*R_ap)/(4*K_ap*K_p2),-x_9);
CC2=subs(CC2,(R_ap*(m_1 + m_2 + m_3 + m_uav + K_1*K_d2))/(4*K_1*K_ap*K_p2),x_10);
CC2=subs(CC2,(K_d3*R_ap)/(4*K_ap*K_p2),x_11);
CC2=subs(CC2,R_ap/(4*K_1*K_ap*K_p2),x_12);
CC2=subs(CC2,-R_ap/(4*K_1*K_ap*K_p2),-x_12);

CC2=subs(CC2,(K_d1*R_ap)/(4*K_ap*K_p3),x_13);
CC2=subs(CC2,(K_d2*R_ap)/(4*K_ap*K_p3),x_14);
CC2=subs(CC2,(R_ap*(m_1 + m_2 + m_3 + m_uav + K_1*K_d3))/(4*K_1*K_ap*K_p3),x_15);
CC2=subs(CC2,(K_d4*R_ap)/(4*K_ap*K_p3),x_16);
CC2=subs(CC2,R_ap/(4*K_1*K_ap*K_p3),x_17);
CC2=subs(CC2,-R_ap/(4*K_1*K_ap*K_p3),-x_17);
CC2=subs(CC2,R_ap/(4*K_ap*K_p4*b_1),x_18);
CC2=subs(CC2,K_d1*b_1 ,x_19);
CC2=subs(CC2,-R_ap/(4*K_ap*K_p4*b_1),-x_18);
CC2=subs(CC2,-R_ap/(8*K_ap*K_p4*b_1),-x_18/2);
CC2=subs(CC2,K_d2*b_1  ,x_20);
CC2=subs(CC2,(K_d3*R_ap)/(4*K_ap*K_p4) ,x_21);
CC2=subs(CC2,R_ap/(8*K_ap*K_p4*b_1),x_18/2);
CC2=subs(CC2,2*J_uav - 2*K_d4*b_1,x_22);
CC2=subs(CC2,l_2*x_5 + l_2*x_6 + l_3*x_7,x_23);
CC2=subs(CC2,R_a/(K_a*K_p),x_24);
CC2=subs(CC2,-R_a/(K_a*K_p),-x_24);
CC2=subs(CC2,K_d ,x_25);

CC2=subs(CC2, -(K_bp - 4*K_p1*a_1)/(4*K_p1*a_1),x_26);
CC2=subs(CC2, -(m_1 + m_2 + m_3 + m_uav),-x_27);
CC2=subs(CC2, -(K_bp - 4*K_p2*a_1)/(4*K_p2*a_1),-x_28);
CC2=subs(CC2, -(K_bp - 4*K_p3*a_3)/(4*K_p3*a_3),-x_29);
CC2=subs(CC2, (K_ap*b_1*(K_bp - 4*K_p4*a_4))/(R_ap*a_4),x_30);
CC2=subs(CC2, (K_a*(K_b - K_p))/R_a,x_31);
CC2=subs(CC2, -(K_b - K_p)/K_p,-x_32);

CC2=simplify(CC2);

CC3=subs(CC2,cos(q_1),'C_1');
CC3=subs(CC3,sin(q_1),'S_1');
CC3=subs(CC3,cos(q_2),'C_2');
CC3=subs(CC3,sin(q_2),'S_2');
CC3=subs(CC3,cos(q_3),'C_3');
CC3=subs(CC3,sin(q_3),'S_3');
CC3=subs(CC3,cos(2*q_2),'C_22');
CC3=subs(CC3,cos(2*q_2 + q_3),'C_223');
CC3=subs(CC3,cos(2*q_2 + 2*q_3),'C_2223');
CC3=subs(CC3,sin(2*q_2),'S_22');
CC3=subs(CC3,sin(2*q_2 + q_3),'S_223');
CC3=subs(CC3,sin(2*q_2 + 2*q_3),'S_2223');
CC3=subs(CC3,cos(q_2 + q_3),'C_23');
CC3=subs(CC3,sin(q_2 + q_3),'S_23');
CC3=simplify(CC3);