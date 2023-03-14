GG2=subs(GG, (R_ap*(m_1 + m_2 + m_3 + m_uav - K_1*K_d1))/(4*K_1*K_ap*K_p1),x_1);
GG2=subs(GG2,-(K_d2*R_ap)/(4*K_ap*K_p1),-x_2);
GG2=subs(GG2,(K_d3*R_ap)/(4*K_ap*K_p1),x_3);
GG2=subs(GG2,-R_ap/(4*K_1*K_ap*K_p1),-x_4);
GG2=subs(GG2,R_ap/(4*K_1*K_ap*K_p1),x_4);
GG2=subs(GG2,l_2*m_2,x_5);
GG2=subs(GG2,l_2*m_3,x_6);
GG2=subs(GG2,l_3*m_3,x_7);
GG2=subs(GG2,- K_1*K_d4,-x_8);

GG2=subs(GG2,-(K_d1*R_ap)/(4*K_ap*K_p2),-x_9);
GG2=subs(GG2,(R_ap*(m_1 + m_2 + m_3 + m_uav + K_1*K_d2))/(4*K_1*K_ap*K_p2),x_10);
GG2=subs(GG2,(K_d3*R_ap)/(4*K_ap*K_p2),x_11);
GG2=subs(GG2,R_ap/(4*K_1*K_ap*K_p2),x_12);
GG2=subs(GG2,-R_ap/(4*K_1*K_ap*K_p2),-x_12);

GG2=subs(GG2,(K_d1*R_ap)/(4*K_ap*K_p3),x_13);
GG2=subs(GG2,(K_d2*R_ap)/(4*K_ap*K_p3),x_14);
GG2=subs(GG2,(R_ap*(m_1 + m_2 + m_3 + m_uav + K_1*K_d3))/(4*K_1*K_ap*K_p3),x_15);
GG2=subs(GG2,(K_d4*R_ap)/(4*K_ap*K_p3),x_16);
GG2=subs(GG2,R_ap/(4*K_1*K_ap*K_p3),x_17);
GG2=subs(GG2,-R_ap/(4*K_1*K_ap*K_p3),-x_17);
GG2=subs(GG2,R_ap/(4*K_ap*K_p4*b_1),x_18);
GG2=subs(GG2,K_d1*b_1 ,x_19);
GG2=subs(GG2,-R_ap/(4*K_ap*K_p4*b_1),-x_18);
GG2=subs(GG2,-R_ap/(8*K_ap*K_p4*b_1),-x_18/2);
GG2=subs(GG2,K_d2*b_1  ,x_20);
GG2=subs(GG2,(K_d3*R_ap)/(4*K_ap*K_p4) ,x_21);
GG2=subs(GG2,R_ap/(8*K_ap*K_p4*b_1),x_18/2);
GG2=subs(GG2,2*J_uav - 2*K_d4*b_1,x_22);
GG2=subs(GG2,l_2*x_5 + l_2*x_6 + l_3*x_7,x_23);
GG2=subs(GG2,R_a/(K_a*K_p),x_24);
GG2=subs(GG2,-R_a/(K_a*K_p),-x_24);
GG2=subs(GG2,K_d ,x_25);

GG2=subs(GG2, (m_1 + m_2 + m_3 + m_uav),x_27);

GG2=simplify(GG2);

GG3=subs(GG2,cos(q_2),'C_2');
GG3=subs(GG3,sin(q_2),'S_2');
GG3=subs(GG3,cos(q_2 + q_3),'C_23');
GG3=subs(GG3,sin(q_2 + q_3),'S_23');
GG3=simplify(GG3);