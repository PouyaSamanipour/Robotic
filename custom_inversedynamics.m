function [tau1,tau2,tau3]=custom_inversedynamics(a,b,c,d,e,f,g,h,i)
syms q1(t) q2(t) q3(t) Dq1(t) Dq2(t) Dq3(t) L1 L2 L3 Lm1 Lm2 Lm3;
Rz1 = [cos(q1(t)) -sin(q1(t)) 0 0; sin(q1(t)) cos(q1(t)) 0 0; 0 0 1 0; 0 0 0 1];
Tz1 = [1 0 0 0; 0 1 0 0; 0 0 1 L1; 0 0 0 1];
Rx1 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1];
T01 = Rz1*Tz1*Rx1*[1 0 0 L1/2;0 1 0 0;0 0 1 0;0 0 0 1];
R01 = T01(1:3,1:3);

Rz2 = [cos(q2(t)) -sin(q2(t)) 0 0; sin(q2(t)) cos(q2(t)) 0 0; 0 0 1 0; 0 0 0 1];
Tx2 = [1 0 0 L2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T02 = T01*Rz2*Tx2*[1 0 0 L1/2;0 1 0 0;0 0 1 0;0 0 0 1];
R02 = T02(1:3,1:3);

Rz3 = [cos(q3(t)) -sin(q3(t)) 0 0; sin(q3(t)) cos(q3(t)) 0 0; 0 0 1 0; 0 0 0 1];
Tx3 = [1 0 0 L3; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T03 = T02*Rz3*Tx3*[1 0 0 L1/2;0 1 0 0;0 0 1 0;0 0 0 1];
R03 = T03(1:3,1:3);
d1=T01(1:3,4);
d2=T02(1:3,4);
d3 = T03(1:3,4);
d1_d0=d1;
d2_d0=d2;
d2_d1=d2-d1;
d3_d0=d3;
d3_d1=d3-d1;
d3_d2=d3-d2;


J11 = [cross([0; 0; 1], d1_d0) [0 0;0 0;0 0]];
J12 = [cross(R01(:,3), d2_d0) cross(R02(:,3), d2_d1) [0;0;0]];
J13 = [cross([0;0;1], d3_d0) cross(R01(:,3), d3_d1) cross(R02(:,3), d3_d2)];

J21 = [[0; 0; 1] [0;0;0] [0;0;0]];
J22 = [[0; 0; 1] R01(:,3) [0;0;0]];
J23 = [[0;0;1] R01(:,3) R02(:,3)];

J = [J11 J12 J13; J21 J22 J23];
J=simplify(J);
I1 = [1 0 0; 0 .083 0; 0 0 1];
I2 = [1 0 0; 0 .083 0; 0 0 1];
I3 = [1 0 0; 0 .33 0; 0 0 1];

DQ = [Dq1(t); Dq2(t); Dq3(t)];
m1 = 1;
m2 = 1;
m3 = 1;
g=9.81;
KE = 0.5*transpose(DQ)*(transpose(J21)*R01*I1*transpose(R01)*J21+m1*transpose(J11)*J11+transpose(J22)*R02*I2*transpose(R02)*J22+m2*transpose(J12)*J12+transpose(J23)*R03*I3*transpose(R03)*J23+m3*transpose(J13)*J13)*DQ;
PE= (m1*g*L1/2)+m2*g*(L1+L2*sin(q2(t))/2)+m3*g*(L1+L2*sin(q2(t))+(L3/2)*sin(q3(t)));
Lagrange=KE-PE;
Lagrange=simplify(Lagrange);
D=(transpose(J21)*R01*I1*transpose(R01)*J21+m1*transpose(J11)*J11+transpose(J22)*R02*I2*transpose(R02)*J22+m2*transpose(J12)*J12+transpose(J23)*R03*I3*transpose(R03)*J23+m3*transpose(J13)*J13);
dL1_dq1=diff(Lagrange,Dq1(t));
dL1_dq2=diff(Lagrange,Dq2(t));
dL1_dq3=diff(Lagrange,Dq3(t));
dL1_q1=diff(Lagrange,q1(t));
dL1_q2=diff(Lagrange,q2(t));
dL1_q3=diff(Lagrange,q3(t));
Tau_1=diff(dL1_dq1,t)-dL1_q1;
Tau_2=diff(dL1_dq2,t)-dL1_q2;
Tau_3=diff(dL1_dq2,t)-dL1_q3;
tau1=subs(Tau_1,[q1(t),q2(t),q3(t),Dq1(t),Dq2(t),Dq3(t),diff(q1(t), t),diff(q2(t), t),diff(q2(t), t),diff(Dq1(t), t),diff(Dq2(t), t),diff(Dq3(t), t),L1,L2,L3],[a,b,c,d,e,f,d,e,f,g,h,i,1,1,1])
tau2=subs(Tau_2,[q1(t),q2(t),q3(t),Dq1(t),Dq2(t),Dq3(t),diff(q1(t), t),diff(q2(t), t),diff(q2(t), t),diff(Dq1(t), t),diff(Dq2(t), t),diff(Dq3(t), t),L1,L2,L3],[a,b,c,d,e,f,d,e,f,g,h,i,1,1,1])
tau3=subs(Tau_3,[q1(t),q2(t),q3(t),Dq1(t),Dq2(t),Dq3(t),diff(q1(t), t),diff(q2(t), t),diff(q2(t), t),diff(Dq1(t), t),diff(Dq2(t), t),diff(Dq3(t), t),L1,L2,L3],[a,b,c,d,e,f,d,e,f,g,h,i,1,1,1]) 
end



