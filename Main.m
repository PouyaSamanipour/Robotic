clc,clear all;close all;
syms q1(t) q2(t) q3(t) Dq1(t) Dq2(t) Dq3(t) L1 L2 L3 Lm1 Lm2 Lm3;
q1=input("please enter q1(radian):");
q2=input("please enter q2(radian):");
q3=input("please enter q3(radian):");
dq1=input("please enter dq1:");
dq2=input("please enter dq2:");
dq3=input("please enter dq3:");
ddq1=input("please enter ddq1:");
ddq2=input("please enter ddq2:");
ddq3=input("please enter ddq3:");
[tau1 tau2 tau3]=custom_inversedynamics(q1,q2,q3,dq1,dq2,dq3,ddq1,ddq2,ddq3)
% sympref('FloatingPointOutput',true);
% vpaSols = vpa(numSols,4)
