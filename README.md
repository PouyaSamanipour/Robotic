# Robotic
in Order to run the 10 DOF robot you need to add the code in the path
code:
cd git clone https://github.com/hpoonawala/rmc-s21 
https://github.com/PouyaSamani/desktop-tutorial/tree/assignment1_ME699 
then add the code from my github to the folder of (rmc-s21)
then in Julia use this command to run the 10 dof robot:
`include("Main_New.jl")`
for inverse kinematic use 
`include("test_new.jl")`
in order to update the endeffector point 
`update_desired(state,vis,point,body,x1,x2,x3)`
