in Order to run the 10 DOF robot you need to add the code in the path

then in Julia use this command to run the 10 dof robot: 
	
	`include("Main_New.jl")` 
To change the states 
	
	`set_configuration!(state, [0,0,0,pi/18,-pi/18,-pi/18,pi/6,-pi/6,-pi/18,pi/18])` 
and 
	
	`set_configuration!(mvis, configuration(state))` 
for inverse kinematic use

`include("test_new.jl")` 


in order to update the endeffector point 	

`update_desired(state,vis,point,body,x1,x2,x3)` 

The ball showed the former position of the endeffector in the vis.
