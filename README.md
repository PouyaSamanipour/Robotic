# Robotic
you just need to clone all these codes and add them to the path.

Firts, you need to run the following in the command terminal.

`include("Main_New.jl")`

Then, for the first part, you just need to run the: `Traj(t)` function to find the desired path. 

In the second part, you need to run `Controller(1)` for the PD controller.


In the last part, you need to run `Controller(2)` for the CTC controller.


The results shows that we need a high derivative and proprtional gain for PD controller to have the error less than the specified value. However, for the CTC the small derivative and proportional gain will provide a satisfactory result.
