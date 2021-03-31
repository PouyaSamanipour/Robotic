import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics, RigidBodySim
using StaticArrays, LinearAlgebra
using MeshCat, MeshCatMechanisms
using Gadfly, Cairo, Fontconfig




###############################################################################

q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

deltaq = qd - q0
T = 10

function Traj(t)

   theta = q0 + (t/T)*deltaq
   dtheta = (1/T)*deltaq
   ddtheta = 0*deltaq


   print("\n q: \n")
   print(theta)
   print("\n Dq: \n")
   print(dtheta)
   print("\n DDq: \n")
   print(ddtheta)
   return theta, dtheta, ddtheta
end

################################################################################

function display_urdf(urdfPath,vis)
    mechanism = parse_urdf(Float64,urdfPath)
    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    for bd in bodies(mechanism)
        setelement!(mvis,default_frame(bd),0.5,"$bd")
    end
    return mvis, mechanism
end

#################################################################################
function PDcontrol!(τ, t, state)
    # Do some PD
    q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
    qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]
    T=10
    deltaq = qd - q0
    theta = q0 + (t/T)*deltaq
    dtheta = (1/T)*deltaq
    ddtheta = 0*deltaq
    Kd=50
    Kp=450
    τ .= -diagm(Kd*[1,1, 1, 1, 1, 1, 1,1,1]) * (velocity(state)-dtheta)- diagm(Kp*[1,1, 1, 1, 1, 1, 1,1,1])*(configuration(state) -theta)
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end
####################################################################
function plot_sol(p,sol,colorarg,saveflag,savename)
    qsol = vcat(sol[:]'...)
    for i=1:7
        push!(p,layer(x=sol.t,y=qsol[:,i],Geom.line,color=colorarg))
    end
    p
    if saveflag
        p |> PDF(savename)
    end
end
##############################
function Control_CTC!(τ, t, state)
    T=10
    qprev = configuration(state)
    kp = 50
    kd = 20
    qdes = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]
    M = mass_matrix(state)
    theta = q0 + (t/T)*deltaq
    dtheta = (1/T)*deltaq
    ddtheta = 0*deltaq
    q = configuration(state)
    dq = velocity(state)
    e = theta-q
    edot = dtheta-dq
    τ .= M*(ddtheta + kp * e + kd * edot)+dynamics_bias(state)
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

#############################
function Controller(i)
  if i==1
    system=PDcontrol!
  end
  if i==2
    system=Control_CTC!
  end
  vis = Visualizer();open(vis)
  # Refresh visualization
  delete!(vis)
  # Load mechanism info
  urdfPath = "panda.urdf"
  mvis, mechanism = display_urdf(urdfPath,vis)
  # Create state and set initial config and velocity
  state = MechanismState(mechanism)
  set_configuration!(state,[0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1])
  zero_velocity!(state)
  # Update mechanism visual
  set_configuration!(mvis, configuration(state))

  # Define ODE Problem, which defines closed loop using  control
  problem = ODEProblem(Dynamics(mechanism,system), state, (0., 10.));
  # Solve ODE problem using Tsit5 scheme, and given numerical tolerances
  sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
  # Animate solution
  setanimation!(mvis, sol; realtime_rate = 1.0);
  for i in 1:7
      println("final joint angle $i : $(sol[end][i])")
  end
  error=sol[end][1:7]-[0;0;0.0;0.0;0.0;pi;0.01]
  println("error is:$(error)")
  error_norm=norm(error,2)
  println("error_norm is:$(error_norm)")
end
