import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics, RigidBodySim
using StaticArrays, LinearAlgebra
using MeshCat, MeshCatMechanisms
using Gadfly, Cairo, Fontconfig




###############################################################################

q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qd = [0;0;0;0.0;0.0;pi;0.01;0.01;0.01]

deltaq = qd - q0
T = 10

function Traj(t)
   Q= q0 + (t/T)*deltaq
   DQ = (1/T)*deltaq
   DDQ = 0*deltaq
   return Q, DQ, DDQ;
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
    Q,DQ,DDQ=Traj(t)
    Kd=90
    Kp=450
    τ .= -diagm(Kd*[1,1, 1, 1, 1, 1, 1,1,1]) * (velocity(state)-DQ)- diagm(Kp*[1,1, 1, 1, 1, 1, 1,1,1])*(configuration(state) -Q)
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

##############################
function Control_CTC!(τ, t, state)
    T=10
    q_real = configuration(state)
    kp = 0.5
    kd = 0.8
    M = mass_matrix(state)
    Q,DQ,DDQ=Traj(t)
    q = configuration(state)
    dq = velocity(state)
    e = Q-q
    edot = DQ-dq
    τ .= M*(DDQ + kp * e + kd * edot)+dynamics_bias(state)
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
  error=sol[end][1:7]-qd[1:7]
  println("error is:$(error)")
  error_norm=norm(error,2)
  println("error_norm is:$(error_norm)")
end
