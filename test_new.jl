import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics
using StaticArrays
using MeshCat, MeshCatMechanisms, Blink
#vis = Visualizer();
using Random
true::Bool
Random.seed!(42);
srcdir = dirname(pathof(RigidBodyDynamics))
#urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
urdf = "planar3R.urdf"
mechanism = parse_urdf(urdf)
state = MechanismState(mechanism)
mechanism
body = findbody(mechanism, "lower_link")
point = Point3D(default_frame(body), 3, 0, 0.3)
# Create the visualizer
vis = MechanismVisualizer(mechanism, URDFVisuals(urdf))

# Render our target point attached to the robot as a sphere with radius 0.07
setelement!(vis, point, 0.07)
open(vis,Window());
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));
function jacobian_transpose_ik!(state::MechanismState,
                               body::RigidBody,
                               point::Point3D,
                               desired::Point3D;
                               α=0.1,
                               iterations=100)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for i in 1:iterations
        # Update the position of the point
        point_in_world = transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
    end
    state
end
rand!(state)
set_configuration!(vis, configuration(state))
desired_tip_location = Point3D(root_frame(mechanism),2.5,-1,0.6)
jacobian_transpose_ik!(state, body, point, desired_tip_location)
set_configuration!(vis, configuration(state))
transform(state, point, root_frame(mechanism))
function update_desired(state,vis,point,body,x1,x2,x3)
   desired_tip_location = Point3D(root_frame(mechanism),x1,x2,x3)
   jacobian_transpose_ik!(state, body, point, desired_tip_location)
   set_configuration!(vis, configuration(state))
   transform(state, point, root_frame(mechanism))
end
