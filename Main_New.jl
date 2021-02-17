import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics
using LinearAlgebra
# using StaticArrays#, Plots
using MeshCat, MeshCatMechanisms
using GeometryTypes, CoordinateTransformations, ColorTypes
vis = Visualizer();open(vis)
#import PandaRobot # for visualizing Panda

function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    #urdfPath = "planar3R.urdf"

    mechanism = parse_urdf(Float64,urdfPath)

    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    for bd in bodies(mechanism)
       setelement!(mvis,default_frame(bd),1,"$bd")
    end
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mvis, mechanism
end

function plot_sphere(vis,c1,radius,mat,name="")
    geom = HyperSphere(Point3f0(c1), convert(Float32,radius))
    setobject!(vis["sph"][name],geom,mat)
end

mvis, mechanism = display_urdf("CustomizedRobot.urdf",vis)
state=MechanismState(mechanism)
 set_configuration!(state, [0,0,0,pi/18,-pi/18,-pi/18,pi/6,-pi/6,-pi/18,pi/18])
 set_configuration!(mvis, configuration(state))


# Point3D(body_frame, 0.2, 0.2, 0.2)
