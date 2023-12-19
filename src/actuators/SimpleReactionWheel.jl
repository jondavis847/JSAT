mutable struct SimpleReactionWheel <: AbstractActuator
    name::Symbol
    inertia::Float64 # spatial force, first 3 elements are force, last 3 are torque
    motor_constant::Float64 #kt, current to torque   
    current_momentum::Float64
    current_speed::Float64
    current_torque::Float64
    current_joint_force::SVector{6,Float64} #spatial force, so includes torque
    body_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to body frame
    joint_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to inner joint frame, could be rotated from body so still need to do for wheel
    xindex::SVector{1,Int16}
    body::AbstractBody #TODO need to make this parametric
    command::AbstractSoftware #TODO: parameterize
    SimpleReactionWheel(name, inertia, kt) = new(name, inertia, kt, 0)
    SimpleReactionWheel(name, inertia, kt, H::Real) = new(name, inertia, kt, H)
end

function set_state!(A::SimpleReactionWheel, x)
    A.current_momentum = x[A.xindex][1]
    A.current_speed = A.current_momentum / A.inertia
    return nothing
end

function get_actuator_force!(A::SimpleReactionWheel)
    A.current_torque = A.motor_constant * A.command.current_value
    # wheel motion is always about wheel frame Z
    # negative since this is a reaction torque
    A.current_joint_force = -A.joint_transform * SVector{6,Float64}(0, 0, A.current_torque, 0, 0, 0)
    return nothing
end

get_actuator_momentum(A::SimpleReactionWheel) = A.joint_transform * SVector{6,Float64}(0, 0, A.current_momentum, 0, 0, 0)

get_q(A::SimpleReactionWheel) = A.current_momentum
get_dq(A::SimpleReactionWheel) = A.current_torque

function get_savedict(A::SimpleReactionWheel, i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(A.name)_torque",
        typeof(A.current_torque),
        integrator -> integrator.p.sys.actuators[i].current_torque
    )

    save_dict!(
        save_config,
        "$(A.name)_momentum",
        typeof(A.current_momentum),
        integrator -> integrator.p.sys.actuators[i].current_momentum
    )

    save_dict!(
        save_config,
        "$(A.name)_speed",
        typeof(A.current_speed),
        integrator -> integrator.p.sys.actuators[i].current_speed
    )

    save_dict!(
        save_config,
        "$(A.name)_u",
        typeof(A.command.current_value),
        integrator -> integrator.p.sys.actuators[i].command.current_value
    )
    return save_config
end