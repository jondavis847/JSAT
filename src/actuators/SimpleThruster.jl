#actuator frame is that force is always in direction of z
mutable struct SimpleThruster <: AbstractActuator
    name::Symbol
    force::Float64 # spatial force, first 3 elements are force, last 3 are torque
    current_force::Float64
    command::Float64
    state::ActuatorState
    frame::Cartesian # cartesian frame from actuator to body frame
    body_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to body frame
    ijof_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to inner joint frame
    xindex::SVector{1,Int16}
    current_ijof_force::SVector{6,Float64}
    body::AbstractBody #TODO need to make this parametric
    callback::DiscreteCallback # really a FunctionCallingCallback
    SimpleThruster(name, force) = new(name, force, 0, false, ActuatorState())
end

function get_actuator_force!(A::SimpleThruster)
    A.current_force = A.command * A.force
    A.current_ijof_force = A.ijof_transform * SVector{6,Float64}(0, 0, 0, 0, 0, A.current_force)
    return nothing
end

get_actuator_momentum(A::SimpleThruster) = @SVector zeros(6)
set_state!(A::SimpleThruster, x) = nothing

get_dq(A::SimpleThruster) = 0.0 #changed via callback
get_q(A::SimpleThruster) = A.current_force

function get_savedict(A::SimpleThruster, i)
    save_config = Dict[]

    states = fieldnames(typeof(A.state))
    for state in states
        save_dict!(
            save_config,
            "$(A.name)_$(string(state))",
            typeof(getfield(A.state, state)),
            integrator -> getfield(integrator.p.sys.actuators[i].state, state)
        )
    end

    save_dict!(
        save_config,
        "$(A.name)_force",
        typeof(A.current_force),
        integrator -> integrator.p.sys.actuators[i].current_force
    )

    save_dict!(
        save_config,
        "$(A.name)_current_ijof_force",
        typeof(A.current_ijof_force),
        integrator -> integrator.p.sys.actuators[i].current_ijof_force
    )

    save_dict!(
        save_config,
        "$(A.name)_u",
        typeof(A.command),
        integrator -> integrator.p.sys.actuators[i].command
    )
    return save_config
end