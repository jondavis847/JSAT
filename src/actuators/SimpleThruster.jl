#actuator frame is that force is always in direction of x
mutable struct SimpleThruster <: AbstractActuator
    name::Symbol    
    force::Float64 # spatial force, first 3 elements are force, last 3 are torque
    current_force::Float64
    command::AbstractSoftware
    body_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to body frame
    joint_transform::SMatrix{6,6,Float64,36} # transform from actuator frame to inner joint frame
    xindex::SVector{1,Int16}
    current_joint_force::SVector{6,Float64} 
    body::AbstractBody #TODO need to make this parametric
    callback::DiscreteCallback # really a FunctionCallingCallback
    SimpleThruster(name,force) = new(name,force,0)
end

function get_actuator_force!(A::SimpleThruster)
    A.current_force = A.command.current_value * A.force
    A.current_joint_force = A.joint_transform * SVector{6,Float64}(0,0,0,0,0,A.current_force)
    return nothing
end

get_actuator_momentum(A::SimpleThruster) = @SVector zeros(6)
set_state!(A::SimpleThruster,x) = nothing

get_dq(A::SimpleThruster) = 0.0 #changed via callback
get_q(A::SimpleThruster) = A.current_force