#actuator frame is that force is always in direction of x
mutable struct SimpleThruster <: AbstractActuator
    name::Symbol    
    force::SVector{6,Float64} # spatial force, first 3 elements are force, last 3 are torque
    command::AbstractSoftware
    transform::SMatrix{6,6,Float64,36} # transform from actuator frame to body frame
    u_index::SVector{6,Int16}
    current_force::SVector{6,Float64}
    body::AbstractBody #TODO need to make this parametric
    SimpleThruster(name,force) = new(name,SVector{6,Float64}(0,0,0,force,0,0))
end

get_actuator_force(A::SimpleThruster) = A.command.current_value ? A.transform * A.force : SVector{6,Float64}(0.,0.,0.,0.,0.,0.)

function get_actuator_force!(A::SimpleThruster)
    A.current_force = A.command.current_value * A.transform * A.force # : SVector{6,Float64}(0.,0.,0.,0.,0.,0.)
    return nothing
end