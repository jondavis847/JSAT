mutable struct SimplePID <: AbstractSoftware
    name::Symbol
    P::Float64
    I::Float64
    D::Float64
    period::Float64
    p_input::AbstractSensor    
    p_reference::AbstractSoftware
    d_input::AbstractSensor    
    d_reference::AbstractSoftware    
    xindex::Int16   
    current_value::Float64
    p_error::Float64
    d_error::Float64
    p_accum::Float64
    SimplePID(name,P,I,D) = new(name,P,I,D,0)
    SimplePID(name,P,I,D,T) = new(name,P,I,D,T)
end

function get_callback(S::SimplePID, i)    
    function affect!(integrator) 
        pid = integrator.p.sys.software[i]
        pid.p_error = get_measurement(pid.p_input) - get_target(pid.p_reference)
        pid.d_error = get_measurement(pid.d_input) - get_target(pid.d_reference)
        pid.p_accum += pid.p_input # this should be safe since it occurs in a callback
        integrator.p.sys.software[i].current_value = pid.P * pid.p_error + pid.D * pid.d_error + pid.I * pid.p_accum #TODO: should this be negative
        return nothing
    end
    return PeriodicCallback(S.period,affect!)    
end

function connect!(software::AbstractSoftware,pid::SimplePID,p_or_d::Symbol)
    if p_or_d == :P
        pid.p_reference = software
    elseif p_or_d == :D
        pid.d_reference = software
    else
        error("software reference for SimplePID must be specified as either :P or :D")
    end
    return nothing
end

function connect!(sensor::AbstractSensor,pid::SimplePID,p_or_d::Symbol)
    if p_or_d == :P
        pid.p_input = sensor
    elseif p_or_d == :D
        pid.d_input = sensor
    else
        error("sensor input for SimplePID must be specified as either either :P or :D")
    end
    return nothing
end