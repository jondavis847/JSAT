
mutable struct TimedCommand{T<:AbstractFloat} <: AbstractSoftware
    name::Symbol
    initial_value::Bool
    t_starts::Vector{T}
    t_stops::Vector{T}
    callback::Vector{DiscreteCallback}
    xindex::Int16   
    current_value::Bool 
    sensors::Vector{AbstractSensor}
    TimedCommand(name,val,t_starts,t_stops) = new{Float64}(name,val,t_starts,t_stops)
end

function get_callback(tc::TimedCommand, i)
    # make a callback for t_starts
    start_affect! = (integrator) -> integrator.p.sys.software[i].current_value = true
    start_cb = PresetTimeCallback(tc.t_starts,start_affect!)

    # make a callback for t_stops
    stop_affect! = (integrator) -> integrator.p.sys.software[i].current_value = false
    stop_cb = PresetTimeCallback(tc.t_stops,stop_affect!)

    return [start_cb,stop_cb]    
end