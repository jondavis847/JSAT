
mutable struct TimedCommand{T<:AbstractFloat} <: AbstractSoftware
    name::Symbol
    initial_value::Bool
    t_starts::Vector{T}
    t_stops::Vector{T}
    callback::Vector{DiscreteCallback}
    u_index::Int16   
    current_value::Bool 
    TimedCommand(name,val,t_starts,t_stops) = new{Float64}(name,val,t_starts,t_stops)
end

function create_callbacks!(tc::TimedCommand, i)
    # make a callback for t_starts
    start_affect! = (integrator) -> integrator.u[integrator.p.sys.software[i].u_index] = true
    start_cb = PresetTimeCallback(tc.t_starts,start_affect!)

    # make a callback for t_stops
    stop_affect! = (integrator) -> integrator.u[integrator.p.sys.software[i].u_index] = false
    stop_cb = PresetTimeCallback(tc.t_stops,stop_affect!)

    tc.callback = [start_cb,stop_cb]
    return nothing
end

function set_state!(tc::TimedCommand,x)
    tc.current_value = x[tc.u_index]
    return nothing
end

function run_software!(tc::TimedCommand)
    return nothing
end