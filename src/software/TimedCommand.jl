
mutable struct TimedCommand{T<:AbstractFloat} <: AbstractSoftware
    name::Symbol
    initial_value::Bool
    t_starts::Vector{T}
    t_stops::Vector{T}        
    connections::SoftwareConnection
    current_value::Bool 
    
    TimedCommand(name,val,t_starts,t_stops) = new{Float64}(name,val,t_starts,t_stops,SoftwareConnection())
end

function get_callback(tc::TimedCommand, i)
    # make a callback for t_starts
    start_affect! = (integrator) -> setfield!.(integrator.p.sys.software[i].connections.actuators, :command, true)
    start_cb = PresetTimeCallback(tc.t_starts,start_affect!)

    # make a callback for t_stops
    stop_affect! = (integrator) -> setfield!.(integrator.p.sys.software[i].connections.actuators, :command, false)
    stop_cb = PresetTimeCallback(tc.t_stops,stop_affect!)

    return [start_cb,stop_cb]    
end

function get_savedict(S::TimedCommand,i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_u",
        typeof(S.initial_value),
        integrator -> integrator.p.sys.software[i].current_value
    )
    return save_config
end

get_initial_value(SW::TimedCommand) = []
set_xindex!(SW::TimedCommand,i) = nothing
set_state!(SW::TimedCommand,x) = nothing