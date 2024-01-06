
mutable struct TimedCommand{T} <: AbstractSoftware
    name::Symbol
    tsteps::Vector{Float64}
    values::Vector{T}
    connections::SoftwareConnection
    current_value::T

    function TimedCommand(name, tsteps, values)
        if !(typeof(tsteps) <: AbstractVector)
            values = [tsteps]
        end
        if !(typeof(values) <: AbstractVector)
            values = [values]
        end
        x = new{eltype(values)}()
        x.name = name
        x.tsteps = tsteps
        x.values = values
        x.connections = SoftwareConnection()
        x.current_value = zero(eltype(values))
        return x
    end
end

function get_callback(tc::TimedCommand, i)
    # make a callback for each tstep
    cbs = []
    for j in eachindex(tc.tsteps)
        affect! = (integrator) -> begin
            integrator.p.sys.software[i].current_value = tc.values[j]
            setfield!.(integrator.p.sys.software[i].connections.actuators, :command, Float64(integrator.p.sys.software[i].current_value))
        end
        push!(cbs,PresetTimeCallback(tc.tsteps[j], affect!))
    end    
    return cbs
end

function get_savedict(S::TimedCommand, i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_u",
        eltype(S.values),
        integrator -> integrator.p.sys.software[i].current_value
    )
    return save_config
end

get_initial_value(SW::TimedCommand) = []

set_xindex!(SW::TimedCommand, i) = nothing
set_state!(SW::TimedCommand, x) = nothing