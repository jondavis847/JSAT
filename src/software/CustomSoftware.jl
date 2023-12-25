mutable struct CustomSoftware <: AbstractSoftware
    name::Symbol
    entry::Function
    period::Float64    
    parameters::NamedTuple
    variables::ComponentArray    
    required_sensors::Vector{Symbol}
    required_actuators::Vector{Symbol}
    connections::SoftwareConnection

    CustomSoftware(name,entry,period) = new(name,entry,period,NamedTuple(),ComponentArray(),[],[],SoftwareConnection())    
    CustomSoftware(name,entry,period,parameters) = new(name,entry,period,parameters,ComponentArray(),[],[],SoftwareConnection())        
    CustomSoftware(name,entry,period,parameters,variables) = new(name,entry,period,parameters,variables,[],[],SoftwareConnection())        
    CustomSoftware(name,entry,period,parameters,variables,sensors,actuators) = new(name,entry,period,parameters,variables,sensors,actuators,SoftwareConnection())        
end

get_initial_value(SW::CustomSoftware) = []
set_xindex!(SW::CustomSoftware,i) = nothing
set_state!(SW::CustomSoftware,x) = nothing

function get_callback(SW::CustomSoftware,i)
    affect! = (integrator) -> integrator.p.sys.software[i].entry(integrator.p.sys.software[i])
    return PeriodicCallback(affect!,SW.period; initial_affect = false)
end

function get_savedict(SW::CustomSoftware, i)
    save_config = Dict[]    
    states = keys(SW.variables)
    for state in states
        save_dict!(
            save_config,
            "$(SW.name)_$(string(state))",
            typeof(SW.variables[state]),
            integrator -> integrator.p.sys.software[i].variables[state]
        )
    end    
    return save_config
end

includet("custom//pole_cart_example.jl")

function get_custom_software()
    return Dict(
        "pole_cart_software" => create_pole_cart_software()
        )
end

