save_dict!(sv, name, type, func) = push!(sv,Dict("name" => name, "type" => type, "func" => func))

function configure_saving(sys::MultibodySystem)
    save_config = []

    for i in eachindex(sys.bodies)
        append!(save_config,get_savedict(sys.bodies[i],i))
    end    

    for i in eachindex(sys.joints)
        append!(save_config,get_savedict(sys.joints[i],i))
    end

    for i in eachindex(sys.sensors)
        append!(save_config,get_savedict(sys.sensors[i],i))
    end

    for i in eachindex(sys.software)
        append!(save_config,get_savedict(sys.software[i],i))
    end

    for i in eachindex(sys.actuators)
        append!(save_config,get_savedict(sys.actuators[i],i))
    end

    save_values = SavedValues(Float64, Tuple{getindex.(save_config, "type")...})
    function save_function(u, t, integrator)
        #update after integration complete so values aren't based on RK4 steps
        update_model!(integrator.p.sys, u)
        model!(integrator.p.sys)
        Tuple([f(integrator) for f in getindex.(integrator.p.save_config, "func")])
    end
    save_cb = SavingCallback(save_function, save_values)
    return save_config, save_values, save_cb
end

function nt_save_values(save_values, save_config)
    d = Dict()
    d[:t] = save_values.t
    for i in eachindex(save_config)
        name = Symbol(save_config[i]["name"])
        values = getindex.(save_values.saveval, i)
        d[name] = values
    end
    NamedTuple(d)
end

function df_save_values(save_values, save_config)
    D = DataFrame()
    D[!, "t"] = save_values.t
    for i in eachindex(save_config)
        name = String(save_config[i]["name"])
        type = save_config[i]["type"]

        if type <: AbstractVector
            for j in eachindex(save_values.saveval[1][i])
                this_name = "$(name)[$(j)]"
                this_values = map(x -> x[i][j], save_values.saveval)
                D[!, this_name] = this_values
            end
        elseif type <: AbstractMatrix
            for k in axes(save_values.saveval[1][i])[2]
                for j in axes(save_values[1][i])[1]
                    this_name = "$(name)[$(j),$(k)]"
                    this_values = map(x -> x[i][j, k], save_values.saveval)
                    D[!, this_name] = this_values
                end
            end
        else
            values = getindex.(save_values.saveval, i)
            D[!, name] = values
        end
    end
    return D
end