function save_dict!(config, name, type, func)
    d = Dict(
        "name" => name,
        "type" => type,
        "func" => func
    )
    push!(config, d)
    nothing
end

function configure_saving(sys::MultibodySystem)
    save_config = []
    for i in eachindex(sys.joints)
        if !isa(sys.joints[i], FixedJoint)
            #save joint state info        
            states = fieldnames(typeof(sys.joints[i].state))
            for state in states
                save_dict!(
                    save_config,
                    "$(sys.joints[i].meta.name)_$(string(state))",
                    typeof(getfield(sys.joints[i].state, state)),
                    integrator -> getfield(integrator.p.sys.joints[i].state, state)
                )
            end
        end
    end
    for i in 1:length(sys.bodies)

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_v",
            typeof(sys.bodies[i].state.v),
            integrator -> integrator.p.sys.bodies[i].state.v
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_a",
            typeof(sys.bodies[i].state.a),
            integrator -> integrator.p.sys.bodies[i].state.a
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_external_force",
            typeof(sys.bodies[i].external_force),            
            integrator -> integrator.p.sys.bodies[i].external_force
        )

        # get base frame position vars for animation        

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_q_base",
            typeof(sys.bodies[i].state.q_base),            
            integrator -> (integrator.p.sys.bodies[i].state.q_base)
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_r_base",
            typeof(sys.bodies[i].state.r_base),            
            integrator -> (integrator.p.sys.bodies[i].state.r_base)
        )

        for i in eachindex(sys.software)
            save_dict!(
                save_config,
                "$(sys.software[i].name)_u",
                typeof(sys.software[i].initial_value),
                #integrator -> integrator.u[integrator.p.sys.software[i].u_index]
                integrator -> integrator.p.sys.software[i].current_value
            )
        end

        for i in eachindex(sys.actuators)
            save_dict!(
                save_config,
                "$(sys.actuators[i].name)_f",
                typeof(sys.actuators[i].current_force),
                #integrator -> integrator.u[integrator.p.sys.actuators[i].u_index]
                integrator -> integrator.p.sys.actuators[i].current_force
            )

            save_dict!(
                save_config,
                "$(sys.actuators[i].name)_u",
                typeof(sys.actuators[i].command.current_value),
                integrator -> integrator.p.sys.actuators[i].command.current_value
            )

        end
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