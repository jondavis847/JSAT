function ode_func!(dx, x, p, t)
    update_model!(p.sys, x)
    model!(p.sys)
    pack_dq_in_dx!(dx, p.sys)
    nothing
end

function update_model!(sys, x)
    set_state!.(sys.joints, Ref(x))
    set_state!.(sys.software, Ref(x))
    set_state!.(sys.actuators, Ref(x))
    calculate_transforms!(sys)  # spatial transforms
    calculate_body_positions!(sys) # update generalized coords
    calculate_actuator_positions!(sys)
    nothing
end

#needed each timestep to fill dx with ode state derivatives [dq,q̈] (dq not q̇ since q̇ can = ω and dq is quaternion deriv)
function pack_dq_in_dx!(dx, sys)
    for joint in sys.joints
        if !isa(joint, FixedJoint)
            dx[joint.meta.xindex] = get_dq(joint)
            dx[joint.meta.ẋindex] = joint.state.q̈
        end
    end
    for actuator in sys.actuators
        dx[actuator.xindex] .= get_dq(actuator)
    end
    nothing
end

#only need stuff in model! that need to be integrated, all algeriac states are handled by callbacks
function model!(sys)    
    #sensors!(sys) #currently handled by callbacks
    #software!(sys) #currently handled by callbacks
    actuators!(sys)
    environments!(sys) # currently handled by callbacks
    dynamics!(sys)
    return nothing
end

function simulate(sys::MultibodySystem, tspan; dt=nothing, nruns=0, output_type=nothing)
    #get callbacks           
    joint_cb = []
    for i in eachindex(sys.joints)
        cbs = get_callback(sys.joints[i], i)
        append!(joint_cb, cbs)
    end
    sensor_cb = [get_callback(sys.sensors[i], i) for i in eachindex(sys.sensors)]
    software_cb = []
    for i in eachindex(sys.software)
        cbs = get_callback(sys.software[i], i)
        append!(software_cb, cbs)
    end
    cb = [joint_cb; sensor_cb; software_cb] #must follow order

    prob = make_prob(0, sys, cb, tspan, dt, output_type)
    #dump(prob.p.sys)
    sol = solve(prob, Tsit5())

    nominal_results = output_func(sol,0)

    if nruns > 0
        prob = make_prob(0, sys, cb, tspan, dt, output_type) #make a new prob or nominal resutls are stored in sys
        ens_prob = EnsembleProblem(prob, output_func=output_func, prob_func=prob_func)
        sim = solve(ens_prob, Tsit5(), EnsembleThreads(), trajectories=nruns)
        return (nominal_results, sim)
    else
        return nominal_results
    end
end

prob_func(prob, i, repeat) = make_prob(i, prob.p.sys, prob.p.callbacks, prob.p.tspan, prob.p.dt, prob.p.output_type)

function make_prob(i, orig_sys, cbs, tspan, dt, output_type)
    sys = deepcopy(orig_sys)# make a copy so we can rerun orig sys without mutating it during previous sim   
    #sys = orig_sys
    if i > 0
        disperse!.(sys.bodies)
    end

    #initialize    
    initialize_inertias!.(sys.bodies)
    initialize_actuators!.(sys.actuators)
    u0 = initialize_state_vectors(sys)

    save_config, save_values, save_cb = configure_saving(sys)
    p = (
        sys=sys,
        save_config=save_config,
        save_values=save_values,
        callbacks=cbs,
        dt=dt,
        output_type=output_type,
        tspan=tspan)

    cb = CallbackSet(cbs..., save_cb) #must follow order

    if isnothing(dt)
        prob = ODEProblem(ode_func!, u0, tspan, p, callback=cb)
    else
        prob = ODEProblem(ode_func!, u0, tspan, p, callback=cb, adaptive=false, dt=dt)
    end

    return prob
end

function output_func(sol, i)
    save_values = sol.prob.p.save_values
    save_config = sol.prob.p.save_config

    if sol.prob.p.output_type == DataFrame
        results = df_save_values(save_values, save_config)
    elseif sol.prob.p.output_type == NamedTuple
        results = nt_save_values(save_values, save_config)
    else
        results = sol
    end
    # return just results if nominal, otherwise retun false as well to not rerun
    if i > 0
        return (results,false)
    else
        return results
    end 
end
