function ode_func!(dx, x, p, t)
    update_model!(p.sys, x)
    model!(p.sys)
    pack_dq_in_dx!(dx, p.sys)
    nothing
end

function update_model!(sys, x)    
    set_state!.(sys.joints,Ref(x))
    set_state!.(sys.software,Ref(x))
    calculate_transforms!(sys)  # spatial transforms
    calculate_r!(sys) # update generalized coords
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
    nothing
end

function model!(sys)
    #sensors!(sys)
    software!(sys)
    actuators!(sys)
    environments!(sys)
    dynamics!(sys)
    return nothing
end

function simulate(orig_sys::MultibodySystem, tspan; output_type=nothing)
    sys = deepcopy(orig_sys)# make a copy so we can rerun orig sys without mutating it during previous sim   
       
    u0 = initialize_state_vectors(sys)

    save_config, save_values, save_cb = configure_saving(sys)
    p = (sys=sys, save_config=save_config)

    #get callbacks
    callbacks = [save_cb; getfield.(sys.software, :callback)...]
    cb = CallbackSet(callbacks...)

    prob = ODEProblem(ode_func!,u0, tspan, p)
    sol = solve(prob, callback=cb, adaptive=false, dt=0.01) # higher sample rate until three animation keyframetracks are better understood for interpolation

    if output_type == DataFrame
        simout = df_save_values(save_values, save_config)
    elseif output_type == NamedTuple
        simout = nt_save_values(save_values, save_config)
    else
        simout = sol
    end
    return simout
end

