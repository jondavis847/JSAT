using DifferentialEquations

mutable struct Model    
    initial_state::Float64
    step_time::Vector{Float64}
    signal_value::Float64    
    Model(initial_state,step_time) = new(initial_state,step_time,0.0)
end

function odefunc!(du,u,p,t)
    for i in eachindex(p.models)
        du[i] = p.models[i].signal_value * 1.0 #arbitrary function
    end
    return nothing
end

function initialize_models(models)
    u = getfield.(models,:initial_state)
    p = (;models = models)    
    cbs = []
    for i in eachindex(models)
        push!(cbs, PresetTimeCallback(
            models[i].step_time, #condition
            (integrator) -> integrator.p.models[i].signal_value = 1.0) #affect!
            )
    end
    cbs = CallbackSet(cbs...)
    return u,p,cbs
end

function simulate(original_models, adaptive = true)

    # copy since we're goign to mutate values during sim so we can rerun simualte(models)
    models = deepcopy(original_models) 
    u0,p,cbs = initialize_models(models)
    prob = ODEProblem(odefunc!, u0, (0.,10.), p)
    if adaptive
        sol = solve(prob, Tsit5(), callback = cbs)
    else
        sol = solve(prob, Tsit5(), adaptive = false, dt = 1, callback = cbs)
    end
    return sol
end


model1 = Model(0.0,[1.0])
model2 = Model(-1.0,[2.0])

models = [model1,model2]

sol = simulate(models, true) #doesnt seem to work

#sol = simulate(models, false) #works


