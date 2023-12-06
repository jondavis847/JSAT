using DifferentialEquations, LinearAlgebra, StaticArrays

struct Model{I}
    uindex::I
end

struct MySystem{N,M<:Model}
    # Type stable StaticArrays code requires a statically sized system,
    # hence using NTuple instead of Vector (StaticVector would also work)
    models::NTuple{N,M}
end

computeX(x) = -1e-2 * x * x'
run_model(x) = computeX(x) * x
run_models(sys, u) = vcat((run_model(u[model.uindex]) for model in sys.models)...)
odefunc(u, p, t) = run_models(p.sys, u)

function simulate(sys, u; tspan=(0, 10), solver=Tsit5())
    p = (; sys=sys)   # No need for deepcopy here, everything is immutable

    _save(u, models) = Tuple(computeX(u[m.uindex]) for m in models)
    save_function(u, t, integrator) = _save(u, integrator.p.sys.models)
    save_values = SavedValues(eltype(u), typeof(_save(u, p.sys.models)))
    save_cb = SavingCallback(save_function, save_values)

    prob = ODEProblem(odefunc, u, tspan, p)
    sol = solve(prob, solver, callback=save_cb)
    return sol, save_values
end

u = @SArray randn(6)
model1 = Model(SA[1:3...])  # Need statically sized slices here
model2 = Model(SA[4:6...])

sys = MySystem((model1, model2))
sol, save_values = simulate(sys, u; solver=Rosenbrock23())
# Rosenbrock23() works!