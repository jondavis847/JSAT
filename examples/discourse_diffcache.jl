# More specific imports to reduce precompilation time
using DifferentialEquations, LinearAlgebra, PreallocationTools

struct Model{I,DC<:DiffCache}
    uindex::I
    Xcache::DC

    function Model(u, uindex)
        x = u[uindex]
        Xcache = DiffCache(x * x')
        return new{typeof(uindex),typeof(Xcache)}(uindex, Xcache)
    end
end

struct MySystem{M<:Model}
    models::Vector{M}
end

# Factor out the computation of X such that it can be called
# both in run_model! and the callback
function computeX!(Xcache, x)
    X = get_tmp(Xcache, x)
    # x and X are not static arrays anymore, so we need
    # in-place mul! to avoid allocations. In this case, we could
    # also have used broadcasting like this:
    # @. X = -1e-2 * x * x'
    mul!(X, x, x', -1e-2, 0.0)  # arbitrary example function
    return X                    # minus sign for ODE stability
end

function run_models!(sys, u, du)
    # Use @views to avoid allocating copies
    @views for model in sys.models
        x = u[model.uindex]
        X = computeX!(model.Xcache, x)
        mul!(du[model.uindex], X, x)
    end
    return nothing
end

function odefunc!(du, u, p, t)
    run_models!(p.sys, u, du)
    return nothing
end

function simulate(sys, u; tspan=(0, 10), solver=Tsit5())
    p = (; sys=deepcopy(sys))
    
    # Note: calling computeX! in the callback to get the
    # correct X for the current u, then making a copy for
    # saving; without the copy, you'd be saving the same object
    # every time
    _save(u, models) = Tuple(copy(computeX!(m.Xcache, u[m.uindex])) for m in models)
    save_function(u, t, integrator) = _save(u, integrator.p.sys.models)
    # Let the compiler/runtime figure out the types
    save_values = SavedValues(eltype(u), typeof(_save(u, p.sys.models)))
    save_cb = SavingCallback(save_function, save_values)

    prob = ODEProblem(odefunc!, u, tspan, p)
    sol = solve(prob, solver; callback=save_cb)
    return sol, save_values
end;

u = randn(6)
model1 = Model(u, 1:3)  # ranges rather than StaticVector to
model2 = Model(u, 4:6)  # slice `u`

sys = MySystem([model1, model2])
sol, save_values = simulate(sys, u; solver=Rosenbrock23())
# Rosenbrock23() works!