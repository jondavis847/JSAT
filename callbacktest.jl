using DifferentialEquations

function odefunc!(du,u,p,t)
    du[1] = u[2]
    du[2] = 0

    nothing
end
affect! = (integrator) -> integrator.u[2] = -1
cb = PresetTimeCallback(2,affect!,save_positions = (true,true))

prob = ODEProblem(odefunc!,[0,1],(0,10),1)
sol = solve(prob, callback = cb)
