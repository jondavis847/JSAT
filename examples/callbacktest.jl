using LinearAlgebra,DifferentialEquations,ComponentArrays,UnPack

Base.@kwdef struct ObservedStates
    x
    H::Function = (x)
end

x = ComponentArray(
    θ = zeros(3), #angles
    ω = [50/1000,0,0], #rates
    T = zeros(3), #torque 
    Hᵇ = [50,0,0], #ang momentum
    Hˢ = [50,0,50],
    wxh = cross([50/1000,0,0],[50,0,50])
)
p = (
    J = 1000*I(3), #inertia tensor   
    Hⁱ = [0,0,50]
)

# integration function?
function euler!(d,x,p,t)
    @unpack ω,wxh,T = x
    @unpack J = p
    
    d.θ = ω
    d.ω = J \ (T - wxh)
    nothing
end

# FunctionCallback function
function euler_cb!(u,t,s)
    u.Hᵇ = s.p.J * u.ω
    u.Hˢ = u.Hᵇ + s.p.Hⁱ 
    u.wxh = u.ω×u.Hˢ
    nothing
end

# start torquing to have something to look at
function torque_cb!(s)
    s.u.T[1] = 0
end
torque_cb = PresetTimeCallback(1,torque_cb!)

# sim

prob = ODEProblem(euler!, x, (0, 5), p)
sol = solve(prob,Tsit5(),callback = CallbackSet(torque_cb), adaptive = false, dt = 1.)
