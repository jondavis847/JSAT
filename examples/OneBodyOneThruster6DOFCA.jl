using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays

# Parameters
p = (
    m = 1000, # mass
    m̄ = SMatrix{3,3}(1000*I(3)), # mass matrix
    Ī = SMatrix{3,3}(1000*I(3)), # inertia tensor
    # Assumptions
    # thruster is 1N, 1m along b̂₁ pointing in b̂₂
    F = SVector{3}([0,1,0]), #N from thruster    
    T = SVector{3}([0,0,-1]), #Nm from thruster Fxr    
    αₜ = SVector{3}(zeros(3)), # no remainder accelerations
    # Partial Velocity matrices
    Ω = SMatrix{3,6}([I(3) zeros(3,3)]), #(1) Equation 29
    V = SMatrix{3,6}([zeros(3,3) I(3)]), #(1) Equation 30    
)

# Initial Conditions
🍕 = ComponentArray(
    u=ComponentArray(        
        ω=[π / 4, 0, 0], # body attitude rate
        v=zeros(3), # initial translational velocity
    ),
    x=ComponentArray(
        q=[0, 0, 0, 1], # ᴺqᴮ attitude quaternion    
        r=zeros(3), # initial translational position        
    ),
    Hₛ = p.Ī*[π / 4, 0, 0],
    thruster_command = 0.
)

# Equations of Motion
function eom!(Δ🍕, 🍕, p, t)
    # map states and parameters    
    ω = SVector{3}(🍕.u.ω)
    v = SVector{3}(🍕.u.ω)
    q = SVector{4}(🍕.x.q)
    r = SVector{3}(🍕.x.r)
    Hₛ = SVector{3}(🍕.Hₛ) # system momentum    
    thruster_command = 🍕.thruster_command
    
    @unpack m̄, Ī, F, T, αₜ, Ω, V = p    

    # u̇ equations (1) Equation 42
    COEF = Ω' * Ī * Ω + V' * m̄ * V
    RHS = Ω' * (T*thruster_command - Ī * αₜ - ω × Hₛ) + V' * (F*thruster_command - m̄ * αₜ)

    Δ🍕.u = COEF \ RHS

    # ẋ equations (1) Equations 43:45
    Δ🍕.x.q = 0.5 * SA[
               q[4] -q[3] q[2]
               q[3] q[4] -q[1]
               -q[2] q[1] q[4]
               -q[1] -q[2] -q[3]
           ] * ω
    Δ🍕.x.r = v
    return nothing
end


f_thrust_start!(s) = s.u.thruster_command = 1.    
thrust_start = PresetTimeCallback(5.,f_thrust_start!)

f_thrust_stop!(s) = s.u.thruster_command = 0.
thrust_stop = PresetTimeCallback(6.,f_thrust_stop!)

#define any algebraic relationships here
function func_cb!(u,t,s)
    s.u.Hₛ = s.p.Ī * s.u.u.ω    
    return nothing
end
func_cb = FunctionCallingCallback(func_cb!)
function per_cb!(s)
    s.u.Hₛ = s.p.Ī * s.u.u.ω    
    return nothing
end
per_cb = PeriodicCallback(per_cb!,1)

prob = ODEProblem(eom!, 🍕, (0, 10), p)
sol = solve(prob, Tsit5(), callback = CallbackSet(thrust_start,thrust_stop,per_cb),dt = 1, adaptive=false)