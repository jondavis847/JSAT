using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays

# Parameters
p = (
    m=1000, # mass
    m̄=1000 * I(3), # mass matrix
    Ī=1000 * I(3), # inertia tensor
    # Assumptions
    F=zeros(3), # no forces
    T=zeros(3), # no torques  
    αₜ=zeros(3), # no remainder accelerations
    # Partial Velocity matrices
    Ω=[I(3) zeros(3, 3)], #(1) Equation 29
    V=[zeros(3, 3) I(3)], #(1) Equation 30
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
    )
)

# Equations of Motion
function EOMCA!(Δ🍕, 🍕, p, t)
    # map states and parameters    
    @unpack u, x = 🍕
    @unpack ω, v = u
    @unpack q, r = x
    @unpack m̄, Ī, F, T, αₜ, Ω, V = p

    Hₛ = Ī * ω # system momentum

    # u̇ equations (1) Equation 42
    COEF = Ω' * Ī * Ω + V' * m̄ * V
    RHS = Ω' * (T - Ī * αₜ - ω × Hₛ) + V' * (F - m̄ * αₜ)

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

# no StaticArrays - 2852 allocations, 242.39 KiB, 144.4μs

prob = ODEProblem(EOMCA!, 🍕, (0, 10), p)
sol = solve(prob, Tsit5())