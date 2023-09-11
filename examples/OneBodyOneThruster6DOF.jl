using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack

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
q₀ = SVector{4}([0, 0, 0, 1]) # ᴺqᴮ attitude quaternion
ω₀ = SVector{3}([π/4, 0, 0]) # body attitude rate
r₀ = SVector{3}(zeros(3)) # initial translational position
v₀ = SVector{3}(zeros(3)) # initial translational velocity
thruster_command = 0. # thruster command, 0 or 1, handled by callbacks
🖖₀ = SVector{14}([ω₀; v₀; q₀; r₀; thruster_command])

# Equations of Motion
# 🖖 = [ω,v,q,r]
function EOM!(🖖, p, t)
    # map states and parameters
    ω = 🖖[SVector{3}(1:3)]
    v = 🖖[SVector{3}(4:6)]
    q = 🖖[SVector{4}(7:10)]
    r = 🖖[SVector{3}(11:13)]
    thruster_command = 🖖[14]

    @unpack m̄,Ī,F,T,αₜ,Ω,V = p
    
    Hₛ = Ī * ω # system momentum

    # u̇ equations (1) Equation 42
    COEF = Ω' * Ī * Ω + V' * m̄ * V
    RHS = Ω' * (T*thruster_command - Ī * αₜ - ω×Hₛ) + V' * (F*thruster_command - m̄ * αₜ)

    u̇ = COEF \ RHS

    # ẋ equations (1) Equations 43:45
    q̇ = 0.5 * SA[
             q[4] -q[3] q[2]
             q[3] q[4] -q[1]
             -q[2] q[1] q[4]
             -q[1] -q[2] -q[3]
         ] * ω
    ṙ = v

    return [u̇;q̇;ṙ;0]    
end


f_thrust_start!(s) = s.u[14] = 1.    
thrust_start = PresetTimeCallback(5.,f_thrust_start!)

f_thrust_stop!(s) = s.u[14] = 0.
thrust_stop = PresetTimeCallback(6.,f_thrust_stop!)

saved_values = SavedValues(Float64, Tuple{SVector{3, Float64},SVector{3, Float64}})
save_cb = SavingCallback((u,t,s) -> (s.p.F,s.p.T), saved_values)

prob = ODEProblem(EOM!, 🖖₀, (0, 10),p)
sol = solve(prob, Tsit5(), callback = CallbackSet(thrust_start,thrust_stop,save_cb))

ẇ = inv(I)*(T- w×H)

