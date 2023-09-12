using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays

include("..\\blue42.jl")

N = WorldFrame()

B = Body(
    name=:B,
    m=100,
    I=SMatrix{3,3,Float64}(I(3) * 100),
    cm=zeros(3),    
)

dof6 = DOF6(name = :ᴺGᴮ)

ᴺUᴮ = Connection(
    Bᵢ = N,
    Bₒ = B,
    G = dof6
)

sys = System(
    name=:oneBody6DOF,
    world = N,
    bodies=B,
    joints=dof6,
    connections= ᴺUᴮ
)
#=
# Initial Conditions
🖖 = ComponentArray(
    body=ComponentArray(
        q=[0, 0, 0, 1],
        ω=zeros(3), # body attitude rate
        r=zeros(3),
        v=zeros(3), # initial translational velocity
    ),
    pendulum=ComponentArray(
        θ=0,
        σ=pi / 6,
    ),
    joint=(
        rᵦⱼ=[0, 1, 0],
        rₚⱼ=[0, 1, 0],
        r₂=[0, 0, 0]
    )
)

scross(x) = SA[
    0 -x[3] x[2]
    x[3] 0 -x[1]
    -x[2] x[1] 0
]
function qtoa(q)
    i123 = SA[1, 2, 3]
    a = diagm(q[4]^2 - q[i123]' * q[i123]) + 2 * q[i123] * q[i123]' - 2 * scross(q[i123] * q[4])
end

# Equations of Motion
function eom!(d, 🖖, p, t)
    # map states and parameters    
    ω = SVector{3}(🖖.body.ω)
    v = SVector{3}(🖖.body.v)
    q = SVector{4}(🖖.body.q)
    r = SVector{3}(🖖.body.r)
    θ = 🖖.pendulum.θ
    σ = 🖖.pendulum.σ
    rᵦⱼ = 🖖.joint.rᵦⱼ
    rₚⱼ = 🖖.joint.rₚⱼ
    r₂ = 🖖.joint.r₂

    @unpack m̄, Ī = p

    Γ = SA[0, 0, 1]

    ᴺCᴮ = qtoa(q)
    ᴺCᵖ = [
        cos(θ) sin(θ) 0
        sin(θ) -cos(θ) 0
        0 0 1
    ]
    Ω = [
        I(3) zeros(3) zeros(3, 3)
        C Γ zeros(3, 3)
    ]

    ρ₂₁ =
        V = [
            zeros(3, 3) zeros(3) I(3)
            r2
        ]

    # u̇ equations (1) Equation 42
    COEF = Ω' * Ī * Ω + V' * m̄ * V
    RHS = Ω' * (T - Ī * αₜ - ω × (Ī * ω)) + V' * (F - m̄ * αₜ)

    d.u = COEF \ RHS

    # ẋ equations (1) Equations 43:45
    d.x.q = 0.5 * SA[
                q[4] -q[3] q[2]
                q[3] q[4] -q[1]
                -q[2] q[1] q[4]
                -q[1] -q[2] -q[3]
            ] * ω
    d.x.r = v
    nothing
end


f_thrust_start!(s) = s.u.thruster_command = 1.0
thrust_start = PresetTimeCallback(5.0, f_thrust_start!)

f_thrust_stop!(s) = s.u.thruster_command = 0.0
thrust_stop = PresetTimeCallback(6.0, f_thrust_stop!)

#define any algebraic relationships here
function func_cb!(u, t, s)
    s.u.Hₛ = s.p.Ī * s.u.u.ω
    nothing
end
func_cb = FunctionCallingCallback(func_cb!)
function per_cb!(s)
    s.u.Hₛ = s.p.Ī * s.u.u.ω
    nothing
end
per_cb = PeriodicCallback(per_cb!, 1)

prob = ODEProblem(eom!, 🖖, (0, 10), p)
sol = solve(prob, Tsit5(), callback=CallbackSet(thrust_start, thrust_stop, per_cb), dt=1, adaptive=false)
=#