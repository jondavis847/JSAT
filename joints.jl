##NOTE we may need to add S∘ for joint space derivatives if translations looks really bad for the base 6dof joint!!!!


abstract type Joint end
abstract type JointState{T} end

mutable struct JointMeta{T<:Integer}
    name::Symbol
    id::T
    qspan::Vector{T}
    q̇span::Vector{T}
    DOF::T
    nq::T
    nq̇::T
end

"""
    DOF6(;name,[q,ω,r,v])

6DOF joint for rotation and translation

States:
- `q::SVector{4,Float64}` quaternion rotation of Bₒ in Bᵢ
- `ω::SVector{3,Float64}`  angular rate

Joint frame:
- x right, y up, z out the page
- identity quaternion means body x,y,z aligns with joint x,y,z        
    
"""
mutable struct DOF6State{T<:AbstractFloat} <: JointState{T}
    q::SVector{4,T}
    ω::SVector{3,T}
    r::SVector{3,T}
    v::SVector{3,T}        
end
struct DOF6{TI<:Integer,TF<:AbstractFloat} <: Joint
    meta::JointMeta{TI}
    state::DOF6State{TF}
end

function DOF6(name,
    q{T} = SVector{4,Float64}(0, 0, 0, 1),
    ω{T} = SVector{3,Float64}(0, 0, 0),
    r{T} = SVector{3,Float64}(0, 0, 0),
    v{T} = SVector{3,Float64}(0, 0, 0),
    ) where T <: AbstractFloat
    jm = JointMeta(name,0,[],[],6,7,6)
    js = DOF6State(q,ω,r,v)
    DOF6(jm,js)
end

get_q(G::DOF6) = [G.q;G.r]
get_q̇(G::DOF6) = [G.ω;G.v]

function set_state!(G::DOF6,x)
    G.q = x.q[G.qspan][SVector{4,Int8}(1,2,3,4)]
    G.r = x.q[G.qspan][SVector{3,Int8}(5,6,7)]
    G.ω = x.q̇[G.q̇span][SVector{3,Int8}(1,2,3)]
    G.v = x.q̇[G.q̇span][SVector{3,Int8}(4,5,6)]
end

Φ(G::DOF6) = qtoa(G.q)
ρ(G::DOF6) = G.r
𝒮(::DOF6) = SMatrix{6,6,Float64}(I(6))
𝒯(::DOF6) = SMatrix{6,6,Float64}(I(6))

"""
Revolute Joint

    1DOF rotation in the x-y plane about z

States:
    θ - rotation angle
    ω - angular rate
Joint frame:
    - right hand rule
    - x to the right, y up, z out of the page
    - θ referenced from +x        
"""
mutable struct RevoluteState{T<:AbstractFloat} <: JointState{T}
    θ::SVector{1,T}
    ω::SVector{1,T}    
end
struct Revolute{TI<:Integer,TF<:AbstractFloat} <: Joint
    meta::JointMeta{TI}
    state::DOF6State{TF}
end

function Revolute(name, θ = SVector{1,T}(0), ω = SVector{1,T}(0)) where T <: AbstractFloat
    jm = JointMeta(name,0,[],[],1,1,1)
    js = RevoluteState(θ,ω)
    Revolute(jm,js)
end

Φ(G::Revolute) = SA[cos(G.θ) -sin(G.θ) 0.0; sin(G.θ) cos(G.θ) 0.0; 0.0 0.0 1.0]
ρ(::Revolute) = SVector{3,Float64}(0,0,0)
𝒮(::Revolute) = SMatrix{6,1,Float64}(0,0,1,0,0,0)
get_q(G::Revolute) = SVector{1,Float64}(G.θ)
get_q̇(G::Revolute) = SVector{1,Float64}(G.ω)

function set_state!(G::Revolute,x)
    G.θ = x.q[G.qspan]
    G.ω = x.q̇[G.q̇span]
end

# transforms joint frame to cartesian frame type
Cartesian(G::Joint) = Cartesian(Φ(G),ρ(G))

# transforms joint frame to spatial frame types
ℳ(G::Joint) = ℳ(Φ(G),ρ(G))
ℳ⁻¹(G::Joint) = ℳ⁻¹(Φ(G),ρ(G))
ℱ(G::Joint) = ℱ(Φ(G),ρ(G))
ℱ⁻¹(G::Joint) = ℱ⁻¹(Φ(G),ρ(G))
