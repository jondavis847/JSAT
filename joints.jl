##NOTE we may need to add S∘ for joint space derivatives if translations looks really bad for the base 6dof joint!!!!


abstract type Joint end
abstract type JointState{T} end

mutable struct JointMeta
    name::Symbol
    id::Int16
    qindex::Vector{Int16} # index for generalized coords in sys.q
    q̇index::Vector{Int16} # index for generalized speeds in sys.q
    xindex::Vector{Int16} # index for generalzied coords in sys.x
    ẋindex::Vector{Int16} # index for generalized speeds in sys.x
    nq::Int8
    nq̇::Int8
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
struct DOF6{T<:AbstractFloat} <: Joint
    meta::JointMeta
    state::DOF6State{T}
end

function DOF6(name,
    q::AbstractVector{T}=SVector{4,T}(0, 0, 0, 1),
    ω::AbstractVector{T}=SVector{3,T}(0, 0, 0),
    r::AbstractVector{T}=SVector{3,T}(0, 0, 0),
    v::AbstractVector{T}=SVector{3,T}(0, 0, 0),
) where {T<:AbstractFloat}
    jm = JointMeta(name, 0, [], [], [], [], 7, 6)
    js = DOF6State{T}(q, ω, r, v)
    DOF6{T}(jm, js)
end

get_q(G::DOF6) = [G.state.q; G.state.r]
get_q̇(G::DOF6) = [G.state.ω; G.state.v]
get_dq(G::DOF6) = [quaternion_derivative(G.state.q, G.state.ω);G.state.v]


# put this in quaternions.jl
function quaternion_derivative(q, ω)
    Q = @SMatrix [
        q[4] -q[3] q[2]
        q[3] q[4] -q[1]
        -q[2] q[1] q[4]
        -q[1] -q[2] -q[3]
    ]
    return 0.5 * Q * ω
end


function set_state!(G::DOF6, x)
    G.state.q = x[G.meta.xindex][SVector{4,Int8}(1, 2, 3, 4)]
    G.state.r = x[G.meta.xindex][SVector{3,Int8}(5, 6, 7)]
    G.state.ω = x[G.meta.ẋindex][SVector{3,Int8}(1, 2, 3)]
    G.state.v = x[G.meta.ẋindex][SVector{3,Int8}(4, 5, 6)]
end

Φ(G::DOF6{T}) where {T<:AbstractFloat} = qtoa(G.state.q) #TODO make rotations convert on rotation types
ρ(G::DOF6{T}) where {T<:AbstractFloat} = G.state.r
𝒮(::DOF6{T}) where {T<:AbstractFloat} = SMatrix{6,6,T}(I(6))

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
    θ::T
    ω::T
end
struct Revolute{T<:AbstractFloat} <: Joint
    meta::JointMeta
    state::RevoluteState{T}
end

function Revolute(name, θ::T, ω::T) where {T<:AbstractFloat}
    jm = JointMeta(name, 0, [], [], [],[], 1, 1)
    js = RevoluteState(θ, ω)
    Revolute(jm, js)
end

Φ(G::Revolute{T}) where {T<:AbstractFloat} = SA[cos(G.state.θ) -sin(G.state.θ) 0.0; sin(G.state.θ) cos(G.state.θ) 0.0; 0.0 0.0 1.0]
ρ(::Revolute{T}) where {T<:AbstractFloat} = SVector{3,T}(0, 0, 0)
𝒮(::Revolute{T}) where {T<:AbstractFloat} = SMatrix{6,1,T}(0, 0, 1, 0, 0, 0)
get_q(G::Revolute{T}) where {T<:AbstractFloat} = SVector{1,T}(G.state.θ)
get_q̇(G::Revolute{T}) where {T<:AbstractFloat} = SVector{1,T}(G.state.ω)
get_dq(G::Revolute{T}) where {T<:AbstractFloat} = SVector{1,T}(G.state.ω)

function set_state!(G::Revolute, x)
    G.state.θ = x[G.meta.xindex][1]
    G.state.ω = x[G.meta.ẋindex][1]
end

# transforms joint frame to cartesian frame type
Cartesian(G::Joint) = Cartesian(Φ(G), ρ(G))

# transforms joint frame to spatial frame types
ℳ(G::Joint) = ℳ(Φ(G), ρ(G))
ℳ⁻¹(G::Joint) = ℳ⁻¹(Φ(G), ρ(G))
ℱ(G::Joint) = ℱ(Φ(G), ρ(G))
ℱ⁻¹(G::Joint) = ℱ⁻¹(Φ(G), ρ(G))
