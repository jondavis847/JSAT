##NOTE we may need to add S∘ for joint space derivatives if translations looks really bad for the base 6dof joint!!!!


abstract type AbstractJoint end
abstract type AbstractJointState end

mutable struct JointMeta
    name::Symbol
    nq::Int8
    nq̇::Int8
    id::Int16
    qindex::Vector{Int16} # index for generalized coords in sys.q
    q̇index::Vector{Int16} # index for generalized speeds in sys.q
    xindex::Vector{Int16} # index for generalzied coords in sys.x
    ẋindex::Vector{Int16} # index for generalized speeds in sys.x
    function JointMeta(name,nq,nq̇)    
        x = new()
        x.name = name
        x.nq = nq
        x.nq̇ = nq̇
        x
    end
end

mutable struct JointConnection
    predecessor::AbstractBody # predecessor body
    successor::AbstractBody # successor body
    Fp::Cartesian # transform from joint to precessor body frames
    Fs::Cartesian # transform from joint to successor body frames
    JointConnection() = new()
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
mutable struct DOF6State <: AbstractJointState
    q::SVector{4,Float64}
    ω::SVector{3,Float64}
    r::SVector{3,Float64}
    v::SVector{3,Float64}
end
struct DOF6 <: AbstractJoint
    meta::JointMeta
    state::DOF6State
    connection::JointConnection
end

function DOF6(name,
    q::AbstractVector=SVector{4,Float64}(0, 0, 0, 1),
    ω::AbstractVector=SVector{3,Float64}(0, 0, 0),
    r::AbstractVector=SVector{3,Float64}(0, 0, 0),
    v::AbstractVector=SVector{3,Float64}(0, 0, 0),
)
    jm = JointMeta(name,7,6)
    js = DOF6State(q, ω, r, v)
    DOF6(jm, js, JointConnection())
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

Φ(G::DOF6) = qtoa(G.state.q) #TODO make rotations convert on rotation types
ρ(G::DOF6) = G.state.r
𝒮(::DOF6) = SMatrix{6,6,Float64}(I(6))

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
mutable struct RevoluteState <: AbstractJointState
    θ::Float64
    ω::Float64
end
struct Revolute <: AbstractJoint
    meta::JointMeta
    state::RevoluteState
    connection::JointConnection
end

function Revolute(name, θ = 0., ω = 0.)
    jm = JointMeta(name, 1, 1)
    js = RevoluteState(θ, ω)
    Revolute(jm, js, JointConnection())
end

Φ(G::Revolute) = SA[cos(G.state.θ) -sin(G.state.θ) 0.0; sin(G.state.θ) cos(G.state.θ) 0.0; 0.0 0.0 1.0]
ρ(::Revolute) = SVector{3,Float64}(0, 0, 0)
𝒮(::Revolute) = SMatrix{6,1,Float64}(0, 0, 1, 0, 0, 0)
get_q(G::Revolute) = SVector{1,Float64}(G.state.θ)
get_q̇(G::Revolute) = SVector{1,Float64}(G.state.ω)
get_dq(G::Revolute) = SVector{1,Float64}(G.state.ω)

function set_state!(G::Revolute, x)
    G.state.θ = x[G.meta.xindex][1]
    G.state.ω = x[G.meta.ẋindex][1]
end

# transforms joint frame to cartesian frame type
Cartesian(G::AbstractJoint) = Cartesian(Φ(G), ρ(G))

# transforms joint frame to spatial frame types
ℳ(G::AbstractJoint) = ℳ(Φ(G), ρ(G))
ℳ⁻¹(G::AbstractJoint) = ℳ⁻¹(Φ(G), ρ(G))
ℱ(G::AbstractJoint) = ℱ(Φ(G), ρ(G))
ℱ⁻¹(G::AbstractJoint) = ℱ⁻¹(Φ(G), ρ(G))

#method for connecting bodies to joints
function connect!(
    G::AbstractJoint,
    p::AbstractBody, #predecessor
    s::AbstractBody, #successor
    Fp::Cartesian = Cartesian(I(3),[0,0,0]), #transform from joint to predecessor
    Fs::Cartesian = Cartesian(I(3),[0,0,0]) #transform from joint to successor
    ) 

    G.connection.predecessor = p
    G.connection.successor = s
    G.connection.Fs = Fs
    G.connection.Fp = Fp
    nothing

end