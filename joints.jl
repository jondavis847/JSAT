##NOTE we may need to add S∘ for joint space derivatives if translations looks really bad for the base 6dof joint!!!!

abstract type AbstractJoint end
abstract type AbstractJointState end
mutable struct JointMeta
    name::Symbol
    nq::Int64
    nq̇::Int64
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

# transforms joint frame to spatial frame types
ℳ(G::AbstractJoint) = ℳ(G.frame)
ℳ⁻¹(G::AbstractJoint) = ℳ⁻¹(G.frame)
ℱ(G::AbstractJoint) = ℱ(G.frame)
ℱ⁻¹(G::AbstractJoint) = ℱ⁻¹(G.frame)

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
mutable struct DOF6 <: AbstractJoint
    meta::JointMeta
    state::DOF6State
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,6,Float64}
end

function DOF6(name,
    q::AbstractVector=SVector{4,Float64}(0, 0, 0, 1),
    ω::AbstractVector=SVector{3,Float64}(0, 0, 0),
    r::AbstractVector=SVector{3,Float64}(0, 0, 0),
    v::AbstractVector=SVector{3,Float64}(0, 0, 0),
)
    jm = JointMeta(name,7,6)
    js = DOF6State(q, ω, r, v)
    S = SMatrix{6,6,Float64,36}(I(6))
    joint = DOF6(jm, js, JointConnection(),eye(Cartesian), S)
    update_joint_frame!(joint,q,r)
    return joint
end

function get_vj(G::DOF6)
    @unpack ω,v,r = G.state

    vj = MVector{6,Float64}(undef)
    vj[i3] = ω
    vj[i6] = v# + ω×r
    SVector{6,Float64}(vj)
end

function get_q(G::DOF6)
    #E = qtoa(G.state.q) # need to transform translation quantities to outer joint frame
    [G.state.q; G.state.r]
end
function get_q̇(G::DOF6)     
    [G.state.ω; G.state.v]
end
function get_dq(G::DOF6)     
    [quaternion_derivative(G.state.q, G.state.ω);G.state.v]
end


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

function set_state!(G::DOF6, q, q̇)
    # joint frame translation is in the inner joint frame, but q is expressed in outer joint frame
    # rotate all translation to inner joint frame using q
    G.state.q = q[SVector{4,Int8}(1, 2, 3, 4)]
    E = qtoa(G.state.q)
    #G.state.r = E' * q[SVector{3,Int8}(5, 6, 7)]
    G.state.r = q[SVector{3,Int8}(5, 6, 7)]
    G.state.ω = q̇[SVector{3,Int8}(1, 2, 3)]
    G.state.v = q̇[SVector{3,Int8}(4, 5, 6)]
    update_joint_frame!(G,G.state.q,G.state.r)
    nothing
end

function update_joint_frame!(G::DOF6,q = G.state.q, r = G.state.r)
    R = qtoa(G.state.q)
    r = G.state.r
    G.frame = Cartesian(R,R'*r)
    nothing
end

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
mutable struct Revolute <: AbstractJoint
    meta::JointMeta
    state::RevoluteState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,1,Float64}
end

function Revolute(name, θ = 0., ω = 0.)
    jm = JointMeta(name, 1, 1)
    js = RevoluteState(θ, ω)
    S = SMatrix{6,1,Float64}(0,0,1,0,0,0)
    joint = Revolute(jm, js, JointConnection(),eye(Cartesian),S)
    update_joint_frame!(joint,θ)
    return joint
end


get_q(G::Revolute) = SVector{1,Float64}(G.state.θ)
get_q̇(G::Revolute) = SVector{1,Float64}(G.state.ω)
get_dq(G::Revolute) = SVector{1,Float64}(G.state.ω)

function set_state!(G::Revolute, q ,q̇)
    G.state.θ = q[1]
    G.state.ω = q̇[1]    
    update_joint_frame!(G::Revolute,G.state.θ)
    nothing
end

function update_joint_frame!(G::Revolute, θ = G.state.θ)
    R = SA[
        cos(θ) sin(θ) 0 
        -sin(θ) cos(θ) 0 
        0 0 1
    ]
    G.frame = Cartesian(R,@SVector zeros(3))
    nothing
end

"""
Spherical Joint

    3DOF rotation

States:
    q - quaternion
    ω - angular rate
Joint frame:
    - right hand rule
    - x to the right, y up, z out of the page    
"""
mutable struct SphericalState <: AbstractJointState
    q::SVector{4,Float64}
    ω::SVector{3,Float64}
end
mutable struct Spherical <: AbstractJoint
    meta::JointMeta
    state::SphericalState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,3,Float64}
end

function Spherical(name, q = [0.,0.,0.,1.], ω = zeros(3))
    jm = JointMeta(name, 4, 3)
    js = SphericalState(q, ω)
    S = SMatrix{6,3,Float64}([
        1 0 0
        0 1 0
        0 0 1
        0 0 0
        0 0 0
        0 0 0
    ])
    joint = Spherical(jm, js, JointConnection(),eye(Cartesian),S)
    update_joint_frame!(joint,q)
    return joint
end

get_q(G::Spherical) = SVector{4,Float64}(G.state.q)
get_q̇(G::Spherical) = SVector{3,Float64}(G.state.ω)
get_dq(G::Spherical) = quaternion_derivative(G.state.q, G.state.ω)

function set_state!(G::Spherical, q ,q̇)
    G.state.q = q
    G.state.ω = q̇
    update_joint_frame!(G::Spherical)
    nothing
end

function update_joint_frame!(G::Spherical,q = G.state.q,r = @SVector zeros(3))    
    R = qtoa(q)
    G.frame = Cartesian(R,r)
    nothing
end
