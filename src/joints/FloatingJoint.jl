"""
    FloatingJoint(;name,[q,ω,r,v])

6DOF joint for rotation and translation

States:
- `q::SVector{4,Float64}` quaternion rotation of Fs wrt Fp
- `ω::SVector{3,Float64}` angular rate of Fs wrt Fp
- `r::SVector{3,Float64}` position vector in Gᵢ from Fp to Fs
- `v::SVector{3,Float64}` velocity vector in Gᵢ of Fs wrt Fp
 

Joint frame:
- x right, y up, z out the page
- identity quaternion means body x,y,z aligns with joint x,y,z        
    
"""
mutable struct FloatingJointState <: AbstractJointState
    q::SVector{4,Float64}
    ω::SVector{3,Float64}
    r::SVector{3,Float64}
    v::SVector{3,Float64}
    τ::SVector{6,Float64}
    q̈::SVector{6,Float64}
end
mutable struct FloatingJoint <: AbstractJoint
    meta::JointMeta
    state::FloatingJointState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,6,Float64}
end

function FloatingJoint(name,
    q::AbstractVector=SVector{4,Float64}(0, 0, 0, 1),
    ω::AbstractVector=SVector{3,Float64}(0, 0, 0),
    r::AbstractVector=SVector{3,Float64}(0, 0, 0),
    v::AbstractVector=SVector{3,Float64}(0, 0, 0),
)
    jm = JointMeta(name, 7, 6)
    js = FloatingJointState(q, ω, r, v, SVector{6,Float64}(zeros(6)), SVector{6,Float64}(zeros(6)))
    S = SMatrix{6,6,Float64,36}(I(6))
    joint = FloatingJoint(jm, js, JointConnection(), eye(Cartesian), S)
    update_joint_frame!(joint, q, r)
    return joint
end

function get_vj(G::FloatingJoint)
    @unpack ω, v, r = G.state

    vj = MVector{6,Float64}(undef)
    vj[i3] = ω
    vj[i6] = v# + ω×r
    SVector{6,Float64}(vj)
end

function get_q(G::FloatingJoint)
    #E = qtoa(G.state.q) # need to transform translation quantities to outer joint frame
    [G.state.q; G.state.r]
end
function get_q̇(G::FloatingJoint)
    [G.state.ω; G.state.v]
end
function get_dq(G::FloatingJoint)
    [quaternion_derivative(G.state.q, G.state.ω); G.frame.Φ.value' * G.state.v]
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

function set_state!(G::FloatingJoint, x)
    q = @view x[G.meta.xindex]
    q̇ = @view x[G.meta.ẋindex]

    G.state.q = q[SVector{4,Int8}(1, 2, 3, 4)]
    G.state.r = q[SVector{3,Int8}(5, 6, 7)]
    G.state.ω = q̇[SVector{3,Int8}(1, 2, 3)]
    G.state.v = q̇[SVector{3,Int8}(4, 5, 6)]
    update_joint_frame!(G)
    nothing
end

function update_joint_frame!(G::FloatingJoint, q=G.state.q, r=G.state.r)
    R = qtoa(G.state.q)
    r = G.state.r
    #G.frame = Cartesian(R,R'*r)
    G.frame = Cartesian(R, r)
    nothing
end

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::FloatingJoint) = nothing