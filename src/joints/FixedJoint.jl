"""
    FixedJoint(;name,[q,ω,r,v])

0DOF joint for rotation and translation

States:
- `q::SVector{4,Float64}` quaternion rotation of Fs wrt Fp
- `r::SVector{3,Float64}` position vector in Gᵢ from Fp to Fs

Joint frame:
- x right, y up, z out the page
- identity quaternion means body x,y,z aligns with joint x,y,z        
    
"""
mutable struct FixedJointState <: AbstractJointState    
    q::SVector{4,Float64}    
    r::SVector{3,Float64}    
    τ::SVector{6,Float64}
    q̈::SVector{6,Float64}
end
mutable struct FixedJoint <: AbstractJoint
    meta::JointMeta
    state::FixedJointState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,6,Float64}
end

function FixedJoint(name,
    q::AbstractVector=SVector{4,Float64}(0, 0, 0, 1),    
    r::AbstractVector=SVector{3,Float64}(0, 0, 0),    
)
    jm = JointMeta(name,0,0)
    js = FixedJointState(q, r,SVector{6, Float64}(zeros(6)),SVector{6, Float64}(zeros(6)))
    S = SMatrix{6,6,Float64,36}(I(6))
    joint = FixedJoint(jm, js, JointConnection(),eye(Cartesian), S)        
    R = qtoa(SVector{4,Float64}(q))    
    joint.frame = Cartesian(R,r)
    return joint
end

get_q(G::FixedJoint) = [G.state.q; G.state.r]

get_q̇(G::FixedJoint) = @SVector zeros(6)

get_dq(G::FixedJoint) = @SVector zeros(7)

set_state!(G::FixedJoint, x) = nothing

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::FixedJoint) = nothing

get_savedict(G::FixedJoint) = []
