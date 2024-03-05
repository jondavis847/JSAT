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
    # temp variables for the ABA
    c::SVector{6,Float64}
    biasforce::SVector{6,Float64}
    function FixedJointState(q,r) 
        x = new()
        x.q = q
        x.r = r
        #need zeros placeholders so saving_dicts can initialize
        x.τ = @SVector zeros(6)
        x.q̈ = @SVector zeros(6)        
        x.c = @SVector zeros(6)
        x.biasforce = @SVector zeros(6)       
        return x
    end
end
mutable struct FixedJoint <: AbstractJoint
    meta::JointMeta
    state::FixedJointState
    connection::JointConnection
    frame::Cartesian
    locked::Bool
    S::SMatrix{6,6,Float64}
end

function FixedJoint(name,
    q::AbstractVector=SVector{4,Float64}(0, 0, 0, 1),
    r::AbstractVector=SVector{3,Float64}(0, 0, 0),
)
    jm = JointMeta(name, 0, 0)
    js = FixedJointState(q, r)
    S = SMatrix{6,6,Float64,36}(I(6))
    joint = FixedJoint(jm, js, JointConnection(), eye(Cartesian), true, S)
    R = qtoa(SVector{4,Float64}(q))
    joint.frame = Cartesian(R, r)
    return joint
end

get_q(G::FixedJoint) = [G.state.q; G.state.r]

get_q̇(G::FixedJoint) = @SVector zeros(6)

get_dq(G::FixedJoint) = @SVector zeros(7)

set_state!(G::FixedJoint, x) = nothing

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::FixedJoint, pA) = nothing

get_savedict(G::FixedJoint) = []

get_callback(J::FixedJoint, i) = []
