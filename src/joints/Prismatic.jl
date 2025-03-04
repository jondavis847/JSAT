"""
Prismatic Joint

    1DOF translation

States:
    r - position
    v - velocity
Joint frame:
    - x to the right, y up, z out of the page    
"""
mutable struct PrismaticState <: AbstractJointState
    r::Float64
    v::Float64
    τ::SVector{1,Float64}
    q̈::SVector{1,Float64}
    # temp variables for the ABA
    c::SVector{6,Float64}
    biasforce::SVector{6,Float64}
    a′::SVector{6,Float64}
    U::SMatrix{6,1,Float64} # (6,nDOF)
    D::SMatrix{1,1,Float64} # (nDOF,nDOF)
    u::SVector{1,Float64} # nDOF
    function PrismaticState(r, v)
        x = new()
        x.r = r
        x.v = v        
        #need zeros placeholders so saving_dicts can initialize
        x.c = @SVector zeros(6)
        x.biasforce = @SVector zeros(6)
        x.a′ = @SVector zeros(6)
        x.U = @SMatrix zeros(6, 1)
        x.D = @SMatrix zeros(1, 1)
        x.u = @SVector zeros(1)
        return x
    end
end
mutable struct Prismatic <: AbstractJoint
    meta::JointMeta
    state::PrismaticState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,1,Float64}
end

function Prismatic(name, r=0.0, v=0.0)
    jm = JointMeta(name, 1, 1)
    js = PrismaticState(r, v)
    S = SMatrix{6,1,Float64}(0, 0, 0, 1, 0, 0)
    joint = Prismatic(jm, js, JointConnection(), eye(Cartesian), S)
    update_joint_frame!(joint, r)
    return joint
end


get_q(G::Prismatic) = SVector{1,Float64}(G.state.r)
get_q̇(G::Prismatic) = SVector{1,Float64}(G.state.v)
get_dq(G::Prismatic) = SVector{1,Float64}(G.state.v)

function set_state!(G::Prismatic, x)
    q = @view x[G.meta.xindex]
    q̇ = @view x[G.meta.ẋindex]

    G.state.r = q[1]
    G.state.v = q̇[1]
    update_joint_frame!(G::Prismatic, G.state.r)
    nothing
end

function update_joint_frame!(G::Prismatic, r=G.state.r)
    Φ = SMatrix{3,3,Float64,9}(1, 0, 0, 0, 1, 0, 0, 0, 1)
    G.frame = Cartesian(Φ, SVector{3,Float64}(r, 0, 0))
    nothing
end

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::Prismatic) = nothing

get_callback(J::Prismatic, i) = []
