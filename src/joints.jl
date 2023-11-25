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


include(joinpath("joints","FixedJoint.jl"))
include(joinpath("joints","FloatingJoint.jl"))
include(joinpath("joints","Revolute.jl"))
include(joinpath("joints","Spherical.jl"))