using UnPack

abstract type AbstractBody end
#abstract type AbstractJoint{T} end

includet("frames.jl")
includet("joints.jl")


# mutable interface while creating models
mutable struct BodyConnection
    inner_joint::AbstractJoint
    outer_joints::Vector{AbstractJoint}
    function BodyConnection() 
        x = new()
        x.outer_joints = AbstractJoint[]
        return x
    end
end

# immutable static version for run time
struct BodyConnection🖖{N,J<:AbstractJoint,TJ<:NTuple{N,AbstractJoint}}
    inner_joint::J
    outer_joints::TJ
    BodyConnection🖖(b::BodyConnection) = new{
        length(b.outer_joints),
        typeof(b.inner_joint),typeof(Tuple(b.outer_joints))
    }(b.inner_joint, Tuple(b.outer_joints))
end

# mutable interface while creating models
mutable struct BodyMeta
    id::Int64
    BodyMeta() = new()
end

# immutable static version for run time
struct BodyMeta🖖{T<:Int64}
    id::T
    BodyMeta🖖(b::BodyMeta) = new{Int64}(b.id)
end

struct BaseFrame{M<:BodyMeta,C<:BodyConnection} <: AbstractBody
    name::Symbol
    meta::M
    connection::C
    BaseFrame{M,C}(name) where {M<:BodyMeta,C<:BodyConnection} = new(name, BodyMeta(), BodyConnection())
    BaseFrame(name) = new{BodyMeta,BodyConnection}(name)
end

struct MassProperties{M,C,I}
    mass::M
    cm::C
    inertia::I
    MassProperties(mass, cm, inertia) = new{Float64,SVector{3,Float64},SMatrix{3,3,Float64}}(
        Float64(mass),
        SVector{3,Float64}(cm),
        SMatrix{3,3,Float64}(inertia)
    )
end

# mutable interface while creating models
struct Body <: AbstractBody
    name::Symbol
    mass_props::MassProperties
    meta::BodyMeta
    connection::BodyConnection
    function Body(name, mass, cm, inertia)
        mp = MassProperties(mass, cm, inertia)
        meta = BodyMeta()
        connection = BodyConnection()
        return new(name, mp, meta, connection)
    end
end

struct Body🖖{S<:Symbol,MP<:MassProperties,M<:BodyMeta🖖,C<:BodyConnection🖖} <: AbstractBody
    name::S
    mass_props::MP
    meta::M
    connection::C
    function Body🖖(b::Body)                
        meta = BodyMeta🖖(b.meta)
        connection = BodyConnection🖖(b.connection)
        return new{Symbol,typeof(b.mass_props),typeof(meta),typeof(connection)}(b.name, b.mass_props, meta, connection)
    end
end