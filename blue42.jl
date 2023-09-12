
include("utils//quaternion.jl")
using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames
import Base: show

abstract type Joint end

Base.@kwdef struct WorldFrame
    name::Symbol = :N
end

Base.@kwdef struct FrameRef
    r::SVector{3,Float64} # ᵇrʲ - location of joint origin in body frame 
    Φ::SMatrix{3,3,Float64,9} # ᵇΦʲ - rotation of joint frame in body frame  
    FrameRef(r, Φ) = new(SVector{3,Float64}(r), SMatrix{3,3,Float64}(Φ))
end

Base.@kwdef mutable struct Body
    name::Union{String,Symbol}
    #parameters
    m::Float64 # mass
    I::SMatrix{3,3,Float64} # inertia tensor
    cm::SVector{3,Float64} # center of mass    
    n::Int64 =0 # body number for table designation (applied automatically by sys)
    Body(name, m, I, cm,n) = new(name, m, SMatrix{3,3,Float64}(I), SVector{3,Float64}(cm),n)
end

Base.@kwdef struct Connection
    Bᵢ::Union{WorldFrame,Body}
    Fᵢ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    Bₒ::Body
    Fₒ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    G::Joint
end

struct PathTable
    table::Matrix{String}
end

Base.@kwdef mutable struct System
    name::Union{String,Symbol}
    world::WorldFrame
    bodies::Vector{Body}
    joints::Vector{Joint}
    connections::Vector{Connection}
    bodypath::PathTable = PathTable(Matrix{String}(undef,0,0))
    jointpath::Matrix{String} = Matrix{String}(undef,0,0)
    System(name, world, bodies, joints, connections,bodypath,jointpath) = new(
        name, world,
        ifelse(isa(bodies, Vector), bodies, [bodies]),
        ifelse(isa(joints, Vector), joints, [joints]),
        ifelse(isa(connections, Vector), connections, [connections]),
        bodypath,jointpath
    )
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
Base.@kwdef mutable struct DOF6 <: Joint
    const name::Union{String,Symbol}
    const Γ::Function = q -> qtoa(q)# joint partial function
    const DOF::Int64 = 6 # number of joint speeds
    q::SVector{4,Float64} = SVector{4,Float64}(0, 0, 0, 1)
    ω::SVector{3,Float64} = SVector{3,Float64}(0, 0, 0)
    r::SVector{3,Float64} = SVector{3,Float64}(0, 0, 0)
    v::SVector{3,Float64} = SVector{3,Float64}(0, 0, 0)
    n::Int64 = 0# body number for table designation (applied automatically by sys)
    DOF6(name, Γ, DOF, q, ω, r, v, n) = new(name, Γ, DOF, SVector{4,Float64}(q), SVector{3,Float64}(ω), SVector{3,Float64}(r), SVector{3,Float64}(v),n)
end


"""
Revolute Joint

    1DOF rotation in the x-z plane about y (to be consistent with Three.js geometry)

States:
    θ - rotation angle
    ω - angular rate
Joint frame:
    - right hand rule
    - x to the right, y up, z out of the page
    - θ referenced from +x        

"""
Base.@kwdef mutable struct Revolute <: Joint
    const name::Union{String,Symbol}
    const Γ::Function = θ -> SA[sin(θ), 0, cos(θ)] # joint partial function
    const DOF::Int64 = 1 # number of joint speeds
    θ::Float64 = 0.0
    ω::Float64 = 0.0
    n::Int64 = 0    
end

function find_roots(sys::System)
    # find any connections with inner body set to the world frame
    root_connections = sys.connections[map(x->isa(x.Bᵢ,WorldFrame),sys.connections)]
    # return the outer bodies of the root connections
    map(x->x.Bₒ, root_connections)        
end


function get_path!(sys::System)
    root_bodies = find_roots(sys)
    body_n = 0
    joint_n = 0
    map_tree!(root_bodies,body_n,joint_n,sys)    
end

#updates the bodys and joints with their identifying integers
function map_path!(bodies::Vector{Body},body_n,joint_n,sys)
    #loop over each of these bodies
    for this_body in bodies        
        #increment body counter and set to this body
        body_n += 1
        this_body.n = body_n
        #find all outer connections of thie body
        outer_connections = sys.connections[map(x->x.Bᵢ===this_body,sys.connections)]
        #loop over outer connections, number joints, and map outer bodies
        for this_outer in outer_connections
            joint_n += 1
            this_outer.G.n = joint_n
            body_n,joint_n = map_path!([this_outer.Bₒ],body_n,joint_n,sys)
        end
    end
    #return counters with updated values
    return body_n, joint_n
end

#over load show to make pathtale printing nice
function Base.show(io::IO, path::PathTable)
    #copy it so matrix version is still readable on sys
    path_table = copy(path.table)
    #make each element length 5 by filling with white space
    for i in eachindex(path_table)
        space_to_add = 5 - length(path_table[i])
        if (space_to_add == 5) && (i != 1)
            path_table[i] = ".    "
        else
            path_table[i] *= repeat(" ",space_to_add)
        end
    end
    for row in eachrow(path_table)
        println(io,join(row), "    ")
    end
end

#creates the table to be stored in sys
function bodypath!(sys)    
    table = fill("",(length(sys.bodies)+1,length(sys.bodies)+1))        
    for this_body in sys.bodies
        table[1,this_body.n+1] = string(this_body.name) #label column
        table[this_body.n+1,1] = string(this_body.name) #label row
        table[this_body.n+1,this_body.n+1] = "x" #mark diagonal element
        path = get_bodypath(this_body,sys)
        for inner_body in path
            table[this_body.n+1,inner_body.n+1] = "x" #mark inner body elements
        end
    end
    sys.bodypath = PathTable(table)
end

#returns all bodies in another bodies path
function get_bodypath(body,sys;path=[]) #call without path, path is applied recursively
    inner_connections = sys.connections[map(x->x.Bₒ===body,sys.connections)]
    if !isempty(inner_connections)
        inner_bodies = map(x->x.Bᵢ,inner_connections)        
        for inner_body in inner_bodies
            if !isa(inner_body,WorldFrame) #remove this if we remove joints to worldframe
                push!(path,inner_body)
            end
            path = get_bodypath(inner_body,sys;path=path)
        end
    end
    return path
end