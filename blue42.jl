
include("utils//quaternion.jl")
include("joints.jl")
using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames
import Base: show
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
    id::Int64 = 0 # body number for table designation (applied automatically by sys)
    Body(name, m, I, cm, id) = new(name, m, SMatrix{3,3,Float64}(I), SVector{3,Float64}(cm), id)
end

Base.@kwdef struct Connection
    Bᵢ::Union{WorldFrame,Body}
    Fᵢ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    Bₒ::Body
    Fₒ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    G::Joint
end

struct PathTable
    string_table::Matrix{String}
    bool_table::Matrix{Bool}
end

struct System
    name::Union{String,Symbol}
    N::WorldFrame
    B::Vector{Body}
    G::Vector{Joint}
    U::Vector{Connection}
    bodypath::PathTable
    #jointpath::PathTable
    rotations::Matrix{SMatrix{3,3,Float64,9}}
    Γ::Vector{Union{SVector,SMatrix}}
    Ω::Matrix{Float64}
end

#constructor
function System(; name, N, B, G, U)

    # 1) find the body path of the system
    #ensure we start id's with the root bodies
    root_B = find_roots(B, U)

    # map the bodies and joints 
    map_path!(root_B, U, 0, 0)

    # get the body path
    bodypath = body_path(B, U)
    jointpath = joint_path(B,U)

    # 2) find all the rotations of the system
    C = fill(SMatrix{3,3,Float64}(zeros(3, 3)), (length(B), length(B)))
    get_rotations!(C, B, U, bodypath)

    # 3) calculate joint partials Vector
    Γ = joint_partials(U)

    # 4) calculate Ω
    Ω = calculate_Ω(C, Γ, bodypath)

    System(name, N, B, G, U, bodypath, C, Γ, Ω)
end

function get_rotations!(M, B, U, bodypath=body_path(B, U))
    for i in eachindex(B)
        for j in eachindex(B)
            if bodypath.bool_table[i, j]
                if i == j
                    M[i, j] = SMatrix{3,3,Float64}(1, 0, 0, 0, 1, 0, 0, 0, 1)
                else
                    Bₒ = B[map(x -> x.id == i, B)][1]
                    Bᵢ = B[map(x -> x.id == j, B)][1]
                    M[i, j] = get_C(Bₒ, Bᵢ, U)
                end
            end
        end
    end
    return nothing
end

#find the C matrix from one body to another in its path
function get_C(Bₒ, Bᵢ, Us)
    #find the connection where input arg Bₒ is the ... Bₒ
    U = find_U_by_Bₒ(Bₒ, Us)
    # get C for this layer
    C = U.Fᵢ.Φ * U.Fₒ.Φ * Φ(U.G)
    # dig deeper if needed
    if U.Bᵢ != Bᵢ
        C *= get_C(U.Bᵢ, Bᵢ, Us)
    end
    return C
end

function find_roots(B, U)
    # find any connections with inner body set to the world frame    
    root_U = U[map(x -> isa(x.Bᵢ, WorldFrame), U)]
    # return the outer bodies of the root connections
    map(x -> x.Bₒ, root_U)
end

#updates the bodys and joints with their identifying integers
function map_path!(B, U, body_id, joint_id)
    #loop over each of these bodies
    for body in B        
        if !isa(body,WorldFrame) #starting with worldframe but dont want it in the body path table, but want to mark the joint
            #increment body counter and set to this body
            body_id += 1
            body.id = body_id
        end
        #find all outer connections of the body
        Uₒ = U[map(x -> x.Bᵢ === body, U)]
        #loop over outer connections, number joints, and map outer bodies
        for this_U in Uₒ
            joint_id += 1
            this_U.G.id = joint_id
            body_id, joint_id = map_path!([this_U.Bₒ], U, body_id, joint_id)
        end
    end
    #return counters with updated values
    return body_id, joint_id
end

#over load show to make pathtale printing pretty
function Base.show(io::IO, path::PathTable)
    #copy it so matrix version is still readable on sys
    path_table = copy(path.string_table)
    #make each element length 5 by filling with white space
    for i in eachindex(path_table)
        space_to_add = 5 - length(path_table[i])
        if (space_to_add == 5) && (i != 1)
            path_table[i] = ".    "
        else
            path_table[i] *= repeat(" ", space_to_add)
        end
    end
    for row in eachrow(path_table)
        println(io, join(row), "    ")
    end
end

#creates the table to be stored in sys
function body_path(B, U)
    table = fill("", (length(B) + 1, length(B) + 1))
    bt = falses(length(B), length(B))
    for body in B
        table[1, body.id+1] = string(body.name) #label column
        table[body.id+1, 1] = string(body.name) #label row
        table[body.id+1, body.id+1] = "x" #mark diagonal element
        bt[body.id, body.id] = true
        path = get_body_path(body, U)
        for inner_body in path
            table[body.id+1, inner_body.id+1] = "x" #mark inner body elements
            bt[body.id, inner_body.id] = true #mark inner body elements
        end
    end
    PathTable(table, bt)
end

#returns all bodies in another bodies path
function get_body_path(body, Us; path=[]) #call without path, path is applied recursively
    Uᵢ = find_U_by_Bₒ(body, Us)
    if !isempty(Uᵢ)
        Bᵢs = map(x -> x.Bᵢ, Uᵢ)
        for Bᵢ in Bᵢs
            if !isa(Bᵢ, WorldFrame) #remove this if we remove joints to worldframe
                push!(path, Bᵢ)
            end
            path = get_body_path(Bᵢ, Us; path=path)
        end
    end
    return path
end

function joint_path(Bs, Us)
    table = fill("", (length(Bs) + 1, length(Bs) + 1))
    bt = falses(length(Bs), length(Bs))
    for U in Us
        table[1, U.G.id+1] = string(U.G.name) #label column
    end

    for B in Bs
        table[B.id+1, 1] = string(B.name) #label row                
        path = get_joint_path(B, Us)
        for Gᵢ in path
            table[B.id+1, Gᵢ.id+1] = "x" #mark inner body elements
            bt[B.id, Gᵢ.id] = true #mark inner body elements
        end
    end
    PathTable(table, bt)
end

#returns all bodies in another bodies path
function get_joint_path(Bₒ, Us; path=[]) #call without path, path is applied recursively
    U = find_U_by_Bₒ(Bₒ, Us)        
    push!(path, U.G)
    if !isa(U.Bᵢ,WorldFrame)
        path = get_joint_path(U.Bᵢ, Us; path=path)
    end
    return path
end

function joint_partials(U)
    # I believe map path is set up to handle the ordering here, may need to test to ensure
    map(x -> Γ(x.G), U)
end

function calculate_Ω(Cs, Γs, bodypath=trues(size(Cs))) #default to calc all
    m = 3 * length(Γs) #guaranteed to be 3 elements per Γ
    sc = size.(Γs, 2)
    n = sum(sc) #num cols = sum of DOF of each joint

    Ω = zeros(m, n)
    for i in eachindex(Γs) # C is guaranteed to be length(B),length(B) and Γ is guaranteed to be length(B)        
        rows = (3*i:3*i+2) .- 2
        for j in eachindex(Γs)
            #only do these calcs if theres a connect between bodies, otherwise leave as zeros
            if bodypath.bool_table[i, j]
                ncols = sc[j]
                icol = sum(sc[1:j-1]) + 1
                cols = icol:icol+ncols-1

                Ω[rows, cols] = Cs[i, j] * Γs[j]
            end
        end
    end
    return Ω
end
function find_B_by_id(id, Bs)
    for B in Bs
        if Bs.id == id
            return B
        end
    end
end

function find_G_by_id(id, Gs)
    for G in Gs
        if Gs.id == id
            return G
        end
    end
end

function find_U_by_Bₒ(B, Us)
    for U in Us
        if U.Bₒ == B
            return U
        end
    end
end

function find_U_by_Bᵢ(B, Us)
    for U in Us
        if U.Bₒ == B
            return U
        end
    end
end

function calculate_ρ(id_B, id_G, Bs, Gs, Us)
    B = find_B_by_id(id_B, Bs)
    G = find_G_by_id(id_G, Gs)
    U = find_U_by_Bₒ(B, Us)

    ρ = U.Fₒ.r - B.cm
    if U.G == G
        return ρ
    else
        return calculateρ(U.Bᵢ.id, id_G, Bs, Gs, Us) + ρ
    end
end