function find_roots(B, U)
    # find any connections with inner body set to the world frame    
    root_U = U[map(x -> isa(x.Bᵢ, WorldFrame), U)]
    # return the outer bodies of the root connections
    map(x -> x.Bₒ, root_U)
end

function find_B_by_id(id, Bs)
    for B in Bs
        if B.id == id
            return B
        end
    end
end

function find_G_by_id(id, Gs)
    for G in Gs
        if G.id == id
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
function find_U_by_G(G::Joint, Us)
    for U in Us
        if U.G == G
            return U
        end
    end
end

function find_U_by_G(G::Int64, Us)
    for U in Us
        if U.G.id == G
            return U
        end
    end
end

function find_U_by_Bᵢ(B, Us)
    out_U = []
    for U in Us
        if U.Bᵢ == B
            push!(out_U, U)
        end
    end
    out_U
end

function find_U_by_N(Us)
    out_U = []
    for U in Us
        if isa(U.Bᵢ, WorldFrame)
            push!(out_U, U)
        end
    end
    out_U
end

#updates the bodys and joints with their identifying integers
function map_path!(Bs, Us)
    body_id = 0 

    #length(Bs)-1 since most of these dont care about N, or fixes for 0 based indexing
    p = zeros(Int64, length(Us)) # G predecessor
    s = zeros(Int64, length(Us)) # G successor
    λ = zeros(Int64, length(Bs)-1) # B parent array
    κ = [[] for _ in 1:length(Bs)-1] # all G before B[i]
    μ = OffsetArray([[] for _ in 1:length(B)],0:length(B)-1) # B child array
    ν = OffsetArray([[] for _ in 1:length(B)],0:length(B)-1) # all B after B[i]

    #find base connections    
    base_U = find_U_by_N(Us)    
    
    for U in base_U
        # one outer body per connection, id them and their joint
        body_id += 1
        U.Bₒ.id = body_id
        U.G.id = body_id

        #recursively get next bodies until tip
        #done in the for loop so new bodies 
        #attached to base are done after this system is complete
        body_id = get_B!(body_id, U.Bₒ, Us)
    end

    #update helpful arrays
    for U in Us
        p[U.G.id] = U.Bᵢ.id
        s[U.G.id] = U.Bₒ.id
    end
    for B in Bs
        if !isa(B,WorldFrame)
            #traverse body to base, updating κ and μ
            Uᵢ = find_U_by_Bₒ(B, Us)
            (B.id != 0) ? λ[B.id] = Uᵢ.Bᵢ.id : nothing
            #update μ 1 time        
            push!(μ[Uᵢ.Bᵢ.id], B.id)
            #update κ for all
            while !isa(Uᵢ.Bᵢ, WorldFrame)
                push!(κ[B.id], Uᵢ.G.id)
                Uᵢ = find_U_by_Bₒ(Uᵢ.Bᵢ, Us)
            end
            #loop ended on worldframe, add its joint to κ            
            push!(κ[B.id], Uᵢ.G.id)
        end

        #traverse body to tip, updating ν (\nu+tab)        
        get_ν!(ν[B.id], B, Us)
    end

    return p, s, λ, κ, μ, ν
end
function get_B!(id, B, Us)
    outer_U = find_U_by_Bᵢ(B, Us)
    for Uₒ in outer_U
        id += 1
        Uₒ.Bₒ.id = id
        Uₒ.G.id = id
        id = get_B!(id, Uₒ.Bₒ, Us)
    end
    return id
end
function get_ν!(ν, B, Us)
    outer_U = find_U_by_Bᵢ(B, Us)
    for Uₒ in outer_U
        push!(ν, Uₒ.Bₒ.id)
        get_ν!(ν, Uₒ.Bₒ, Us)
    end
end
#over load show to make pathtale printing pretty
function Base.show(io::IO, path::PathTable)
    #copy it so matrix version is still readable on sys
    path_table = copy(path.string_table)
    #make each element length 5 by filling with white space
    for i in eachindex(path_table)
        space_to_add = 7 - length(path_table[i])
        if (space_to_add == 7) && (i != 1)
            path_table[i] = "." * repeat(" ", space_to_add - 1)
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
function get_body_path(Bₒ, Us; path=[]) #call without path, path is applied recursively
    U = find_U_by_Bₒ(Bₒ, Us)
    if !isa(U.Bᵢ, WorldFrame) #remove this if we remove joints to worldframe
        push!(path, U.Bᵢ)
        path = get_body_path(U.Bᵢ, Us; path=path)
    end
    return path
end

function joint_path(Bs, Us)
    table = fill("", (length(Bs) + 1, length(Bs) + 1))
    bool_table = falses(length(Bs), length(Bs))
    for B in Bs
        table[B.id+1, 1] = string(B.name) #label row                
        path = get_joint_path(B, Us)
        for Gᵢ in path
            table[B.id+1, Gᵢ.id+1] = "x" #mark inner body elements
            bool_table[B.id, Gᵢ.id] = true #mark inner body elements
        end
    end
    PathTable(table, bool_table)
end

#returns all bodies in another bodies path
function get_joint_path(Bₒ, Us; path=[]) #call without path, path is applied recursively
    U = find_U_by_Bₒ(Bₒ, Us)
    push!(path, U.G)
    if !isa(U.Bᵢ, WorldFrame)
        path = get_joint_path(U.Bᵢ, Us; path=path)
    end
    return path
end

nothing