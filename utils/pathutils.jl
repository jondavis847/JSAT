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
        if G.meta.id == id
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
        if U.G.meta.id == G
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
    μ = OffsetArray([[] for _ in 1:length(Bs)],0:length(Bs)-1) # B child array
    γ = OffsetArray([[] for _ in 1:length(Bs)],0:length(Bs)-1) # all B after B[i]

    #find base connections    
    base_U = find_U_by_N(Us)    
    
    for U in base_U
        # one outer body per connection, id them and their joint
        body_id += 1
        U.Bₒ.id = body_id
        U.G.meta.id = body_id

        #recursively get next bodies until tip
        #done in the for loop so new bodies 
        #attached to base are done after this system is complete
        body_id = get_B!(body_id, U.Bₒ, Us)
    end

    #update helpful arrays
    for U in Us
        p[U.G.meta.id] = U.Bᵢ.id
        s[U.G.meta.id] = U.Bₒ.id
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
                push!(κ[B.id], Uᵢ.G.meta.id)
                Uᵢ = find_U_by_Bₒ(Uᵢ.Bᵢ, Us)
            end
            #loop ended on worldframe, add its joint to κ            
            push!(κ[B.id], Uᵢ.G.meta.id)
        end

        #traverse body to tip, updating γ (\nu+tab)        
        get_γ!(γ[B.id], B, Us)
    end

    return p, s, λ, κ, μ, γ
end
function get_B!(id, B, Us)
    outer_U = find_U_by_Bᵢ(B, Us)
    for Uₒ in outer_U
        id += 1
        Uₒ.Bₒ.id = id
        Uₒ.G.meta.id = id
        id = get_B!(id, Uₒ.Bₒ, Us)
    end
    return id
end
function get_γ!(γ, B, Us)
    outer_U = find_U_by_Bᵢ(B, Us)
    for Uₒ in outer_U
        push!(γ, Uₒ.Bₒ.id)
        get_γ!(γ, Uₒ.Bₒ, Us)
    end
end

nothing