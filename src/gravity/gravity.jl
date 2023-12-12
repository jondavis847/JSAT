includet("GravityConstant.jl")

connect!(b::AbstractBody,g::AbstractGravity) = push!(b.gravity,g)