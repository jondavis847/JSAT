includet("GravityConstant.jl")

connect!(b::AbstractBody,g::AbstractGravity) = push!(b.gravity,g)

function gravity!(sys::MultibodySystem)
    calculate_gravity!.(sys.bodies)
    return nothing
end