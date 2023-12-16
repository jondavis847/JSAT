includet("GravityConstant.jl")

connect!(b::AbstractBody,g::AbstractGravity) = push!(b.gravity,g)

function gravity!(sys::MultibodySystem)
    calculate_gravity!.(sys.bodies)
    return nothing
end

function calculate_gravity!(body::AbstractBody) 
    #reset each step
    body.gravity *= 0

    for gravity in body.models.gravity        
        body.gravity += body.inertia_joint * body.transforms.base_to_body_motion * calculate_gravity(gravity)
    end
    return nothing
end