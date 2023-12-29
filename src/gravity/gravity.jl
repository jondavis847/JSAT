connect!(b::AbstractBody,g::AbstractGravity) = push!(b.models.gravity,g)
function gravity!(sys::MultibodySystem)
    calculate_gravity!.(sys.bodies)
end

function calculate_gravity!(body::AbstractBody) 
    #reset each step
    body.gravity *= 0

    for gravity in body.models.gravity        
        body.gravity += body.inertia_joint * body.transforms.base_to_body_motion * calculate_gravity(gravity, body.state.r_base)
    end
    return nothing
end

includet("GravityConstant.jl")
includet("TwoBody.jl")