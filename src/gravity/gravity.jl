connect!(b::AbstractBody,g::AbstractGravity) = push!(b.models.gravity,g)
function gravity!(sys::MultibodySystem)
    calculate_gravity!.(sys.bodies)
end

function calculate_gravity!(body::AbstractBody) 
    #reset each step
    body.gravity *= 0
    body.gravity_ijof *= 0

    for gravity in body.models.gravity        
        body.gravity_ijof += body.inertia.ijof * body.transforms.base_to_ijof_motion * calculate_gravity(gravity, body.state.r_base)
        body.gravity += ℱ(inv(body.innerjoint.connection.Fs))*body.gravity_ijof
    end
    return nothing
end

includet("GravityConstant.jl")
includet("TwoBody.jl")