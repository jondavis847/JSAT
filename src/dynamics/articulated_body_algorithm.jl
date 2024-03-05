function articulated_body_algorithm!(sys)
    first_pass!.(sys.bodies)
    second_pass!.(Iterators.reverse(sys.bodies))
    third_pass!.(sys.bodies)
    return nothing
end

first_pass!(body::BaseFrame) = nothing
function first_pass!(body)
    joint = body.innerjoint    
    parent = joint.connection.predecessor
    if isa(joint, FixedJoint)
        body.state.v_ijof = body.transforms.parent_to_ijof_motion * parent.state.v_ijof
        joint.state.c = @SVector zeros(6)
    else        
        S = joint.S
        vj = S * get_q̇(joint) # velocity across just the inner joint frame, not to be confused with v_ijof which is body velocity in the outer joint frame
        body.state.v_ijof = body.transforms.parent_to_ijof_motion * parent.state.v_ijof + vj
        joint.state.c = body.state.v_ijof ×ᵐ vj # + cj #commented until we need S∘                
    end

    body.inertia.articulated = body.inertia.ijof
    body.state.gyroscopic_force_ijof = body.state.v_ijof ×ᶠ (body.inertia.ijof * body.state.v_ijof + body.state.internal_momentum_ijof)
    body.state.gyroscopic_force = ℱ(inv(joint.connection.Fs)) * body.state.gyroscopic_force_ijof
    joint.state.biasforce = body.state.gyroscopic_force_ijof - body.state.external_force_ijof
    return nothing
end

second_pass!(body::BaseFrame) = nothing
function second_pass!(body)
    joint = body.innerjoint
    parent = joint.connection.predecessor

    if isa(joint, FixedJoint)
        if !isa(parent, BaseFrame)
            parent.inertia.articulated = parent.inertia.articulated +
                                         body.transforms.ijof_to_parent_force *
                                         body.inertia.articulated *
                                         body.transforms.parent_to_ijof_motion

            parent.innerjoint.state.biasforce = parent.innerjoint.state.biasforce + body.transforms.ijof_to_parent_force * joint.state.biasforce
        end
    else
        S = joint.S
        joint.state.U = body.inertia.articulated * S
        joint.state.D = S' * joint.state.U
        calculate_τ!(joint)
        joint.state.u = joint.state.τ - S' * joint.state.biasforce
        if !isa(parent, BaseFrame)
            Ia = body.inertia.articulated - joint.state.U * inv(joint.state.D) * joint.state.U'
            pa = joint.state.biasforce + Ia * joint.state.c + joint.state.U * inv(joint.state.D) * joint.state.u
            parent.inertia.articulated = parent.inertia.articulated
            +body.transforms.ijof_to_parent_force * Ia * body.transforms.parent_to_ijof_motion
            parent.innerjoint.state.biasforce = parent.innerjoint.state.biasforce + body.transforms.ijof_to_parent_force * pa
        end
    end
    return nothing
end

third_pass!(body::BaseFrame) = nothing
function third_pass!(body)
    joint = body.innerjoint
    parent = joint.connection.predecessor
    if isa(joint, FixedJoint)
        joint.state.q̈ = 0.0 * joint.state.q̈
        body.state.a_ijof = body.transforms.parent_to_ijof_motion * parent.state.a_ijof
    else        
        S = joint.S
        joint.state.a′ = body.transforms.parent_to_ijof_motion * parent.state.a_ijof + joint.state.c
        joint.state.q̈ = inv(joint.state.D) * (joint.state.u - joint.state.U' * joint.state.a′)
        body.state.a_ijof = joint.state.a′ + S * joint.state.q̈
    end
    return nothing
end