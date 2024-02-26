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
    if joint.locked || isa(joint, FixedJoint)
        body.state.v_ijof = body.transforms.parent_to_ijof_motion * parent.state.v_ijof
        body.tmp.c = @SVector zeros(6)
    else        
        S = joint.S
        vj = S * get_q̇(joint) # velocity across just the inner joint frame, not to be confused with v_ijof which is body velocity in the outer joint frame
        body.state.v_ijof = body.transforms.parent_to_ijof_motion * parent.state.v_ijof + vj
        body.tmp.c = body.state.v_ijof ×ᵐ vj # + cj #commented until we need S∘                
    end

    body.inertia.articulated = body.inertia.ijof
    body.state.gyroscopic_force = body.state.v_ijof ×ᶠ (body.inertia.ijof * body.state.v_ijof + body.state.internal_momentum_ijof)
    body.tmp.pᴬ = body.state.gyroscopic_force_ijof - body.state.external_force_ijof
    return nothing
end

second_pass!(body::BaseFrame) = nothing
function second_pass!(body)
    joint = body.innerjoint
    parent = joint.connection.predecessor

    if joint.locked || isa(joint, FixedJoint)
        if !isa(parent, BaseFrame)
            parent.inertia.articulated = parent.inertia.articulated +
                                         body.transforms.ijof_to_parent_force *
                                         body.inertia.articulated *
                                         body.transforms.parent_to_ijof_motion

            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.ijof_to_parent_force * body.tmp.pᴬ
        end
    else
        S = joint.S
        body.tmp.U = body.inertia.articulated * S
        body.tmp.D = S' * body.tmp.U
        body.tmp.u = joint.state.τ - S' * body.tmp.pᴬ
        if !isa(parent, BaseFrame)
            Ia = body.inertia.articulated - body.tmp.U * inv(body.tmp.D) * body.tmp.U'
            pa = body.tmp.pᴬ + Ia * body.tmp.c + body.tmp.U * inv(body.tmp.D) * body.tmp.u
            parent.inertia.articulated = parent.inertia.articulated
            +body.transforms.ijof_to_parent_force * Ia * body.transforms.parent_to_ijof_motion
            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.ijof_to_parent_force * pa
        end
    end
    return nothing
end

third_pass!(body::BaseFrame) = nothing
function third_pass!(body)
    joint = body.innerjoint
    parent = joint.connection.predecessor
    if joint.locked || isa(joint, FixedJoint)
        joint.state.q̈ = 0.0 * joint.state.q̈
        body.state.a_ijof = body.transforms.parent_to_ijof_motion * parent.state.a_ijof
    else        
        S = joint.S
        body.tmp.a′ = body.transforms.parent_to_ijof_motion * parent.state.a_ijof + body.tmp.c
        joint.state.q̈ = inv(body.tmp.D) * (body.tmp.u - body.tmp.U' * body.tmp.a′)
        body.state.a_ijof = body.tmp.a′ + S * joint.state.q̈
    end
    return nothing
end