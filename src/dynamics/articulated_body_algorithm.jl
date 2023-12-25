function articulated_body_algorithm!(sys)
    first_pass!.(sys.bodies)
    second_pass!.(Iterators.reverse(sys.bodies))
    third_pass!.(sys.bodies)
    return nothing
end

first_pass!(body::BaseFrame) = nothing
function first_pass!(body)
    joint = body.inner_joint

    if isa(joint, FixedJoint)
        body.state.v_body = body.transforms.parent_to_body_motion * joint.connection.predecessor.state.v_body
        body.tmp.c = @SVector zeros(6)
    else
        v_joint = joint.S * get_q̇(joint)
        body.state.v_body = body.transforms.parent_to_body_motion * joint.connection.predecessor.state.v_body + v_joint
        body.tmp.c = body.state.v_body ×ᵐ v_joint # + cj #commented until we need S∘                
    end

    body.inertia_articulated = body.inertia_joint
    body.tmp.pᴬ = body.state.v_body ×ᶠ (body.inertia_joint * body.state.v_body + body.internal_momentum) - body.external_force
    return nothing
end

second_pass!(body::BaseFrame) = nothing
function second_pass!(body)
    joint = body.inner_joint
    parent = joint.connection.predecessor

    if isa(joint, FixedJoint)
        if !isa(parent, BaseFrame)
            parent.inertia_articulated = parent.inertia_articulated +
                                         body.transforms.body_to_parent_force *
                                         body.inertia_articulated *
                                         body.transforms.parent_to_body_motion

            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.body_to_parent_force * body.tmp.pᴬ
        end
    else
        S = joint.S
        body.tmp.U = body.inertia_articulated * S
        body.tmp.D = S' * body.tmp.U
        body.tmp.u = joint.state.τ - S' * body.tmp.pᴬ
        if !isa(parent, BaseFrame)
            Ia = body.inertia_articulated - body.tmp.U * inv(body.tmp.D) * body.tmp.U'
            pa = body.tmp.pᴬ + Ia * body.tmp.c + body.tmp.U * inv(body.tmp.D) * body.tmp.u
            parent.inertia_articulated = parent.inertia_articulated
            +body.transforms.body_to_parent_force * Ia * body.transforms.parent_to_body_motion
            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.body_to_parent_force * pa
        end
    end
    return nothing
end

third_pass!(body::BaseFrame) = nothing
function third_pass!(body)
    joint = body.inner_joint
    parent = joint.connection.predecessor
    if isa(joint, FixedJoint)
        body.state.a_body = body.transforms.parent_to_body_motion * parent.state.a_body
    else
        S = joint.S
        body.tmp.a′ = body.transforms.parent_to_body_motion * parent.state.a_body + body.tmp.c
        joint.state.q̈ = inv(body.tmp.D) * (body.tmp.u - body.tmp.U' * body.tmp.a′)
        body.state.a_body = body.tmp.a′ + S * joint.state.q̈
    end
    return nothing
end