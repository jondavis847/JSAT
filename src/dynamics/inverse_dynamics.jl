function inverse_dynamics(sys)
    first_pass_id!.(sys.bodies)
    second_pass_id!.(Iterators.reverse(sys.bodies))
end

function first_pass_id!(body, q̈ = 0)
    joint = body.innerjoint
    parent = joint.predecessor

    v_joint = joint.S * get_q̇(joint)
    body.state.v_body = body.transforms.parent_to_body_motion * parent.state.v_body + v_joint
    body.state.a_body = body.transforms.parent_to_body_motion * parent.state.a_body + joint.S * q̈ + body.state.v_body ×ᵐ v_joint
    body.tmp.pᴬ = body.inertia.ijof * body.state.a_body + body.state.v_body ×ᶠ (body.inertia.ijof * body.state.v_body) - body.state.external_force    
    return nothing
end

function second_pass_id!(body)
    joint = body.innerjoint
    parent = joint.predecessor

    joint.state.τ = joint.S' * body.tmp.pᴬ
    if !isa(parent,BaseFrame)
        parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.body_to_parent_force * body.tmp.pᴬ
    end
    return nothing
end