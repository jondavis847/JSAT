function composite_rigid_body(sys)
    first_pass_crb!.(sys.bodies)
    second_pass_crb!.(sys.bodies)
end

function first_pass_crb!(body)
    body.inertia_articulated = body.inertia_joint
    return nothing
end

function second_pass_crb!(body)
    joint = body.inner_joint
    parent = joint.predecessor

    if !isa(parent,BaseFrame)
        parent.inertia_articulated = parent.inertia_articulated + body.transforms.body_to_parent_force * body.inertia_articulated * body.transforms.parent_to_body_motion        
    end
end