function composite_rigid_body(sys)
    first_pass_crb!.(sys.bodies)
    second_pass_crb!.(sys.bodies)
end

function first_pass_crb!(body)
    body.inertia.articulated = body.inertia.ijof
    return nothing
end

function second_pass_crb!(body)
    joint = body.innerjoint
    parent = joint.predecessor

    if !isa(parent,BaseFrame)
        parent.inertia.articulated = parent.inertia.articulated + body.transforms.body_to_parent_force * body.inertia.articulated * body.transforms.parent_to_body_motion        
    end
end