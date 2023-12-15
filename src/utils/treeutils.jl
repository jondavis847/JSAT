#updates the bodys and joints with their identifying integers
function map_tree!(base)
    body_id = 0
    root_joints = base.outer_joints
    
    for joint in root_joints
        # one outer body per joint, id them and their joint
        body_id += 1
        joint.meta.id = body_id
        joint.connection.successor.id = body_id

        #recursively get next bodies until tip
        #done in the for loop so new bodies 
        #attached to base are done after this system is complete
        body_id = get_next_body!(body_id, joint.connection.successor)
    end
end
function get_next_body!(id, body)
    for joint in body.outer_joints
        id += 1
        joint.meta.id = id
        joint.connection.successor.id = id        
        id = get_next_body!(id, joint.connection.successor)
    end
    return id
end

nothing