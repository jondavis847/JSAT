find_root_joints(joints) = joints[map(x -> isa(x.connection.predecessor, BaseFrame), joints)]

find_joints_by_predecessor(body, joints) = joints[map(x -> x.connection.predecessor == body, joints)]

function find_joint_by_successor(body, joints)
    for joint in joints
        if joint.connection.successor == body
            return joint
        end
    end
end

#updates the bodys and joints with their identifying integers
function map_tree!(bodies, joints)
    body_id = 0 

    #length(bodies)-1 since most of these dont care about BaseFrame
    p = zeros(Int16, length(joints)) # joint predecessor
    s = zeros(Int16, length(joints)) # joint successor
    λ = zeros(Int16, length(bodies)-1) # body parent array
    κ = [Int16[] for _ in 1:length(bodies)-1] # all joints before body [i]
    μ = OffsetArray([Int16[] for _ in 1:length(bodies)],0:length(bodies)-1) # body child array
    γ = OffsetArray([Int16[] for _ in 1:length(bodies)],0:length(bodies)-1) # all bodies after body[i]

    #find root joints    
    root_joints = find_root_joints(joints)
    
    for joint in root_joints
        # one outer body per joint, id them and their joint
        body_id += 1
        joint.meta.id = body_id
        joint.connection.successor.id = body_id

        #recursively get next bodies until tip
        #done in the for loop so new bodies 
        #attached to base are done after this system is complete
        body_id = get_next_body!(body_id, joint.connection.successor, joints)
    end

    #update helpful arrays
    for joint in joints
        p[joint.meta.id] = joint.connection.predecessor.id
        s[joint.meta.id] = joint.connection.successor.id
    end
    for body in bodies
        if !isa(body,BaseFrame)
            #traverse body to base, updating κ and μ
            joint = find_joint_by_successor(body, joints)
            λ[body.id] = joint.connection.predecessor.id
            #update μ 1 time        
            push!(μ[joint.connection.predecessor.id], body.id)
            #update κ for all
            while !isa(joint.connection.predecessor, BaseFrame)
                push!(κ[body.id], joint.meta.id)
                joint = find_joint_by_successor(joint.connection.predecessor, joints)
            end
            #loop ended on BaseFrame, add its joint to κ            
            push!(κ[body.id], joint.meta.id)
        end

        #traverse body to tip, updating γ
        get_γ!(γ[body.id], body, joints)
    end

    return p, s, λ, κ, μ, γ
end
function get_next_body!(id, body, joints)
    outer_joints = find_joints_by_predecessor(body, joints)
    for joint in outer_joints
        id += 1
        joint.meta.id = id
        joint.connection.successor.id = id        
        id = get_next_body!(id, joint.connection.successor, joints)
    end
    return id
end

function get_γ!(γ, body, joints)
    outer_joints = find_joints_by_predecessor(body, joints)
    for joint in outer_joints
        push!(γ, joint.connection.successor.id)
        get_γ!(γ, joint.connection.successor, joints)
    end
end

nothing