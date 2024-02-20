function calculate_transforms!(body::Body)
    joint = body.inner_joint
    parent = joint.connection.predecessor

    if !isa(joint, FixedJoint)
        #parent_to_body = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp        
        #body_to_parent = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

        #Featherstone transforms only go from Fp to outer joint frame, not to Fs. See figure 4.7
        parent_to_body = joint.frame * joint.connection.Fp                
        before_body_to_parent = inv(joint.connection.Fp) * inv(joint.frame) 
        body_to_parent = before_body_to_parent * joint.connection.Fs

        body.transforms.body_to_parent_force = ℱ(before_body_to_parent)        
        body.transforms.before_body_to_parent_motion = ℳ(before_body_to_parent)
        body.transforms.body_to_parent_motion = ℳ(body_to_parent)
        # body.transforms.parent_to_body_force = ℱ(parent_to_body) #doesnt seem to get useds        
        body.transforms.parent_to_body_motion = ℳ(parent_to_body)
    end

    if parent isa BaseFrame
        #body.transforms.body_to_base_force = body.transforms.body_to_parent_force
        body.transforms.body_to_base_motion = body.transforms.body_to_parent_motion
        body.transforms.before_body_to_base_motion = body.transforms.before_body_to_parent_motion 
        #body.transforms.base_to_body_force = body.transforms.parent_to_body_force
        body.transforms.base_to_body_motion = body.transforms.parent_to_body_motion
    else    
        #body.transforms.body_to_base_force = parent.transforms.body_to_base_force * body.transforms.body_to_parent_force
        body.transforms.body_to_base_motion = parent.transforms.body_to_base_motion * body.transforms.body_to_parent_motion
        body.transforms.before_body_to_parent_motion = parent.transforms.before_body_to_base_motion * body.transforms.before_body_to_parent_motion
        #body.transforms.base_to_body_force = body.transforms.parent_to_body_force * parent.transforms.base_to_body_force
        body.transforms.base_to_body_motion = body.transforms.parent_to_body_motion * parent.transforms.base_to_body_motion
    end

    return nothing
end
calculate_transforms!(sys::MultibodySystem) = calculate_transforms!.(sys.bodies)


# only have to do this once at initialization
function calculate_transforms_FixedJoints!(body)
        joint = body.inner_joint    
        if isa(joint, FixedJoint)            
            #parent_to_body = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
            #body_to_parent = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

            #Featherstone transforms only go from Fp to outer joint frame, not to Fs. See figure 4.7
            parent_to_body = joint.frame * joint.connection.Fp        
            before_body_to_parent = inv(joint.connection.Fp) * inv(joint.frame)
            body_to_parent = before_body_to_parent * joint.connection.Fs

            body.transforms.body_to_parent_motion = ℳ(body_to_parent)
            body.transforms.before_body_to_parent_motion = ℳ(before_body_to_parent)
            body.transforms.body_to_parent_force = ℱ(before_body_to_parent)
            body.transforms.parent_to_body_motion = ℳ(parent_to_body)
            #body.transforms.parent_to_body_force = ℱ(parent_to_body)            
        end    
    nothing
end
calculate_transforms_FixedJoints!(sys::MultibodySystem) = calculate_transforms_FixedJoints!.(sys.bodies)

function calculate_body_positions!(body::AbstractBody)
    if !isa(body, BaseFrame)
        joint = body.inner_joint

        r_Fs_to_Bi_in_Fs_frame = joint.connection.Fs.Φ.value * (-joint.connection.Fs.r)
        r_Fp_to_Bi_in_Fp_frame = joint.frame.Φ.value' * r_Fs_to_Bi_in_Fs_frame + joint.frame.r
        r_Bλ_to_Bi_in_Bλ_frame = joint.connection.Fp.Φ.value' * r_Fp_to_Bi_in_Fp_frame + joint.connection.Fp.r

        if !isa(joint.connection.predecessor, BaseFrame)

            predecessor = joint.connection.predecessor

            r_Bλ_to_Bi_in_base_frame = predecessor.transforms.body_to_base_motion[i3, i3] * r_Bλ_to_Bi_in_Bλ_frame
            r_base_to_Fp_in_base_frame = r_Bλ_to_Bi_in_base_frame + predecessor.state.r_base
            body.state.q_base = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value * qtoa(predecessor.state.q_base))
        else
            r_base_to_Fp_in_base_frame = r_Bλ_to_Bi_in_Bλ_frame
            body.state.q_base = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value)
        end
        body.state.r_base = r_base_to_Fp_in_base_frame
    end
    return nothing
end

# this assumes that vector of bodies are ordered by id. that must be true or this wont work, since the calc is recursive
calculate_body_positions!(sys::MultibodySystem) = calculate_body_positions!.(sys.bodies)