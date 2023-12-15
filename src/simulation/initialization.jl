function initialize_state_vectors(sys)    
    x = []
    for joint in sys.joints
        if !isa(joint, FixedJoint)
            this_q = get_q(joint)
            this_q̇ = get_q̇(joint)            
            joint.meta.xindex = SVector{length(this_q),Int16}((length(x)+1):(length(x)+length(this_q)))
            append!(x, this_q)
            joint.meta.ẋindex = SVector{length(this_q̇),Int16}((length(x)+1):(length(x)+length(this_q̇)))
            append!(x, this_q̇)            
        end
    end

    for software in sys.software
        append!(x, software.initial_value)
        software.u_index = length(x)
    end

    for actuator in sys.actuators
        actuator.u_index = SVector{6,Int16}(length(x)+1:length(x)+6) # body frame spatial force
        append!(x, SVector{6,Float64}(zeros(6))) # TODO: may need to make this the initial value of actuator instead of assuming its 0        
    end
    
    x = MVector{length(x),Float64}(x)
    return x
end

function initialize_inertias!(body::Body)
    # make the assumption for revolute or spherical joints that the body frame is coincident with the joint Fs frame
    # shift mass properties to the joint frame
    if typeof(body.inner_joint) in [Revolute, Spherical]
        Fs = body.inner_joint.connection.Fs # Fs is joint frame expressed in body frame, or transform from body to joint
        ᵇXⱼᵐ = ℳ(inv(Fs)) # need motion transformation from joint to body
        ʲXᵦᶠ = ℱ(Fs) # need force transformation from body to joint        
        body.inertia_joint = ʲXᵦᶠ * body.inertia_body * ᵇXⱼᵐ # Featherstone equations 2.66 for transform of spatial inertia                
    end

    # make the assumption for FloatingJoints that the body frame is coincident with the com
    # shift mass properties to the com
    if typeof(body.inner_joint) == FloatingJoint
        body.inertia_joint = mcI(body.m, SVector{3,Float64}(zeros(3)), body.I)
    end
    return nothing
end

function initialize_actuators!(actuator::AbstractActuator)
    body = actuator.body        
    body_to_joint_frame = body.inner_joint.connection.Fs
    actuator_to_joint_frame = ℱ(body_to_joint_frame) * actuator.body_transform
    actuator.joint_transform = actuator_to_joint_frame #spatial force transform from actuator frame to joint frame
    return nothing    
end