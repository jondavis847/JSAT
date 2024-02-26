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

    #TODO: Maybe dont need this
    for software in sys.software
        append!(x, get_initial_value(software))
        set_xindex!(software,length(x))
    end

    for actuator in sys.actuators
        this_q = get_q(actuator)
        actuator.xindex = SVector{length(this_q),Int16}(length(x)+1:(length(x)+length(this_q))) # body frame spatial force
        append!(x, this_q) 
    end
    
    x = MVector{length(x),Float64}(x)
    return x
end

function initialize_inertias!(body::Body)
    body.inertia.body = mcI(body)
    body.inertia.ijof = body.inertia.body
    # make the assumption for revolute or spherical joints that the body frame is coincident with the joint Fs frame
    # shift mass properties to the joint frame
    if typeof(body.innerjoint) in [Revolute, Spherical]
        Fs = body.innerjoint.connection.Fs # Fs is joint frame expressed in body frame, or transform from body to joint
        ᵇXⱼᵐ = ℳ(inv(Fs)) # need motion transformation from joint to body
        ʲXᵦᶠ = ℱ(Fs) # need force transformation from body to joint        
        body.inertia.ijof = ʲXᵦᶠ * body.inertia.body * ᵇXⱼᵐ # Featherstone equations 2.66 for transform of spatial inertia                
    end

    # make the assumption for FloatingJoints that the body frame is coincident with the com
    # shift mass properties to the com
    if typeof(body.innerjoint) == FloatingJoint
        body.inertia.ijof = mcI(body.m.value, SVector{3,Float64}(zeros(3)), getfield.(body.I,:value))
    end
    return nothing
end

function initialize_actuators!(actuator::AbstractActuator)
    body = actuator.body        
    body_to_ijof = body.innerjoint.connection.Fs
    actuator_to_ijof = ℱ(body_to_ijof) * actuator.body_transform
    actuator.ijof_transform = actuator_to_ijof #spatial force transform from actuator frame to joint frame
    return nothing    
end