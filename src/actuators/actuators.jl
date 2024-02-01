mutable struct ActuatorState
    q_base::SVector{4,Float64}
    r_base::SVector{3,Float64}
    ActuatorState() = new()
end

function actuators!(sys)
    get_actuator_force!.(sys.actuators)
    return nothing
end

#F is transform from actuator frame to body frame
function connect!(B::AbstractBody,A::AbstractActuator,F::Cartesian = Cartesian(I(3),[0,0,0]))
    A.body = B
    A.frame = F
    body_to_actuator_frame = F
    actuator_to_body_frame = inv(body_to_actuator_frame)    
    A.body_transform = ℱ(actuator_to_body_frame) #spatial force transform from actuator frame to body frame

    if isdefined(B, :inner_joint)
        body_to_joint_frame = B.inner_joint.connection.Fs
        actuator_to_joint_frame = body_to_joint_frame * actuator_to_body_frame
        A.joint_transform = ℱ(actuator_to_joint_frame) #spatial force transform from actuator frame to joint frame
    end
    
    push!(B.models.actuators,A)
    return nothing
end

function connect!(SW::AbstractSoftware,A::AbstractActuator) 
    push!(SW.connections.actuators,A)    
    return nothing 
end

#just used for animation, TODO: make this optional based on if animations required
function calculate_actuator_positions!(A::AbstractActuator)
    dcm_body_to_base = A.body.transforms.body_to_base_motion[i3,i3]    
    dcm_actuator_to_body = A.frame.Φ.value 
    
    r_actuator = A.body.state.r_base + dcm_body_to_base * (-A.frame.r) #minus for body to actuator
    q_actuator =  atoq(transpose(dcm_body_to_base * dcm_actuator_to_body))
    
    A.state.r_base = r_actuator    
    A.state.q_base = q_actuator 

    return nothing    
end
calculate_actuator_positions!(sys::MultibodySystem) = calculate_actuator_positions!.(sys.actuators)

include("SimpleThruster.jl")
include("SimpleReactionWheel.jl")