include("SimpleThruster.jl")
include("SimpleReactionWheel.jl")
function actuators!(sys)
    get_actuator_force!.(sys.actuators)
    return nothing
end

#F is transform from actuator frame to body frame
function connect!(B::AbstractBody,A::AbstractActuator,F::Cartesian = Cartesian(I(3),[0,0,0]))
    A.body = B
    
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

function connect!(A::AbstractActuator,S::AbstractSoftware)
    A.command = S
    return nothing
end

