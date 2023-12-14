include("SimpleThruster.jl")


#F is transform from actuator frame to body frame
function connect!(B::AbstractBody,A::AbstractActuator,F::Cartesian = Cartesian(I(3),[0,0,0]))
    A.body = B
    
    body_to_actuator_frame = F
    actuator_to_body_frame = inv(body_to_actuator_frame)
    body_to_joint_frame = B.inner_joint.connection.Fs
    actuator_to_joint_frame = body_to_joint_frame * actuator_to_body_frame
    
    A.transform = ℱ(actuator_to_joint_frame) #spatial force transform from actuator frame to joint frame
    push!(B.models.actuators,A)
    return nothing
end

function connect!(A::AbstractActuator,S::AbstractSoftware)
    A.command = S
    return nothing
end

