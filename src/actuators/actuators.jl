include("SimpleThruster.jl")


#F is transform from actuator frame to body frame
function connect!(B::AbstractBody,A::AbstractActuator,F::Cartesian = Cartesian(I(3),[0,0,0])) 
    A.transform = ℱ(F) #spatial force transform
    push!(B.actuators,A)
    return nothing
end

function connect!(A::AbstractActuator,S::AbstractSoftware)
    A.command = S
    return nothing
end

