function dynamics!(sys)
    #calculate_joint_forces!(sys)  # generalized actuator forces    do this in the ABA now to account for pA
    calculate_external_forces!(sys) # body external forces
    calculate_internal_momentums!(sys)
    articulated_body_algorithm!(sys)
    calculate_body_velocities!(sys) # after v_ijof is set in ABA, get body velocities for sensors
    return nothing
end

#calculate_joint_forces!(sys::MultibodySystem) = calculate_τ!.(sys.joints) do this in the ABA now to account for pA

function calculate_external_forces!(body::Body)
    #TODO: move grav to accel, transform to transforms
    body.state.external_force_ijof = ℱ(body.innerjoint.connection.Fs) * body.gravity

    for actuator in body.models.actuators
        body.state.external_force_ijof += actuator.current_ijof_force        
    end

    #add environments

    # convert ijof to body external force for output
    #TODO: move transform to transforms
    body.state.external_force = ℱ(inv(body.innerjoint.connection.Fs)) * body.state.external_force_ijof 

    return nothing
end
calculate_external_forces!(sys::MultibodySystem) = calculate_external_forces!.(sys.bodies)

function calculate_internal_momentums!(body::Body)
    # this function is only used for momentum that isnt stored as separate bodies, for example reaction wheels
    # separate bodies are calculated independently

    #reset each time step
    body.state.internal_momentum *= 0 #TODO: could be a 3 vector, but then transform needs to change a little. taking the hit for now

    for actuator in body.models.actuators
        body.state.internal_momentum += get_actuator_momentum(actuator)
    end
    return nothing
end
calculate_internal_momentums!(sys::MultibodySystem) = calculate_internal_momentums!.(sys.bodies)