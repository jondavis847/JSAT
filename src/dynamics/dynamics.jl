function dynamics!(sys)
    calculate_joint_forces!(sys)  # generalized actuator forces    
    calculate_external_forces!(sys) # body external forces
    articulated_body_algorithm!(sys)
    return nothing
end

calculate_joint_forces!(sys::MultibodySystem) = calculate_τ!.(sys.joints)

function calculate_external_forces!(body::Body)
    body.external_force = body.gravity

    for actuator in body.models.actuators
        body.external_force += actuator.current_force #converted to body frame in actuator get_actuator_force
    end

    #add environments
    return nothing
end

calculate_external_forces!(sys::MultibodySystem) = calculate_external_forces!.(sys.bodies);