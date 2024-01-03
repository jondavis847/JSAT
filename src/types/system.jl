struct MultibodySystem{B,J,A,SW,S}
    name::Symbol
    base::BaseFrame
    bodies::B
    joints::J    
    actuators::A
    software::SW
    sensors::S

    function MultibodySystem(name, base, bodies, joints; actuators=AbstractActuator[], software=AbstractSoftware[], sensors=AbstractSensor[])
        #convert to array if not already
        bodies = isa(bodies, Vector) ? bodies : [bodies]
        joints = isa(joints, Vector) ? joints : [joints]
        actuators = isa(actuators, Vector) ? actuators : [actuators]
        software = isa(software, Vector) ? software : [software]
        sensors = isa(sensors, Vector) ? sensors : [sensors]
    
        # id all the bodies and joints in the tree 
        map_tree!(base)
    
        # sort arrays so we can index directly by ID    
        permute!(bodies, sortperm(map(x -> x.id, bodies)))
        permute!(joints, sortperm(map(x -> x.meta.id, joints)))

         #convert to tuples after sorting for concreteness
         bodies = Tuple(bodies)
         joints = Tuple(joints)
         actuators = Tuple(actuators)
         software = Tuple(software)
         sensors = Tuple(sensors)    
    
        # copy base gravity into bodies TODO needs to be for only child bodies of this base
        if !isempty(base.gravity)
            for body in bodies
                append!(body.models.gravity, deepcopy(base.gravity))
            end
        end

        sys = new{typeof(bodies),typeof(joints),typeof(actuators),typeof(software),typeof(sensors)}(name, base, bodies, joints, actuators, software, sensors)
    
        #initialize FixedJoints transforms so we don't have to calculate again
        calculate_transforms_FixedJoints!(sys)
        return sys
    end
end
