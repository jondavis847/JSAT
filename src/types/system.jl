struct MultibodySystem{J,A,S}
    name::Symbol
    base::BaseFrame
    bodies::Vector{Body}
    joints::J    
    actuators::A
    software::S

    function MultibodySystem(name, base, bodies, joints, actuators=AbstractActuator[], software=AbstractSoftware[])

        bodies = isa(bodies, Vector) ? bodies : [bodies]
        joints = isa(joints, Vector) ? joints : [joints]
        actuators = isa(actuators, Vector) ? actuators : [actuators]
        software = isa(software, Vector) ? software : [software]
    
        # id all the bodies and joints in the tree 
        map_tree!(base)
    
        # sort arrays so we can index directly by ID    
        permute!(bodies, sortperm(map(x -> x.id, bodies)))
        permute!(joints, sortperm(map(x -> x.meta.id, joints)))
    
        # initialize to joint specific values 
        initialize_inertias!.(bodies)
        initialize_actuators!.(actuators)        
    
        # copy base gravity into bodies TODO needs to be for only child bodies of this base
        if !isempty(base.gravity)
            for body in bodies
                append!(body.models.gravity, deepcopy(base.gravity))
            end
        end
    
        # make the software callbacks
        for i in eachindex(software)
            create_callbacks!(software[i], i)
        end
    
        sys = new{typeof(joints),typeof(actuators),typeof(software)}(name, base, bodies, joints, actuators, software)
    
        #initialize FixedJoints X so we don't have to calculate again
        calculate_transforms_FixedJoints!(sys)
        return sys
    end
end
