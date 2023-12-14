mutable struct BodyModels{A<:Vector{AbstractActuator},E<:Vector{AbstractEnvironment},G<:Vector{AbstractGravity}}
    actuators::A
    environments::E
    gravity::G
    BodyModels() = new{Vector{AbstractActuator},Vector{AbstractEnvironment},Vector{AbstractGravity}}(AbstractActuator[],AbstractEnvironment[],AbstractGravity[])
end

mutable struct BodyState
    q_base::SVector{4,Float64}
    ω_base::SVector{3,Float64}
    r_base::SVector{3,Float64}
    v_base::SVector{3,Float64}    
    BodyState() = new()
end

mutable struct BodyTransforms
    base_to_body_force::SMatrix{6,6,Float64,36}
    base_to_body_motion::SMatrix{6,6,Float64,36}
    body_to_base_force::SMatrix{6,6,Float64,36}
    body_to_base_motion::SMatrix{6,6,Float64,36}
    parent_to_body_force::SMatrix{6,6,Float64,36}
    parent_to_body_motion::SMatrix{6,6,Float64,36}
    body_to_parent_force::SMatrix{6,6,Float64,36}
    body_to_parent_motion::SMatrix{6,6,Float64,36}
    BodyTransforms() = new()
end

mutable struct Body{M<:BodyModels} <: AbstractBody
    name::Symbol
    #parameters
    m::Float64 # mass
    I::SMatrix{3,3,Float64,9} # inertia tensor
    cm::SVector{3,Float64} # center of mass    
    id::Int64 # body number for table designation (applied automatically by sys)  
    inertia_body::SMatrix{6,6,Float64,36}      
    inertia_joint::SMatrix{6,6,Float64,36}    
    external_force::SVector{6,Float64}
    gravity::SVector{6,Float64}
    models::M
    inner_joint::AbstractJoint #need to make these parametric - dont use abstracts as fields
    outer_joints::Vector{AbstractJoint} #need to make these parametric  - dont use abstracts as fields
    transforms::BodyTransforms
    state::BodyState
    
    function Body(name, m, cm, I)
        models = BodyModels()
        x = new{typeof(models)}()
        x.name = Symbol(name)
        x.m = Float64(m)
        x.cm = SVector{3,Float64}(cm)
        x.I = SMatrix{3,3,Float64,9}(I)                        
        x.inertia_body = mcI(m,cm,I)
        x.inertia_joint = x.inertia_body #remove this and do at initialization
        x.models = models
        x.transforms = BodyTransforms()
        x.outer_joints = AbstractJoint[]   
        x.state = BodyState()     
        return x
    end
end

function calculate_gravity!(body::AbstractBody) 
    #reset each step
    body.gravity *= 0

    for gravity in body.models.gravity        
        body.gravity += body.inertia_joint * body.transforms.base_to_body_motion * calculate_gravity(gravity)
    end
    return nothing
end

mcI(body::T) where T<:Body = mcI(body.m,body.cm,body.I)