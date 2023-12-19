mutable struct BodyModels{A<:Vector{AbstractActuator},S<:Vector{AbstractSensor},E<:Vector{AbstractEnvironment},G<:Vector{AbstractGravity}}
    actuators::A
    sensors::S
    environments::E
    gravity::G
    BodyModels() = new{Vector{AbstractActuator},Vector{AbstractSensor},Vector{AbstractEnvironment},Vector{AbstractGravity}}(AbstractActuator[], AbstractSensor[], AbstractEnvironment[], AbstractGravity[])
end

mutable struct BodyState
    q_base::SVector{4,Float64}
    r_base::SVector{3,Float64}
    v_base::SVector{3,Float64}
    ω_base::SVector{3,Float64}
    a_body::SVector{6,Float64}
    v_body::SVector{6,Float64} #spatial vector, ω_body = v[1:3]      
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

#temporary arrays for calculation
mutable struct BodyTmp
    c::SVector{6,Float64}
    pᴬ::SVector{6,Float64}
    U::SMatrix{SU1,SU2,Float64} where {SU1,SU2}
    D::SMatrix{SD1,SD2,Float64} where {SD1,SD2}
    u::SVector{Su,Float64} where {Su}
    BodyTmp() = new()
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
    inertia_articulated::SMatrix{6,6,Float64,36}
    external_force::SVector{6,Float64}
    internal_momentum::SVector{6,Float64} # used for like reaction wheels where we don't specify a new body for them
    gravity::SVector{6,Float64}
    models::M
    inner_joint::AbstractJoint #need to make these parametric - dont use abstracts as fields
    outer_joints::Vector{AbstractJoint} #need to make these parametric  - dont use abstracts as fields
    transforms::BodyTransforms
    state::BodyState
    tmp::BodyTmp

    function Body(name, m, cm, I)
        models = BodyModels()
        x = new{typeof(models)}()
        x.name = Symbol(name)
        x.m = Float64(m)
        x.cm = SVector{3,Float64}(cm)
        x.I = SMatrix{3,3,Float64,9}(I)
        x.inertia_body = mcI(m, cm, I)
        x.models = models
        x.transforms = BodyTransforms()
        x.outer_joints = AbstractJoint[]
        x.state = BodyState()
        x.tmp = BodyTmp()
        return x
    end
end

mcI(body::T) where {T<:Body} = mcI(body.m, body.cm, body.I)

function get_savedict(B::Body, i)
    save_config = Dict[]
    #save body state info        
    states = fieldnames(typeof(B.state))
    for state in states
        save_dict!(
            save_config,
            "$(B.name)_$(string(state))",
            typeof(getfield(B.state, state)),
            integrator -> getfield(integrator.p.sys.bodies[i].state, state)
        )
    end

    #get force info
    save_dict!(
        save_config,
        "$(B.name)_external_force",
        typeof(B.external_force[SVector{3,Int16}(4, 5, 6)]),
        integrator -> integrator.p.sys.bodies[i].external_force[SVector{3,Int16}(4, 5, 6)]
    )

    save_dict!(
        save_config,
        "$(B.name)_external_torque",
        typeof(B.external_force[SVector{3,Int16}(1, 2, 3)]),
        integrator -> integrator.p.sys.bodies[i].external_force[SVector{3,Int16}(1, 2, 3)]
    )

    #get force info
    save_dict!(
        save_config,
        "$(B.name)_external_force",
        typeof(B.external_force),
        integrator -> integrator.p.sys.bodies[i].external_force
    )
    return save_config
end
