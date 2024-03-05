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
    a_ijof::SVector{6,Float64}
    v_body::SVector{6,Float64} #spatial vector, ω_body = v[1:3]      
    v_ijof::SVector{6,Float64} #spatial vector, ω_body = v[1:3]      
    gyroscopic_force_ijof::SVector{6,Float64}
    gyroscopic_force::SVector{6,Float64}
    external_force::SVector{6,Float64} # defined in body frame
    external_force_ijof::SVector{6,Float64}
    internal_momentum::SVector{6,Float64} # used for like reaction wheels where we don't specify a new body for them
    internal_momentum_ijof::SVector{6,Float64} 
    BodyState() = new()
end

mutable struct BodyInertia
    body::SMatrix{6,6,Float64,36}
    ijof::SMatrix{6,6,Float64,36}
    articulated::SMatrix{6,6,Float64,36}
    BodyInertia() = new()
end

mutable struct BodyTransforms
    base_to_ijof_force::SMatrix{6,6,Float64,36}    
    base_to_ijof_motion::SMatrix{6,6,Float64,36}    
    ijof_to_base_force::SMatrix{6,6,Float64,36}    
    ijof_to_base_motion::SMatrix{6,6,Float64,36}    
    base_to_body_force::SMatrix{6,6,Float64,36}        
    base_to_body_motion::SMatrix{6,6,Float64,36}    
    body_to_base_force::SMatrix{6,6,Float64,36}    
    body_to_base_motion::SMatrix{6,6,Float64,36}
    
    parent_to_ijof_force::SMatrix{6,6,Float64,36}    
    parent_to_ijof_motion::SMatrix{6,6,Float64,36}    
    ijof_to_parent_force::SMatrix{6,6,Float64,36}    
    ijof_to_parent_motion::SMatrix{6,6,Float64,36}    
    parent_to_body_force::SMatrix{6,6,Float64,36}    
    parent_to_body_motion::SMatrix{6,6,Float64,36}    
    body_to_parent_force::SMatrix{6,6,Float64,36}
    body_to_parent_motion::SMatrix{6,6,Float64,36}    
    BodyTransforms() = new()
end
mutable struct Body{M<:BodyModels} <: AbstractBody
    name::Symbol
    #parameters
    m::M where {M<:SimVal} # mass    
    cm::CM where {CM<:Vector{SimVal}} # center of mass in body frame 
    I::I where {I<:Matrix{SimVal}} # inertia tensor in body frame defined about the cm
    id::Int64 # body number for table designation (applied automatically by sys)  
    inertia::BodyInertia
    gravity::SVector{6,Float64}
    gravity_ijof::SVector{6,Float64}
    models::M
    innerjoint::AbstractJoint #need to make these parametric - dont use abstracts as fields
    outerjoints::Vector{AbstractJoint} #need to make these parametric  - dont use abstracts as fields
    transforms::BodyTransforms
    state::BodyState    

    function Body(name, m, cm, ixx, iyy, izz, ixy, ixz, iyz)
        ixx = (ixx isa SimValue) ? ixx : SimValue(ixx)
        iyy = (iyy isa SimValue) ? iyy : SimValue(iyy)
        izz = (izz isa SimValue) ? izz : SimValue(izz)
        ixy = (ixy isa SimValue) ? ixy : SimValue(ixy)
        ixz = (ixz isa SimValue) ? ixz : SimValue(ixz)
        iyz = (iyz isa SimValue) ? iyz : SimValue(iyz)

        inertia = [
            ixx ixy ixz
            ixy iyy iyz
            ixz iyz izz
        ]
        return Body(name,m,cm,inertia)
    end

    function Body(name, m, cm, inertia)
        models = BodyModels()

        #convert to simvalue to support monte carlo
        m = (m isa SimVal) ? m : SimVal(m)
        if !isa(cm,Vector{SimVal})
            cm = map(x-> (!isa(x,SimVal)) ? SimVal(x) : x , cm)
        end

        if !isa(inertia,Matrix{SimVal})
            inertia = map(x-> (!isa(x,SimVal)) ? SimVal(x) : x , inertia)
        end

        x = new{typeof(models)}()
        x.name = Symbol(name)
        x.m = m
        x.cm = cm
        x.I = inertia
        x.inertia = BodyInertia()
        x.models = models
        x.transforms = BodyTransforms()
        x.outerjoints = AbstractJoint[]
        x.state = BodyState()        
        return x
    end
end

mcI(body::T) where {T<:Body} = mcI(body.m.value, getfield.(body.cm,:value), getfield.(body.I,:value))

function disperse!(body::Body)
    (isdefined(body.m, :dispersion)) ? body.m.value = rand(body.m.dispersion) : nothing
    map(x-> (isdefined(x, :dispersion)) ? x.value = rand(x.dispersion) : nothing, body.cm)
    map(x-> (isdefined(x, :dispersion)) ? x.value = rand(x.dispersion) : nothing, body.I)    
    return nothing
end

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
    #=
    tmps = fieldnames(typeof(B.tmp))
    for tmp in tmps
        save_dict!(
            save_config,
            "$(B.name)_$(string(tmp))",
            typeof(getfield(B.tmp, tmp)),
            integrator -> getfield(integrator.p.sys.bodies[i].tmp, tmp)
        )
    end    
    =#
    return save_config
end
