using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames, OffsetArrays, PlotThemes
import Base: show
import Plots: plot, plot!

theme(:juno)

includet("utils//quaternion.jl")
includet("spatial.jl")

abstract type AbstractBody end

Base.@kwdef struct WorldFrame <: AbstractBody
    name::Symbol = :N
    id::Int64 = 0
end

mutable struct Body <: AbstractBody
    name::Symbol
    #parameters
    m::Float64 # mass
    I::SMatrix{3,3,Float64} # inertia tensor
    cm::SVector{3,Float64} # center of mass    
    id::Int64 # body number for table designation (applied automatically by sys)
    function Body(name, m, I, cm)
        x = new()
        x.name = Symbol(name)
        x.m = Float64(m)
        x.I = SMatrix{3,3,Float64}(I)
        x.cm = SVector{3,Float64}(cm)
        return x
    end
end
mcI(b::Body) = mcI(b.m,b.cm,b.I)

includet("joints.jl")
includet("utils//pathutils.jl")

struct MultibodySystem
    name::Union{String,Symbol}
    bodies::OffsetVector{AbstractBody,Vector{AbstractBody}}
    joints::Vector{AbstractJoint}
    p::Vector{Int16} # joint predecessor body array
    s::Vector{Int16} # joint successor body array
    λ::Vector{Int16} # body parent array
    κ::Vector{Vector{Int16}} # all joints between body i and base
    μ::OffsetVector{Vector{Int16}} # body children array
    γ::OffsetVector{Vector{Int16}} # all bodies from body i to tip (this is nu in Featherstone but nu just looks like v in Julia, so \gamma it is)    
    U::Vector{SArray{S,Float64,2} where S<:Tuple}
    D::Vector{SArray{S,Float64,2} where S<:Tuple}
    u::Vector{SArray{S,Float64,1} where S<:Tuple}
    c::Vector{SVector{6,Float64}}
    pᴬ::Vector{SVector{6,Float64}}
    ᵖXᵢᵐ::Vector{SMatrix{6,6,Float64,36}} #spatial motion transform from body i to predecessor of body i
    ᵖXᵢᶠ::Vector{SMatrix{6,6,Float64,36}} #spatial force transform from body i to predecessor of body i
    ⁱXₚᵐ::Vector{SMatrix{6,6,Float64,36}} #spatial motion transform from predecessor of body i to body i
    ⁱXₚᶠ::Vector{SMatrix{6,6,Float64,36}} #spatial force transform from predecessor of body i to body i       
    ᵒXᵢᵐ::OffsetVector{SMatrix{6,6,Float64,36}} #spatial motion transform from body i to worldframe
    ᵒXᵢᶠ::OffsetVector{SMatrix{6,6,Float64,36}} #spatial force transform from body i to worldframe
    ⁱXₒᵐ::OffsetVector{SMatrix{6,6,Float64,36}} #spatial motion transform from worldframe to body i
    ⁱXₒᶠ::OffsetVector{SMatrix{6,6,Float64,36}} #spatial force transform from worldframe to body i         
    q::MVector #generalized coordinates
    q̇::MVector #generalized speeds
    q̈::MVector #generalized accel
    x::MVector #ode state vector = [q;q̇]
    H::MMatrix #generalized mass matrix
    C::MVector #generalized bias force
    τ::MVector #generalized applied force (control force only for my application?)
    fˣ::MVector #external forces applied in the inertial frame 
    fᵇ::MVector #external forces applied in the body frame 
    f_gyro #gyroscopic force (in body frame?)
    Iᵇ::Vector{SMatrix{6,6,Float64}} #body frame inertia
    Iᴬ::Vector{SMatrix{6,6,Float64}} #articulated body inertia    
    r::MVector #body frame spatial position
    v::OffsetVector #body frame spatial velocity
    a::OffsetVector #body frame spatial acceleration    
end

#MultibodySystem constructor
function MultibodySystem(name, bodies, joints)
    # preallocate vectors/matrices at MultibodySystem definition, mutate in place during sim
    # map the bodies and joints 
    p, s, λ, κ, μ, γ = map_tree!(bodies, joints)

    # sort arrays so we can index directly by ID    
    permute!(bodies, sortperm(map(x -> x.id, bodies)))
    permute!(joints, sortperm(map(x -> x.meta.id, joints)))

    Iᵇ, Iᴬ = initialize_inertias(bodies, joints) # Iᵇ for now to not conflict with LinearAlgebra: I


    #grab q,q̇ initial conditions
    q, q̇, q̈, x = initialize_state_vectors(joints)

    nb = length(bodies)

    U = [SMatrix{6,joints[i].meta.nq̇,Float64}(zeros(6, joints[i].meta.nq̇)) for i in eachindex(joints)]
    D = [SMatrix{joints[i].meta.nq̇,joints[i].meta.nq̇,Float64}(zeros(joints[i].meta.nq̇, joints[i].meta.nq̇)) for i in eachindex(joints)]
    u = [SVector{joints[i].meta.nq̇,Float64}(zeros(joints[i].meta.nq̇)) for i in eachindex(joints)]
    c = fill(SVector{6,Float64}(zeros(6)), nb - 1)
    pᴬ = fill(SVector{6,Float64}(zeros(6)), nb - 1)


    # body frame spatial vectors 
    r = MVector{nb - 1,SVector{7,Float64}}(fill(SVector{7,Float64}(zeros(7)), nb - 1))
    v = OffsetVector(fill(SVector{6,Float64}(zeros(6)), nb), 0:nb-1)
    a = OffsetVector(fill(SVector{6,Float64}(zeros(6)), nb), 0:nb-1)    
    fˣ = MVector{nb - 1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)), nb - 1))
    fᵇ = MVector{nb - 1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)), nb - 1))
    f_gyro = MVector{nb - 1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)), nb - 1))

    # generalized vectors
    nq̇ = length(q̇)
    τ = MVector{nq̇}(zeros(nq̇))
    C = MVector{nq̇,Float64}(zeros(nq̇))
    H = MMatrix{nq̇,nq̇,Float64}(zeros(nq̇, nq̇))

    # spatial transformations from body i to predecessor body
    identity_X = SMatrix{6,6,Float64}(I(6))
    ᵖXᵢᵐ = fill(identity_X, nb - 1)
    ᵖXᵢᶠ = fill(identity_X, nb - 1)
    ⁱXₚᵐ = fill(identity_X, nb - 1)
    ⁱXₚᶠ = fill(identity_X, nb - 1)

    # spatial transformations from body i to worldframe
    # superscript little o here since can't start with 0 to represent 0th body   
    # could use n but it looks wierd since superscript is capital and subscript is lower case    
    ᵒXᵢᵐ = OffsetVector(fill(identity_X, nb), 0:nb-1)
    ᵒXᵢᶠ = OffsetVector(fill(identity_X, nb), 0:nb-1)
    ⁱXₒᵐ = OffsetVector(fill(identity_X, nb), 0:nb-1)
    ⁱXₒᶠ = OffsetVector(fill(identity_X, nb), 0:nb-1)

    MultibodySystem(name, bodies, joints, p, s, λ, κ, μ, γ, U, D, u, c, pᴬ, ᵖXᵢᵐ, ᵖXᵢᶠ, ⁱXₚᵐ, ⁱXₚᶠ, ᵒXᵢᵐ, ᵒXᵢᶠ, ⁱXₒᵐ, ⁱXₒᶠ, q, q̇, q̈, x, H, C, τ, fˣ, fᵇ, f_gyro, Iᵇ, Iᴬ, r, v, a)
end

function model!(sys)
    #sensors!(sys)
    #software!(sys)
    #actuators!(sys)
    environments!(sys)
    dynamics!(sys)
    nothing
end

# uses Featherstone notation
function dynamics!(sys)
    calculate_τ!(sys)  # generalized actuator forces    
    articulated_body_algorithm!(sys)
    #=
    calculate_J!(sys)  # body jacobians
    calculate_C!(sys)  # bias forces (gravity, environments, disturbances, gyroscopic, coriolis, etc.)
    calculate_H!(sys)  # mass matrix
    calculate_q̈!(sys)  # generalized acceleration
    =#
    nothing
end

calculate_X!(sys::MultibodySystem) = calculate_X!(sys.ᵖXᵢᵐ, sys.ᵖXᵢᶠ, sys.ⁱXₚᵐ, sys.ⁱXₚᶠ, sys.ᵒXᵢᵐ, sys.ᵒXᵢᶠ, sys.ⁱXₒᵐ, sys.ⁱXₒᶠ, sys.λ, sys.joints)
calculate_J!(sys::MultibodySystem) = calculate_J!(sys.J, sys.κ, sys.joints, sys.bodies)
#calculate_C!(sys::MultibodySystem) = inverse_dynamics!(sys.C,0*sys.q̈,sys.q̇,sys.fˣ,sys.Iᵇ,sys.ⁱXₚᵐ,sys.ⁱXₒᶠ,sys.ᵖXᵢᶠ,sys.v,sys.a,sys.f,sys.λ,sys.joints)
calculate_C!(sys::MultibodySystem) = inverse_dynamics!(sys.C, 0 * sys.q̈, sys.q̇, sys.fˣ, sys.Iʲ, sys.ⁱXₚᵐ, sys.ⁱXₒᶠ, sys.ᵖXᵢᶠ, sys.v, sys.a, sys.f, sys.λ, sys.joints)
calculate_H!(sys::MultibodySystem) = forward_dynamics!(sys.H, sys.Iᶜ, sys.Iᵇ, sys.ᵖXᵢᶠ, sys.ⁱXₚᵐ, sys.λ, sys.joints)


function calculate_τ!(τ)
    #until we figure out how to do software
    for i in eachindex(τ)
        τ[i] = 0
    end
    nothing
end
calculate_τ!(sys::MultibodySystem) = calculate_τ!(sys.τ)

function calculate_X!(ᵖXᵢᵐ, ᵖXᵢᶠ, ⁱXₚᵐ, ⁱXₚᶠ, ᵒXᵢᵐ, ᵒXᵢᶠ, ⁱXₒᵐ, ⁱXₒᶠ, λ, joints)
    for i in eachindex(joints)
        joint = joints[i]

        p_to_i = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
        i_to_p = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

        ᵖXᵢᵐ[i] = ℳ(i_to_p)
        ᵖXᵢᶠ[i] = ℱ(i_to_p)
        ⁱXₚᵐ[i] = ℳ(p_to_i)
        ⁱXₚᶠ[i] = ℱ(p_to_i)

        ᵒXᵢᵐ[i] = ᵒXᵢᵐ[λ[i]] * ᵖXᵢᵐ[i]
        ᵒXᵢᶠ[i] = ᵒXᵢᶠ[λ[i]] * ᵖXᵢᶠ[i]
        ⁱXₒᵐ[i] = ᵖXᵢᵐ[i] * ⁱXₒᵐ[λ[i]]
        ⁱXₒᶠ[i] = ᵖXᵢᶠ[i] * ⁱXₒᶠ[λ[i]]
    end
    nothing
end

function articulated_body_algorithm!(sys)
    @unpack v, a, c, D, U, u, ⁱXₚᵐ, Iᵇ, Iᴬ, pᴬ, ⁱXₒᶠ, ᵖXᵢᶠ, ⁱXₚᵐ,fᵇ, fˣ, f_gyro, τ, λ, q̈, q̇, joints = sys

    # pass 1
    for i in 1:length(sys.bodies)-1
        S = joints[i].S
        vj = S * q̇[joints[i].meta.q̇index]
        #vj = get_vj(joints[i])
        v[i] = ⁱXₚᵐ[i] * v[λ[i]] + vj
        #c[i] = v[i] ×ᵐ vj # + cj #commented until we need S∘        
        c[i] = v[i] ×ᶠ vj # + cj #commented until we need S∘        
        Iᴬ[i] = Iᵇ[i]
        #fᵇ[i] = ⁱXₒᶠ[i] * fˣ[i]
        fᵇ[i] = fˣ[i]
        f_gyro[i] = v[i] ×ᶠ (Iᵇ[i] * v[i])
        pᴬ[i] = f_gyro[i] - fᵇ[i]
    end

    # pass 2
    for i in reverse(1:length(sys.bodies)-1)
        S = joints[i].S
        U[i] = Iᴬ[i] * S
        D[i] = S' * U[i]
        u[i] = τ[joints[i].meta.q̇index] - S' * pᴬ[i]
        if λ[i] != 0
            Ia = Iᴬ[i] - U[i] * inv(D[i]) * U[i]'
            pa = pᴬ[i] + Ia * c[i] + U[i] * inv(D[i]) * u[i]
            Iᴬ[λ[i]] = Iᴬ[λ[i]] + ᵖXᵢᶠ[i] * Ia * ⁱXₚᵐ[i]
            pᴬ[λ[i]] = pᴬ[λ[i]] + ᵖXᵢᶠ[i] * pa
        end
    end

    # pass 3   

    for i in 1:length(sys.bodies)-1
        S = joints[i].S
        a′ = ⁱXₚᵐ[i] * a[λ[i]] + c[i]
        q̈[joints[i].meta.q̇index] = inv(D[i]) * (u[i] - U[i]' * a′)
        a[i] = a′ + S * q̈[joints[i].meta.q̇index]
    end
    nothing
end

function initialize_state_vectors(joints)
    q = []
    q̇ = []
    x = []
    for joint in joints
        this_q = get_q(joint)
        this_q̇ = get_q̇(joint)
        joint.meta.qindex = SVector{length(this_q),Int16}((length(q)+1):(length(q)+length(this_q)))
        joint.meta.q̇index = SVector{length(this_q̇),Int16}((length(q̇)+1):(length(q̇)+length(this_q̇)))
        joint.meta.xindex = SVector{length(this_q),Int16}((length(x)+1):(length(x)+length(this_q)))
        append!(x, this_q)
        joint.meta.ẋindex = SVector{length(this_q̇),Int16}((length(x)+1):(length(x)+length(this_q̇)))
        append!(x, this_q̇)
        append!(q, this_q)
        append!(q̇, this_q̇)
    end
    q = MVector{length(q),Float64}(q)
    q̇ = MVector{length(q̇),Float64}(q̇)
    q̈ = MVector{length(q̇),Float64}(zeros(length(q̇)))
    x = MVector{length(x),Float64}(x)
    return (q, q̇, q̈, x)
end

function initialize_inertias(bodies, joints)
    Iᵇ = Vector{SMatrix{6,6,Float64}}(undef, length(bodies) - 1)
    for i in 1:length(bodies)-1
        body = bodies[i]       
        Iᵇ[i] = mcI(body)
    end    
    Iᴬ = copy(Iᵇ)
    return Iᵇ, Iᴬ
end

function environments!(sys)
    gravity!(sys)
end


function gravity!(sys::MultibodySystem)
    for i in 1:length(sys.bodies)-1
        sys.fˣ[i] = sys.fˣ[i] .+ sys.Iᵇ[i] * sys.ⁱXₒᵐ[i] * SVector{6,Float64}(0, 0, 0, 0, -1, 0)        
    end
    nothing
end

function reset_f!(sys)
    for i in eachindex(sys.fˣ)
        sys.fˣ[i] *= 0
    end
    nothing
end

function update_model!(sys, x)
    reset_f!(sys)
    sys.x .= x #we probably dont need to do this other than FYI
    for joint in sys.joints        
        set_state!(joint,x[joint.meta.xindex],x[joint.meta.ẋindex])
        sys.q[joint.meta.qindex] = x[joint.meta.xindex]
        sys.q̇[joint.meta.q̇index] = x[joint.meta.ẋindex]
    end
    calculate_X!(sys)  # spatial transforms
    nothing
end

function ode_func!(dx, x, p, t)
    update_model!(p.sys, x)
    model!(p.sys)
    pack_dq_in_dx!(dx, p.sys)
    nothing
end

function save_dict!(config,name,type,func)
    d = Dict(
            "name" => name,
            "type" => type,
            "func" => func
        )
        push!(config, d)
        nothing
end

function configure_saving(sys::MultibodySystem)
    save_config = []
    for i in eachindex(sys.joints)
        #save joint state info
        states = fieldnames(typeof(sys.joints[i].state))
        for state in states
            d = Dict(
                "name" => "$(sys.joints[i].meta.name)_$(string(state))",
                "type" => typeof(getfield(sys.joints[i].state, state)),
                "func" => sys -> getfield(sys.joints[i].state, state)
            )
            push!(save_config, d)
        end
        #save joint frame
        d = Dict(
            "name" => "$(sys.joints[i].meta.name)_F",
            "type" => typeof(sys.joints[i].frame),
            "func" => sys -> sys.joints[i].frame
        )
        push!(save_config, d)

        # save q̈
        d = Dict(
            "name" => "$(sys.joints[i].meta.name)_q̈",
            "type" => typeof(sys.q̈[sys.joints[i].meta.q̇index]),
            "func" => sys -> sys.q̈[sys.joints[i].meta.q̇index]
        )
        push!(save_config, d)
        #save body r,v,a,I
    end
    for i in 1:length(sys.bodies)-1
        d = Dict(
            "name" => "$(sys.bodies[i].name)_r",
            "type" => typeof(sys.r[i]),
            "func" => sys -> sys.r[i]
        )
        push!(save_config, d)
        d = Dict(
            "name" => "$(sys.bodies[i].name)_v",
            "type" => typeof(sys.v[i]),
            "func" => sys -> sys.v[i]
        )
        push!(save_config, d)
        d = Dict(
            "name" => "$(sys.bodies[i].name)_a",
            "type" => typeof(sys.a[i]),
            "func" => sys -> sys.a[i]
        )
        push!(save_config, d)

        d = Dict(
            "name" => "$(sys.bodies[i].name)_Xf",
            "type" => typeof(sys.ᵒXᵢᶠ[i]),
            "func" => sys -> sys.ᵒXᵢᶠ[i]
        )

        push!(save_config, d)

        d = Dict(
            "name" => "$(sys.bodies[i].name)_Xm",
            "type" => typeof(sys.ᵒXᵢᵐ[i]),
            "func" => sys -> sys.ᵒXᵢᵐ[i]
        )

        push!(save_config, d)

        d = Dict(
            "name" => "$(sys.bodies[i].name)_c",
            "type" => typeof(sys.c[i]),
            "func" => sys -> sys.c[i]
        )
        push!(save_config, d)

        d = Dict(
            "name" => "$(sys.bodies[i].name)_Ia",
            "type" => typeof(sys.Iᴬ[i]),
            "func" => sys -> sys.Iᴬ[i]
        )
        push!(save_config, d)

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_fᵇ",
            typeof(sys.fᵇ[i]),
            sys -> sys.fᵇ[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_U",
            typeof(sys.U[i]),
            sys -> sys.U[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_D",
            typeof(sys.D[i]),
            sys -> sys.D[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_pᴬ",
            typeof(sys.pᴬ[i]),
            sys -> sys.pᴬ[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_u",
            typeof(sys.u[i]),
            sys -> sys.u[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_f_gyro",
            typeof(sys.f_gyro[i]),
            sys -> sys.f_gyro[i]
        )


    end

    save_values = SavedValues(Float64, Tuple{getindex.(save_config, "type")...})
    save_function(u, t, integrator) = Tuple([f(integrator.p.sys) for f in getindex.(integrator.p.save_config, "func")])
    save_cb = SavingCallback(save_function, save_values)
    return save_config, save_values, save_cb
end

function simulate(orig_sys::MultibodySystem, tspan)
    sys = deepcopy(orig_sys)# make a copy so we can rerun orig sys without mutating it during previous sim    
    save_config, save_values, save_cb = configure_saving(sys)
    p = (sys=sys, save_config=save_config)
    prob = ODEProblem(ode_func!, sys.x, tspan, p)
    solve(prob, Tsit5(), callback=save_cb, adaptive=false, dt=0.01)
    simout = wrap_save_values(save_values, save_config)
    return simout
end

function wrap_save_values(save_values, save_config)
    d = Dict()
    d[:t] = save_values.t
    for i in eachindex(save_values.saveval[1])
        name = Symbol(save_config[i]["name"])
        values = getindex.(save_values.saveval, i)
        d[name] = values
    end
    NamedTuple(d)
end

#needed each timestep to fill dx with ode state derivatives [dq,q̈] (dq not q̇ since q̇ can = ω and dq is quaternion deriv)
function pack_dq_in_dx!(dx, sys)
    for joint in sys.joints
        dx[joint.meta.xindex] = get_dq(joint)
        dx[joint.meta.ẋindex] = sys.q̈[joint.meta.q̇index]
    end
    nothing
end

function Plots.plot(t::Vector{T}, x::Vector{SVector{S,T}} ) where {S,T<:AbstractFloat}    
    p = plot(t,getindex.(x,1))
    if length(x[1]) > 1
        for i in 2:length(x[1])
            plot!(t,getindex.(x,i))
        end
    end
    return p
end
