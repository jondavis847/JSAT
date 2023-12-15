using ConcreteStructs
using CSV
using DataFrames
using Dates
using DifferentialEquations
using LinearAlgebra
using OffsetArrays
using Plots
using PlotThemes
using StaticArrays
using UnPack

import Base: show
import Plots: plot, plot!

theme(:juno)

includet(joinpath("math", "quaternion.jl"))
includet(joinpath("math", "spatial.jl"))


includet("abstract.jl")

includet(joinpath("bodies", "bodies.jl"))
includet(joinpath("joints", "joints.jl"))
includet(joinpath("software", "software.jl"))
includet(joinpath("actuators", "actuators.jl"))
includet(joinpath("gravity", "gravity.jl"))
includet(joinpath("bases", "base.jl"))
includet(joinpath("utils", "pathutils.jl"))


@concrete struct MultibodySystem
    name
    base
    bodies
    joints
    software
    actuators
    p       # joint predecessor body array
    s       # joint successor body array
    λ       # body parent array
    κ       # all joints between body i and base
    μ       # body children array
    γ       # all bodies from body i to tip (this is nu in Featherstone but nu just looks like v in Julia, so \gamma it is)        
    q
    q̇
    q̈
    x
    τ
end

#MultibodySystem constructor
function MultibodySystem(name, base, bodies, joints, actuators=AbstractActuator[], software=AbstractSoftware[])

    bodies = isa(bodies, Vector) ? bodies : [bodies]
    joints = isa(joints, Vector) ? joints : [joints]
    actuators = isa(actuators, Vector) ? actuators : [actuators]
    software = isa(software, Vector) ? software : [software]

    # convert bodies to an offsetarray with the base as index 0
    bodies = OffsetArray([base; bodies], 0:length(bodies))

    # preallocate vectors/matrices at MultibodySystem definition, mutate in place during sim
    # map the bodies and joints 
    p, s, λ, κ, μ, γ = map_tree!(bodies, joints)

    # sort arrays so we can index directly by ID    
    permute!(bodies, sortperm(map(x -> x.id, bodies)))
    permute!(joints, sortperm(map(x -> x.meta.id, joints)))

    # initialize inertias to joint specific values
    initialize_inertias!(bodies)

    # copy base gravity into bodies TODO needs to be for only child bodies of this base
    if !isempty(base.gravity)
        for body in bodies
            if !isa(body, BaseFrame)
                append!(body.models.gravity, deepcopy(base.gravity))
            end
        end
    end

    #grab q,q̇ initial conditions
    q, q̇, q̈, x = initialize_state_vectors(joints, software, actuators)

    nb = length(bodies)

    # generalized vectors
    nq̇ = length(q̇)
    τ = MVector{nq̇}(zeros(nq̇))

    # make the software callbacks
    for i in eachindex(software)
        create_callbacks!(software[i], i)
    end

    sys = MultibodySystem(name, base, bodies, joints, software, actuators, p, s, λ, κ, μ, γ, q, q̇, q̈, x, τ)

    #initialize FixedJoints X so we don't have to calculate again
    calculate_transforms_FixedJoints!(sys)
    return sys
end

function model!(sys)
    #sensors!(sys)
    software!(sys)
    actuators!(sys)
    environments!(sys)
    dynamics!(sys)
    return nothing
end

function software!(sys)
    run_software!.(sys.software, Ref(sys))
    return nothing
end

function actuators!(sys)
    get_actuator_force!.(sys.actuators)
    return nothing
end


# uses Featherstone notation
function dynamics!(sys)
    calculate_τ!(sys)  # generalized actuator forces    
    calculate_fˣ!(sys) # body external forces
    articulated_body_algorithm!(sys)
    return nothing
end

function calculate_τ!(τ)
    #until we figure out how to do software
    for i in eachindex(τ)
        τ[i] = 0.0
    end
    nothing
end
calculate_τ!(sys::MultibodySystem) = calculate_τ!(sys.τ)

function calculate_fˣ!(body::Body)
    body.external_force = body.gravity

    for actuator in body.models.actuators
        body.external_force += actuator.current_force #converted to body frame in actuator get_actuator_force
    end

    #add environments
    return nothing
end

calculate_fˣ!(sys::MultibodySystem) = calculate_fˣ!.(sys.bodies);
return nothing;

function calculate_transforms!(body)
    joint = body.inner_joint
    parent = joint.connection.predecessor

    if !isa(joint, FixedJoint)
        parent_to_body = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
        body_to_parent = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

        body.transforms.body_to_parent_force = ℱ(body_to_parent)
        body.transforms.body_to_parent_motion = ℳ(body_to_parent)
        body.transforms.parent_to_body_force = ℳ(parent_to_body)
        body.transforms.parent_to_body_motion = ℱ(parent_to_body)
    end

    body.transforms.body_to_base_force = parent.transforms.body_to_base_force * body.transforms.body_to_parent_force
    body.transforms.body_to_base_motion = parent.transforms.body_to_base_motion * body.transforms.body_to_parent_motion
    body.transforms.base_to_body_force = body.transforms.parent_to_body_force * parent.transforms.base_to_body_force
    body.transforms.base_to_body_motion = body.transforms.parent_to_body_force * parent.transforms.base_to_body_force
    nothing
end

calculate_transforms!(sys::MultibodySystem) = calculate_transforms!.(sys.bodies)


# only have to do this once at initialization
function calculate_transforms_FixedJoints!(body)
        joint = body.inner_joint    
        if isa(joint, FixedJoint)            
            parent_to_body = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
            body_to_parent = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

            body.transforms.body_to_parent_motion = ℳ(body_to_parent)
            body.transforms.body_to_parent_force = ℱ(body_to_parent)
            body.transforms.parent_to_body_motion = ℳ(parent_to_body)
            body.transforms.parent_to_body_force = ℱ(parent_to_body)            
        end    
    nothing
end
calculate_transforms_FixedJoints!(sys::MultibodySystem) = calculate_X_FixedJoints!.(sys.bodies)

function calculate_r!(body::AbstractBody)
    if !isa(body, BaseFrame)
        joint = body.inner_joint

        r_Fs_to_Bi_in_Fs_frame = (joint.connection.Fs.Φ.value)' * (-joint.connection.Fs.r)
        r_Fs_to_Bi_in_Fs_frame = (joint.connection.Fs.Φ.value)' * (-joint.connection.Fs.r)
        r_Fp_to_Fs_in_Fp_frame = joint.frame.Φ.value' * r_Fs_to_Bi_in_Fs_frame + joint.frame.r
        r_Bλ_to_Fp_in_Bλ_frame = joint.connection.Fp.Φ.value' * r_Fp_to_Fs_in_Fp_frame + joint.connection.Fp.r

        if !isa(joint.connection.predecessor, BaseFrame)

            predecessor = joint.connection.predecessor

            r_Bλ_to_Fp_in_base_frame = predecessor.transforms.body_to_base_motion[i3, i3] * r_Bλ_to_Fp_in_Bλ_frame
            r_base_to_Fp_in_base_frame = r_Bλ_to_Fp_in_base_frame + predecessor.state.r_base
            body.state.q_base = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value * qtoa(predecessor.state.q_base))
        else
            r_base_to_Fp_in_base_frame = r_Bλ_to_Fp_in_Bλ_frame
            body.state.q_base = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value)
        end
        body.state.r_base = r_base_to_Fp_in_base_frame
    end
    return nothing
end

# this assumes that vector of bodies are ordered by id. that must be true or this wont work, since the calc is recursive
calculate_r!(sys::MultibodySystem) = calculate_r!.(sys.bodies)

function articulated_body_algorithm!(sys)
    first_pass!.(sys.bodies, Ref(sys))
    second_pass!.(Iterators.reverse(sys.bodies), Ref(sys))
    third_pass!.(sys.bodies, Ref(sys))
    return nothing
end

first_pass!(body::BaseFrame, sys) = nothing
function first_pass!(body, sys)
    joint = body.inner_joint

    if isa(joint, FixedJoint)
        body.state.v = body.transforms.parent_to_body_motion * joint.connection.predecessor.state.v
        body.tmp.c = @SVector zeros(6)
    else
        vj = joint.S * sys.q̇[joint.meta.q̇index]
        body.state.v = body.transforms.parent_to_body_motion * joint.connection.predecessor.state.v + vj
        body.tmp.c = body.state.v ×ᵐ vj # + cj #commented until we need S∘                
    end

    body.inertia_articulated = body.inertia_joint
    body.tmp.pᴬ = body.state.v ×ᶠ (body.inertia_joint * body.state.v) - body.external_force
    return nothing
end

second_pass!(body::BaseFrame, sys) = nothing
function second_pass!(body, sys)
    joint = body.inner_joint
    parent = joint.connection.predecessor

    if isa(joint, FixedJoint)
        if !isa(parent, BaseFrame)
            parent.inertia_articulated = parent.inertia_articulated +
                                         body.transforms.body_to_parent_force * body.inertia_articulated * body.transforms.parent_to_body_motion
            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.body_to_parent_force * body.tmp.pᴬ
        end
    else
        S = joint.S
        body.tmp.U = body.inertia_articulated * S
        body.tmp.D = S' * body.tmp.U
        body.tmp.u = sys.τ[joint.meta.q̇index] - S' * body.tmp.pᴬ
        if !isa(parent, BaseFrame)
            Ia = body.inertia_articulated - body.tmp.U * inv(body.tmp.D) * body.tmp.U'
            pa = body.tmp.pᴬ + Ia * body.tmp.c + body.tmp.U * inv(body.tmp.D) * body.tmp.u
            parent.inertia_articulated = parent.inertia_articulated + body.transforms.body_to_parent_force * Ia * body.transforms.parent_to_body_motion
            parent.tmp.pᴬ = parent.tmp.pᴬ + body.transforms.body_to_parent_force * pa
        end
    end
    return nothing
end

third_pass!(body::BaseFrame, sys) = nothing
function third_pass!(body, sys)
    joint = body.inner_joint
    parent = joint.connection.predecessor
    if isa(joint, FixedJoint)
        body.state.a = body.transforms.parent_to_body_motion * parent.state.a
    else
        S = joint.S
        a′ = body.transforms.parent_to_body_motion * parent.state.a + body.tmp.c
        sys.q̈[joint.meta.q̇index] = body.tmp.D \ (body.tmp.u - body.tmp.U' * a′)
        body.state.a = a′ + S * sys.q̈[joint.meta.q̇index]
    end
    return nothing
end

function initialize_state_vectors(joints, softwares, actuators)
    q = []
    q̇ = []
    x = []
    for joint in joints
        if !isa(joint, FixedJoint)
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
    end

    for software in softwares
        append!(x, software.initial_value)
        software.u_index = length(x)
    end

    for actuator in actuators
        actuator.u_index = SVector{6,Int16}(length(x)+1:length(x)+6) # body frame spatial force
        append!(x, SVector{6,Float64}(zeros(6))) # TODO: may need to make this the initial value of actuator instead of assuming its 0        
    end

    q = MVector{length(q),Float64}(q)
    q̇ = MVector{length(q̇),Float64}(q̇)
    q̈ = MVector{length(q̇),Float64}(zeros(length(q̇)))
    x = MVector{length(x),Float64}(x)
    return (q, q̇, q̈, x)
end

function initialize_inertias!(body)
    # make the assumption for revolute or spherical joints that the body frame is coincident with the joint Fs frame
    # shift mass properties to the joint frame
    if typeof(body.inner_joints) in [Revolute, Spherical]
        Fs = body.inner_joint.connection.Fs # Fs is joint frame expressed in body frame, or transform from body to joint
        ᵇXⱼᵐ = ℳ(inv(Fs)) # need motion transformation from joint to body
        ʲXᵦᶠ = ℱ(Fs) # need force transformation from body to joint
        body.inertia_joint = ʲXᵦᶠ * body.inertia_body * ᵇXⱼᵐ # Featherstone equations 2.66 for transform of spatial inertia                
    end

    # make the assumption for FloatingJoints that the body frame is coincident with the com
    # shift mass properties to the com
    if typeof(body.inner_joint) == FloatingJoint
        body.inertia_joint = mcI(body.m, SVector{3,Float64}(zeros(3)), body.I)
    end
    return nothing
end

function environments!(sys)
    gravity!(sys)
    return nothing
end


function gravity!(sys::MultibodySystem)
    calculate_gravity!.(sys.bodies)
    return nothing
end

function reset_forces!(sys)
    for i in eachindex(sys.fˣ)
        sys.fˣ[i] *= 0
    end
    nothing
end

function update_model!(sys, x)
    reset_forces!(sys)
    sys.x .= x #we probably dont need to do this other than FYI
    for joint in sys.joints
        if !isa(joint, FixedJoint)
            set_state!(joint, x[joint.meta.xindex], x[joint.meta.ẋindex])
            sys.q[joint.meta.qindex] = x[joint.meta.xindex]
            sys.q̇[joint.meta.q̇index] = x[joint.meta.ẋindex]
        end
    end
    calculate_transforms!(sys)  # spatial transforms
    calculate_r!(sys) # update generalized coords
    nothing
end

function ode_func!(dx, x, p, t)
    update_model!(p.sys, x)
    model!(p.sys)
    pack_dq_in_dx!(dx, p.sys)
    nothing
end

function save_dict!(config, name, type, func)
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
        if !isa(sys.joints[i], FixedJoint)
            #save joint state info        
            states = fieldnames(typeof(sys.joints[i].state))
            for state in states
                save_dict!(
                    save_config,
                    "$(sys.joints[i].meta.name)_$(string(state))",
                    typeof(getfield(sys.joints[i].state, state)),
                    integrator -> getfield(integrator.p.sys.joints[i].state, state)
                )
            end

            # save q̈
            save_dict!(
                save_config,
                "$(sys.joints[i].meta.name)_q̈",
                typeof(sys.q̈[sys.joints[i].meta.q̇index]),
                integrator -> integrator.p.sys.q̈[sys.joints[i].meta.q̇index]
            )
        end
    end
    for i in 1:length(sys.bodies)-1
        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_r",
            typeof(sys.r[i]),
            integrator -> integrator.p.sys.r[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_v",
            typeof(sys.v[i]),
            integrator -> integrator.p.sys.v[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_a",
            typeof(sys.a[i]),
            integrator -> integrator.p.sys.a[i]
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_fᵇ",
            typeof(sys.fᵇ[i]),
            #integrator -> integrator.p.sys.fᵇ[i]
            integrator -> integrator.p.sys.bodies[i].external_force
        )

        # get base frame position vars for animation        

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_q_base",
            typeof(sys.q_base[i]),
            #integrator -> (integrator.p.sys.q_base[i])
            integrator -> (integrator.p.sys.bodies[i].state.q_base)
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_r_base",
            typeof(sys.r_base[i]),
            #integrator -> (integrator.p.sys.r_base[i])
            integrator -> (integrator.p.sys.bodies[i].state.r_base)
        )

        for i in eachindex(sys.software)
            save_dict!(
                save_config,
                "$(sys.software[i].name)_u",
                typeof(sys.software[i].initial_value),
                #integrator -> integrator.u[integrator.p.sys.software[i].u_index]
                integrator -> integrator.p.sys.software[i].current_value
            )
        end

        for i in eachindex(sys.actuators)
            save_dict!(
                save_config,
                "$(sys.actuators[i].name)_f",
                typeof(sys.actuators[i].current_force),
                integrator -> integrator.u[integrator.p.sys.actuators[i].u_index]
            )

            save_dict!(
                save_config,
                "$(sys.actuators[i].name)_u",
                typeof(sys.actuators[i].command.current_value),
                integrator -> integrator.p.sys.actuators[i].command.current_value
            )

        end
    end

    save_values = SavedValues(Float64, Tuple{getindex.(save_config, "type")...})
    function save_function(u, t, integrator)
        #update after integration complete so values aren't based on RK4 steps
        update_model!(integrator.p.sys, u)
        model!(integrator.p.sys)
        Tuple([f(integrator) for f in getindex.(integrator.p.save_config, "func")])
    end
    save_cb = SavingCallback(save_function, save_values)
    return save_config, save_values, save_cb
end

function simulate(orig_sys::MultibodySystem, tspan; output_type=nothing)
    sys = deepcopy(orig_sys)# make a copy so we can rerun orig sys without mutating it during previous sim    
    save_config, save_values, save_cb = configure_saving(sys)
    p = (sys=sys, save_config=save_config)

    #get callbacks
    callbacks = [save_cb; getfield.(sys.software, :callback)...]
    cb = CallbackSet(callbacks...)

    prob = ODEProblem(ode_func!, sys.x, tspan, p)
    sol = solve(prob, callback=cb, adaptive=false, dt=0.01) # higher sample rate until three animation keyframetracks are better understood for interpolation

    if output_type == DataFrame
        simout = df_save_values(save_values, save_config)
    elseif output_type == NamedTuple
        simout = nt_save_values(save_values, save_config)
    else
        simout = sol
    end
    return simout
end

function nt_save_values(save_values, save_config)
    d = Dict()
    d[:t] = save_values.t
    for i in eachindex(save_config)
        name = Symbol(save_config[i]["name"])
        values = getindex.(save_values.saveval, i)
        d[name] = values
    end
    NamedTuple(d)
end

function df_save_values(save_values, save_config)
    D = DataFrame()
    D[!, "t"] = save_values.t
    for i in eachindex(save_config)
        name = String(save_config[i]["name"])
        type = save_config[i]["type"]

        if type <: AbstractVector
            for j in eachindex(save_values.saveval[1][i])
                this_name = "$(name)[$(j)]"
                this_values = map(x -> x[i][j], save_values.saveval)
                D[!, this_name] = this_values
            end
        elseif type <: AbstractMatrix
            for k in axes(save_values.saveval[1][i])[2]
                for j in axes(save_values[1][i])[1]
                    this_name = "$(name)[$(j),$(k)]"
                    this_values = map(x -> x[i][j, k], save_values.saveval)
                    D[!, this_name] = this_values
                end
            end
        else
            values = getindex.(save_values.saveval, i)
            D[!, name] = values
        end
    end
    return D
end

#needed each timestep to fill dx with ode state derivatives [dq,q̈] (dq not q̇ since q̇ can = ω and dq is quaternion deriv)
function pack_dq_in_dx!(dx, sys)
    for joint in sys.joints
        if !isa(joint, FixedJoint)
            dx[joint.meta.xindex] = get_dq(joint)
            dx[joint.meta.ẋindex] = sys.q̈[joint.meta.q̇index]
        end
    end
    nothing
end

function Plots.plot(t::Vector{T}, x::Vector{SVector{S,T}}) where {S,T<:AbstractFloat}
    p = plot(t, getindex.(x, 1))
    if length(x[1]) > 1
        for i in 2:length(x[1])
            plot!(t, getindex.(x, i))
        end
    end
    return p
end

function plot(sol::DataFrame, col)
    p = plot(sol[!, "t"], sol[!, col])
    return p
end
