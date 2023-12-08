using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, OffsetArrays, PlotThemes, DataFrames, CSV, Dates, ConcreteStructs
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
mcI(b::Body) = mcI(b.m, b.cm, b.I)

includet("joints.jl")
includet("software//software.jl")
includet("utils//pathutils.jl")

struct Force
    magnitude::Float64
    direction::SVector{3,Float64}
end

struct Torque
    magnitude::Float64
    direction::SVector{3,Float64}
end

@concrete struct MultibodySystem
    name
    bodies
    joints
    software
    p       # joint predecessor body array
    s       # joint successor body array
    λ       # body parent array
    κ       # all joints between body i and base
    μ       # body children array
    γ       # all bodies from body i to tip (this is nu in Featherstone but nu just looks like v in Julia, so \gamma it is)    
    U
    D
    u
    c
    pᴬ
    ᵖXᵢᵐ        #spatial motion transform from body i to predecessor of body i
    ᵖXᵢᶠ        #spatial force transform from body i to predecessor of body i
    ⁱXₚᵐ        #spatial motion transform from predecessor of body i to body i
    ⁱXₚᶠ        #spatial force transform from predecessor of body i to body i       
    ᵒXᵢᵐ        #spatial motion transform from body i to worldframe
    ᵒXᵢᶠ        #spatial force transform from body i to worldframe
    ⁱXₒᵐ        #spatial motion transform from worldframe to body i
    ⁱXₒᶠ        #spatial force transform from worldframe to body i  
    q
    q̇
    q̈
    x
    H
    C
    τ
    fˣ
    fᵇ
    f_gyro
    Iᵇ
    Iᴬ
    r
    v
    a
    r_base
    q_base
end


#MultibodySystem constructor
function MultibodySystem(name, bodies, joints, software=AbstractSoftware[])
    # preallocate vectors/matrices at MultibodySystem definition, mutate in place during sim
    # map the bodies and joints 
    p, s, λ, κ, μ, γ = map_tree!(bodies, joints)

    # sort arrays so we can index directly by ID    
    permute!(bodies, sortperm(map(x -> x.id, bodies)))
    permute!(joints, sortperm(map(x -> x.meta.id, joints)))

    Iᵇ, Iᴬ = initialize_inertias(bodies, joints) # Iᵇ for now to not conflict with LinearAlgebra: I


    #grab q,q̇ initial conditions
    q, q̇, q̈, x = initialize_state_vectors(joints, software)

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

    r_base = MVector{nb - 1,SVector{3,Float64}}(fill(SVector{3,Float64}(zeros(3)), nb - 1))
    q_base = MVector{nb - 1,SVector{4,Float64}}(fill(SVector{4,Float64}(zeros(4)), nb - 1))

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


    # make the software callbacks
    for i in eachindex(software)
        create_callbacks!(software[i], i)
    end

    sys = MultibodySystem(name, bodies, joints, software, p, s, λ, κ, μ, γ, U, D, u, c, pᴬ, ᵖXᵢᵐ, ᵖXᵢᶠ, ⁱXₚᵐ, ⁱXₚᶠ, ᵒXᵢᵐ, ᵒXᵢᶠ, ⁱXₒᵐ, ⁱXₒᶠ, q, q̇, q̈, x, H, C, τ, fˣ, fᵇ, f_gyro, Iᵇ, Iᴬ, r, v, a, r_base, q_base)

    #initialize FixedJoints X so we don't have to calculate again
    calculate_X_FixedJoints!(sys)
    return sys
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
    nothing
end



function calculate_τ!(τ)
    #until we figure out how to do software
    for i in eachindex(τ)
        τ[i] = 0.0
    end
    nothing
end
calculate_τ!(sys::MultibodySystem) = calculate_τ!(sys.τ)

calculate_X!(sys::MultibodySystem) = calculate_X!(sys.ᵖXᵢᵐ, sys.ᵖXᵢᶠ, sys.ⁱXₚᵐ, sys.ⁱXₚᶠ, sys.ᵒXᵢᵐ, sys.ᵒXᵢᶠ, sys.ⁱXₒᵐ, sys.ⁱXₒᶠ, sys.λ, sys.joints)

function calculate_X!(ᵖXᵢᵐ, ᵖXᵢᶠ, ⁱXₚᵐ, ⁱXₚᶠ, ᵒXᵢᵐ, ᵒXᵢᶠ, ⁱXₒᵐ, ⁱXₒᶠ, λ, joints)
    for i in eachindex(joints)
        if !isa(joints[i], FixedJoint)
            joint = joints[i]
            p_to_i = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
            i_to_p = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

            ᵖXᵢᵐ[i] = ℳ(i_to_p)
            ᵖXᵢᶠ[i] = ℱ(i_to_p)
            ⁱXₚᵐ[i] = ℳ(p_to_i)
            ⁱXₚᶠ[i] = ℱ(p_to_i)
        end

        ᵒXᵢᵐ[i] = ᵒXᵢᵐ[λ[i]] * ᵖXᵢᵐ[i]
        ᵒXᵢᶠ[i] = ᵒXᵢᶠ[λ[i]] * ᵖXᵢᶠ[i]
        ⁱXₒᵐ[i] = ⁱXₚᵐ[i] * ⁱXₒᵐ[λ[i]]
        ⁱXₒᶠ[i] = ⁱXₚᶠ[i] * ⁱXₒᶠ[λ[i]]

    end
    nothing
end

# only have to do this once at initialization
function calculate_X_FixedJoints!(ᵖXᵢᵐ, ᵖXᵢᶠ, ⁱXₚᵐ, ⁱXₚᶠ, λ, joints)
    for i in eachindex(joints)
        if isa(joints[i], FixedJoint)
            joint = joints[i]
            p_to_i = inv(joint.connection.Fs) * joint.frame * joint.connection.Fp
            i_to_p = inv(joint.connection.Fp) * inv(joint.frame) * joint.connection.Fs

            ᵖXᵢᵐ[i] = ℳ(i_to_p)
            ᵖXᵢᶠ[i] = ℱ(i_to_p)
            ⁱXₚᵐ[i] = ℳ(p_to_i)
            ⁱXₚᶠ[i] = ℱ(p_to_i)
        end
    end
    nothing
end
calculate_X_FixedJoints!(sys::MultibodySystem) = calculate_X_FixedJoints!(sys.ᵖXᵢᵐ, sys.ᵖXᵢᶠ, sys.ⁱXₚᵐ, sys.ⁱXₚᶠ, sys.λ, sys.joints)

calculate_r!(sys::MultibodySystem) = calculate_r!(sys.joints, sys.r_base, sys.q_base, sys.ᵒXᵢᵐ, sys.λ)
function calculate_r!(joints, r_base, q_base, ᵒXᵢᵐ, λ)
    for i in eachindex(joints)
        joint = joints[i]
        r_Fs_to_Bi_in_Fs_frame = (joint.connection.Fs.Φ.value)' * (-joint.connection.Fs.r)
        r_Fp_to_Fs_in_Fp_frame = joint.frame.Φ.value' * r_Fs_to_Bi_in_Fs_frame + joint.frame.r
        r_Bλ_to_Fp_in_Bλ_frame = joint.connection.Fp.Φ.value' * r_Fp_to_Fs_in_Fp_frame + joint.connection.Fp.r
        if λ[i] != 0
            r_Bλ_to_Fp_in_base_frame = (ᵒXᵢᵐ[λ[i]][i3, i3]) * r_Bλ_to_Fp_in_Bλ_frame
            r_base_to_Fp_in_base_frame = r_Bλ_to_Fp_in_base_frame + r_base[λ[i]]
            q_base[i] = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value * qtoa(q_base[λ[i]]))
        else
            r_base_to_Fp_in_base_frame = r_Bλ_to_Fp_in_Bλ_frame
            q_base[i] = atoq(joint.connection.Fs.Φ.value' * joint.frame.Φ.value * joint.connection.Fp.Φ.value)
        end
        r_base[i] = r_base_to_Fp_in_base_frame

    end
    nothing
end

function articulated_body_algorithm!(sys)
    @unpack v, a, c, D, U, u, ⁱXₚᵐ, Iᵇ, Iᴬ, pᴬ, ⁱXₒᶠ, ᵖXᵢᶠ, ⁱXₚᵐ, fᵇ, fˣ, f_gyro, τ, λ, q̈, q̇, joints = sys

    # pass 1
    for i in 1:length(sys.bodies)-1

        if isa(joints[i], FixedJoint)
            v[i] = ⁱXₚᵐ[i] * v[λ[i]]
            c[i] = @SVector zeros(6)
        else
            S = joints[i].S
            vj = S * q̇[joints[i].meta.q̇index]
            v[i] = ⁱXₚᵐ[i] * v[λ[i]] + vj
            c[i] = v[i] ×ᵐ vj # + cj #commented until we need S∘                
        end

        Iᴬ[i] = Iᵇ[i]
        fᵇ[i] = fˣ[i]
        f_gyro[i] = v[i] ×ᶠ (Iᵇ[i] * v[i])
        pᴬ[i] = f_gyro[i] - fᵇ[i]
    end

    # pass 2
    for i in reverse(1:length(sys.bodies)-1)

        if isa(joints[i], FixedJoint)
            if λ[i] != 0
                Iᴬ[λ[i]] = Iᴬ[λ[i]] + ᵖXᵢᶠ[i] * Iᴬ[i] * ⁱXₚᵐ[i]
                pᴬ[λ[i]] = pᴬ[λ[i]] + ᵖXᵢᶠ[i] * pᴬ[i]
            end
        else
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
    end

    # pass 3   

    for i in 1:length(sys.bodies)-1
        if isa(joints[i], FixedJoint)
            a[i] = ⁱXₚᵐ[i] * a[λ[i]]
        else
            S = joints[i].S
            a′ = ⁱXₚᵐ[i] * a[λ[i]] + c[i]
            q̈[joints[i].meta.q̇index] = D[i] \ (u[i] - U[i]' * a′)
            a[i] = a′ + S * q̈[joints[i].meta.q̇index]
        end
    end
    nothing
end

function initialize_state_vectors(joints, softwares)
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

        # make the assumption for revolute or spherical joints that the body frame is coincident with the joint Fs frame
        # shift mass properties to the joint frame
        if typeof(joints[i]) in [Revolute, Spherical]
            Fs = joints[i].connection.Fs # Fs is joint frame expressed in body frame, or transform from body to joint
            ᵇXⱼᵐ = ℳ(inv(Fs)) # need motion transformation from joint to body
            ʲXᵦᶠ = ℱ(Fs) # need force transformation from body to joint
            Iʲ = ʲXᵦᶠ * Iᵇ[i] * ᵇXⱼᵐ # Featherstone equations 2.66 for transform of spatial inertia
            Iᵇ[i] = Iʲ
        end

        # make the assumption for FloatingJoints that the body frame is coincident with the com
        # shift mass properties to the com
        if typeof(joints[i]) == FloatingJoint
            Iᵇ[i] = mcI(body.m, SVector{3,Float64}(zeros(3)), body.I)
        end
    end
    Iᴬ = copy(Iᵇ)
    return Iᵇ, Iᴬ
end

function environments!(sys)
    gravity!(sys)
end


function gravity!(sys::MultibodySystem)
    for i in 1:length(sys.bodies)-1
        sys.fˣ[i] = sys.fˣ[i] .+ sys.Iᵇ[i] * sys.ⁱXₒᵐ[i] * SVector{6,Float64}(0, 0, 0, 0, -9.8, 0)
    end
    nothing
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
    calculate_X!(sys)  # spatial transforms
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
            integrator -> integrator.p.sys.fᵇ[i]
        )

        # get base frame position vars for animation        

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_q_base",
            typeof(sys.q_base[i]),
            integrator -> (integrator.p.sys.q_base[i])
        )

        save_dict!(
            save_config,
            "$(sys.bodies[i].name)_r_base",
            typeof(sys.r_base[i]),
            integrator -> (integrator.p.sys.r_base[i])
        )

        for i in eachindex(sys.software)
            save_dict!(
                save_config,
                "$(sys.software[i].name)_u",
                typeof(sys.software[i].initial_value),
                integrator -> integrator.u[integrator.p.sys.software[i].u_index]
            )
        end
    end

    save_values = SavedValues(Float64, Tuple{getindex.(save_config, "type")...})
    save_function(u, t, integrator) = Tuple([f(integrator) for f in getindex.(integrator.p.save_config, "func")])
    save_cb = SavingCallback(save_function, save_values)
    return save_config, save_values, save_cb
end

function simulate(orig_sys::MultibodySystem, tspan; output_type=nothing)
    sys = deepcopy(orig_sys)# make a copy so we can rerun orig sys without mutating it during previous sim    
    save_config, save_values, save_cb = configure_saving(sys)
    p = (sys=sys, save_config=save_config)

    #get callbacks
    callbacks = [save_cb; getfield.(sys.software,:callback)...;]
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

function plot(sol::DataFrame,col)
    p = plot(sol[!,"t"], sol[!,col])
    return p
end
