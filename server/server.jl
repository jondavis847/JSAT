using HTTP, Sockets, JSON3, JLD2
includet("..\\src\\jsat.jl")
function jsat_server()
    ROUTER = HTTP.Router()

    routerIndex(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\index.html"))
    routerCss(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\css\\jsat.css"))
    routerJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"], read("server\\public\\js\\jsat.js"))
    routerMeatball(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/png"], read("server\\public\\images\\nasa_aquamarine.png"))
    routerEarth(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\earth.jpeg"))

    HTTP.register!(ROUTER, "GET", "/", routerIndex)
    HTTP.register!(ROUTER, "GET", "/css/jsat.css", routerCss)
    HTTP.register!(ROUTER, "GET", "/js/jsat.js", routerJs)
    HTTP.register!(ROUTER, "GET", "/images/nasa_aquamarine.png", routerMeatball)
    HTTP.register!(ROUTER, "GET", "/images/earth.jpeg", routerEarth)

    HTTP.register!(ROUTER, "POST", "/simulate", routerSimulate)
    HTTP.register!(ROUTER, "GET", "/simfiles", routerSimFiles)
    HTTP.register!(ROUTER, "GET", "/customsoftware", routerCustomSoftware)
    HTTP.register!(ROUTER, "GET", "/loadmodels", routerLoadModels)
    HTTP.register!(ROUTER, "POST", "/loadstates", routerLoadStates)
    HTTP.register!(ROUTER, "POST", "/plotstates", routerPlot)
    HTTP.register!(ROUTER, "POST", "/animation", routerAnimate)
    HTTP.register!(ROUTER, "POST", "/createmodel", routerCreateModel)

    server = HTTP.serve!(ROUTER, ip"127.0.0.1", 80)
    printstyled("servers up! ctrl+click url to go : http://127.0.0.1:80 \n", color=:light_yellow)
    return server

end

function routerSimFiles(req::HTTP.Request)
    tmp = []
    files = readdir("sim")
    for file in files
        simfilename = replace(file, ".csv" => "")
        simfiledate = Dates.unix2datetime(mtime("sim\\$(file)"))        
        D = Dict("fileName" => simfilename, "fileDate" => simfiledate)
        push!(tmp, D)
    end
    sort!(tmp,by=x->x["fileDate"], rev=true)
    msg = JSON3.write(tmp)

    HTTP.Response(200, JSON3.write(tmp))
end

function routerSimulate(req::HTTP.Request)
    message = JSON3.read(req.body)
    println(message)
    sim = message[:sim]
    base = message[:base]
    bodies = message[:bodies]
    joints = message[:joints]
    actuators = message[:actuators]
    softwares = message[:software]
    gravitys = message[:gravity]
    #need to make these things in this order to make children first before connecting

    Software = []
    for k in keys(softwares)
        software = softwares[k]
        type = software[:type]
        if type == "timedCommand"
            init = eval(Meta.parse(software[:init]))
            tstarts = eval(Meta.parse(software[:tstarts]))
            if !(typeof(tstarts) <: Vector)
                tstarts = [tstarts]
            end
            tstops = eval(Meta.parse(software[:tstops]))
            if !(typeof(tstops) <: Vector)
                tstops = [tstops]
            end
            S = TimedCommand(Symbol(software[:name]), init, tstarts, tstops,)
        else
            error("bad software type provided")
            return
        end
        push!(Software, S)
    end

    Actuators = AbstractActuator[]
    for k in keys(actuators)
        actuator = actuators[k]
        type = actuator[:type]
        if type == "thruster"
            force = eval(Meta.parse(actuator[:thrust]))
            A = SimpleThruster(Symbol(actuator[:name]), force)
        else
            error("bad actuator type provided")
            return
        end
        command = Software[getfield.(Software, :name).==Symbol(actuator[:command])]
        connect!(A, command...)
        push!(Actuators, A)
    end

    Gravitys = AbstractGravity[]
    for k in keys(gravitys)
        gravity = gravitys[k]
        if gravity[:type] == "constant"
            value = eval(Meta.parse(gravity[:value]))
            G = GravityConstant(Symbol(gravity[:name]), value)
        else
            error("bad gravity type provided")
            return
        end
        push!(Gravitys, G)
    end


    base_gravity = base[:gravity]
    BG = AbstractGravity[]
    if !isempty(base_gravity)
        for bg in base_gravity            
            append!(BG, Gravitys[getfield.(Gravitys, :name) .== Symbol(bg)])            
        end
    else
        push!(BG,GravityNone())
    end
    base = BaseFrame(:base, BG)    
    
    Bodies = AbstractBody[]
    for k in keys(bodies)
        body = bodies[k]

        mass = Float64(eval(Meta.parse(body[:mass])))
        cm = Float64.(eval(Meta.parse(body[:cm])))
        ixx = Float64(eval(Meta.parse(body[:ixx])))
        iyy = Float64(eval(Meta.parse(body[:iyy])))
        izz = Float64(eval(Meta.parse(body[:izz])))
        ixy = Float64(eval(Meta.parse(body[:ixy])))
        ixz = Float64(eval(Meta.parse(body[:ixz])))
        iyz = Float64(eval(Meta.parse(body[:iyz])))

        connected_actuators = eval(Meta.parse.(body[:actuators]))

        inertia = [
            ixx ixy ixz
            ixy iyy iyz
            ixz iyz izz
        ]

        B = Body(
            body[:name],
            mass,
            cm,
            inertia
        )

        for actuator_name in connected_actuators
            actuator = actuators[actuator_name]
            rotation = inv(eval(Meta.parse(actuator[:rotation]))) #inv because server gives rotation from body to act, need act to body
            translation = -eval(Meta.parse(actuator[:translation])) #inv because server gives translation from body to act, need act to body
            frame = Cartesian(rotation, translation)
            A = Actuators[getfield.(Actuators, :name).==Symbol(actuator_name)]
            connect!(B, A..., frame)
        end

        connected_gravity = body[:gravity]
        for gravity_name in connected_gravity
            G = Gravitys[getfield.(Gravitys, :name).==Symbol(gravity_name)]
            connect!(B, G)
        end


        push!(Bodies, B)
    end    

    Joints = []
    for k in keys(joints)
        joint = joints[k]
        type = joint[:type]
        if type == "revolute"
            θ = eval(Meta.parse(joint[:theta]))
            ω = eval(Meta.parse(joint[:omega]))
            J = Revolute(Symbol(joint[:name]), θ, ω)
        elseif type == "fixed"
            q = eval(Meta.parse(joint[:q]))
            r = eval(Meta.parse(joint[:position]))
            J = FixedJoint(Symbol(joint[:name]), q, r)
        elseif type == "floating"
            q = eval(Meta.parse(joint[:q]))
            ω = eval(Meta.parse(joint[:omega]))
            r = eval(Meta.parse(joint[:position]))
            v = eval(Meta.parse(joint[:velocity]))
            J = FloatingJoint(Symbol(joint[:name]), q, ω, r, v)
        else
            error("bad joint type provided")
            return
        end

        predecessor_name = joint[:predecessor]
        successor_name = joint[:successor]

        if predecessor_name in ["base", "earth"]
            predecessor = [base]
        else
            predecessor = Bodies[getfield.(Bodies, :name).==Symbol(predecessor_name)]
        end
        successor = Bodies[getfield.(Bodies, :name).==Symbol(successor_name)]

        FpΦ = eval(Meta.parse(joint[:FpPhi]))
        FsΦ = eval(Meta.parse(joint[:FsPhi]))
        Fpρ = eval(Meta.parse(joint[:FpRho]))
        Fsρ = eval(Meta.parse(joint[:FsRho]))
        Fp = Cartesian(FpΦ, Fpρ)
        Fs = Cartesian(FsΦ, Fsρ)

        connect!(J, predecessor..., successor..., Fp, Fs)
        push!(Joints, J)
    end



    sys = MultibodySystem(Symbol(sim[:name]), base,  Bodies, Joints, Actuators, Software)

    t = time()
    sol = simulate(sys, eval(Meta.parse(sim[:tspan])); output_type=DataFrame)
    dt = time() - t
    if (isempty(sim[:name]))
        t = lpad(string(abs(rand(Int16))), 5, "0")
        sim_name = "sim$(t)"
    else
        sim_name = sim[:name]
    end

    if !isdir("sim\\$(sim_name)")
        mkdir("sim\\$(sim_name)")
    end

    #save simulation data file
    CSV.write("sim\\$(sim_name)\\run0.csv", sol)

    #save system file
    open("sim\\$(sim_name)\\system.json", "w") do io
        JSON3.pretty(io, message)
    end


    HTTP.Response(200, "$(dt)")
end

function routerLoadStates(req::HTTP.Request)
    message = JSON3.read(req.body)
    states = String[]
    for file in message
        df = CSV.read("sim\\$(file)\\run0.csv", DataFrame; limit=0) # limit just reads headers for now
        union!(states, string.(names(df)))
    end
    D = Dict("states" => states)
    HTTP.Response(200, JSON3.write(D))
end

function routerPlot(req::HTTP.Request)
    data = JSON3.read(req.body)
    sims = data[:sims]
    xs = data[:states]
    simdata = []
    for i in eachindex(sims)
        loc = "sim\\$(sims[i])"
        f = readdir(loc)
        runs = f[occursin.(".csv", f)]
        rundata = []
        for r in eachindex(runs)
            rd = CSV.read("$(loc)\\$(runs[r])", DataFrame, select=["t", xs...])
            push!(rundata, rd)
        end
        push!(simdata, Dict("sim" => sims[i], "runData" => rundata))
    end
    HTTP.Response(200, JSON3.write(simdata))
end

function routerAnimate(req::HTTP.Request)
    data = JSON3.read(req.body)
    sim = data[:sim]
    run = data[:run]
    loc = "sim\\$(sim)"
    f = readdir(loc)
    sys = JSON3.read("$(loc)\\system.json")
    t = CSV.read("$(loc)\\$(run).csv", DataFrame, select=["t"])
    rd = DataFrame(t)

    for body in sys.bodies
        body_name = body[2].name
        states = [
            "$(body_name)_q_base[1]"
            "$(body_name)_q_base[2]"
            "$(body_name)_q_base[3]"
            "$(body_name)_q_base[4]"
            "$(body_name)_r_base[1]"
            "$(body_name)_r_base[2]"
            "$(body_name)_r_base[3]"
        ]
        this_rd = CSV.read("$(loc)\\$(run).csv", DataFrame, select=[states...])
        rd = hcat(rd, this_rd)
    end

    D = Dict("sys" => sys, "data" => rd)
    HTTP.Response(200, JSON3.write(D))#JSON3.write(simdata))
end

function routerCreateModel(req::HTTP.Request)
    file = "server//models.jld2"

    data = JSON3.read(req.body, Dict)
    name = data["name"]
    model = data["model"]

    if isfile(file)
        model_dict = load(file)
        rm(file)
    else
        model_dict = Dict()
    end
    model_dict[name] = model

    save(file, model_dict)

    HTTP.Response(200, "success")
end

function routerLoadModels(req::HTTP.Request)
    file = "server//models.jld2"

    if !isfile(file)
        models = Dict()
    else
        models = load("server//models.jld2")
    end
    HTTP.Response(200, JSON3.write(models))
end

function routerCustomSoftware(req::HTTP.Request)
    tmp = []
    files = readdir("src//software//custom")
    for file in files
        modulename = replace(file, ".jl" => "")
        push!(tmp, modulename)
    end

    HTTP.Response(200, JSON3.write(tmp))
end