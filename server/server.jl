using HTTP, Sockets, JSON3, JLD2
includet("..\\src\\jsat.jl")
function jsat_server()
    ROUTER = HTTP.Router()

    routerIndex(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\index.html"))
    routerCss(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\css\\jsat.css"))
    routerJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"], read("server\\public\\js\\jsat.js"))
    routerAnimJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"], read("server\\public\\js\\animation.js"))
    routerPlotJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"], read("server\\public\\js\\plotting.js"))
    routerCytoJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"], read("server\\public\\js\\cyto.js"))
    routerMeatball(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/png"], read("server\\public\\images\\nasa_aquamarine.png"))
    routerEarth(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\earth.jpeg"))
    routerEarth16k(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\earth_16k.jpg"))
    routerEarthBump16k(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\earth_bump_16k.jpg"))
    routerEarthSpec4k(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\8081_earthspec4k.jpg"))
    routerStarfield(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "image/jpeg"], read("server\\public\\images\\starfield.jpg"))
    
    HTTP.register!(ROUTER, "GET", "/", routerIndex)
    HTTP.register!(ROUTER, "GET", "/css/jsat.css", routerCss)
    HTTP.register!(ROUTER, "GET", "/js/jsat.js", routerJs)
    HTTP.register!(ROUTER, "GET", "/js/animation.js", routerAnimJs)
    HTTP.register!(ROUTER, "GET", "/js/plotting.js", routerPlotJs)
    HTTP.register!(ROUTER, "GET", "/js/cyto.js", routerCytoJs)
    HTTP.register!(ROUTER, "GET", "/images/nasa_aquamarine.png", routerMeatball)
    HTTP.register!(ROUTER, "GET", "/images/earth.jpeg", routerEarth)
    HTTP.register!(ROUTER, "GET", "/images/earth_16k.jpg", routerEarth16k)
    HTTP.register!(ROUTER, "GET", "/images/earth_bump_16k.jpg", routerEarthBump16k)
    HTTP.register!(ROUTER, "GET", "/images/8081_earthspec4k.jpg", routerEarthSpec4k)
    HTTP.register!(ROUTER, "GET", "/images/starfield.jpg", routerStarfield)

    HTTP.register!(ROUTER, "POST", "/simulate", routerSimulate)
    HTTP.register!(ROUTER, "GET", "/simfiles", routerSimFiles)
    HTTP.register!(ROUTER, "GET", "/customsoftware", routerCustomSoftware)
    HTTP.register!(ROUTER, "GET", "/loadmodels", routerLoadModels)
    HTTP.register!(ROUTER, "POST", "/loadstates", routerLoadStates)
    HTTP.register!(ROUTER, "POST", "/plotstates", routerPlot)
    HTTP.register!(ROUTER, "POST", "/animation", routerAnimate)
    HTTP.register!(ROUTER, "POST", "/savescenario", routerSaveScenario)
    HTTP.register!(ROUTER, "GET", "/getscenarios", routerGetScenarios)
    HTTP.register!(ROUTER, "POST", "/loadscenario", routerLoadScenario)

    server = HTTP.serve!(ROUTER, ip"127.0.0.1", 80)
    printstyled("servers up! ctrl+click url to go : http://127.0.0.1:80 \n", color=:light_yellow)
    return server

end

function routerSimFiles(req::HTTP.Request)
    tmp = []
    if !isdir("sim")
        mkdir("sim")
    end
    files = readdir("sim")
    for file in files
        simfilename = replace(file, ".csv" => "")
        simfiledate = Dates.unix2datetime(mtime("sim\\$(file)"))
        D = Dict("fileName" => simfilename, "fileDate" => simfiledate)
        push!(tmp, D)
    end
    sort!(tmp, by=x -> x["fileDate"], rev=true)
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
    sensors = message[:sensors]
    gravitys = message[:gravity]

    #need to make these things in this order to make children first before connecting

    Sensors = AbstractSensor[]
    for k in keys(sensors)
        sensor = sensors[k]
        type = sensor[:type]
        if type == "simpleAttitudeSensor"
            S = SimpleAttitudeSensor(sensor[:name])
        elseif type == "simpleAttitudeSensor4"
            S = SimpleAttitudeSensor4(sensor[:name])
        elseif type == "simpleRateSensor"
            S = SimpleRateSensor(sensor[:name])
        elseif type == "simpleRateSensor3"
            S = SimpleRateSensor3(sensor[:name])
        end
        push!(Sensors, S)
    end

    Actuators = AbstractActuator[]
    for k in keys(actuators)
        actuator = actuators[k]
        type = actuator[:type]
        if type == "thruster"
            force = eval(Meta.parse(actuator[:thrust]))
            A = SimpleThruster(Symbol(actuator[:name]), force)
        elseif type == "reactionWheel"
            inertia = eval(Meta.parse(actuator[:inertia]))
            kt = eval(Meta.parse(actuator[:kt]))
            H = eval(Meta.parse(actuator[:H]))
            A = SimpleReactionWheel(Symbol(actuator[:name]),inertia,kt,H)
        else
            error("bad actuator type provided")
            return
        end
        push!(Actuators, A)
    end

    Software = AbstractSoftware[]
    for k in keys(softwares)
        software = softwares[k]
        type = software[:type]
        if type == "timedCommand"
            values = eval(Meta.parse(software[:values]))
            if !(typeof(values) <: Vector)
                values = [values]
            end
            tsteps = eval(Meta.parse(software[:tsteps]))
            if !(typeof(tsteps) <: Vector)
                tsteps = [tsteps]
            end            
            SW = TimedCommand(Symbol(software[:name]), tsteps, values)
        elseif type == "custom"
            modulename = software[:module]
            custom_software_dict = get_custom_software()
            SW = custom_software_dict[modulename]
        else
            error("bad software type provided")
            return
        end
        matched_sensors = intersect(getfield.(Sensors, :name), Symbol.(software[:sensors]))
        for sensor in Sensors
            if sensor.name in matched_sensors
                connect!(sensor, SW)
            end
        end

        matched_actuators = intersect(getfield.(Actuators, :name), Symbol.(software[:actuators]))
        for actuator in Actuators
            if actuator.name in matched_actuators
                connect!(SW, actuator)
            end
        end

        push!(Software, SW)
    end

    Gravitys = AbstractGravity[]
    for k in keys(gravitys)
        gravity = gravitys[k]
        if gravity[:type] == "constant"
            value = eval(Meta.parse(gravity[:value]))
            G = GravityConstant(Symbol(gravity[:name]), value)
        elseif gravity[:type] == "twoBodyEarth"
            G = TwoBodyEarth(Symbol(gravity[:name]))
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
            append!(BG, Gravitys[getfield.(Gravitys, :name).==Symbol(bg)])
        end
    else
        push!(BG, GravityNone())
    end
    base = BaseFrame(:base, BG)

    Bodies = AbstractBody[]
    for k in keys(bodies)
        body = bodies[k]

        mass = make_SimVal(body[:mass])
        cmx = make_SimVal(body[:cmx])
        cmy = make_SimVal(body[:cmy])
        cmz = make_SimVal(body[:cmz])
        ixx = make_SimVal(body[:ixx])
        iyy = make_SimVal(body[:iyy])
        izz = make_SimVal(body[:izz])
        ixy = make_SimVal(body[:ixy])
        ixz = make_SimVal(body[:ixz])
        iyz = make_SimVal(body[:iyz])

        connected_actuators = eval(Meta.parse.(body[:actuators]))
        connected_sensors = eval(Meta.parse.(body[:sensors]))

        cm = [cmx,cmy,cmz]

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

        for sensor_name in connected_sensors
            sensor = sensors[sensor_name]
            rotation = inv(eval(Meta.parse(sensor[:rotation]))) #inv because server gives rotation from body to act, need act to body
            translation = -eval(Meta.parse(sensor[:translation])) #inv because server gives translation from body to act, need act to body
            frame = Cartesian(rotation, translation)
            S = Sensors[getfield.(Sensors, :name).==Symbol(sensor_name)]
            connect!(B, S..., frame)
        end

        connected_gravity = body[:gravity]
        for gravity_name in connected_gravity
            G = Gravitys[getfield.(Gravitys, :name).==Symbol(gravity_name)]
            connect!(B, G...)
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
            f = eval(Meta.parse(joint[:force]))
            κ = eval(Meta.parse(joint[:kappa]))
            ζ = eval(Meta.parse(joint[:zeta]))
            pos_low_lim = eval(Meta.parse(joint[:poslowlim]))
            pos_up_lim = eval(Meta.parse(joint[:posuplim]))
            J = Revolute(Symbol(joint[:name]), θ, ω; f=f, κ=κ,ζ=ζ, pos_upper_limit = pos_up_lim, pos_lower_limit = pos_low_lim)
        elseif type == "prismatic"
            r = eval(Meta.parse(joint[:position]))
            v = eval(Meta.parse(joint[:velocity]))
            J = Prismatic(Symbol(joint[:name]), r, v)
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



    sys = MultibodySystem(Symbol(sim[:name]), base, Bodies, Joints, actuators=Actuators, software=Software, sensors=Sensors)

    sim_dt = eval(Meta.parse(sim[:dt]))
    sim_nruns = sim[:nruns] isa String ? eval(Meta.parse(sim[:nruns])) : sim[:nruns]

    t = time()
    sol = simulate(sys, eval(Meta.parse(sim[:tspan])); nruns=sim_nruns, dt=sim_dt, output_type=DataFrame)

    simtime = time() - t
    if (isempty(sim[:name]))
        t = lpad(string(abs(rand(Int16))), 5, "0")
        sim_name = "sim$(t)"
    else
        sim_name = sim[:name]
    end

    if !isdir("sim")
        mkdir("sim")
    end

    if !isdir("sim\\$(sim_name)")
        mkdir("sim\\$(sim_name)")
    end

    #save simulation data file
    if sol isa DataFrame
        CSV.write("sim\\$(sim_name)\\run0.csv", sol)
    elseif sol isa Tuple # (nominal,dispersed)
        CSV.write("sim\\$(sim_name)\\run0.csv", sol[1])
        for i in eachindex(sol[2])
            CSV.write("sim\\$(sim_name)\\run$(i).csv", sol[2][i])
        end
    else 
        error("why is sol not a dataframe or tuple?")
    end

    #save system file
    open("sim\\$(sim_name)\\system.json", "w") do io
        JSON3.pretty(io, message)
    end

    println("Simulation completed in $(simtime) seconds.")

    HTTP.Response(200, "$(simtime)")    
end


function make_SimVal(expr)
    
    nominal = eval(Meta.parse(expr[:nominal]))
    dispersed = eval(Meta.parse(expr[:dispersed]))
    sv = isnothing(dispersed) ? SimVal(nominal) : SimVal(nominal,dispersed)
    return sv
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
    #t = CSV.read("$(loc)\\$(run).csv", DataFrame, select=["t"])
    #rd = DataFrame(t)
    full_states = []

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
        append!(full_states, states)
    end

    for actuator in sys.actuators
        actuator_name = actuator[2].name
        states = [
            "$(actuator_name)_q_base[1]"
            "$(actuator_name)_q_base[2]"
            "$(actuator_name)_q_base[3]"
            "$(actuator_name)_q_base[4]"
            "$(actuator_name)_r_base[1]"
            "$(actuator_name)_r_base[2]"
            "$(actuator_name)_r_base[3]"
            "$(actuator_name)_force"
        ]
        append!(full_states, states)
    end
    rd = CSV.read("$(loc)\\$(run).csv", DataFrame, select=["t", full_states...])

    D = Dict("sys" => sys, "data" => rd)
    HTTP.Response(200, JSON3.write(D))
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
    cs = keys(get_custom_software())
    HTTP.Response(200, JSON3.write(cs))
end

function routerSaveScenario(req::HTTP.Request)
    message = JSON3.read(req.body)
    scenario_name = message[:name]
    open("scenarios\\$(scenario_name).json", "w") do io
        JSON3.pretty(io, message)
    end
    HTTP.Response(200, "saved")
end

function routerGetScenarios(req::HTTP.Request)
    tmp = []
    files = readdir("scenarios")
    for file in files
        scenario_name = replace(file, ".json" => "")
        push!(tmp, scenario_name)
    end
    HTTP.Response(200, JSON3.write(tmp))
end

function routerLoadScenario(req::HTTP.Request)
    message = JSON3.read(req.body)
    println(message)
    scenario = JSON3.read("scenarios\\$(message[:name]).json")
    HTTP.Response(200, JSON3.write(scenario))
end