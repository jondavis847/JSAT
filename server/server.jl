using HTTP, Sockets, JSON3
includet("..\\src\\blue42.jl")
function jsat_server()
    ROUTER = HTTP.Router()

    routerIndex(req::HTTP.Request) = HTTP.Response(200,read("server\\public\\index.html"))
    routerCss(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\css\\jsat.css"))
    routerJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"],read("server\\public\\js\\jsat.js"))
    #routerJs(req::HTTP.Request) = HTTP.Response(200, ["Content-Type" => "application/javascript"],read("server\\public\\dist\\bundle.js"))
    routerLoadModels(req::HTTP.Request) = HTTP.Response(200, string(JSON3.read("server\\models.json")))

    HTTP.register!(ROUTER, "GET", "/", routerIndex)
    HTTP.register!(ROUTER, "GET", "/css/jsat.css", routerCss)
    HTTP.register!(ROUTER, "GET", "/js/jsat.js", routerJs)
    #HTTP.register!(ROUTER, "GET", "/dist/bundle.js", routerJs)

    HTTP.register!(ROUTER, "POST", "/simulate", routerSimulate)
    HTTP.register!(ROUTER, "GET", "/simfiles", routerSimFiles)
    HTTP.register!(ROUTER, "GET", "/loadmodels", routerLoadModels)
    HTTP.register!(ROUTER, "POST", "/loadstates", routerLoadStates)
    HTTP.register!(ROUTER, "POST", "/plotstates", routerPlot)
    HTTP.register!(ROUTER, "POST", "/animation", routerAnimate)

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
    msg = JSON3.write(tmp)

    HTTP.Response(200, JSON3.write(tmp))
end

function routerSimulate(req::HTTP.Request)
    message = JSON3.read(req.body)
    sim = message[:sim]
    bodies = message[:bodies]
    joints = message[:joints]

    N = WorldFrame()
    Bodies = AbstractBody[N]
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

        inertia = [
            ixx ixy ixz
            ixy iyy iyz
            ixz iyz izz
        ]

        B = Body(
            body[:name],
            mass,
            inertia,
            cm
        )
        push!(Bodies, B)
    end

    Bodies = OffsetArray(Bodies, 0:length(Bodies)-1)

    Joints = []
    for k in keys(joints)
        joint = joints[k]
        type = joint[:type]
        if type == "revolute"
            θ = eval(Meta.parse(joint[:theta]))
            ω = eval(Meta.parse(joint[:omega]))
            J = Revolute(Symbol(joint[:name]), θ, ω)
        else
            error("bad joint type provided")
            return
        end

        predecessor_name = joint[:predecessor]
        successor_name = joint[:successor]

        if predecessor_name == "base"
            predecessor = [N]
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

    sys = MultibodySystem(Symbol(sim[:name]), Bodies, Joints)

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

    println(message)
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
        rd = hcat(rd,this_rd)        
    end    
    
    D = Dict("sys"=> sys, "data"=>rd)
    HTTP.Response(200, JSON3.write(D))#JSON3.write(simdata))
end