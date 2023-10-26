using HTTP, Sockets, JSON3
includet("..\\src\\blue42.jl")
function jsat_server()
    ROUTER = HTTP.Router()

    routerIndex(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\index.html"))
    routerCss(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\css\\jsat.css"))
    routerJs(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\js\\jsat.js"))
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

        sol = simulate(sys, eval(Meta.parse(sim[:tspan])); output_type=DataFrame)

        if (isempty(sim[:name]))
            t = lpad(string(abs(rand(Int16))), 5, "0")
            sim_name = "sim$(t)"
        else
            sim_name = sim[:name]
        end

        CSV.write("sim\\$(sim_name).csv", sol)

        HTTP.Response(200, "success")
    end

    function routerSimFiles(req::HTTP.Request) 
        tmp = []
        files = readdir("sim")
        for file in files
            simfilename = replace(file,".csv" => "")
            simfiledate = Dates.unix2datetime(mtime("sim\\$(file)"))
            D = Dict("fileName" => simfilename, "fileDate" => simfiledate)
            push!(tmp,D)
        end
        msg = JSON3.write(tmp)

        println(msg)
        HTTP.Response(200, JSON3.write(tmp))
    end

    HTTP.register!(ROUTER, "GET", "/", routerIndex)
    HTTP.register!(ROUTER, "GET", "/css/jsat.css", routerCss)
    HTTP.register!(ROUTER, "GET", "/js/jsat.js", routerJs)

    HTTP.register!(ROUTER, "POST", "/simulate", routerSimulate)
    HTTP.register!(ROUTER, "GET", "/simfiles", routerSimFiles)

    server = HTTP.serve!(ROUTER, ip"127.0.0.1", 80)
    printstyled("servers up! ctrl+click url to go : http://127.0.0.1:80", color=:light_yellow)
    return server

end

