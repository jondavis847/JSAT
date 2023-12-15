include("TimedCommand.jl")

function software!(sys)
    run_software!.(sys.software)
    return nothing
end