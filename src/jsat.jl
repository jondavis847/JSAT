using ComponentArrays
#using ConcreteStructs
using CSV
using DataFrames
using Dates
using DifferentialEquations
using LinearAlgebra
using Plots
using PlotThemes
using StaticArrays
using UnPack

import Base: show
import Plots: plot, plot!
theme(:juno)

includet(joinpath("math", "quaternion.jl"))
includet(joinpath("math", "spatial.jl"))
includet(joinpath("math", "rotations.jl"))
includet(joinpath("types","abstract.jl"))
includet(joinpath("bases", "base.jl"))
includet(joinpath("bodies", "bodies.jl"))
includet(joinpath("joints", "joints.jl"))
includet(joinpath("software", "software.jl"))
includet(joinpath("actuators", "actuators.jl"))
includet(joinpath("sensors", "sensors.jl"))
includet(joinpath("types", "system.jl"))
includet(joinpath("gravity", "gravity.jl"))
includet(joinpath("environments", "environments.jl"))
includet(joinpath("dynamics", "kinematics.jl"))
includet(joinpath("dynamics", "dynamics.jl"))
includet(joinpath("dynamics", "articulated_body_algorithm.jl"))
includet(joinpath("utils", "treeutils.jl"))
includet(joinpath("simulation","initialization.jl"))
includet(joinpath("simulation","saving.jl"))
includet(joinpath("simulation","simulate.jl"))
includet(joinpath("simulation","plotting.jl"))