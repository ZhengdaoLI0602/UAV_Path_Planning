module TestFunctions
export Circle
export draw
using LinearAlgebra
using Random

mutable struct Circle
    x::Float64
    y::Float64
    R::Float64
    Points::Vector{Int}
    Circles::Vector{Int}
end

function draw(A,theta1::Float64,theta2::Float64)
    if theta1 > theta2
        theta2 = theta2 + 2*pi
    end
    arr = LinRange(theta1,theta2,101)
    return A.x .+ A.R*cos.(arr), A.y .+ A.R*sin.(arr)
end

end


using .TestFunctions
a = Circle(0,0,5,[],[])
outp= draw(a, π/4, π/2)

using Plots
using LaTeXStrings
plotlyjs()
plot(outp[1], outp[2], 
    aspect_ratio=1,
    legend=:outertopright)

title!(L"Trigonometric functions 2πx^2")
title!(L"Trigonometric \\pi functions x_{2π}")


