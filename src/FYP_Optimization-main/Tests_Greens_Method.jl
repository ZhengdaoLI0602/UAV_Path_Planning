include("Greens_Method.jl")
using .Greens_Method
include("Base_Functions.jl")
using .Base_Functions

using Test
using Random


@testset "Area" begin
    # (1) Non-intersecting circles
    # Circle A centred at (-1,-1) with Radius = 1
    # Circle B centred at (1,1) with Radius = 1
    z = Vector{Float64}([-1.0,1,-1,1,1,1])

    @test Greens_Problem(z).area == 2*(pi*1*1)

    # (2) Tangent circles
    # Circle A centred at (-1,0) with Radius = 1
    # Circle B centred at (1,0) with Radius = 1
    z = Vector{Float64}([-1,1,0,0,1,1])

    @test Greens_Problem(z).area == 2*(pi*1*1)

    # (3) Intersecting circles
    # Circle A centred at (0,0) with Radius = 1
    # Circle B centred at (1,0) with Radius = 1
    z = Vector{Float64}([0,1,0,0,1,1])

    @test isapprox(Greens_Problem(z).area, (4*pi/3) + (sqrt(3)/2), atol=0.1)

    # (4) Coincident circles
    x, y, R = rand(3)
    z = Vector{Float64}([x,x,y,y,R,R])

    @test isapprox(Greens_Problem(z).area, pi*R*R)

end;

