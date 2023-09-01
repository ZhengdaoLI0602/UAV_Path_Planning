include("Base_Functions.jl")
using .Base_Functions

using Test
using Random
using LinearAlgebra

@testset "Distance Function" begin
    x1,y1,x2,y2 = rand(-1000.0:1000.0,4)
    R1,R2 = rand(1:1000,2)

    A = Circle(x1,y1,R1,[],[])
    B = Circle(x2,y2,R2,[],[])

    @test distance(A,B) == sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
    @test distance(x1,y1,x2,y2) == sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
end

@testset "Coincident Function" begin
    x, y, R = rand(3)
    A = Circle(x,y,R,[],[])
    B = Circle(x,y,R,[],[])
    C = Circle(x+0.1,y+0.1,R+0.1,[],[])

    @test coincident(A,B) == true
    @test coincident(A,C) == false
end

@testset "Intersection Function" begin
    A = Circle(0,0,1,[],[])
    B = Circle(1,0,1,[],[])
    C = Circle(2,0,1,[],[])
    D = Circle(3,0,1,[],[])

    @test intersection(A,A) === nothing
    @test intersection(A,B) == (0.5,-0.5*sqrt(3),0.5,+0.5*sqrt(3))
    @test intersection(A,C) === nothing
    @test intersection(A,D) === nothing
end

@testset "Contained Function" begin
    A = Circle(0,0,1,[],[])
    B = Circle(0,0,0.5,[],[])
    C = Circle(1,0,1,[],[])
    D = Circle(2,0,1,[],[])
    E = Circle(0.5,0,0.5,[],[])

    @test contained(A,B) == B

    @test contained(A,C) === nothing

    @test contained(A,D) === nothing

    @test contained(A,E) == E
end

@testset "Outside Function" begin
    A = Circle(0,0,1,[],[])
    Point1 = Point(0,1,[])
    Point2 = Point(0,0,[])
    Point3 = Point(1/sqrt(2),1/sqrt(2),[])

    @test outside(A,Point1) == true
    @test outside(A,Point2) == false
    @test outside(A,Point3) == true
end

@testset "ACW Ascending Sort Function" begin
    x,y = rand(-1000:1000,2)
    R = rand(1:1000,2)[1]

    A = Circle(x,y,R,[],[])

    N = rand(2:1000,1)[1]
    angles = LinRange(0.,2*pi,N)

    True_Points = [Point(x + R*cos(angle), y + R*sin(angle),[]) for angle in angles]

    Output_Points = shuffle(True_Points)
    sort_asc_angle!(A,Output_Points)

    @test Output_Points == True_Points
end;

@testset "Shoelace Function" begin
    square = [Point(-1,-1,[]),
              Point(-1,1,[]),
              Point(1,1,[]),
              Point(1,-1,[])]
    
    x_circle, y_circle = draw(Circle(0,0,2,[],[]), 0.0, 2*pi)
    circle = [Point(x_circle[i],y_circle[i],[]) for i in 1:length(x_circle)]

    @test shoelace(square) == 4
    @test isapprox(shoelace(circle), pi*2*2, atol=0.1)
end;

@testset "Associate Function" begin
input = []
push!(input,[[1,2,3],"A"],[[2,4,6],"B"],[[6,7,8],"C"],[[10,11,12],"D"],[[12,24,36],"E"])

output = []
push!(output,[[[1,2,3],"A"], [[2,4,6],"B"], [[6,7,8],"C"]],[[[10,11,12],"D"], [[12,24,36],"E"]])

var = associate(input)

@test var == output

end;