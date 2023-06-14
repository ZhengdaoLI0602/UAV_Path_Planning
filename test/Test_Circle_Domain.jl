# Just some comments for now.
include("../src/Circle_Domain.jl")
# using .Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
# using .Base_Functions

using Test

@testset "Distance Function" begin
    A = Circle_Domain.Base_Functions.Circle(0,0,2,[],[])

    @test Circle_Domain.domain_area(A) == pi*4

end