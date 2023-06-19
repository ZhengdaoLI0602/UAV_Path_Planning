include("Circle_Domain.jl")
using .Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions


this_history = Domains_Problem(Vector{Circle}(),10)
process(this_history, 50, 5)




