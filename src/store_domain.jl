include("Circle_Domain.jl")
using .Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

using Plots
plotlyjs()
Plots.default(show = true)


this_history = Domains_Problem(Vector{Circle}(),10)
this_history.process(20, 2)

for i in 1: size(this_history.Domain_History,1)
    this_cir = this_history.Domain_History[i]
    plot!(draw(this_cir,0.0,2*π),legend=:outertopright, aspect_ratio=1)
end

savefig("My_Repo/src/maps.pdf")






