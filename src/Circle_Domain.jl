module Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

export domain_area
export Domains_Problem
export process


mutable struct Domains_Problem
    Domain_History :: Vector{Circle}
    Total_epochs :: Int64

    # function Domains_Problem(Total_epochs:: Int64)
    #     # Domain_History = Vector{Circle}(undef, epochs)
    #     Domain_History = [Circle() for _ in 1:epochs]
    #     Total_epochs = epochs

    #     new(Domain_History, Total_epochs)
    # end
end

function process(history:: Domains_Problem, ini_R::Int64, rate_R::Int64)
    for i in 1: history.Total_epochs
        # history.Domain_History[i].R = rate_R * (i-1) +ini_R
        this_R = rate_R * (i-1) +ini_R
        push!(history.Domain_History, Circle(0,0, this_R,[],[]))
        println("At time $i, Domain Radius: $(history.Domain_History[i].R); Area: $(domain_area(history.Domain_History[i])) at (0, 0)")
        sleep(1)
    end
end

function domain_area(cir::Circle)
    return pi*cir.R^2
end

end

