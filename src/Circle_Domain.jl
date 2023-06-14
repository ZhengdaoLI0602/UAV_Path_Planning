module Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

export domain_area
export Domains_Problem
export process


# Domain_R_min = 50
# cir_domain = Circle(0,0,Domain_R_min,[],[])


# mutable struct Domains_Throughout_Time
#     Domain_history :: Vector{Circle}
#     function Domains_Throughout_Time(total_epochs::Int64)
#         Domain_history = Vector{Circle}(undef, total_epochs)
#         new(Domain_history)
#     end
# end

mutable struct Domains_Problem
    Domain_History :: Vector{Circle}
    Total_epochs::Int64

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
        println("At time $i, Domain Radius: $(history.Domain_History[i].R) at (0, 0)")
        sleep(1)
    end
end

function domain_area(cir::Circle)
    return pi*cir.R^2
end

# for i in 1:10
#     cir_domain.x += 5
#     cir_domain.R += 10
#     println("At time $i,Circle domain has radius of $(cir_domain.R) at ($(cir_domain.x), $(cir_domain.y))")
#     sleep(1) # wait for 1 second before computing the next area
# end

# elapsed_time = @elapsed my_func()
# println("Elapsed time: $elapsed_time seconds")
end


