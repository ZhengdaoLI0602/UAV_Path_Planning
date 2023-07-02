module Circle_Domain

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

# export domain_area
export Domains_Problem
# export process


mutable struct Domains_Problem
    Domain_History :: Vector{Circle}
    Total_epochs :: Int64
    process
    domain_area

    function Domains_Problem(Domain_History, Total_epochs :: Int64)
        this = new()

        this.Domain_History = Domain_History

        this.Total_epochs = Total_epochs

        this.process = function (ini_R::Int64, rate_R::Int64)
            for i in 1: this.Total_epochs
                this_R = rate_R * (i-1) +ini_R
                push!(this.Domain_History, Circle(0,0, this_R,[],[]))
                println("At time $i, Domain Radius: $(this.Domain_History[i].R); Area: $(this.domain_area(this.Domain_History[i])) at (0, 0)")
                sleep(1)
            end
        end

        this.domain_area = function (cir::Circle)
            return pi*cir.R^2
        end

        return this
    end

end

end

