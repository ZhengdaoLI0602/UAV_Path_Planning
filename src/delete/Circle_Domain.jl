module Circle_Domain
include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

# export domain_area
export Domains_Problem
# export process


mutable struct Domains_Problem
    Domain_History :: Vector{Circle}
    Radius :: Float64
    Expand_rate :: Float64

    domain_expand
    domain_area

    function Domains_Problem(Domain_History, Radius::Float64, Expand_rate::Float64)
        this = new()

        this.Domain_History = Domain_History

        this.Radius = Radius
        push!(this.Domain_History, Circle(0,0, this.Radius,[],[]))

        # this.Total_epochs = Total_epochs
        this.Expand_rate = Expand_rate

        this.domain_expand = function ()
            # for i in 1: this.Total_epochs
            #     this_R = rate_R * (i-1) + ini_R
            #     push!(this.Domain_History, Circle(0,0, this_R,[],[]))
            #     println("At time $i, Domain Radius: $(this.Domain_History[i].R); Area: $(this.domain_area(this.Domain_History[i])) at (0, 0)")
            #     sleep(1)
            # end
            this.Radius = this.Radius + this.Expand_rate
            push!(this.Domain_History, Circle(0,0, this.Radius,[],[]))
        end

        this.domain_area = function (cir::Circle)
            return pi*cir.R^2
        end

        return this
    end

end

end


# a = Circle_Domain.Domains_Problem([],20.0,2.0)
# a.domain_expand()