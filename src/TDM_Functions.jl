module TDM_Functions
include("../../FYP_Optimization-main/Base_Functions.jl")
using Plots
using .Base_Functions
plotlyjs()
Plots.default(show = true)
using Random
Random.seed!(111)


export show_circles
export make_MADS
export allocate_random_circles
export Domains_Problem

function show_circles(circles, cir_domain)
    # palette = ["blue", "orange", "green", "purple", "pink", "gray", "olive", "cyan"]
    palette = ["blue", "orange", "green", "purple", "pink"]
    # number of color = number of UAV
    for i in eachindex(circles)
        this_color = palette[mod1(i,length(palette))]
        this_cir = circles[i]
        x,y = Base_Functions.draw(this_cir, 0.0, 2*π)
        if i==1
            plot(x,y, aspect_ratio=1, color = this_color, legend=:outertopright)
        else
            plot!(x,y, aspect_ratio=1, color = this_color, legend=:outertopright)
        end
    end
    
    if cir_domain !== nothing
        # Show the shape of domain
        domain_x, domain_y = Base_Functions.draw(cir_domain, 0.0, 2π)
        plot!(domain_x, domain_y, aspect_ratio=1, color="red", legend=:outertopright, linewidth = 5)
        # savefig("./circles_and_domain.pdf")
    end
end


function make_MADS(circles)::Vector{Float64}  # <-> make_circles in "Base_Functions.jl"
    x_ = []
    y_ = []
    r_ = []
    for i in 1: length(circles)
        this_cir = circles[i]
        push!(x_, this_cir.x)
        push!(y_, this_cir.y)
        push!(r_, this_cir.R)
    end
    return [x_; y_; r_]
end


function allocate_random_circles(cir_domain, N, r_max)
    this_domain = cir_domain.Domain_History[end]
    # Generate circles
    x_arr = Vector{Float64}(undef,0)
    y_arr = Vector{Float64}(undef,0)
    r_arr = Vector{Float64}(undef,0)

    for i in 1:N
        # NOTE: A * rand() -> return float values in [0, A]

        ref_dis = this_domain.R *rand()
        ref_angle = 2*π*rand()
        
        x = ref_dis * cos(ref_angle)
        y = ref_dis * sin(ref_angle)
        r = min((this_domain.R - ref_dis),r_max) * rand()
        
        push!(x_arr,x)
        push!(y_arr,y)
        push!(r_arr,r)
    end

    return [x_arr;y_arr;r_arr]

end


function allocate_even_circles(r_centering_cir::Float64, N, r_uav::Float64)
    # Generate circles
    x_arr = Vector{Float64}(undef,0)
    y_arr = Vector{Float64}(undef,0)
    r_arr = Vector{Float64}(undef,0)

    for i in 1:N
        # NOTE: A * rand() -> return float values in [0, A]

        # ref_dis = this_domain.R *rand()
        ref_angle = 2*π/N * (i-1)
        
        x = r_centering_cir * cos(ref_angle)
        y = r_centering_cir * sin(ref_angle)
        r = r_uav

        push!(x_arr,x)
        push!(y_arr,y)
        push!(r_arr,r)
    end

    return [x_arr;y_arr;r_arr]

end


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