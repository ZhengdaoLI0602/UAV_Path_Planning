# "FYP_Optimization-main" oriented from Logan's codes: https://github.com/Logan1904/FYP_Optimization

include("FYP_Optimization-main/Base_Functions.jl")
include("FYP_Optimization-main/Greens_Method.jl")
include("FYP_Optimization-main/Plotter.jl")
using Random
using Plots
plotlyjs()


include("TDM_Functions.jl")
include("TDM_STATIC_opt.jl")

# Parameters settings
M = 10               # (user-defined) Total number of map updates
N = 5                # (user-defined) Total number of UAVs 
R1 = 150.0           # (user-defined) Initial radius 
ΔR = 5.0             # Expanding rate: 5m/update
FOV = 80/180*π       # FOV in radians
h_min = 1            # (user-defined) Flying altitude lower bound (exclude initialization)
h_max = 30           # (user-defined) Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
r_max = h_max * tan(FOV/2) 
d_lim = critical_dis       # (user-defined) limitations on displacement of group UAV induced from optimization 
N_iter = 100         # (use-defined) set the limit of iterations for coverage maximization

# Define objective function in step 2
function obj_main1(x)
    objective_circles = union(circles_pool, Base_Functions.make_circles(x))
    objective_MADS = TDM_Functions.make_MADS(objective_circles)
    my_problem = Greens_Method.Greens_Problem(objective_MADS)
    return -my_problem.area
end

# Define extreme and progressive constraints
include("TDM_Constraints.jl")
cons_ext = [cons1, cons2, cons3]
cons_prog = []

# Main loop for change of domain and coverage optimization
cir_domain = TDM_Functions.Domains_Problem([], R1, ΔR)   # Initialize the time-dependent map (TDM)

# Allocate circles in the beginning
global STATIC_input = TDM_Functions.allocate_even_circles(5.0, N, 5.0)               # Even initialization
# global STATIC_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max)  # Random initialization

# random five circles (example in the report)
# global STATIC_input = [-0.39202819916964404,34.872393940656,5.5239352511433015,-29.73691435996299,-22.73383564724028,
#                     3.113693723318892,10.794042832857249,-2.3783477662587384,27.63667034017559,-1.1279665973334239,
#                     5.025207348818531,4.676120462178847,8.538346582322927,1.6090792603854056,2.0937907421476356]

# random two circles (example in the report)
# abc = TDM_Functions.make_MADS(ini_circles)
# global STATIC_input = [ 9.127587405538227, 16.601464822611025 , 
#                         3.6274369269999824, -18.875662873621483, 
#                         2.919235905281591, 1.9936226689791061]

# # random eight circles (example in the report)
# global STATIC_input = [9.419326534638106, -14.069976031679154, 0.5496529806736414, -1.8435110796149294, -10.803563459691215,18.465026332348856,24.87086813236132,-21.105129214395415,
#                        5.343328346547971, 20.873790896855237, 0.3887417746181918, -25.105394616237355, 21.688213923103323, -17.10912547394886,-3.888353157875017,13.720528152419483,
#                        9.664363626984361, 7.982352667811984, 10.18724049935073, 1.3871589000030617 ,9.907355059342985, 4.814607976990199, 9.90862169756836, 5.027266465645969
#                     ]


# Obtain and show initial set of circles 
ini_circles = Base_Functions.make_circles(STATIC_input) 
plot()
TDM_Functions.show_epoch(ini_circles, cir_domain.Domain_History[1]) 


global circles_pool = ini_circles          # Initialize circle pool
global pre_optimized_circles = ini_circles # Initialize pre-optimized circles
history_input_pb = []                       # document the circles in all historical epochs
history_output_pb = []
single_input_pb = []                        # document the circles at each epoch
single_output_pb = []

for k in 1: M   # epoch index k∈[1,M]
    new_inputs =[]

    # I. Set up the input at each epoch
    global STATIC_input = TDM_Functions.make_MADS(pre_optimized_circles)
    single_input = Greens_Method.Greens_Problem(STATIC_input)   # document current pre-opt

    if k == 1
        global updated_MADS = TDM_Functions.make_MADS(ini_circles)
    end

    input_problem = Greens_Method.Greens_Problem(updated_MADS)
    # Plotter.Plot(input_problem,[30,30],"Input_$k") ## TEST PASSED

    # II. Main optimization function
    global STATIC_output = TDM_STATIC_opt.optimize(STATIC_input, obj_main1, cons_ext, cons_prog, N_iter, r_min, r_max, d_lim, new_inputs, k,  circles_pool)
    if STATIC_output == false
        println("The problem cannot be well solved s.t. all constraints")
        break
    end

    single_output = Greens_Method.Greens_Problem(STATIC_output) # document current post-opt

    # III. Update circle pools
    post_optimized_circles = Base_Functions.make_circles(STATIC_output)
    global circles_pool = union(circles_pool, post_optimized_circles)

    updated_MADS = TDM_Functions.make_MADS(circles_pool)
    output_problem = Greens_Method.Greens_Problem(updated_MADS)

    # IV. Store the information in each epoch
    push!(history_input_pb, input_problem)
    push!(history_output_pb, output_problem)

    push!(single_input_pb, single_input)
    push!(single_output_pb, single_output)

    # V. Update the map 
    cir_domain.domain_expand()
    if k != M
        # VI. Update the circles group variable
        global pre_optimized_circles = post_optimized_circles
    end
end




global STATIC_output = false

if STATIC_output == true
    # using Plots
    # plotlyjs()

    history_TDM = cir_domain.Domain_History


    ## Plot 1: Show the initial circles
    include("TDM_Functions.jl")
    plot()
    TDM_Functions.show_epoch(ini_circles, cir_domain.Domain_History[1])
    plot!(
        legend=:topright,
        legendfontsize=12,
        # legend=:none,
        size=(600, 650),
        xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
        ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
        xlims = (-205, 205),
        ylims = (-205, 205),
    )

    # # for Zoom-in figure
    # plot()
    # TDM_Functions.show_epoch(ini_circles, nothing)
    # plot!(
    #     # legend=:topright,
    #     # legendfontsize=12,
    #     legend=:none,
    #     size=(600, 650),
    #     xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
    #     ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
    #     xlims = (-50, 50),
    #     ylims = (-35, 50),
    # )


    ## Plot 2: Show Coverage from start to end with the domain expansion
    # include("TDM_Functions.jl")
    plot()
    # store the trajectory of center
    all_center_x=[]
    all_center_y=[]
    for j in 1:N   
        xs_circle = []
        ys_circle = []
        for i in eachindex(single_input_pb)  
            circle_in = single_input_pb[i].circles[j]
            push!(xs_circle, circle_in.x)
            push!(ys_circle, circle_in.y)
            if i== M
                circle_out = single_output_pb[i].circles[j]
                push!(xs_circle, circle_out.x)
                push!(ys_circle, circle_out.y)
            end
        end
        push!(all_center_x, xs_circle)
        push!(all_center_y, ys_circle)
    end
    global palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
    for i in 1:N
        # palette = ["blue", "orange", "green", "purple", "cyan"]
        this_color = palette[mod1(i,length(palette))]
        plot!(all_center_x[i][1:end], all_center_y[i][1:end], 
            aspect_ratio=1, color = this_color,
            markershape =:cross, 
            markersize =:1,
            markerstrokestyle = :solid,
            label =:none, 
            legend =:none)
    end
    for rnd in eachindex(history_TDM)
        this_domain = history_TDM[rnd]
        TDM_Functions.show_coverage(circles_pool, this_domain, rnd, M, N)
    end
    plot!(
        legend=:topright,
        legendfontsize=11,
        size=(600, 650),
        xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
        ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
        xlims = (-205, 205),
        ylims = (-205, 205),
    )



    ## Plot 3: Show boundaries
    include("FYP_Optimization-main/Plotter.jl")
    # Plotting outline with domain boundary
    final_MADS = TDM_Functions.make_MADS(circles_pool)
    final_pb = Greens_Method.Greens_Problem(final_MADS)
    Plotter.Plot(final_pb,[70,70],"Final") ## TEST PASSED

    domain_x, domain_y = Base_Functions.draw(cir_domain.Domain_History[end], 0.0, 2π)
    plot!(domain_x, domain_y, aspect_ratio=1, label="Domain", 
          color = "red", linewidth = 5)
    plot!(legend=:topright,
        legendfontsize=12,
        size=(600, 650),
        xlabel="x [m]", xguidefontsize=15, xtickfontsize= 10, 
        ylabel="y [m]", yguidefontsize=15, ytickfontsize= 10, 
        # xlims = (-155, 180),
        # ylims = (-155, 180),
        xlims = (-205, 205),
        ylims = (-205, 205),
    )


    # Calculate the covered area
    for i in eachindex(history_input_pb)
        println("Epoch$i -- Domain area: $(π*(history_TDM[i].R)^2); 
                Initially UAVs cover: $(history_input_pb[i].area); 
                Finally UAVs cover: $(history_output_pb[i].area)")
    end


    ## Plot 4: Change in area line graph
    areas_uav = []
    areas_domain = []
    for i in 1:M
        if i ==1
            push!(areas_uav, history_input_pb[i].area)
            push!(areas_domain, π*(history_TDM[i].R)^2)
        end
        push!(areas_uav, history_output_pb[i].area)
        push!(areas_domain, π*(history_TDM[i].R)^2)
    end


    #store all the curves the variabel "areas"
    if !@isdefined(areas) || length(areas)==0
        global areas = []
        push!(areas, areas_domain)
    end
    push!(areas, areas_uav)


    ## Showcase the line graphs 
    plot()
    plot!(areas[6], lw = 1.5, label = "Evenly-distributed 2 UAVs",
            markershape =:diamond, 
            markersize =:3,
            color = "blue")
    plot!(areas[7], lw = 1.5, label = "Randomly-distributed 2 UAVs",
            markershape =:utriangle, 
            markersize =:3,
            linestyle = :dash,
            linealpha=0.5, fillalpha=0.5,
            color = "blue")
    plot!(areas[4], lw = 1.5, label = "Evenly-distributed 5 UAVs",
            markershape =:diamond, 
            markersize =:3,
            color = "orange")
    plot!(areas[5], lw = 1.5, label = "Randomly-distributed 5 UAVs",
            markershape =:utriangle, 
            markersize =:3,
            linestyle = :dash,
            linealpha=0.5, fillalpha=0.5,
            color = "orange")
    plot!(areas[3], lw = 1.5, label = "Evenly-distributed 8 UAVs",
            markershape =:diamond, 
            markersize =:3,
            markerstrokestyle = :solid,
            color = "green")
    plot!(areas[2], lw = 1.5, label = "Randomly-distributed 8 UAVs",
            markershape =:utriangle, 
            markersize =:3,
            linestyle = :dash,
            markerstrokestyle = :dash,
            color = "green")

    plot()
    for i in 1:10
        if i == 10
            lb = "Domain"
        else
            lb =:none
        end
        plot!(i:i+1,[areas[1][i+1], areas[1][i+1]],
            lw = 1.5, label = lb,
            markershape =:circle, 
            markersize =:3,
            color = "red")
    end
    plot!(
        # legend=:outertopright,
        legend=:top,
        legendfontsize=12,   #(2:2:10, ["two", "four", "six", "eight", "ten"])
        size=(500, 450),
        xlims=(0.5,11.5),
        # ylims=(0,5.5e4),
        xlabel="Time [epoch]", xguidefontsize=15, xtickfontsize= 10, 
            xticks = (1:1:11,["0","1","2","3","4","5","6","7","8","9","10"]),
        ylabel="Area [meter square]", yguidefontsize=15, ytickfontsize= 10, 
        # yticks = -0.5e3: 2e3,
        # size=(600, 650),
    )
end
