include("../../FYP_Optimization-main/Base_Functions.jl")
# using .Base_Functions
include("../../FYP_Optimization-main/Greens_Method.jl")
# using .Greens_Method
include("../../FYP_Optimization-main/Plotter.jl")
using Random
# Random.seed!(123)

include("TDM_Functions.jl")


# Parameters settings
M = 10               # (user-defined) Total number of map updates 
N = 5               # (user-defined) Total number of UAVs 
R1 = 100.0          # (user-defined) Initial radius 
ΔR = 5.0            # Expanding rate: 2m/update
FOV = 80/180*π      # FOV in radians
h_min = 1           # (user-defined, replaced later) Flying altitude lower bound (exclude initialization)
h_max = 20          # Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
# r_min = 0           # (user-defined, replaced later)
r_max = h_max * tan(FOV/2) 
d_lim = 8.6 #critical_dis           # (user-defined) limitations on displacement of group UAV induced from optimization 
N_iter = 100        # (use-defined) set the limit of iterations for coverage maximization

# Define objective (COPIED)
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
include("TDM_STATIC_opt.jl")


# Initialize variables
# cir_domain = Base_Functions.Circle(0,0,R1,[],[])  # Initialize the time-dependent map (TDM)
cir_domain = TDM_Functions.Domains_Problem([], R1, ΔR)

# Randomly allocate circles in the beginning
# global STATIC_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max) #20230815 

global STATIC_input = TDM_Functions.allocate_even_circles(5.0, N, 5.0) #20230815 
# global STATIC_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max)
# random five circles
# global STATIC_input = [-0.39202819916964404,34.872393940656,5.5239352511433015,-29.73691435996299,-22.73383564724028,
#                     3.113693723318892,10.794042832857249,-2.3783477662587384,27.63667034017559,-1.1279665973334239,
#                     5.025207348818531,4.676120462178847,8.538346582322927,1.6090792603854056,2.0937907421476356]




ini_circles = Base_Functions.make_circles(STATIC_input) 
using Plots
plotlyjs()
plot()
TDM_Functions.show_epoch(ini_circles, cir_domain.Domain_History[1]) #20230815 
# title!("Evenly-distributed $(N) circles", titlefontsize=12)
# xlabel!("x (m)")
# ylabel!("y (m)")
# annotate!([(0, 25, text("Custom Title Position", :center, 16))])

# savefig("E:/IRP/myplot.png")


global circles_pool = ini_circles   # Initialize circle pool
global pre_optimized_circles = ini_circles

history_input_pb = []
history_output_pb = []
# history_TDM = []

for k in 1: M 
    new_inputs =[]
    # I. Initialize an array of the UAV group in this epoch
    # this_circle_group = [Base_Functions.Circle(0,0,0,[],[]) for i in 1:N]

    # II. Maximize the union of circles, get the 2D positions and radius of UAVs
    global STATIC_input = TDM_Functions.make_MADS(pre_optimized_circles)
    # for i in 1:N
    #     global STATIC_input[end-i+1] = STATIC_input[end-i+1]+0.1
    # end
    input_problem = Greens_Method.Greens_Problem(STATIC_input)
    # Plotter.Plot(input_problem,[30,30],"Input_$k") ## TEST PASSED

    # III. STATIC_output = TDM_STATIC_opt.optimize(STATIC_input, obj, cons_ext, cons_prog, N_iter, r_max, domain_x, domain_y)
    global STATIC_output = TDM_STATIC_opt.optimize(STATIC_input, obj_main1, cons_ext, cons_prog, N_iter, r_min, r_max, d_lim, new_inputs, k,  circles_pool)
    if STATIC_output == false
        println("The problem cannot be well solved s.t. all constraints")
        break
    end
    output_problem = Greens_Method.Greens_Problem(STATIC_output)
    # Plotter.Plot(output_problem,[30,30],"Output_$k") ## TEST PASSED

    # IV. Update circle pools
    post_optimized_circles = Base_Functions.make_circles(STATIC_output)
    global circles_pool = union(circles_pool, post_optimized_circles)
    # TDM_Functions.show_circles(circles_pool, cir_domain) # (optional) depict the circles configuration

    # V. Store the information in each epoch
    push!(history_input_pb, input_problem)
    push!(history_output_pb, output_problem)
    # push!(history_TDM, cir_domain)

    # TDM_Functions.show_epoch(post_optimized_circles, nothing) # for testing


    if k != M
        # VI. Update the map (substitude with Class functions later on)
        # global cir_domain = Base_Functions.Circle(0, 0, R1 + k * ΔR, [], [])
        cir_domain.domain_expand()

        # VII. Update the circles group variable
        global pre_optimized_circles = post_optimized_circles
    end
end



if STATIC_output!= false
    using Plots
    plotlyjs()

    history_TDM = cir_domain.Domain_History
    # Show the locations of circles in each epoch
    # include("TDM_Functions.jl")

    # for rnd in eachindex(history_TDM)
    #     plot()
    #     string = "TDM_Circles_$rnd" 
    #     show_domain = true
    #     show_coverage = false
    #     TDM_Functions.show_epoch(history_output_pb[rnd].circles, history_TDM[rnd]) 
    #     savefig(string)
    # end


    # Show the UAVs coverage from first epoch to the last
    include("TDM_Functions.jl")
    plot()
    # store the trajectory of center
    all_center_x=[]
    all_center_y=[]
    for j in 1:N
        xs_circle = []
        ys_circle = []
        for i in eachindex(history_input_pb)
            circle_in = history_input_pb[i].circles[j]
            push!(xs_circle, circle_in.x)
            push!(ys_circle, circle_in.y)
            if i== M
                circle_out = history_output_pb[i].circles[j]
                push!(xs_circle, circle_out.x)
                push!(ys_circle, circle_out.y)
            end
        end
        push!(all_center_x, xs_circle)
        push!(all_center_y, ys_circle)
    end
    palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
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
        string = "TDM_Coverage_$rnd"
        # this_pool = circles_pool[1:N*rnd]
        this_pool = circles_pool
        
        this_domain = history_TDM[rnd]
        TDM_Functions.show_coverage(this_pool, this_domain, rnd, M, N)
        # savefig(string)

        if rnd == M
            this_domain = history_TDM[rnd]
            # this_pool = circles_pool[1:N*(rnd+1)]
            TDM_Functions.show_coverage(this_pool, this_domain, rnd+1, M, N)
        else
            # show_domain = false
        end
    end




    include("../../FYP_Optimization-main/Plotter.jl")
    # Plotting outline with domain boundary
    final_MADS = TDM_Functions.make_MADS(circles_pool)
    final_pb = Greens_Method.Greens_Problem(final_MADS)
    Plotter.Plot(final_pb,[70,70],"Final") ## TEST PASSED

    domain_x, domain_y = Base_Functions.draw(cir_domain.Domain_History[end], 0.0, 2π)
    plot!(domain_x, domain_y, aspect_ratio=1, 
          legend=:outertopright, label="Domain", 
          color = "red", linewidth = 5)


    # Calculate the covered area
    for i in eachindex(history_input_pb)
        println("Epoch$i -- Domain area: $(π*(history_TDM[i].R)^2); Initially UAVs cover: $(history_input_pb[i].area); Finally UAVs cover: $(history_output_pb[i].area)")
    end
end



# using Plots
# plotlyjs()

# x = 1:1.0:10
# y = rand(10)

# plot(x, y, linealpha=1, fillalpha=0.2, label="Line with 50% Transparency")
# plot!(x, x.^2, linealpha=0.5, fillalpha=0.2, label="Line with 50% Transparency")
# plot!(x, x.^3, linealpha=0.1, fillalpha=0.2, label="Line with 50% Transparency")

# scatter!(x, y, markerstrokewidth=0, alpha=0.8, label="Markers with 80% Transparency")