include("../../FYP_Optimization-main/Base_Functions.jl")
# using .Base_Functions
include("../../FYP_Optimization-main/Greens_Method.jl")
# using .Greens_Method
include("../../FYP_Optimization-main/Plotter.jl")
using Random
Random.seed!(123)

include("TDM_Functions.jl")
include("Circle_Domain.jl")


# Parameters settings
M = 8               # (user-defined) Total number of map updates 
N = 5               # (user-defined) Total number of UAVs 
R1 = 20.0            # (user-defined) Initial radius 
ΔR = 2.0              # Expanding rate: 2m/update
FOV = 80/180*π      # FOV in radians
h_min = 5           # (user-defined, replaced later) Flying altitude lower bound (exclude initialization)
h_max = 100         # Flying altitude upper bound
r_min = h_min * tan(FOV/2) # (user-defined, replaced later)
r_min = 0
r_max = h_max * tan(FOV/2) 
d_lim = 35           # (user-defined) limitations on displacement of group UAV induced from optimization 


# Define objective (COPIED)
function obj_main1(x)
    objective_circles = union(circles_pool, Base_Functions.make_circles(x))
    objective_MADS = TDM_Functions.make_MADS(objective_circles)
    my_problem = Greens_Method.Greens_Problem(objective_MADS)

    return -my_problem.area
end

# Define extreme and progressive constraints
include("TDM_Constraints.jl")
cons_ext = [cons1, cons2,cons3]
cons_prog = []

# Main loop for change of domain and coverage optimization
include("TDM_STATIC_opt.jl")


# Initialize variables
# cir_domain = Base_Functions.Circle(0,0,R1,[],[])  # Initialize the time-dependent map (TDM)
cir_domain = Circle_Domain.Domains_Problem([], R1, ΔR)

# Randomly allocate circles in the beginning
global STATIC_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max)
# TDM_Functions.show_circles(circles,cir_domain) # TEST PASSED
circles = Base_Functions.make_circles(STATIC_input)

global circles_pool =  circles   # Initialize circle pool


# r_min = 0 # (copied) for testing
# d_lim = 5 # (copied)for testing
N_iter = 100 # (use-defined) set the limit of iterations for coverage maximization
global pre_optimized_circles = circles

history_input_pb = []
history_output_pb = []
# history_TDM = []

for k in 1: M 
    # I. Initialize an array of the UAV group in this epoch
    # this_circle_group = [Base_Functions.Circle(0,0,0,[],[]) for i in 1:N]

    # II. Maximize the union of circles, get the 2D positions and radius of UAVs
    global STATIC_input = TDM_Functions.make_MADS(pre_optimized_circles)
    input_problem = Greens_Method.Greens_Problem(STATIC_input)
    # Plotter.Plot(input_problem,[30,30],"Input_$k") ## TEST PASSED

    # III. STATIC_output = TDM_STATIC_opt.optimize(STATIC_input, obj, cons_ext, cons_prog, N_iter, r_max, domain_x, domain_y)
    STATIC_output = TDM_STATIC_opt.optimize(STATIC_input, obj_main1, cons_ext, cons_prog, N_iter, r_min, r_max)
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

    if k != M
        # VI. Update the map (substitude with Class functions later on)
        # global cir_domain = Base_Functions.Circle(0, 0, R1 + k * ΔR, [], [])
        cir_domain.domain_expand()

        # VII. Update the circles group variable
        global pre_optimized_circles = post_optimized_circles
    end
end




using Plots
plotlyjs()

history_TDM = cir_domain.Domain_History
# Show the locations of circles in each epoch
for i in eachindex(history_TDM)
    string = "TDM_Circles_$i" 
    TDM_Functions.show_circles(history_output_pb[i].circles, history_TDM[i]) 
    savefig(string)
end


# Show the UAVs coverage from first epoch to the last
for i in eachindex(history_TDM)
    plot()
    string = "TDM_Coverage_$i"
    this_pool = circles_pool[1:N*i]
    this_domain = history_TDM[i]
    TDM_Functions.show_circles(this_pool, this_domain)
    savefig(string)
end


# Plotting outline with domain boundary
final_MADS = TDM_Functions.make_MADS(circles_pool)
final_pb = Greens_Method.Greens_Problem(final_MADS)
Plotter.Plot(final_pb,[70,70],"Final") ## TEST PASSED

domain_x, domain_y = Base_Functions.draw(cir_domain.Domain_History[end], 0.0, 2π)
plot!(domain_x, domain_y, aspect_ratio=1, legend=:outertopright, color = "red", linewidth = 5)

