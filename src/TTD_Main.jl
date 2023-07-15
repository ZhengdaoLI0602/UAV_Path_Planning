include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions
include("../../FYP_Optimization-main/Greens_Method.jl")
using .Greens_Method
using Random
Random.seed!(123)


# Parameters settings
M = 10              # (user-defined) Total number of map updates 
N = 5               # (user-defined) Total number of UAVs 
R1 = 20             # (user-defined) Initial radius 
ΔR = 2              # Expanding rate: 2m/update
FOV = 80/180*π      # FOV in radians
h_min = 5           # (user-defined) Flying altitude lower bound (exclude initialization)
h_max = 100         # Flying altitude upper bound
r_min = h_min * tan(FOV/2)
r_max = h_max * tan(FOV/2) 


# Initialize variables
circles_pool = []   # Initialize circle pool


cir_domain = Base_Functions.Circle(0,0,R1,[],[])

# Generate circles


x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
r_arr = Vector{Float64}(undef,0)

for i in 1:N
    # NOTE: A * rand() -> return float values in [0, A]

    ref_dis = cir_domain.R *rand()
    ref_angle = 2*π*rand()
    
    x = ref_dis * cos(ref_angle)
    y = ref_dis * sin(ref_angle)
    r = min((cir_domain.R - ref_dis),r_max) * rand()
    
    push!(x_arr,x)
    push!(y_arr,y)
    push!(r_arr,r)
end

STATIC_input = [x_arr;y_arr;r_arr]
circles = Base_Functions.make_circles(STATIC_input)

# Define a function convenient for showing the circles
using Plots
plotlyjs()
Plots.default(show = true)
function show_circles(circles, cir_domain)
    for i in 1: length(circles)
        this_cir = circles[i]
        x,y = draw(this_cir, 0.0, 2*π)
        if i==1
            plot(x,y, aspect_ratio=1, legend=:outertopright)
        else
            plot!(x,y, aspect_ratio=1, legend=:outertopright)
        end
    end

    # Show the shape of domain
    domain_x, domain_y = draw(cir_domain,0.0, 2π)
    plot!(domain_x, domain_y, aspect_ratio=1, legend=:outertopright)
    # savefig("./circles_and_domain.pdf")
end

show_circles(circles, cir_domain) # show the randomly-allocated results

# circles1 = Greens_Method.checkCoincident!(circles)
# circles2 = Greens_Method.checkContained!(circles1)
# show_circles(circles2, cir_domain) # show the non-duplicate and non-coincident results

circles_pool = []
circles_pool = union(circles_pool, circles)


# # Define objective (COPIED)
function obj(x)
    my_problem = Greens_Problem(x)
    # distance = sqrt(sum((MADS_input - x).^2))
    return -my_problem.area
end

function make_MADS(circles)  # <-> make_circles in "Base_Functions.jl"
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

# # (use-defined) set the limit of iterations for coverage maximization
N_iter = 100

# # Define extreme and progressive constraints
include("TTD_Constraints.jl")
cons_ext = [cons1]
cons_prog = [cons2, cons3]

include("TTD_STATIC_opt.jl")
# Main loop for change of domain and coverage optimization


pre_optimized_circles = circles_pool
for k in 1: M
    # Initialize an array of the UAV group in this epoch
    this_circle_group = [Base_Functions.Circle(0,0,0,[],[]) for i in 1:N]

    # Maximize the union of circles, get the 2D positions and radius of UAVs
    # (*MADS_input, *obj, 
    # *cons_ext(1), 
    # *cons_prog(2), cons2<-(circles_pool, this_circle_group), ?con3<-(post_optimized_circles, pre_optimized_circles)
    # #N_iter, 
    # #R_lim, 
    # ?domain_x, 
    # ?domain_y)
    STATIC_input = make_MADS(pre_optimized_circles)
    input_problem = Greens_Method.Greens_Problem(STATIC_input)

    STATIC_output = TTD_STATIC_opt.optimize(STATIC_input, obj, cons_ext, cons_prog, N_iter, r_max, domain_x, domain_y)
    output_problem = Greens_Method.Greens_Problem(STATIC_output)
    

    # Exclude coincident and contained circles
    # this_circle_group1 = Greens_Method.checkCoincident!(this_circle_group)
    # this_circle_group2 = Greens_Method.checkCoincident!(this_circle_group1)

    
    # Update circle pools
    post_optimized_circles = Base_Functions.make_circles(STATIC_output)
    circles_pool = union(circles_pool, post_optimized_circles)

    # Update the map (substitude with Class functions later on)
    cir_domain = Base_Functions.Circle(0, 0, R1 + k*ΔR, [], [])

    pre_optimized_circles = post_optimized_circles
end


