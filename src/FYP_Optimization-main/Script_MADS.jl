include("Greens_Method.jl")
using .Greens_Method
include("Position_Optimization.jl")
using .Position_Optimization
include("Base_Functions.jl")
using .Base_Functions


# Parameters
N = 8           # number of MAV
domain_x = 50   # domain x upper boundary
domain_y = 50   # domain y upper boundary
R_lim = 5*pi/2  # upper limit of the detection radius

# MADS Parameters
N_iter = 100

# Generate circles
x_arr = Vector{Float64}(undef,0)
y_arr = Vector{Float64}(undef,0)
R_arr = Vector{Float64}(undef,0)

for i in 1:N
    x = rand(R_lim:domain_x-R_lim)[1]
    y = rand(R_lim:domain_y-R_lim)[1]
    R = rand(pi/4:R_lim)[1]

    push!(x_arr,x)
    push!(y_arr,y)
    push!(R_arr,R)
end

MADS_input = [x_arr;y_arr;R_arr]

# Define objective
function obj(x)
    my_problem = Greens_Problem(x)

    distance = sqrt(sum((MADS_input - x).^2))
    
    return -my_problem.area + distance
end

# Define extreme and progressive constraints
include("Constraints.jl")
cons_ext = [cons1,cons2]
cons_prog = []

# Position optimization
MADS_output = Position_Optimization.optimize(MADS_input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

input_problem = Greens_Problem(MADS_input)
output_problem = Greens_Problem(MADS_output)

# Print areas
println("-----------INPUT----------")
println("Greens Method: ",input_problem.area)
println("")
println("-----------OUTPUT----------")
println("Greens Method: ",output_problem.area)



# Plot input and output figures
include("Plotter.jl")
using .Plotter
using Plots

Plot(input_problem,[domain_x,domain_y],"Input")
# savefig("./FYP_Optimization-main/plots/input.pdf")
Plot(output_problem,[domain_x,domain_y],"Output")
# savefig("./FYP_Optimization-main/plots/output.pdf")





