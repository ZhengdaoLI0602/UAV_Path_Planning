# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization

include("TDM_Functions.jl")
include("TDM_Trajectory_Opt.jl")

using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using TrajectoryOptimization
using Altro
using Random

# Drone Parameters
mass = 0.5                                       # mass of quadrotor # default 0.5
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix # default Diagonal(@SVector[0.0023, 0.0023, 0.004])
gravity = SVector(0,0,-9.81)                     # gravity vector # default SVector(0,0,-9.81) 
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u) # default 1.0
km = 0.0245                                      # motor torque constant (motor torque = km*u) # default 0.0245


# MPC optimization
hor = 3.0           # Prediction horizon length         (3 seconds in total)
dt = 0.1            # Time-step length per horizon      (want to separate 3 seconds into 30 pieces)
Nf = Int(hor/dt)+1  # Number of timesteps per horizon
tf = hor            # for testing on 20230809


# Enum Altro.TerminationStatus:
# UNSOLVED = 0
# LINESEARCH_FAIL = 1
# SOLVE_SUCCEEDED = 2
# MAX_ITERATIONS = 3
# MAX_ITERATIONS_OUTER = 4
# MAXIMUM_COST = 5
# STATE_LIMIT = 6
# CONTROL_LIMIT = 7
# NO_PROGRESS = 8
# COST_INCREASE = 9


include("TDM_Trajectory_Opt.jl")

critical_dis_record = []
positions_stores = []
critical_dis = 0.0
traj_s = []
global sin_phi, cos_phi, sin_theta, cos_theta = √2/√3, 1/√3, √2/2, √2/2 #sind(phi), cosd(phi), sind(theta), cosd(theta)
# global sin_phi, cos_phi, sin_theta, cos_theta = 1, 0, √2/2, √2/2 #sind(phi), cosd(phi), sind(theta), cosd(theta)
# global sin_phi, cos_phi, sin_theta, cos_theta = 0, 1, √2/2, √2/2 #sind(phi), cosd(phi), sind(theta), cosd(theta)


for ind in 1:8  # number of tests to find ̄d
    # I. Starting point
    # x_start_dBar = RBState([0.0,0.0,0.0,  1.0,0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0])

    local x_start_dBar = RBState([5.0*rand()*rand([-1 1]),5.0*rand()*rand([-1 1]),5.0*rand(),  1.0,0.0,0.0,0.0,  0.0,0.0,0.0,  0.0,0.0,0.0])
    local MAV = TDM_TRAJECTORY_opt.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start_dBar)

    # II. Propagation direction
    # Φ ∈ [0, 180], Θ ∈ [0, 360] 
    # Φ = 90*rand();  Θ = 360*rand()
    # sin_phi, cos_phi, sin_theta, cos_theta = sind(Φ) , cosd(Φ) , sind(Θ) ,cosd(Θ)


    for d in 9.0:0.1:100.0
        local X, x0, xf, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, d, sin_phi, cos_phi, sin_theta, cos_theta); 
        println(" Distance: $d; Solver status: ", solver_dBar.stats.status)
        if Int(solver_dBar.stats.status) != 2   
            global critical_dis = d-0.5 
            break
        end
    end

    global X, x0, xf, solver_dBar, prob_dBar, obj_dBar = TDM_TRAJECTORY_opt.find_d_max(MAV, hor, Nf, critical_dis, sin_phi, cos_phi, sin_theta, cos_theta); 
    println("From: $(x0[1:3]), To: $(xf[1:3])"," Distance: $critical_dis; Solver status: ", solver_dBar.stats.status)

    global positions_store = [x0[1:3]; xf[1:3]; sqrt(sum((x0[1:3]-xf[1:3]).^2)); critical_dis]
    push!(positions_stores, positions_store)

    println("The direction is $(sin_phi*cos_theta), $(sin_phi*sin_theta), $(cos_phi). The critical distance is: $critical_dis")
    push!(critical_dis_record, [SVector(MAV.StateHistory[1].r), sin_phi*cos_theta, sin_phi*sin_theta, cos_phi, critical_dis])

    # Document the data and make plots
    local traj = zeros(Nf, 3) # local in for loop
    for i in eachindex(X)
        traj[i,1] = X[i][1]
        traj[i,2] = X[i][2]
        traj[i,3] = X[i][3]
    end
    push!(traj_s, traj)

    # plot!(traj[:,1], traj[:,2], traj[:,3],  markershape=:none, xlims=(-20,20), ylims=(-20,20), zlims=(-20,20), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    using DelimitedFiles
    writedlm("Traj_$ind.csv", traj, ", ")
end


# Store the records of tests on displacement limit
using DelimitedFiles
writedlm("Positions_stored.csv", positions_stores, ", ")


using Plots
plotlyjs()
p = plot()
global palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
scal = 10
global line_num = 0
for ind in eachindex(traj_s)
    this_traj = traj_s[ind]
    this_color = palette[mod1(ind, length(palette))]
    plot!(this_traj[:,1], this_traj[:,2], this_traj[:,3],
        linewidth = 3,
        color = this_color, 
        label="Test $ind",
        xlims=(-scal,scal), ylims=(-scal,scal), zlims=(0,12),
        xlabel="x [m]", xguidefontsize=15, xticks = [-12, -8, -4, 0, 4, 8, 12], xtickfontsize= 10, 
        ylabel="y [m]", yguidefontsize=15, yticks = -12:4:12,  ytickfontsize= 10, 
        zlabel="z [m]", zguidefontsize=15, zticks = 0:4:12, ztickfontsize= 10, zrotation = 60, 
        size=(500, 550),
        camera=(-45, 30),
    )
end
plot!(grid = true, gridwidth = 3,legend=:topright, legendfontsize=12)




# Document all the calculations on dBar
a = []
for i in eachindex(critical_dis_record)
    this_record = [critical_dis_record[i][1][1:3]; critical_dis_record[i][2: end]]
    push!(a, this_record)
    using DelimitedFiles
    writedlm("dBar_record.csv", a, ", ")
end
