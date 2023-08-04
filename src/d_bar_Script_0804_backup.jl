# include("d_bar_Trajectory_Opt.jl")
include("TDM_Functions.jl")
include("Circle_Domain.jl")
include("d_bar_Trajectory_Opt.jl")
# Final Tasks:
# 1. Change Nt to Nf
# 2. Change into functions on different scripts
#using Trajectory_Optimization
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics

using TrajectoryOptimization
using Altro

# Drone Parameters
mass = 0.4                                       # mass of quadrotor # default 0.5
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix # default Diagonal(@SVector[0.0023, 0.0023, 0.004])
# gravity = SVector(0,0,-3.721)                    # gravity vector # default SVector(0,0,-9.81) 
gravity = SVector(0,0,-9.81)
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u) # default 1.0
km = 0.0245                                      # motor torque constant (motor torque = km*u) # default 0.0245

r_max = 100 * tand(80/2)
N = 5
cir_domain = Circle_Domain.Domains_Problem([], 20.0, 2.0)
global MADS_input = TDM_Functions.allocate_random_circles(cir_domain, N, r_max)
# Obtain initial and final states for all drones (Got from Static optimization)
x_start = Trajectory_Optimization.MADS_to_ALTRO(MADS_input) #----- testing on 0802 (with 5 UAVs in total)
# x_final = Trajectory_Optimization.MADS_to_ALTRO(MADS_output)

# for i in 1:N
# push!(MAVs,Trajectory_Optimization.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[1],x_final[1]))
# end

# MPC optimization
hor = 3.0           # Prediction horizon length         (3 seconds in total)
dt = 0.1            # Time-step length per horizon      (want to separate 3 seconds into 30 pieces)
Nt = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

ind = 1


MAVs = Vector{Trajectory_Optimization.Trajectory_Problem}() # ----- testing on just first UAV on 0802
push!(MAVs, Trajectory_Optimization.Trajectory_Problem(mass, J, gravity, motor_dist, kf, km, x_start[1]))
n, m = size(MAVs[ind].Model)
MAV = MAVs[1]


#------------------------------------------------------------------------------------------------#
# function optimize_20230801(MAV::Trajectory_Optimization.Trajectory_Problem, tf::Float64, Nt::Int64)
    # delete Nm: not necessary
    x0 = SVector(MAV.StateHistory[end])
    # xf = SVector(MAV.FinalState)


    # Initialize cost function
    # Q = Diagonal(@SVector fill(1., n))
    # R = Diagonal(@SVector fill(5., m))
    # H = @SMatrix zeros(m, n)
    # q = -Q*xf
    # r = @SVector zeros(m)
    # c = xf'*Q*xf/2

    # cost = QuadraticCost(Q, R, H, q, r, c)  # change to (x[1:3] - x0[1:3]).^2
    # objective = Objective(cost, Nt)

    # COPY from: https://docs.juliahub.com/TrajectoryOptimization/UVgeA/0.1.2/costfunctions.html ---- #
    # ------ Method 1
    # n,m = 2,1
    n,m = size(MAV.Model)       # n: number of states 13; m: number of controls 4
    final_state_ind = 3

    # n = 3 # 3D positions
    # m = size(MAV.Model)[2]

    # Q = Diagonal(0.1I,n)
    # R = Diagonal(0.1I,m)
    Q = Diagonal(@SVector fill(1.0, final_state_ind))
    # Q = Diagonal(0.0I,n)#@SVector fill(0, n))#zeros(n,n);#
    R = Diagonal(@SVector fill(1.0, m))
    # R = Diagonal(0.0I,m)#@SVector fill(0, m))#zeros(m,m);#
    Qf = Diagonal(@SVector fill(1.0, final_state_ind))
    # Qf = -Diagonal(1.0I,n)#@SVector fill(1.0, n)) #ones(n,n);#
    # xf = [π,0]
    xf = SVector(MAV.StateHistory[end]) # actually "x0"
    # xf=[1.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.];
    costfun = LQRCost(Q,R,xf)
    costfun_term = LQRCostTerminal(Qf,xf) 
    objective = Objective(costfun, costfun_term, Nt)

    # -------- Method 2
    psd_xf = xf[1:final_state_ind]  # processed xf
    H = zeros(m, final_state_ind)
    q = -Q * psd_xf 
    r = zeros(m)
    c = psd_xf' * Q * psd_xf/2

    qf = -Qf * psd_xf
    cf = psd_xf' * Qf * psd_xf/2
    costfun      = QuadraticCost(Q, R, H, q, r, c)
    # costfun_term = QuadraticCost(Qf, R*0, H, qf, r*0, cf)
    costfun_term = QuadraticCost(Qf, R*0, H, qf*0, r*0, cf)
    objective = Objective(costfun, costfun_term, Nt)
    # ---------

    # Constraints
    # cons1  ( 
    #   3D position [-10~60;-10~60;0~15]; 
    #   orientation[-2~2]; 
    #   linear velocity[-5~5]; 
    #   angular velocity[-1.5~1.5]       )
    cons = ConstraintList(n, m, Nt)
    x_min = [-10.0,-10.0,0.0,  -2.0,-2.0,-2.0,-2.0,  -5.0,-5.0,-5.0,  -1.5,-1.5,-1.5]
    x_max = [60.0,60.0,15.0,  2.0,2.0,2.0,2.0,  5.0,5.0,5.0,  1.5,1.5,1.5]
    add_constraint!(cons, BoundConstraint(n, m, x_min=x_min, x_max=x_max), 1:Nt-1)

    # cons2;  Add collision constraints if present [No need in determining the d bar]
    # if collision[1] == true
    #     x,y,z = collision[2]
    #     add_constraint!(cons, SphereConstraint(n, [x], [y], [z], [1.5]), 1:Nt)
    # end

    # With random initial positions (with x0=x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, x0=x0, constraints=cons)
    # prob = Problem(MAV.Model, objective, x0, tf, xf = xf, constraints=cons) # previous

    prob = Problem(MAV.Model, objective, xf, hor, constraints=cons) # 20230804

    # prob = Problem(MAV.Model, objective, x0=x0, tf=hor, constraints=cons)  # has parameter: tf->hor


    # Without random initial positions (without x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, constraints=cons)
    

    # State initialization: linear trajectory from start to end
    # Control initialization: hover

    state_guess = zeros(Float64, (n,Nt))
    control_guess = zeros(Float64, (m,Nt-1))

    hover = zeros(MAV.Model)[2]

    for i in 1:Nt-1
        # state_guess[:,i] = x0 + (xf-x0)*(i-1)/(Nt-1)   
        control_guess[:,i] = hover                     # 13 * number of (timesteps-1)
    end

    # state_guess[:,Nt] = xf                             # 13 * number of timesteps

    # initial_states!(prob, state_guess)
    # set_x0!(prob, x0)
    initial_controls!(prob, control_guess)
    altro = ALTROSolver(prob)
    solve!(altro);

    X = states(altro)
    MAV.PredictedStates = X

    for i in 2:Nt
        push!(MAV.StateHistory, X[i])
    end

    # return altro.stats.tsolve, cost, objective
# end

#------------------------------------------------------------------------------------------------#

my_time = []
# t, cost, obj = optimize_20230801(MAVs[ind],hor,Nt)
push!(my_time,t)

d_bar = sqrt(sum((MAVs[ind].StateHistory[1][1:3] - MAVs[ind].StateHistory[end][1:3]).^2))

#------------------------------------------------------------------------------------------------# END















total_converge = false # Check if all MAVs have converged
collision = Vector{Any}(undef,N) # Vector describing collision constraints
for i in 1:N
    collision[i] = [false,[]]
end

# Initialise vector for solution timesF
my_time = Vector{Vector{Any}}(undef,N)
for i in 1:N
    my_time[i] = []
end


# include("Trajectory_Optimization.jl")
# t, cost, objt = Trajectory_Optimization.optimize(MAVs[1],hor,Nt,Nm,collision[1])


# Optimize
while total_converge == false
    global total_converge = true
    
    # Optimize if MAV has not converged to final position
    for i in 1:N
        MAV = MAVs[i]
        if Trajectory_Optimization.converge(MAV) > 0.1
            global total_converge = false
            t = Trajectory_Optimization.optimize(MAV,hor,Nt,Nm,collision[i])
            push!(my_time[i],t)
        end
    end

    # Check for collision
    for i in 1:N
        global collision[i] = [false,[]]
    end
    for i in 1:N
        for j in i+1:N
            MAV1 = MAVs[i]
            MAV2 = MAVs[j]
            distance = sqrt(sum((MAV1.StateHistory[end][1:3] - MAV2.StateHistory[end][1:3]).^2))
            if distance <= R_D
                a,b,c = Trajectory_Optimization.collide(MAV1,MAV2,R_C,Nt) # Output: Boolean; uav1; uav2
                collision[i] = [a,c] # MAV1 -> i -> b; MAV2 -> j -> c
                collision[j] = [a,b]
            end
        end
    end
end

# Normalise state histories
longest = maximum([length(MAVs[i].StateHistory) for i in 1:N])
for i in 1:N
    MAV = MAVs[i]
    size = length(MAV.StateHistory)
    if size < longest
        for j in 1:longest-size
            push!(MAV.StateHistory,MAV.StateHistory[end])
        end
    end
end

# Extract trajectories
X = []
for i in 1:N  # UAV index i
    MAV = MAVs[i]
    x = zeros(Float64, (length(MAV.StateHistory),13))
    for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
        x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
    end
    push!(X,x)
end









# Plot trajectories
using Plots

# Static 3D plot
p = plot(legend=:outertopright, minorgrid=true, minorgridalpha=0.25)
for i in 1:N
    plot!(X[i][:,1],X[i][:,2],X[i][:,3],  markershape=:none, label="MAV $i", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
end

# Animation plot
anim = @animate for i in 1:longest
    for j in 1:N
        if j == 1
            plot(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="MAV $j", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        else
            plot!(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="MAV $j", xlims=(0,50), ylims=(0,50), zlims=(0,15), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        end
    end
end

gif(anim,fps=10)