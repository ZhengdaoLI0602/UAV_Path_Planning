include("TDM_TRAJECTORY_opt.jl")
#using Trajectory_Optimization
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix
# gravity = SVector(0,0,-3.721)                    
gravity = SVector(0,0,-9.81)                     # gravity vector
motor_dist = 0.1750                              # distance between motors
kf = 1.0                                         # motor force constant (motor force = kf*u)
km = 0.0245                                      # motor torque constant (motor torque = km*u)

# MPC optimization
hor = 3.0           # Prediction horizon length
dt = 0.1            # Time-step length per horizon
Nt = Int(hor/dt)+1  # Number of timesteps per horizon
R_D = 10.0          # Danger radius
R_C = 1.0           # Collision radius
Nm = 5              # Number of applied time-steps

# if @isdefined (history_output_pb_backup)
#     global history_input_pb = history_input_pb_backup
#     global history_output_pb = history_output_pb_backup
# end


# Obtain initial and final states for all drones (Got from Static optimization)
# x_start = TDM_TRAJECTORY_opt.MADS_to_ALTRO(MADS_input)
# x_final = TDM_TRAJECTORY_opt.MADS_to_ALTRO(MADS_output)
Xs= []
# global my_time = Vector{Vector{Vector{Any}}}(undef, length(history_input_pb)) # update_iters -> no.UAV -> times used for each optimisation 

global my_time = Array{Vector{Any}}(undef, length(history_input_pb), N) # update_iters -> no.UAV -> times used for each optimisation 
for i in 1:length(history_input_pb), j in 1:N
    my_time[i,j] = []
end

no_movement = []
for i in eachindex(history_input_pb)
    if i!=length(history_input_pb)
        this_circles = history_input_pb[i].circles
        next_circles = history_input_pb[i+1].circles
        for ind = eachindex(this_circles)
            if this_circles[ind].x == next_circles[ind].x &&
                this_circles[ind].y == next_circles[ind].y &&
                this_circles[ind].R == next_circles[ind].R
                push!(no_movement, ind)
            end
        end
    end
end

history_input_pb_backup = history_input_pb
history_output_pb_backup = history_output_pb

for i in eachindex(history_input_pb)
    deleteat!(history_input_pb[i].circles, unique(no_movement))
    deleteat!(history_output_pb[i].circles, unique(no_movement))
end

N = N - length(unique(no_movement))

global jjj = 1
for q in eachindex(history_input_pb)
    global jjj = q
# for q in 1:2
    # q = 1
    # q = 2
    println("===Epoch No. $q starts===")
    if q ==1
        global x_start = TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(history_input_pb[q])
    end
    global x_final = TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(history_output_pb[q])


    global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
    for i in 1:N
        push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i]))
    end

    global total_converge = false # Check if all MAVs have converged
    global collision = Vector{Any}(undef,N) # Vector describing collision constraints
    for i in 1:N
        collision[i] = [false,[]]
    end

    # Initialise vector for solution timesF
    # global this_my_time = my_time[q,:]
    # for i in 1:N
    #     global this_my_time[i] = []
    # end

    global countIter = 0
    # Optimize
    while total_converge == false
        global countIter += 1
        println("-----------------------------Unconverged iteration No. $countIter -----------------------------")
        global total_converge = true
        
        # Optimize if MAV has not converged to final position
        for i in 1:N
            MAV = MAVs[i]
            if TDM_TRAJECTORY_opt.converge(MAV) > 0.1
                global total_converge = false
                t = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt,Nm,collision[i])
                # push!(this_my_time[i],t)  #should be uncommented back
            end
            println("Optimization for MAV no. $i"); # for testing
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
                    a,b,c = TDM_TRAJECTORY_opt.collide(MAV1,MAV2,R_C,Nt) # Output: Boolean; uav1; uav2
                    collision[i] = [a,c] # MAV1 -> i -> b; MAV2 -> j -> c
                    collision[j] = [a,b]
                end
            end
        end
        if countIter > 100
            println("Over 100 MPC trails")
            break
        end
    end

    # Normalise state histories
    global longest = maximum([length(MAVs[i].StateHistory) for i in 1:N])
    for i in 1:N
        MAV = MAVs[i]
        size = length(MAV.StateHistory)
        if size < longest
            println("There is $i be increased in statehistory length")
            for j in 1:longest-size
                push!(MAV.StateHistory,MAV.StateHistory[end])
            end
        end
    end

    # Extract trajectories
    global X = []
    for i in 1:N  # UAV index i
        MAV = MAVs[i]
        x = zeros(Float64, (length(MAV.StateHistory),13))
        # col1-3 (position); col4-7 (quaternion); col8-10(linear velocity); col11-13(angular velocity)
        for j in 1:length(MAV.StateHistory) # trajectory optimization index j (row)
            x[j,:] = MAV.StateHistory[j]    # StateHistory content(column)
        end
        push!(X,x)
        # "x" is the trajectory for each UAV
        # "X" contains all the individual "x"
    end

    # my_time[q,:] = this_my_time 
    push!(Xs, X)

    

    junc_state=Vector{RBState}()
    # junc_state = zeros(N,13)
    for i in 1:N
        # i = 1
        # junc_state[i, 1:3] = X[i][end,:][1:3] # position
        # junc_state[i, 4:7] = X[i][end,:][4:7] # orientation
        # junc_state[i, 8:10] = X[i][end,:][8:10] # linear vel
        # junc_state[i, 11:13] = X[i][end,:][11:13] # angular vel

        # push!(junc_state, RBState(X[i][end,:][1:3], X[i][end,:][4:7], X[i][end,:][8:10], X[i][end,:][11:13]))
        push!(junc_state, RBState(X[i][end,:][1:3], UnitQuaternion(I), zeros(3), zeros(3)))
    end



    global x_start = junc_state
    # 20230812 night
    # 1. Check how to get element in RBVector
    # 2. Add the ending quaternion to the updated state vector
end







# Plot trajectories
using Plots

# Static 3D plot
p = plot(legend=:outertopright, minorgrid=true, minorgridalpha=0.25)
palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]
for j in 1:3 #eachindex(Xs)
    this_X = Xs[j]
    for i in 1:N
        this_color = palette[mod1(i,length(palette))]
        if j==1
            plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3], 
            color = this_color, markershape=:none, label="MAV $i", 
            grid = true,
            xlims=(-50,50), ylims=(-50,50), zlims=(0,100), 
            xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        else
            plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3], 
            color = this_color, markershape=:none, label=:none ,
            grid = true,
            xlims=(-50,50), ylims=(-50,50), zlims=(0,100), 
            xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
        end
    end
end

# Animation plot
# anim = @animate for i in 1:longest
#     for j in 1:N
#         if j == 1
#             plot(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="MAV $j", xlims=(-50,50), ylims=(-50,50), zlims=(-50,50), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
#         else
#             plot!(X[j][1:i,1], X[j][1:i,2],X[j][1:i,3], label="MAV $j", xlims=(-50,50), ylims=(-50,50), zlims=(-50,50), xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
#         end
#     end
# end

# gif(anim,fps=10)