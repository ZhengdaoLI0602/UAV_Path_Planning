# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization

include("TDM_TRAJECTORY_opt.jl")
using StaticArrays, Rotations, LinearAlgebra
using RobotZoo: Quadrotor
using RobotDynamics
using Plots

# Drone Parameters
mass = 0.5                                       # mass of quadrotor
J = Diagonal(@SVector[0.0023, 0.0023, 0.004])    # inertia matrix                    
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




Xs= []

global my_time = Array{Vector{Any}}(undef, length(history_input_pb), N) # update_iters -> no.UAV -> times used for each optimisation 
for i in 1:length(history_input_pb), j in 1:N
    my_time[i,j] = []
end

for q in eachindex(single_input_pb)
    println("===Epoch No. $q starts===")
    if q ==1
        global x_start = TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(single_input_pb[q])
    end
    global x_final = TDM_TRAJECTORY_opt.GreensPb_to_ALTRO(single_output_pb[q])


    global MAVs = Vector{TDM_TRAJECTORY_opt.Trajectory_Problem}()
    for i in 1:N
        push!(MAVs, TDM_TRAJECTORY_opt.Trajectory_Problem(mass,J,gravity,motor_dist,kf,km,x_start[i],x_final[i]))
    end

    global total_converge = false # Check if all MAVs have converged
    global collision = Vector{Any}(undef,N) # Vector describing collision constraints
    for i in 1:N
        collision[i] = [false,[]]
    end


    global countIter = 0
    # Optimize
    while total_converge == false
        global countIter += 1
        println("-----------------------------Unconverged iteration No. $countIter -----------------------------")
        global total_converge = true
        
        # Optimize if MAV has not converged to final position
        for i in 1:N
            local MAV = MAVs[i]
            if countIter == 1
                global total_converge = false
                t = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt,Nm,collision[i])
            else
                # println("We are here")
                if TDM_TRAJECTORY_opt.converge(MAV) > 0.3
                    global total_converge = false    
                    t = TDM_TRAJECTORY_opt.optimize(MAV,hor,Nt,Nm,collision[i])
                    # push!(this_my_time[i],t)  
                end
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

    push!(Xs, X)

    junc_state = Vector{RBState}()
    for i in 1:N
        push!(junc_state, RBState(X[i][end,:][1:3], UnitQuaternion(I), zeros(3), zeros(3)))
    end

    global x_start = junc_state
end



## Plot 3D trajectories

p = plot()
palette = ["blue", "orange", "green", "purple", "cyan", "pink", "gray", "olive"]


pass_ = false
if pass_
    ## Plot 1(a): cylinder 
    using Plots
    plotlyjs()

    r = 150 + M*5
    h = h_max
    m, n =200, 200
    u = range(0, 2pi, length=m)
    v = range(0, h, length=n)
    us = ones(m)*u'
    vs = v*ones(n)'
    #Surface parameterization
    X = r*cos.(us)
    Y = r*sin.(us)
    Z = vs
    p = plot(Plots.surface(X, Y, Z, size=(600,600), 
        cbar=:none, 
        legend=false,
        colorscale="Reds",
        linewidth=0.5, 
        linealpha=0.4,
        alpha=0.4, 
        # linecolor=:red,
        camera = (45, 10), 
    ), label = "Domain")


    # plot()
    ## Plot 1(b): 3D trajectories for the all the UAVs
    for j in eachindex(Xs)                         # epoch index
        local this_X = Xs[j]
        for i in 1:N                                # UAV index
            local this_color = palette[mod1(i,length(palette))]
            if j!= M
                plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3],
                    linewidth = 3,
                    color = this_color, 
                    label=:none, 
                )
            else
                plot!(this_X[i][:,1],this_X[i][:,2],this_X[i][:,3], 
                    linewidth = 3,
                    color = this_color,
                    label="UAV $i",
                )
            end
        end
    end
    scal = r
    plot!(grid = true, gridwidth = 3, 
        legend=:outertopright,
        legendfontsize=10,
        xlims=(-scal,scal), ylims=(-scal,scal), zlims=(0,35),  
        xlabel="x [m]", xguidefontsize=14, xticks = -scal+30:40:scal-30, xtickfontsize= 10,
        ylabel="y [m]", yguidefontsize=14, yticks = -scal+30:40:scal-30,  ytickfontsize= 10,
        zlabel="z [m]", zguidefontsize=14, zticks = 0:5:35, ztickfontsize= 10, zrotation = -90,
        size=(800, 800),
        camera=(45, 30),
    )

end