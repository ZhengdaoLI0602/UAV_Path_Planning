module TDM_TRAJECTORY_opt

export Trajectory_Problem
export converge
export optimize
export GreensPb_to_ALTRO
export GreensPb_to_MADS
export MADS_to_ALTRO
export collide
export find_d_max



using TrajectoryOptimization
using RobotDynamics
using RobotZoo: Quadrotor
using StaticArrays, Rotations, LinearAlgebra
using Altro
# include("../../FYP_Optimization-main/Base_Functions.jl")
# include("TDM_Functions.jl")


"""
    struct Trajectory_Problem

An object describing an MAV

Attributes:
    - 'Mass::Float64': Mass
    - 'J::Diagonal': Diagonal mass inertia matrix
    - 'Gravity::SVector': Gravity vector
    - 'Motor_Distance::Float64': Motor distance
    - 'kf::Float64': Motor force constant
    - 'km::Float64': Motor moment constant
    - 'Model::Quadrotor': Dynamics model
    - 'StateHistory::Vector{RBState}': Vector of the state history
    - 'PredictedStates::Vector{RBState}': Vector of the future predicted states
    - 'FinalState::RBState': Final intended state
"""
mutable struct Trajectory_Problem
    Mass::Float64           # CONST
    J::Diagonal             # CONST
    Gravity::SVector        # CONST
    Motor_Distance::Float64 # CONST
    kf::Float64             # CONST
    km::Float64             # CONST
    Model::Quadrotor

    StateHistory::Vector{RBState}
    PredictedStates::Vector{RBState}

    FinalState::RBState
    

    function Trajectory_Problem(Mass::Float64,J::Diagonal,Gravity::SVector,Motor_Distance::Float64,kf::Float64,km::Float64,InitialState::RBState,FinalState::RBState)
        Model = Quadrotor(mass=Mass, J=J, gravity=Gravity, motor_dist=Motor_Distance, kf=kf, km=km)

        new(Mass, J, Gravity, Motor_Distance, kf, km, Model, [InitialState], [], FinalState)
    end

    function Trajectory_Problem(Mass::Float64,J::Diagonal,Gravity::SVector,Motor_Distance::Float64,kf::Float64,km::Float64,InitialState::RBState)
        Model = Quadrotor(mass=Mass, J=J, gravity=Gravity, motor_dist=Motor_Distance, kf=kf, km=km)

        new( Mass, J, Gravity, Motor_Distance, kf, km, Model, [InitialState], [] )
    end

end



"""
    converge(MAV::Trajectory_Problem)

Returns the Euclidean distance of the MAV from its FinalState
"""
function converge(MAV::Trajectory_Problem)
    current_position = MAV.StateHistory[end][1:3]
    final_position = MAV.FinalState[1:3]
    
    distance = sqrt(sum((current_position-final_position).^2))

    return distance
end



"""
    find_d_max(MAV::Trajectory_Problem, tf::Float64, Nf::Int64)

Returns the X(predicted states), prob (problem object), obj (objective function object) of the d bar determination
"""
function find_d_max(MAV, tf::Float64, Nf::Int64, XF::Float64)
    n,m = size(MAV.Model)       # n: number of states 13; m: number of controls 4
    num_states = n

    # xf = SVector(MAV.StateHistory[end]); # however it is the given x0, 20230810
    weight_Q = 1.0 #1e-30
    weigth_R = 1.0 #1e-30
    weigth_Qf = 5.0 #5.0
    Q = Diagonal(@SVector fill(weight_Q, num_states))
    # Q = Diagonal(SA[weight_Q, weight_Q, weight_Q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    R = Diagonal(@SVector fill(weigth_R, m))
    Qf = Diagonal(@SVector fill(weigth_Qf, num_states)) #xf: 0,0,0, Qf 1,1,1
    # Qf = Diagonal(SA[weigth_Qf, weigth_Qf, weigth_Qf, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #xf: 0,0,0, Qf 1,1,1


    # x0 = [NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN];
    x0 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    xf = [XF, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];




    obj = LQRObjective(Q, R, Qf, xf, Nf)  # only about the 3D position

    cons = ConstraintList(num_states, m, Nf)
    x_min = [-50.0,-50.0,0.0,  -2.0,-2.0,-2.0,-2.0,  -5.0,-5.0,-5.0,  -1.5,-1.5,-1.5]
    x_max = [50.0,50.0,50.0,  2.0,2.0,2.0,2.0,  5.0,5.0,5.0,  1.5,1.5,1.5]
    add_constraint!(cons, BoundConstraint(n, m, x_min=x_min, x_max=x_max), 1:Nf-1)


    prob = Problem(MAV.Model, obj, x0, tf, xf=xf, constraints=cons);
    # prob = Problem(MAV.Model, obj; xf=xf, tf=tf, constraints=cons);

    # zero_SV = zeros(MAV.Model)[1]
    hover = zeros(MAV.Model)[2]
    # hover = @SVector fill(-2*MAV.Model.gravity[3]*MAV.Model.mass/4.0, size(MAV.Model)[2])

    state_guess = zeros(Float64, (num_states,Nf))
    control_guess = zeros(Float64, (m, Nf-1))
    for i in 1: Nf-1  
        state_guess[:,i] = x0 + (xf-x0)*(i-1)/(Nf-1)
        control_guess[:,i] = hover                    # 13 * number of (timesteps-1)
    end
    state_guess[:,Nf] = xf 
    initial_states!(prob, state_guess)
    initial_controls!(prob, control_guess)

    rollout!(prob);   # simulate the system forward in time with the new controls

    solver = ALTROSolver(prob);

    # opts = SolverOptions()
    # opts.cost_tolerance = 1e-5
    solver = ALTROSolver(prob,show_summary=false);

    solve!(solver);

    # status(solver);
    # println("Number of iterations: ", iterations(solver))
    # println("Final cost: ", cost(solver))
    # println("Final constraint satisfaction: ", max_violation(solver))

    X = states(solver);

    return X, solver, prob, obj
end




"""
    optimize(MAV::Trajectory_Problem, tf::Float64, Nt::Int64, collision::Vector{Any})

Performs a trajectory optimization of the MAV from StateHistory[end] (current state) to FinalState

# Arguments:
    - 'tf::Float64': Time horizon
    - 'Nt::Int64': Number of timesteps
    - 'collision::Vector{Any}': Vector of form [Boolean,[x,y,z]], where the Boolean value describes if a collision is imminent with any other MAVs at the location (x,y,z)
"""
function optimize(MAV::Trajectory_Problem, tf::Float64, Nt::Int64, Nm::Int64, collision::Vector{Any})

    x0 = SVector(MAV.StateHistory[end])  # initial 3D positions of MAV
    xf = SVector(MAV.FinalState)         # final 3D positions of MAV

    n,m = size(MAV.Model) # n: number of states; m: number of control inputs

    # Initialize cost function
    Q = Diagonal(@SVector fill(1., n))  # form n*n diagonal matrix filled with the number 1
    R = Diagonal(@SVector fill(5., m))  # form m*m diagonal matrix filled with the number 5
    H = @SMatrix zeros(m, n)  # form m*n matrix
    q = -Q*xf
    r = @SVector zeros(m) # form m*1 vector
    c = xf'*Q*xf/2

    cost = QuadraticCost(Q, R, H, q, r, c)

    objective = Objective(cost, Nt)

    # Constraints
    cons = ConstraintList(n, m, Nt)

    # # modified after 2023.08.09
    # x_min = [-50.0,-50.0,0.0,  -2.0,-2.0,-2.0,  -5.0,-5.0,-5.0,  -1.5,-1.5,-1.5]
    # x_max = [50.0,50.0,50.0,  2.0,2.0,2.0,  5.0,5.0,5.0,  1.5,1.5,1.5]

    x_min = [-100.0,-100.0,0.0,-2.0,-2.0,-2.0,-2.0,-5.0,-5.0,-5.0,-1.5,-1.5,-1.5] 
    # target limits for different variables (positions; orientations; velocity; angular velocity)
    x_max = [100.0,100.0,100.0,2.0,2.0,2.0,2.0,5.0,5.0,5.0,1.5,1.5,1.5]

    add_constraint!(cons, BoundConstraint(n,m, x_min=x_min, x_max=x_max), 1:Nt-1)

    # Add collision constraints if present
    if collision[1] == true
        x,y,z = collision[2]
        add_constraint!(cons, SphereConstraint(n, [x], [y], [z], [1.5]), 1:Nt)
    end

    # With random initial positions (with x0=x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, x0=x0, constraints=cons)
    prob = Problem(MAV.Model, objective, x0, tf, xf = xf, constraints=cons)
    # Without random initial positions (without x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, constraints=cons)
    

    # State initialization: linear trajectory from start to end
    # Control initialization: hover
    state_guess = zeros(Float64, (n,Nt))
    control_guess = zeros(Float64, (m,Nt-1))

    hover = zeros(MAV.Model)[2]

    for i in 1:Nt-1
        state_guess[:,i] = x0 + (xf-x0)*(i-1)/(Nt-1)   # assume linear interpolation
        control_guess[:,i] = hover                     # 13 * number of (timesteps-1)
    end

    state_guess[:,Nt] = xf                             # 13 * number of timesteps

    initial_states!(prob, state_guess)
    initial_controls!(prob, control_guess)
    
    altro = ALTROSolver(prob)

    solve!(altro);

    X = states(altro);

    MAV.PredictedStates = X

    for i in 2:Nm
        push!(MAV.StateHistory, X[i])
    end

    return altro.stats.tsolve
end






function GreensPb_to_ALTRO(GreensPb)
    return MADS_to_ALTRO(GreensPb_to_MADS(GreensPb))
end

function GreensPb_to_MADS(GreensPb)
    cirs = GreensPb.circles
    return TDM_Functions.make_MADS(cirs)
end

"""
    MADS_to_ALTRO(z::Vector{Float64})

Transforms our MADS data structure to the ALTRO data structure
"""
function MADS_to_ALTRO(z::Vector{Float64})
    N = Int(length(z)/3)
    FOV = 80/180*pi

    x = Vector{RBState}()
    for i in range(1,stop=N)
        x_val = z[i]
        y_val = z[N + i]
        R_val = z[N*2 + i]

        z_val = R_val/tan(FOV/2)

        push!(x, RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3))); 
        # positions(3), orientation(4), velocity(3), angular velocity(3)

        # push!(x, RBState([x_val, y_val, z_val], zeros(3), zeros(3), zeros(3))); 
        # ---- modified on 2023.08.09 -----
        # positions(3), 
        # Euler angles(3: phi (roll)|theta (pitch)|psi (yaw)), 
        # Linear velocity(3), 
        # Angular velocity(3) 
    end

    return x
end



"""
    collide(MAV1::Trajectory_Problem, MAV2::Trajectory_Problem, R_C::Float64, Nt::Int64)

Checks if MAV1 is going to collide (defined by R_C) based on their future predicted states
Returns a Boolean value describing if collision is imminent, and the (x,y,z) positions of the 2 MAVs -> to be inserted in the 'collision' vector
"""
function collide(MAV1::Trajectory_Problem, MAV2::Trajectory_Problem, R_C::Float64, Nt::Int64)
    for k in 1:Nt   
        predicted_distance = sqrt(sum((MAV1.PredictedStates[k][1:3] - MAV2.PredictedStates[k][1:3]).^2))
        if predicted_distance <= R_C
            return true, MAV1.PredictedStates[k][1:3], MAV2.PredictedStates[k][1:3]
        end
    end

    return false, [], []
end

end # module end
















