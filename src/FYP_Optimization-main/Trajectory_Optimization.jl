module Trajectory_Optimization

export Trajectory_Problem
export optimize
export optimize_20230802

using TrajectoryOptimization
using RobotDynamics
using RobotZoo: Quadrotor
using StaticArrays, Rotations, LinearAlgebra
using Altro

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

end

"""
    converge(MAV::Trajectory_Problem)

Returns the Euclidean distance of the MAV from its FinalState
"""
function converge(MAV::Trajectory_Problem)
    current_position = MAV.StateHistory[end][1:3]
    final_position = MAV.FinalState[1:3]
    
    distance = sqrt(sum((current_position - final_position).^2))

    return distance
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

    x0 = SVector(MAV.StateHistory[end])
    xf = SVector(MAV.FinalState)

    n,m = size(MAV.Model)

    # Initialize cost function
    Q = Diagonal(@SVector fill(1., n))
    R = Diagonal(@SVector fill(5., m))
    H = @SMatrix zeros(m, n)
    q = -Q*xf
    r = @SVector zeros(m)
    c = xf'*Q*xf/2

    cost = QuadraticCost(Q, R, H, q, r, c)

    objective = Objective(cost, Nt)

    # Constraints
    # cons1
    cons = ConstraintList(n, m, Nt)
    x_min = [-10.0,-10.0,0.0,-2.0,-2.0,-2.0,-2.0,-5.0,-5.0,-5.0,-1.5,-1.5,-1.5]
    x_max = [60.0,60.0,15.0,2.0,2.0,2.0,2.0,5.0,5.0,5.0,1.5,1.5,1.5]
    
    # cons2
    add_constraint!(cons, BoundConstraint(n,m, x_min=x_min, x_max=x_max), 1:Nt-1)

    # cons3;  Add collision constraints if present
    if collision[1] == true
        x,y,z = collision[2]
        add_constraint!(cons, SphereConstraint(n, [x], [y], [z], [1.5]), 1:Nt)
    end

    # With random initial positions (with x0=x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, constraints=cons)
    prob = Problem(MAV.Model, objective, x0, tf, xf = xf, constraints=cons)
    # Without random initial positions (without x0)
    # prob = Problem(MAV.Model, objective,  xf, tf, constraints=cons)
    

    # State initialization: linear trajectory from start to end
    # Control initialization: hover
    state_guess = zeros(Float64, (n,Nt))
    control_guess = zeros(Float64, (m,Nt-1))

    hover = zeros(MAV.Model)[2]

    for i in 1:Nt-1
        state_guess[:,i] = x0 + (xf-x0)*(i-1)/(Nt-1)   
        control_guess[:,i] = hover                     # 13 * number of (timesteps-1)
    end

    state_guess[:,Nt] = xf                             # 13 * number of timesteps

    initial_states!(prob, state_guess)
    initial_controls!(prob, control_guess)
    
    altro = ALTROSolver(prob)
    solve!(altro);

    X = states(altro)

    MAV.PredictedStates = X

    for i in 2:Nm
        push!(MAV.StateHistory, X[i])
    end

    return altro.stats.tsolve, prob, objective
end



# for testing on 07.31


"""
    MADS_to_ALTRO(z::Vector{Float64})

Transforms our MADS data structure to the ALTRO data structure
"""
function MADS_to_ALTRO(z::Vector{Float64})
    N = Int(length(z)/3)
    FOV = pi/2

    x = Vector{RBState}()
    for i in range(1,stop=N)
        x_val = z[i]
        y_val = z[N + i]
        R_val = z[N*2 + i]

        z_val = 2*R_val/FOV

        push!(x, RBState([x_val, y_val, z_val], UnitQuaternion(I), zeros(3), zeros(3)));
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