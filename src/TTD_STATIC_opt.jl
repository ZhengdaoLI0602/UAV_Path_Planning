module Position_Optimization

using DirectSearch

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

include("Greens_Method.jl")
using .Greens_Method



"""
    optimize(input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

Optimize the problem using the Mesh Adaptive Direct Search solver

# Arguments:

    - 'input': Initial point
    - 'obj': Objective function
    - 'cons_ext': Extreme constraints
    - 'cons_prog': Progressive constraints
    - 'N_iter': Number of iterations
    - 'R_lim': Radius limit of circles
    - 'domain_x': x domain size
    - 'domain_y': y domain size
"""
function optimize(input, obj, cons_ext, cons_prog, N_iter, R_lim, domain_x, domain_y)

    # Define optimization problem
    global p = DSProblem(length(input))
    SetInitialPoint(p, input)
    SetObjective(p, obj)
    SetIterationLimit(p, N_iter)

    # Add constraints to problem
    for i in cons_ext
        AddExtremeConstraint(p, i)
    end
    for i in cons_prog
        AddProgressiveConstraint(p, i)
    end

    iter = 0
    while true
        iter += 1
        Optimize!(p)

        # After optimization, return feasible or infeasible solution
        if p.x === nothing
            global result = p.i
        else
            global result = p.x
        end

        if iter > 10
            break
        end

        # check if any circles are contained
        var,new_input = check(result, R_lim, domain_x, domain_y)

        if var
            SetInitialPoint(p, new_input)
            BumpIterationLimit(p, i=N_iter)
            println("Non-optimal solution (a circle is contained). REINITIALIZING...")
        else
            println("Optimal solution found... Optimization ended at Iteration: ", iter)
            break
        end

    end

    return result

end

"""
    check(input,output,R_lim,domain_x,domain_y)

Checks if any Circle objects are contained by other circles, and regenerates them (randomly)

# Arguments:
    - 'output': Concatenated vector of (x,y,R) values, obtained from MADS solver output
    - 'R_lim': Radius limit on circles
    - 'domain_x': x domain size
    - 'domain_y': y domain size
"""
function check(output,R_lim,domain_x,domain_y)
    output_circles = make_circles(output)
    N = length(output_circles)

    is_contained = Vector{Bool}([false for i in 1:N])

    for i in 1:N
        for j in i+1:N
            if contained(output_circles[i],output_circles[j]) == output_circles[i]
                is_contained[i] = true
            elseif contained(output_circles[i],output_circles[j]) == output_circles[j]
                is_contained[j] = true
            end
        end
    end

    if any(is_contained)

        for i in 1:N
            # if circle is contained, regenerate it (randomly)
            if is_contained[i] == true
                output[i] = rand(R_lim:domain_x-R_lim)[1]
                output[N + i] = rand(R_lim:domain_y-R_lim)[1]
                output[2*N + i] = rand(1:R_lim)[1]
            end
        end

        return true, output
    else
        return false, []
    end
end

end # module end