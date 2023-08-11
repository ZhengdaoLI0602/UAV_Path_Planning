module TDM_STATIC_opt

using DirectSearch

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

include("../../FYP_Optimization-main/Greens_Method.jl")
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
function optimize(input, obj, cons_ext, cons_prog, N_iter, r_min, r_max)

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

        if iter > 20
            println("Loop ended with the number of iterations ($iter) over 10")
            break
        end

        # check if any circles are contained
        var,new_input = check(result, r_min, r_max)

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
function check(output, r_min, r_max)
    output_circles = make_circles(output)
    N = length(output_circles)

    is_contained = Vector{Bool}([false for i in 1:N])

    for i in 1:N
        for j in i+1:N
            if contained(output_circles[i], output_circles[j]) == output_circles[i]
                is_contained[i] = true
            elseif contained(output_circles[i], output_circles[j]) == output_circles[j]
                is_contained[j] = true
            end
        end
    end

    if any(is_contained)
        allowed_displacement = 1
        FOV = 80 * pi/180
        for i in 1:N
            # if circle is contained, regenerate it (randomly)
            if is_contained[i] == true

                # output[i] = rand(R_lim:domain_x-R_lim)[1]       # x
                # output[N + i] = rand(R_lim:domain_y-R_lim)[1]   # y
                # output[2*N + i] = rand(1:R_lim)[1]              # R of UAV circle


                # this_distance =  (cir_domain.R - r_max) * rand()
                # this_angle = 2*π*rand()

                # output[i] = this_distance * cos(this_angle)       # x
                # output[N + i] = this_distance * sin(this_angle)   # y
                # output[2*N + i] = r_max * rand()                  # R of UAV circle

                
                variation = - allowed_displacement/2 + allowed_displacement*rand()
                output[i] = output[i] + variation                             # x
                output[N + i] = output[N + i] + variation                     # y
                output[2*N + i] = output[2*N + i] + variation *tan(FOV/2)     # R


            end
        end

        return true, output                          
        # true: has contained circles; output the current circles group in MADS format 
    else
        return false, []
        # true: has no contained circle; output blank array
    end
end

end # module end