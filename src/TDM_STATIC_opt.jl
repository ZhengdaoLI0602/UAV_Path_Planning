module TDM_STATIC_opt

using DirectSearch

include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions

include("../../FYP_Optimization-main/Greens_Method.jl")
using .Greens_Method

include("TDM_Functions.jl")



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
function optimize(input, obj, cons_ext, cons_prog, N_iter, r_min, r_max, d_lim, new_inputs, k)

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
            println("Loop ended with the number of iterations ($iter), which is more than 10")
            # break
            return false
        end

        # check if any circles are contained
        if length(new_inputs)==0
            var,new_input = check(input, result, r_min, r_max, d_lim)
        else
            println("The length of new_inputs is: $(length(new_inputs))")
            var,new_input = check(new_inputs[end], result, r_min, r_max, d_lim)
        end
        

        if var
            # SetInitialPoint(p, new_input)
            # BumpIterationLimit(p, i=N_iter)
            # println("Non-optimal solution (a circle is contained). REINITIALIZING...")

            # push!(new_inputs, new_input)
            # println("New input added in !!!!!")

            # println("Epoch $(k) -> Non-optimal solution (a circle is contained). REIALLOCATED...")
            TDM_Functions.show_epoch(make_circles(new_input), nothing)

            return new_input
        else
            println("Epoch $(k) -> Optimal solutions to all UAVs found... Optimization ended at Iteration: ", iter)
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
function check(input, output, r_min, r_max, d_lim)
    input_circles = make_circles(input)
    output_circles = make_circles(output)

    N = length(output_circles) # input and output should have the same number of circles

    is_contained = Vector{Bool}([false for i in 1:N])
    # is_pure_contained = Vector{Bool}([false for i in 1:N])
    no_movement = Vector{Bool}([false for i in 1:N])

    for i in 1:N
        for j in i+1:N
            if contained(output_circles[i], output_circles[j]) == output_circles[i]
                is_contained[i] = true
            elseif contained(output_circles[i], output_circles[j]) == output_circles[j]
                is_contained[j] = true
            end
        end
    end

    for i in 1:N
        # for j in i+1:N
        if output_circles[i].R<=input_circles[i].R || 
            (output_circles[i].x==input_circles[i].x && output_circles[i].y==input_circles[i].y)
            no_movement[i] = true
        end
        # end
    end

    # for i in eachindex(output_circles)
    #     if Base_Functions.pure_contained(output_circles[i], input_circles[i]) !== nothing
    #         # Base_Functions.pure_contained(pre_optimized_circles[i], this_group[i]) !== nothing
    #         # return false
    #         is_pure_contained[i] = true
    #     end
    # end

    if any(is_contained) || any(no_movement)
    # if any(is_contained)
        println("is_contained: ",is_contained)
        println("no_movement: ",no_movement)
        # println("is_pure_contained: ",is_pure_contained)
        # allowed_displacement = 1
        FOV = 80 * pi/180
        for i in 1:N
            # if circle is contained, regenerate it (randomly)
            if is_contained[i] == true || no_movement[i] == true 

                # output[i] = rand(R_lim:domain_x-R_lim)[1]       # x
                # output[N + i] = rand(R_lim:domain_y-R_lim)[1]   # y
                # output[2*N + i] = rand(1:R_lim)[1]              # R of UAV circle

                # this_distance =  (cir_domain.R - r_max) * rand()
                # this_angle = 2*π*rand()

                # output[i] = this_distance * cos(this_angle)       # x
                # output[N + i] = this_distance * sin(this_angle)   # y
                # output[2*N + i] = r_max * rand()                  # R of UAV circle

                # direction = [output[i], output[N+i], output[2*N+i]/tan(FOV/2)]/sqrt(output[i]^2+output[N+i]^2+(output[2*N+i]/tan(FOV/2))^2)
                direction = [output[i], output[N+i], output[2*N+i]/tan(FOV/2)]/sqrt((output[i])^2 + (output[N+i])^2+ (output[2*N+i]/tan(FOV/2))^2)
                increment = d_lim *direction

                # variation = - allowed_displacement/2 + allowed_displacement*rand()
                output[i] = output[i] + increment[1]                             # x
                output[N + i] = output[N + i] + increment[2]                    # y
                output[2*N + i] = min(output[2*N + i] + increment[3] *tan(FOV/2) *0.9 ,  r_max)   # R

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