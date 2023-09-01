# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization


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
function optimize(input, obj, cons_ext, cons_prog, N_iter, r_min, r_max, d_lim, new_inputs, k, circles_pool)

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
        # if length(new_inputs)==0
        var,new_input = check(input, result, r_min, r_max, d_lim, circles_pool)
        # else
        #     println("The length of new_inputs is: $(length(new_inputs))")
        #     var,new_input = check(new_inputs[end], result, r_min, r_max, d_lim, circles_pool)
        # end
        

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
    - 'input': Pre-optimized circles
    - 'output': The current solution of post-optimized circles 
                (Concatenated vector of (x,y,R) values, obtained from MADS solver output)
    - 'r_min': Minimum radius for detection circle
    - 'r_max': Maximum radius for detection circle
    - 'd_lim': displacement limit
"""
function check(input, output, r_min, r_max, d_lim, circles_pool)
    input_circles = make_circles(input)
    output_circles = make_circles(output)

    N = length(output_circles) # input and output should have the same number of circles

    is_contained = Vector{Bool}([false for i in 1:N])
    # is_pure_contained = Vector{Bool}([false for i in 1:N])
    no_movement = Vector{Bool}([false for i in 1:N])

    # Should not contained by the other UAVs at current epoch
    for i in 1:N
        for j in i+1:N
            if contained(output_circles[i], output_circles[j]) == output_circles[i]
                is_contained[i] = true
            elseif contained(output_circles[i], output_circles[j]) == output_circles[j]
                is_contained[j] = true
            end
        end
    end

    # Should not contained by the detection cirlce of same UAV at previous epoch
    for i in 1:N
        if output_circles[i].R<=input_circles[i].R || 
            (output_circles[i].x==input_circles[i].x && output_circles[i].y==input_circles[i].y)
            no_movement[i] = true
        end
    end


    if any(is_contained) || any(no_movement)
        println("is_contained: ",is_contained)
        println("no_movement: ",no_movement)
        global FOV = 80 /180 *pi
        for i in 1:N
            if is_contained[i] == true || no_movement[i] == true 

                global success = true
                for angle in 1:1:360
                    success = true
                    movement = 0.9*d_lim

                    this_x = output[i] + movement * √2/√3 * cosd(angle)
                    this_y = output[N + i] + movement * √2/√3 * sind(angle)
                    this_r = output[2*N + i] + movement * 1/√3 * tan(FOV/2)

                    global this_circle = Base_Functions.make_circles([this_x, this_y, this_r])
    
                    for m in eachindex(circles_pool)
                        if Base_Functions.intersection(this_circle[1], circles_pool[m]) !== nothing ||
                            Base_Functions.contained(this_circle[1], circles_pool[m]) !== nothing
                            success =  false
                            break
                        end
                    end

                    if success == true
                        output[i] = this_x
                        output[N + i] = this_y
                        output[2*N + i] = this_r
                        println("Reallocate successful!")
                        break
                    end
                end


                if success == false
                    println("Reallocate fail! Interpolate ourwards!")
                    direction = [output[i], output[N+i], output[2*N+i]/tan(FOV/2)]/sqrt((output[i])^2 + (output[N+i])^2+ (output[2*N+i]/tan(FOV/2))^2)
                    increment = d_lim *direction

                    output[i] = output[i] + increment[1]                             # x
                    output[N + i] = output[N + i] + increment[2]                    # y
                    output[2*N + i] = min(output[2*N + i] + increment[3] *tan(FOV/2) *0.9 ,  r_max)   # R
                end
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