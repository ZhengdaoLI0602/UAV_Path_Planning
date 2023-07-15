# Some examples of constraints copied from Logan's codes
include("../../FYP_Optimization-main/Base_Functions.jl")
# include("../../FYP_Optimization-main/Greens_Method.jl")

# #
# Extreme Constraint 1: Radius of drone has to be between r_min and r_max (get based on FOV and h_min & h_max)
function cons1(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        val = R_val >= r_min && R_val <= r_max
        if !val
            return false
        end
    end

    return true
end


# Progressive Constraint 2: Post-optimized positions cannot fall within the previous circles
function cons2(circles_pool, this_circle_group)
        for cirA in circles_pool, cirB in this_circle_group
            if Base_Functions.coincident(cirA, cirB) !== nothing || 
                Base_Functions.contained(cirA, cirB) !== nothing
                return false
            end
        end
    return true
end


# Progressive Constraint 3: Limits on the displacement between "pre-optimized" and "post-optimized" position for each UAV
function cons3(x)


end




