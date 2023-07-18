# Some examples of constraints copied from Logan's codes
include("../../FYP_Optimization-main/Base_Functions.jl")
# include("../../FYP_Optimization-main/Greens_Method.jl")

# #
# Extreme Constraint 1: Radius of drone has to be between r_min and r_max (get based on FOV and h_min & h_max)
function cons1(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        val = (R_val > r_min && R_val <= r_max)
        if !val
            return false
        end
    end

    return true
end


# Progressive Constraint 2: Post-optimized positions cannot fall within the previous circles
function cons2(x)
    this_group = Base_Functions.make_circles(x)
    for cirA in circles_pool
        for cirB in this_group
            if Base_Functions.contained(cirA,cirB) !== nothing
                return false
            end
        end
    end
    return true
end


# Progressive Constraint 3: Limits on the displacement between "pre-optimized" and "post-optimized" position for each UAV
function cons3(x)
    this_group = Base_Functions.make_circles(x)
    for i in eachindex(this_group)
        x1 = pre_optimized_circles[i].x
        y1 = pre_optimized_circles[i].y
        z1 = pre_optimized_circles[i].R / tan(FOV/2)
        # z1 = pre.R / tan(FOV/2)
        
        x2 = this_group[i].x
        y2 = this_group[i].y
        z2 = this_group[i].R / tan(FOV/2)
        
        if sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2) > d_lim
            return false
        end
    end

    # for pre in pre_optimized_circles
    #     x1 = pre.x
    #     y1 = pre.y
    #     z1 = pre.R 
    #     # z1 = pre.R / tan(FOV/2)

    #     for post in this_group
    #         x2 = post.x
    #         y2 = post.y
    #         z2 = post.R 
    #         # z2 = post.R / tan(FOV/2)

    #         if sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2) > d_lim
    #             return false
    #         end
    #     end
    # end
    return true
end




