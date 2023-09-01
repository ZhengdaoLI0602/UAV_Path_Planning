# Modified based on Logan's codes: https://github.com/Logan1904/FYP_Optimization
include("FYP_Optimization-main/Base_Functions.jl")


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


# Extreme Constraint 2: Could not be contained by or contain the same UAV
function cons2(x)
    this_group = Base_Functions.make_circles(x)
    
    for i in eachindex(this_group)
        if Base_Functions.pure_contained(this_group[i], pre_optimized_circles[i]) !== nothing
            return false
        end
    end
    return true
end


# Extreme Constraint 3: Limits on the displacement between "pre-optimized" and "post-optimized" position for each UAV
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
    return true
end




