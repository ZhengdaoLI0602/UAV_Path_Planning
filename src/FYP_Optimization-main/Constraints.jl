# Extreme Constraint: x,y position of drone to remain in domain
function cons1(x)
    for i in range(1,stop=Int(length(x)/3))
        x_val = x[i]
        y_val = x[N + i]
        R_val = x[N*2 + i]
        val = (x_val <= domain_x-R_val && x_val >= R_val) && (y_val <= domain_y-R_val && y_val >= R_val)
        if !val
            return false
        end
    end

    return true
end

# Extreme Constraint: Radius of drone has to be between 1 and R_lim
function cons2(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        val = R_val >= 0 && R_val <= R_lim
        if !val
            return false
        end
    end

    return true
end

# Extreme Constraint: Radius cannot change
function cons3(x)
    for i in range(1,stop=Int(length(x)/3))
        R_val = x[N*2 + i]
        val = (R_val == MADS_input[N*2 + i])
        if !val
            return false
        end
    end

    return true
end

# Progressive Constraint: Constraint on Radius based on x,y location
function cons4(x)
    for i in range(1,stop=Int(length(x)/3))
        x_val = x[i]
        y_val = x[N + i]
        R_val = x[N*2 + i]

        if x_val <= 30
            if R_val > 5
                return false
            end
        end
    end
    return true
end