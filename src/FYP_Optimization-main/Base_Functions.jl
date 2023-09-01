module Base_Functions

export Circle
export Point

export make_circles
export draw
export distance
export coincident
export contained
export intersection
export outside
export sort_asc_angle!
export point_on_circle
export shoelace
export associate

using LinearAlgebra
using Random

"""
    struct Circle

A circle object

# Attributes:

    - 'x::Float64':             x-position of the circle centre on a 2D Cartesian grid
    - 'y::Float64':             y-position of the circle centre on a 2D Cartesian grid
    - 'R::Float64':             Radius
    - 'Points::Vector{Int}':    Vector of intersection point indices
    - 'Circles::Vector{Int}':   Vector of intersection circles indices
"""
mutable struct Circle
    x::Float64
    y::Float64
    R::Float64
    Points::Vector{Int}
    Circles::Vector{Int}
end

"""
    struct Point

A point object

# Attributes:

    - 'x::Float64':             x-position of the point on a 2D Cartesian grid
    - 'y::Float64':             y-position of the point on a 2D Cartesian grid
    - 'Circles::Vector{Int}':   Vector of circle indices that form point
"""
mutable struct Point
    x::Float64
    y::Float64
    Circles::Vector{Int}
end

"""
    make_circles(arr::Vector{Float64})

Returns a Vector of Circle objects

# Arguments:

    - 'arr::{Vector{Float64}}': Vector of x,y coordinates with radii values, in the order [x;y;R]
"""
function make_circles(arr::Vector{Float64})
    N = Int(length(arr)/3)

    circles = Vector{Circle}()
    for i in 1:N
        x = arr[i]
        y = arr[N + i]
        R = arr[2*N + i]
        push!(circles, Circle(x,y,R,[],[]))
    end
    
    return circles
end

"""
    draw(A::Circle, theta1::Float64, theta2::Float64)

Returns 2 vectors of discretised x and y coordinates of points on the circle between the angles

# Arguments:

    - 'A::Circle': A Circle object
    - 'theta1::Float64': First angle
    - 'theta2::Float64': Second angle
"""
# returns x and y vectors of a Circle object (for plotting)
function draw(A,theta1::Float64,theta2::Float64)
    if theta1 > theta2
        theta2 = theta2 + 2*pi
    end
    arr = LinRange(theta1,theta2,101)
    return A.x .+ A.R*cos.(arr), A.y .+ A.R*sin.(arr)
end


"""
    distance(A::Circle, B::Circle)

Returns the Euclidean distance between 2 Circle centres

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function distance(A::Circle,B::Circle)
function distance(A , B)
    return sqrt((A.x-B.x)^2 + (A.y-B.y)^2)
end

"""
    distance(x1::Float64, y1::Float64, x2::Float64, y2::Float64)

Returns the Euclidean distance between the 2 points (x1,y1) and (x2,y2)

# Arguments:

    - 'x1::Float64': x-coordinate of the first point
    - 'y1::Float64': y-coordinate of the first point
    - 'x2::Float64': x-coordinate of the second point
    - 'y2::Float64': y-coordinate of the second point
"""
function distance(x1::Float64,y1::Float64,x2::Float64,y2::Float64)
    return sqrt((x1-x2)^2 + (y1-y2)^2)
end

"""
    coincident(A::Circle, B::Circle)

Returns 'true' if the 2 Circles are coincident, 'false' otherwise

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function coincident(A::Circle,B::Circle)
function coincident(A,B)
    d = round(distance(A,B), digits=1)
    if d == 0 && A.R == B.R
        return true
    else
        return false
    end
end

"""
    contained(A::Circle, B::Circle)

Checks if A is entirely contained inside B and vice-versa

If A is contained in B, return A. If B is contained in A, return B.

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
# function contained(A::Circle,B::Circle)
function contained(A,B)
    d = distance(A,B)
    if d <= abs(A.R - B.R)
    # if d < abs(A.R - B.R)
        if A.R < B.R
            return A
        else
            return B
        end
    end
    return
end


function pure_contained(A,B)
    d = distance(A,B)
    # if d <= abs(A.R - B.R)
    if d < abs(A.R - B.R)
        if A.R < B.R
            return A
        else
            return B
        end
    end
    return
end

"""
    intersection(A::Circle, B::Circle)

Returns the intersection coordinates of 2 Circles, in order 'x1,y1,x2,y2'

Returns 'nothing' if the 2 Circles do not intersect or are tangent

# Arguments:

    - 'A::Circle': First Circle object
    - 'B::Circle': Second Circle object
"""
function intersection(A,B)
    d = distance(A,B)

    if d > A.R + B.R                        # non-intersecting
        return nothing
    elseif d <= abs(A.R - B.R)              # one circle within another
        return nothing
    else
        a = (d^2+A.R^2-B.R^2)/(2*d)
        h = sqrt((A.R^2-a^2))

        varx = A.x + a*(B.x-A.x)/d
        vary = A.y + a*(B.y-A.y)/d

        # Original
        # x1 = varx + h*(B.y-A.y)/d
        # y1 = vary - h*(B.x-A.x)/d
        # x2 = varx - h*(B.y-A.y)/d
        # y2 = vary + h*(B.x-A.x)/d

        # Defined by LZD on 2023.06.08
        x1 = varx - h/a*(vary - A.y)
        y1 = vary + h/a*(varx - A.x)
        x2 = varx + h/a*(vary - A.y)
        y2 = vary - h/a*(varx - A.x)

        if isapprox(distance(x1,y1,x2,y2), 0.0, atol=0.01) # tangent circles -> we take it as non-intersecting
            return nothing
        end

        return x1,y1,x2,y2
        
    end
end

"""
    boundary(A::Circle, P::Point)

Returns 'true' if Point P is inside Circle A, 'false' otherwise

# Arguments:

    - 'A::Circle': The Circle object
    - 'P::Point': The Point object
"""
function outside(A::Circle,P::Point)
    if round((P.x-A.x)^2 + (P.y-A.y)^2 - A.R^2, digits=2) >= 0.0
        return true
    else 
        return false
    end
end

"""
    sort_asc_angle!(A::Circle, Array::Vector{Point})

Sorts a Vector of Points relative to a Circle (centre) in ascending order of polar angle

# Arguments:

    - 'A::Circle': The Circle object (The reference)
    - 'Array::Vector{Point}': The vector of Point objects
"""
function sort_asc_angle!(A::Circle, Array::Vector{Point})
    mean_x = A.x
    mean_y = A.y

    sort_acw!(Array,mean_x,mean_y)
end

"""
    sort_acw!(Array::Any, x::Float64, y::Float64)

Sorts a Vector of Points and/or Circles around the (x,y) coordinates in anticlockwise order

# Arguments:

    - 'Array::Any': A vector of either Circle or Point objects
    - 'x::Float64': x-coordinate (reference point)
    - 'y::Float64': y-coordinate (reference point)
"""
function sort_acw!(Array::Any,x::Float64,y::Float64)
    N = Int(length(Array))

    angles = [mod(atan(point.y-y, point.x-x),2*pi) for point in Array] #???
    for i in 1:N
        for j in i+1:N
            if angles[j] < angles[i]
                tmp = angles[i]
                angles[i] = angles[j]
                angles[j] = tmp

                tmp2 = Array[i]
                Array[i] = Array[j]
                Array[j] = tmp2
            end
        end
    end
end

"""
    point_on_circle(A::Circle, Theta::Float64)

Returns a Point on a Circle given a polar angle

# Arguments:

    -'A::Circle': The Circle object
    -theta::Float64': The angle value (rad)
"""
function point_on_circle(A::Circle,theta::Float64)
    x = A.x + A.R*cos(theta)
    y = A.y + A.R*sin(theta)
    return Point(x,y,[])
end

"""
    shoelace(Array::Vector{Point})

Returns the area of a polygon described by a sorted Vector of Points using the Shoelace algorithm

# Arguments:

    -Array::Vector{Point}': A vector of Point objects
"""
function shoelace(Array::Vector{Point})
    xarr = [point.x for point in Array]
    yarr = [point.y for point in Array]

    dum1 = dot(xarr,circshift(yarr,1))
    dum2 = dot(yarr,circshift(xarr,1))

    area = 0.5*broadcast(abs,(dum1-dum2))
    
    return area
end

"""
    associate(Array::Vector{Any})

Returns a vector, with each row containing rows in Array that have links between the Association_Object(s)

# Arguments:

    'Array::Vector{Any}': A vector with each row of the form [[Association_Object(s)], Link_Object]
    
# Example:

Input = [
    [[1,2,3],A],
    [[2,4,6],B],
    [[6,7,8],C],
    [[10,11,12],D],
    [[12,24,36],E]
]

Output = [
    [[[1,2,3],A], [[2,4,6],B], [[6,7,8],C]],
    [[[10,11,12],D], [[12,24,36],E]]
]

## Explanation:
[1,2,3] is 'associated' with [2,4,6] through element '2', and [2,4,6] is 'associated' with [6,7,8] through element '6'

[10,11,12] is 'associated' with [12,24,36] through element '12'

Note that [1,2,3] is not directly 'associated' with [6,7,8]; this function considers implicit 'association' through the intermediary [2,4,6]
"""
function associate(Array::Vector{Any})

    global dummy = []
    push!(dummy,Array[1])   ## Note: push!() <-> splice!()
    splice!(Array,1)

    final = []

    while size(Array)[1] != 0
        super_break = false

        for i in range(1,stop=size(dummy)[1])
            for j in range(1,stop=size(Array)[1])
                var1 = dummy[i][1]
                var2 = Array[j][1]

                common = intersect(var1,var2)

                if size(common)[1] != 0 # there exists a common object
                    push!(dummy,Array[j])
                    splice!(Array,j)
                    super_break = true
                    break
                end

                if i == size(dummy)[1] && j == size(Array)[1] # no more common Association_Object(s) in Array
                    push!(final,dummy)
                    global dummy = []
                    push!(dummy,Array[1])
                    splice!(Array,1)
                end
                
            end

            if super_break
                break
            end
        end

        if size(Array)[1] == 0
            push!(final,dummy)
        end

    end

    return final
end


end #module end