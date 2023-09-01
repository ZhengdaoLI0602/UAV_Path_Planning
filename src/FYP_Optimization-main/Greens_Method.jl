module Greens_Method

export Greens_Problem

include("Base_Functions.jl")
using .Base_Functions

"""
    struct Greens_Problem

An object describing the problem of finding the area of the union of N intersecting circles
The problem is solved by obtaining the outer contours of the circles, and calculating the area encompassed by Greens Theorem

Attributes:
    - 'circles::Vector{Circle}':                            A vector of Circle objects describing our problem
    - 'intersections::Vector{Point}':                       A vector of Point objects describing the intersection points ONLY LOCATED ON AN OUTER CONTOUR
    - 'contours::Vector{Vector{Circle}}':                    A vector of Circle objects, where each inner vector describes the Circle objects that are 'associated' with one another
    - 'boundaries::Vector{Vector{Vector{Vector{Any}}}}':      A vector of vector of vectors describing the outer boundaries
    - 'area::Float64':                                      The calculated area
"""
struct Greens_Problem
    circles::Vector{Circle}
    intersections::Vector{Point}
    contours::Vector{Vector{Circle}}
    boundaries::Vector{Vector{Vector{Any}}}
    area::Float64

    function Greens_Problem(x::Vector{Float64})
        circles = make_circles(x)

        original_circles = copy(circles)

        circles = checkCoincident!(circles)
        circles = checkContained!(circles)
        intersections = getIntersections(circles)
        contours = getContours(circles)
        boundaries = getBoundaries(circles,intersections,contours)
        area = calculateArea(boundaries)

        new(original_circles,intersections,contours,boundaries,area)
    end
end

"""
    checkCoincident!(circles::Vector{Circle})

Checks if any Circle objects are coincident with each other and removes the redundant Circle objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
# function checkCoincident!(circles::Vector{Circle})
function checkCoincident!(circles)
    clean = false
    while !clean
        N = Int(length(circles))
        clean = true # while has two times (double-check to make sure no duplicate)
        for i in 1:N, j in i+1:N
            A = circles[i]
            B = circles[j]

            if Base_Functions.coincident(A,B)
                clean = false
                deleteat!(circles,j)
                break
            end
        end
    end

    return circles
end

"""
    checkContained!(circles::Vector{Circle})

Checks if any Circle objects are contained within one another and removes the contained Circle objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
# function checkContained!(circles::Vector{Circle})
function checkContained!(circles)
    is_contained = Vector{Bool}([false for i in 1:length(circles)])

    for i in 1:length(circles)
        for j in i+1:length(circles)
            if contained(circles[i],circles[j]) == circles[i]
                is_contained[i] = true
            elseif contained(circles[i],circles[j]) == circles[j]
                is_contained[j] = true
            end
        end
    end

    new_circles = [circles[i] for i in 1:length(circles) if is_contained[i] == false]

    return new_circles
end

"""
    getIntersections(circles::Vector{Circle})


Obtains the intersection Point objects only located on an outer contour

Updates the attributes of the Circle objects in 'circles' accordingly

Returns intersections::Vector{Point} containing the Point objects

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function getIntersections(circles::Vector{Circle})
    intersections = Vector{Point}()
    for i in 1:length(circles)

        for j in i+1:length(circles)
            A = circles[i]
            B = circles[j]

            coords = intersection(circles[i],circles[j])

            if isnothing(coords) == false
                # update the Circles attribute of the Circle objects
                push!(A.Circles,i,j)
                push!(B.Circles,i,j)

                x1,y1,x2,y2 = coords

                # form the Point objects representing the intersection points
                P1 = Point(x1,y1,[i,j])
                P2 = Point(x2,y2,[i,j])

                # check if P1 and P2 are outer contour points
                for P in [P1,P2]
                    contour_point = true
                    for k in 1:length(circles)
                        if outside(circles[k],P) == false
                            contour_point = false
                            break
                        end
                    end
                    
                    if contour_point
                        push!(intersections,P)
                        # update the Points attribute of the Circle objects #--------??
                    
                        dum = length(intersections)
                        push!(A.Points,dum)
                        push!(B.Points,dum)
                    end
                end
            end
        end
    end

    # Ensure no repetitions in the Circle attribute of the Circle objects
    for i in 1:length(circles)
        circles[i].Circles = unique(circles[i].Circles)
    end

    return intersections
end

"""
    getContours(circles::Vector{Circle})

Obtains the contours of our problem

Returns contour::Vector{Vector{Circle}} where each inner vector contains the Circle objects that make up a contour

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
"""
function getContours(circles::Vector{Circle})

    # if there is just one Circle
    if length(circles) == 1
        return [circles]
    end

    # form input format for associate function
    tmp = []
    contour_circles = copy(circles)
    for i in range(1,stop=length(contour_circles))
        A = contour_circles[i]
        push!(tmp,[A.Circles,A])
    end

    tmp = associate(tmp)

    # change into vectors of Circles
    contours = Vector{Vector{Circle}}()
    for i in 1:length(tmp)
        push!(contours,[point[2] for point in tmp[i]])
    end

    return contours
end

"""
    getBoundaries(circles::Vector{Circle}, intersections::Vector{Point}, contours::Vector{Vector{Circle}})

Obtains the boundaries of our problem

Returns boundaries::Vector{Vector{Vector{Vector{Any}}}}

# Arguments:

    - 'circles::Vector{Circle}': Vector of Circle objects
    - 'intersections::Vector{Point}': Vector of Point objects
    - 'contours::Vector{Vector{Circle}}': Vector of vector of Circle objects

# Structure of 'boundaries'

Vector{                 # Each row for a contour
    Vector{             # Each row for a boundary within said contour
        Vector{         # Each row for a circular arc within said boundary (a boundary is composed of multiple circular arcs)
            Vector{Any} # Contains 5 rows describing a circular arc, of form [Circle,Point1,Point2,theta1,theta2]
        }
    }
}
"""
function getBoundaries(circles::Vector{Circle}, intersections::Vector{Point}, contours::Vector{Vector{Circle}})
    boundaries = Vector{Vector{Vector{Vector{Any}}}}()
    for i in range(1,stop=size(contours)[1])
        contour_boundaries = []

        # just the circle
        if size(contours[i])[1] == 1 
            push!(contour_boundaries,[[contours[i][1],Point(0,0,[]),Point(0,0,[]),0.0,2*pi]])
            push!(boundaries,contour_boundaries)
            continue
        end

        # obtain the boundaries for each contour
        for j in 1:length(contours[i]) 
            circle = contours[i][j]
            boundary_points = intersections[circle.Points]

            sort_asc_angle!(circle,boundary_points)
            theta = [mod(atan(point.y-circle.y,point.x-circle.x),2*pi) for point in boundary_points]

            for k in range(1,stop=length(theta))
                next_iterator = mod1(k+1,length(theta))

                if k == length(theta)
                    half_angle = 0.5*(theta[k]+theta[next_iterator]) + pi
                else
                    half_angle = 0.5*(theta[k]+theta[next_iterator])
                end

                point_check = point_on_circle(circle,half_angle)

                A = circles[circle.Circles]

                # check if point_check is inside any of the circles that intersect our current circle
                for l in range(1,stop=length(A)) 
                    if outside(A[l],point_check) == false
                        break
                    elseif l == length(A)
                        push!(contour_boundaries,[circle,boundary_points[k],boundary_points[next_iterator],theta[k],theta[next_iterator]])
                    end
                end

            end

        end

        # sort the boundaries so they form a closed curve
        for j in 1:length(contour_boundaries)-1
            end_sec1 = contour_boundaries[j][3]

            for k in j+1:length(contour_boundaries)
                start_sec2 = contour_boundaries[k][2]

                if end_sec1 == start_sec2
                    tmp = contour_boundaries[j+1]
                    contour_boundaries[j+1] = contour_boundaries[k]
                    contour_boundaries[k] = tmp
                    break
                end
            end
        end

        # split the boundaries so each boundary describes a single closed curve
        tmp = []
        while length(contour_boundaries) > 1
            for j in 1:length(contour_boundaries)-1

                end_sec1 = contour_boundaries[j][3]
                start_sec2 = contour_boundaries[j+1][2]

                # check for discontinuity
                if end_sec1 != start_sec2
                    push!(tmp,contour_boundaries[1:j])
                    deleteat!(contour_boundaries, 1:j)
                    break
                end

                if j == length(contour_boundaries)-1
                    push!(tmp,contour_boundaries)
                    contour_boundaries = []
                end
            end
        end

        push!(boundaries,tmp)
    end

    return boundaries
end

"""
    calculateArea(boundaries::Vector{Vector{Vector{Vector{Any}}}})

Returns the area encompassed by the boundaries

# Arguments:

    - 'boundaries::Vector{Vector{Vector{Vector{Any}}}}': Vector of boundaries
"""
function calculateArea(boundaries::Vector{Vector{Vector{Vector{Any}}}})
    total_area = 0

    # for each contour
    for i in 1:length(boundaries)
        coll_area = []

        # for each boundary
        for j in 1:length(boundaries[i])
            
            # if only one arc in contour
            if length(boundaries[i][j]) == 1
                push!(coll_area,pi*(boundaries[i][1][1][1].R)^2)
                break
            end

            # Vector of points to discretize arc
            vec = Vector{Point}()

            for k in 1:length(boundaries[i][j])

                circle = boundaries[i][j][k][1]
                theta1 = boundaries[i][j][k][4]
                theta2 = boundaries[i][j][k][5]

                if theta1 > theta2
                    theta2 = theta2 + 2*pi
                end

                angles = LinRange(theta1,theta2,101)

                for l in angles
                    x = circle.x + circle.R*cos(l)
                    y = circle.y + circle.R*sin(l)

                    P = Point(x,y,[])
                    push!(vec,P)

                end
            end

            # add area of this boundary
            push!(coll_area, shoelace(vec))
        end

        if length(coll_area) == 1           # one boundary in the contour
            total_area += coll_area[1]
        else                                # several boundaries in contour -> biggest boundary - sum(all other boundaries)
            var,ind = findmax(coll_area)
            deleteat!(coll_area,ind)

            total_area += var - sum(coll_area)
        end
    end

    return total_area
end

end #module end