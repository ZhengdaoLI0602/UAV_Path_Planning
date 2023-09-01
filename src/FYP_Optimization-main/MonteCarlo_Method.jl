module MonteCarlo_Method

using Base_Functions

function Area(arr,N_MC)
    circles = make_circles(arr)

    max_x = max([point.x + point.R for point in circles]...)
    max_y = max([point.y + point.R for point in circles]...)
    min_x = min([point.x - point.R for point in circles]...)
    min_y = min([point.y - point.R for point in circles]...)

    count = 0
    for i in 1:N_MC
        point_x = rand(1)[1]*(max_x-min_x)+min_x
        point_y = rand(1)[1]*(max_y-min_y)+min_y

        P = Point(point_x,point_y,[])

        for j in range(1,stop=size(circles)[1])
            A = circles[j]

            if !(outside(A,P)) #point inside a circle
                count += 1
                break
            end
        end

    end

    area = count/N_MC * ((max_x-min_x)*(max_y-min_y))

    return area
end

end #module end