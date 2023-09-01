# function divs(x::T) where {T<:AbstractArray{Bool}}
#     return sum(x)
# end

# list1 = [true false true 4]
# divs(list1)

# arr = [1 2 3 4 5 6]
# sum1 = [x*log(x) for x in arr]
# println(sum(sum1))

# for i in 1:size(sum1,2)
#     @info "$(i)th element is $(sum1[i])"
# end

# using StatsPlots
# using PlotThemes

# ENV["JULIA_TMPDIR"] = "C:\\Temp\\"
# # my_plot = plot(size=(200,200))
# # savefig(my_plot,"aaa.png")
# println(9)


# function print_num()
#     println(9)
#     println(10)
#     println(11)
# end

# N=5
# for i in 1:N, j in i+1:N
#     println(i,' ',j)
# end



# module MyScript

# export myfunction

# function myfunction()
#     # Your function code here
#     println(9,' ',10,' ',11)
# end

# end




# using .Base_Functions

module MyModule

export f, f1, mystruct

function f()
    println(1)
end

function f1(x)
    println(x+1)
end

struct mystruct
    x::Integer
end

end # MyModule

# function print_x(x)
#     return x
# end
# # print(9)




