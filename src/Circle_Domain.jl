include("../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions


Domain_R_min = 50


cir_domain = Circle(0,0,Domain_R_min,[],[])

for i in 1:10
    cir_domain.x += 5
    cir_domain.R += 10
    println("At time $i,Circle domain has radius of $(cir_domain.R) at ($(cir_domain.x), $(cir_domain.y))")
    sleep(1) # wait for 1 second before computing the next area
end

elapsed_time = @elapsed my_func()
println("Elapsed time: $elapsed_time seconds")


