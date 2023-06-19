# 1. Making plots
using Plots

x = range(0, 10, length=100)
y = sin.(x)
plot(x, y)
savefig("line_graph.png")

# 2. No matched functions (script_MADS.jl)
# 3. error when using "_gettimeinfo()" (script_Altro.jl)
# 4. time-dependent domain version

# no wind, evenly-distributed combustible materials, no Flame retardant region exists

# wildfires or industrial fires: meters per second 
# flames in a laboratory setting or smaller fires: centimeter per second





