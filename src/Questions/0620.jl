# 1. Making plots
using Plots
plotly()
# Plots.default(show = true) # shown in explorer
## Example 1
x = range(0, 10, length=100)
y = sin.(x)
plot(x, y)
## Example 2
x = range(0, 10, length=100)
y = 20 * sin.(x)
f(x) = x.^2+ exp.(x) + 20 * sin.(x)
plot(x, f, color=:red, linestyle=:solid, label="Line 1")
plot!(x, y, color=:blue, linestyle=:dash, label="Line 2")
## Example 3
plot(rand(100, 4), layout = grid(4, 1, heights=[0.1 ,0.4, 0.4, 0.1]))
plot(rand(100, 4), layout = (4, 1))
# savefig("line_graph.png")

# 2. No matched functions (script_MADS.jl)
     # (making plots? identifiers?)

# 3. error when using "_gettimeinfo()" (script_Altro.jl)







