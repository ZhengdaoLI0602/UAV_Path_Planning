# 1. Making plots
using Plots
# plotly()
plotlyjs()
Plots.default(show = true) # shown in explorer
## Example 1
x = range(0, 10, length=100)
y = @. 100*sin(x) + x^2
z = @. exp(x)
plot(x, [y,z], grid = false)
plot(x, [y z], linecolor=["blue" "red"], label=["line 1"  "line 2"],legend=:outertopright, lw=[3 1] ,ls=[:dot :solid],grid = false)


title!("Trigonometric functions")
xlabel!("x")
ylabel!("y")

savefig("My_Repo/src/Questions/Saved/myplot.pdf")



## Example 2
x = range(0, 10, length=100)
y = 20 * sin.(x)
f(x) = @. x^2+ exp(x) + 20 * sin(x)
plot(x, f, color=:red, linestyle=:solid, label="Line 1")
plot!(x, y, color=:blue, linestyle=:dash, label="Line 2")
plot(x, [f,y], label=["Line 1", "Line 2"])
## Example 3
plot(rand(100, 4), layout = grid(4, 1, heights=[0.1 ,0.4, 0.4, 0.1]))
plot(rand(100, 4), layout = (4, 1))
## Example 4
x0 = range(0, 10, length=100)
y1 = @. sin(x0)
y2 = @. cos(x0)
plot(x0, [y1 y2], title="Trigonometric functions", label=["sin(x)" "cos(x)"], linewidth=3)


# 2. No matched functions (script_MADS.jl)
include("../../../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions
a = Circle(0,0,3,[],[])  # a:: Circle
points = draw(a, 0.0, 2*pi) # points: return the x,y positions of arc
plot(points[1], points[2],size=(400,400))

savefig("line_graph.png")
savefig("line_graph.pdf")
savefig("line_graph.svg")
# savefig("line_graph.eps") # having problems

# 3. error when using "_gettimeinfo()" (script_Altro.jl)
# take care of the optional parameters in this function