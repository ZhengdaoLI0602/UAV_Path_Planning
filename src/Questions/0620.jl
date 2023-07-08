## 1. [SOLVED] Making plots error 
using Plots
# plotly()
plotlyjs()
Plots.default(show = true) # shown in explorer
# Example 1
x = range(0, 10, length=100)
y = @. 100*sin(x) + x^2
z = @. exp(x)
f1 = plot(x,y)
f2 = plot!(x,z)


l = @layout [grid(4,1,heights=[0.1 ,0.4, 0.4, 0.1])]
plot(x, [y,z], grid = false)
plot(x, [y z], 
    linecolor=["blue" "red"],   # set line color
    label=["line 1" "line 2"],  # set line legend
    legend=:outertopright,      # set legend position 
    lw=[3 1], ls=[:dot :solid], # set line width and style
    grid = false
)

title!("Trigonometric functions")
xlabel!("x")
ylabel!("y")

savefig("My_Repo/src/Questions/Saved/myplot.pdf")


#  Example 2
plot(rand(100, 4), layout = grid(4, 1, heights=[0.1 ,0.4, 0.4, 0.1]))
plot(rand(100, 4), layout = (4, 1))


#  Example 3
x0 = range(0, 10, length=100)
y1 = @. sin(x0)
y2 = @. cos(x0)
plot(x0, [y1 y2], title="Trigonometric functions", label=["sin(x)" "cos(x)"], linewidth=3)


## 2. [SOLVED] No matched functions (script_MADS.jl) (SOLVED)
include("../../../../FYP_Optimization-main/Base_Functions.jl")
using .Base_Functions
a = Circle(0,0,3,[],[])  # a:: Circle
points = draw(a, 0.0, 2*pi) # points: return the x,y positions of arc
plot(points[1], points[2],size=(400,400))

## 3. error when using "_gettimeinfo()" (script_Altro.jl)
# take care of the optional parameters in this function


# 4. cannot show legend or title string in correct latex format