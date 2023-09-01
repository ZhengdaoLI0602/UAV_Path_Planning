module Plotter

export Plot

include("Base_Functions.jl")
using .Base_Functions
using Plots
plotlyjs()
# plotly()
# gr()
# pgfplotsx() # specialized for latex2pdf command
Plots.default(show = true)

function Plot(problem, domain, string)

    circles = problem.circles
    intersections = problem.intersections
    boundaries = problem.boundaries

    plot(minorgrid=true, minorgridalpha=0.25, xlabel="x (m)", ylabel="y (m)")

    domain_x,domain_y = domain

    # plot circles
    # for i in range(1,stop=length(circles))
    #     # plot!(draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(0,domain_y), xlim=(0,domain_x), legend=:outertopright)
    #     plot!(draw(circles[i],0.0,2*pi), aspect_ratio=1, label="", color="black", ylim=(-domain_y,domain_y), xlim=(-domain_x,domain_x), legend=:outertopright)

    # end

    # plot intersections
    for i in range(1,stop=length(intersections))
        point = intersections[i]
        color = "blue"
        plot!([point.x],[point.y], seriestype=:scatter, label="", color=color, markersize=:2)
    end

    # plot boundaries
    palette = ["blue", "orange", "green", "cyan", "purple", "pink", "gray", "olive"]
    iter = 1
    for i in 1:length(boundaries)
        for j in 1:length(boundaries[i])
            shade = palette[length(palette) - mod1(iter,length(palette))]
            for k in 1:length(boundaries[i][j])

                if k == length(boundaries[i][j])
                    plot!(draw(boundaries[i][j][k][1],boundaries[i][j][k][4],boundaries[i][j][k][5]),
                            aspect_ratio=1, label="Γ$i, B$j", color=shade, 
                            legend=:outertopright, lw = 1.5)
                    # L"Γ_%$i, B_%$j"
                else
                    plot!(draw(boundaries[i][j][k][1],boundaries[i][j][k][4],boundaries[i][j][k][5]),
                            aspect_ratio=1, label="", color=shade, 
                            legend=:outertopright, lw =1.5)
                end

            end
            
            iter += 1
        end
    end
    savefig(string)
end

end #module end