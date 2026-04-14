[Part 1 - 2:00]
[Slide 1: Path Planning in Robotics (Title Slide)]
Welcome everyone. Today we're looking at how autonomous systems, from multi-jointed robotic arms to self-driving cars, navigate complex environments safely. We're discussing the Rapidly-exploring Random Tree, or RRT, and its optimal variant, RRT*.

[Slide 2: What Is Path Planning? / Now Add Obstacles]
Traditional grid-search algorithms like Dijkstra and A* are resolution-complete. This means they will always find the optimal path. They work well for small 2D grids, but when you scale up to 6 DOF robotic arms or 3D drone navigation, the configuration and exploration space becomes too large.

[Slide 3: Why Path Planning Matters / Why Is Optimality Hard?]
To overcome the curse of dimensionality, robotics research shifted toward Sampling-Based Planners. The original RRT was introduced by Professor Steven LaValle in 1998, specifically to handle non-holonomic constraints, meaning systems with dynamic turning and movement limits like cars. This was a paradigm shift because it allowed fast exploration of high-dimensional spaces without exhaustive discretization that many other methods require.

[Part 2 - 3:00]
[Slide 4: RRT: How does it work? (Incremental build)]
Let's look at the mathematical algorithm step by step.
Sample: Select a random state $x_{rand}$ in the free space.
Nearest: Find the existing tree node $x_{nearest}$ closest to $x_{rand}$.
Steer: Move a step size $\Delta t$ from $x_{nearest}$ towards $x_{rand}$ to create $x_{new}$.
ObstacleFree: Mathematically verify no obstacle intersects the line.

[Slide 5: RRT Full Demonstration]
[While advancing through the Manim build of the full tree]
See how the tree naturally biases toward unexplored spaces via Voronoi bias, expanding rapidly.

The main strength of RRT is that it is incredibly fast to find a first feasible path and is probabilistically complete, meaning if a path exists, it will find it as iterations approach infinity. However, its critical weakness is that it is not optimal. The path is often jagged and far from the shortest route, which is unacceptable for energy-constrained or smooth-motion applications.

[Part 3 - 3:30]
[Slide 6: RRT* Line-by-Line Optimization]
In 2011, Karaman and Frazzoli presented a monumental upgrade: RRT*. They proved that classic RRT almost surely converges to a sub-optimal solution. RRT*'s novel contribution is introducing asymptotic optimality to sampling-based planning without destroying the fast exploration properties of RRT.

When a new node $x_{new}$ is added, we don't just connect it to $x_{nearest}$. We run a Near radius search within a dynamic radius $r$, proportional to $(\log n / n)^{1/d}$, for all nearby nodes.

Then in ChooseParent, we evaluate the total path cost from the start to $x_{new}$ through all near nodes and pick the path with minimum cost as the parent.

Then Rewire, which is the core contribution. We look back at those near nodes. If connecting them through $x_{new}$ lowers their total cost from the start, we delete their old parent edge and rewire them to $x_{new}$.

[Slide 7: RRT* Full Demonstration]
[While playing the rewiring animation]
You can see edges dynamically shifting as lower-cost paths are discovered, continually smoothing the tree as it expands. This anytime property means it constantly optimizes.

[Part 4 - 1:30]
[Slide 8: RRT vs RRT* Outcome / Strengths and Drawbacks]
Why is this critical today? In our studies and the broader robotics industry, efficiency and safety are paramount. While RRT gets a robot out of a potentially multi-dimensional maze quickly, traversing an unoptimized, jagged path wastes robot battery, introduces unnecessary wear on actuators, and increases travel time. Although seemingly minor, these costs add up.

[Slide 9: Significance & Extended Impact]
The fundamental rewiring concept of RRT* spawned a whole family of algorithms, including Informed RRT*, which focuses search in elliptical subsets, and Anytime RRT*, which executes a committed trajectory while improving the rest.

By spending slightly more computational time doing local rewiring, RRT* bridges the gap between the speed of sampling-based planners and the optimality of grid-search. It provides the smoothness and cost optimization required for real-world deployment on constrained platforms like car-like robots and UAVs.

[Slide 10: References]
[Pause briefly to show references and conclude]
