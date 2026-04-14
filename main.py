from manim import *
import random
import math
from manim_slides import Slide

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def distance(n1, n2):
    return math.hypot(n1.x - n2.x, n1.y - n2.y)

def check_collision(obstacles, n1, n2, width, height, step=0.1):
    dist = distance(n1, n2)
    if dist == 0:
        return False
    steps = int(dist / step) + 1
    for i in range(steps + 1):
        t = i / steps if steps > 0 else 0
        x = n1.x + t * (n2.x - n1.x)
        y = n1.y + t * (n2.y - n1.y)
        if x < 0 or y < 0 or x >= width or y >= height:
            return True
        for (xmin, ymin, xmax, ymax) in obstacles:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
    return False

def get_random_node(width, height, obstacles, rng=None):
    rng = rng or random
    while True:
        x = rng.uniform(0, width)
        y = rng.uniform(0, height)
        in_obs = False
        for (xmin, ymin, xmax, ymax) in obstacles:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                in_obs = True
                break
        if not in_obs:
            return Node(x, y)

def generate_obstacles(num_obstacles, width, height, start, goal, clear_radius=1.5, rng=None):
    rng = rng or random
    obstacles = []
    for _ in range(num_obstacles):
        w = rng.uniform(1.5, 4.0)
        h = rng.uniform(1.5, 4.0)
        # Limit the maximum y-coordinate to prevent obstacles from overlapping the title at the top
        x = rng.uniform(0, width - w)
        y = rng.uniform(0, height - h - 2.0)
        xmin, ymin, xmax, ymax = x, y, x + w, y + h
        
        # Ensure start and goal are not inside or too close to the obstacle
        def overlaps(px, py):
            return (xmin - clear_radius <= px <= xmax + clear_radius and 
                    ymin - clear_radius <= py <= ymax + clear_radius)
                    
        if overlaps(start.x, start.y) or overlaps(goal.x, goal.y):
            continue
            
        obstacles.append((xmin, ymin, xmax, ymax))
    return obstacles

def extract_path(nodes, goal_target, solved_goal=None):
    if solved_goal is not None:
        curr = solved_goal
    else:
        curr = min(nodes, key=lambda n: distance(n, goal_target))

    path = []
    while curr is not None:
        path.append(curr)
        curr = curr.parent
    path.reverse()
    return path


def run_rrt_trace(
    start,
    goal,
    obstacles,
    width,
    height,
    max_iter=500,
    step_size=1.0,
    star=False,
    rng=None,
    goal_sample_rate=0.1,
    radius=3.0,
    trace_limit=None,
    terminate_on_goal=True,
):
    rng = rng or random

    start_node = Node(start.x, start.y)
    goal_target = Node(goal.x, goal.y)
    nodes = [start_node]
    edits_to_animate = []
    traces = []
    solved_goal = None

    for iteration in range(max_iter):
        goal_biased = rng.random() < goal_sample_rate
        rand_node = Node(goal_target.x, goal_target.y) if goal_biased else get_random_node(width, height, obstacles, rng=rng)

        nearest = min(nodes, key=lambda n: distance(n, rand_node))
        dist = distance(nearest, rand_node)
        if dist == 0:
            continue

        theta = math.atan2(rand_node.y - nearest.y, rand_node.x - nearest.x)
        current_step = min(step_size, dist)
        new_node = Node(
            nearest.x + current_step * math.cos(theta),
            nearest.y + current_step * math.sin(theta),
        )

        in_collision = check_collision(obstacles, nearest, new_node, width, height)
        step_trace = {
            "iteration": iteration + 1,
            "goal_biased": goal_biased,
            "rand_node": rand_node,
            "nearest": nearest,
            "new_node": new_node,
            "collision": in_collision,
            "accepted": False,
            "parent": None,
            "near_nodes": [],
            "candidate_costs": [],
            "rewired": [],
            "goal_connected": False,
        }

        if in_collision:
            traces.append(step_trace)
            if trace_limit is not None and len(traces) >= trace_limit:
                break
            continue

        if not star:
            new_node.parent = nearest
            new_node.cost = nearest.cost + distance(nearest, new_node)
            nodes.append(new_node)
            edits_to_animate.append(("add", nearest, new_node))

            step_trace["accepted"] = True
            step_trace["parent"] = nearest

            if distance(new_node, goal_target) < step_size and not check_collision(obstacles, new_node, goal_target, width, height):
                goal_node = Node(goal_target.x, goal_target.y)
                goal_node.parent = new_node
                goal_node.cost = new_node.cost + distance(new_node, goal_node)
                nodes.append(goal_node)
                edits_to_animate.append(("add", new_node, goal_node))
                solved_goal = goal_node
                step_trace["goal_connected"] = True

                traces.append(step_trace)
                if terminate_on_goal:
                    break
                if trace_limit is not None and len(traces) >= trace_limit:
                    break
                continue

            traces.append(step_trace)
            if trace_limit is not None and len(traces) >= trace_limit:
                break
            continue

        near_nodes = [
            n
            for n in nodes
            if distance(n, new_node) <= radius and not check_collision(obstacles, n, new_node, width, height)
        ]

        min_cost_node = nearest
        min_cost = nearest.cost + distance(nearest, new_node)
        candidate_costs = [(nearest, min_cost)]

        for n in near_nodes:
            candidate_cost = n.cost + distance(n, new_node)
            candidate_costs.append((n, candidate_cost))
            if candidate_cost < min_cost:
                min_cost_node = n
                min_cost = candidate_cost

        new_node.parent = min_cost_node
        new_node.cost = min_cost
        nodes.append(new_node)
        edits_to_animate.append(("add", min_cost_node, new_node))

        rewired_nodes = []
        for n in near_nodes:
            if n is min_cost_node:
                continue
            rewire_cost = new_node.cost + distance(new_node, n)
            if rewire_cost + 1e-9 < n.cost:
                old_parent = n.parent
                n.parent = new_node
                n.cost = rewire_cost
                rewired_nodes.append((n, old_parent, new_node))
                edits_to_animate.append(("rewire", n, old_parent, new_node))

        step_trace["accepted"] = True
        step_trace["parent"] = min_cost_node
        step_trace["near_nodes"] = near_nodes
        step_trace["candidate_costs"] = candidate_costs
        step_trace["rewired"] = rewired_nodes

        if distance(new_node, goal_target) < step_size and not check_collision(obstacles, new_node, goal_target, width, height):
            goal_node = Node(goal_target.x, goal_target.y)
            goal_node.parent = new_node
            goal_node.cost = new_node.cost + distance(new_node, goal_node)
            nodes.append(goal_node)
            edits_to_animate.append(("add", new_node, goal_node))
            solved_goal = goal_node
            step_trace["goal_connected"] = True

            traces.append(step_trace)
            if terminate_on_goal:
                break
            if trace_limit is not None and len(traces) >= trace_limit:
                break
            continue

        traces.append(step_trace)
        if trace_limit is not None and len(traces) >= trace_limit:
            break

    path = extract_path(nodes, goal_target, solved_goal=solved_goal)
    return {
        "start": start_node,
        "goal": goal_target,
        "solved_goal": solved_goal,
        "nodes": nodes,
        "edits": edits_to_animate,
        "path": path,
        "trace": traces,
    }


def run_rrt(start, goal, obstacles, width, height, max_iter=500, step_size=1.0, star=False):
    result = run_rrt_trace(
        start,
        goal,
        obstacles,
        width,
        height,
        max_iter=max_iter,
        step_size=step_size,
        star=star,
    )
    return result["nodes"], result["edits"], result["path"]

class MazePathPlanning(Slide):
    skip_reversing = True
    
    def construct(self):
        random.seed(42)

        def set_caption(caption_obj, text, run_time=0.6):
            new_caption = Text(text, font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)
            self.play(Transform(caption_obj, new_caption), run_time=run_time)

        def highlight_code(code_group, active_idx, color=YELLOW, run_time=0.4):
            anims = []
            for idx, line in enumerate(code_group):
                target_color = color if idx in active_idx else WHITE
                anims.append(line.animate.set_color(target_color))
            self.play(*anims, run_time=run_time)

        def build_map(width, height, scale_factor, start, goal, obstacles):
            bg_rect = Rectangle(width=width * scale_factor, height=height * scale_factor)
            bg_rect.set_fill(DARK_GRAY, opacity=1.0)
            bg_rect.set_stroke(WHITE, width=2)

            grid = VGroup()
            for x in range(width + 1):
                x_coord = (x - width / 2) * scale_factor
                grid.add(Line([x_coord, -height / 2 * scale_factor, 0], [x_coord, height / 2 * scale_factor, 0], color=GRAY, stroke_width=1))
            for y in range(height + 1):
                y_coord = (y - height / 2) * scale_factor
                grid.add(Line([-width / 2 * scale_factor, y_coord, 0], [width / 2 * scale_factor, y_coord, 0], color=GRAY, stroke_width=1))

            obs_group = VGroup()
            for (xmin, ymin, xmax, ymax) in obstacles:
                w = xmax - xmin
                h = ymax - ymin
                rect = Rectangle(width=w * scale_factor, height=h * scale_factor)
                rect.set_fill(WHITE, opacity=1.0)
                rect.set_stroke(WHITE, opacity=1.0)
                center_x = (xmin + w / 2 - width / 2) * scale_factor
                center_y = (ymin + h / 2 - height / 2) * scale_factor
                rect.move_to([center_x, center_y, 0])
                obs_group.add(rect)

            def to_coord_local(node):
                return np.array([(node.x - width / 2) * scale_factor, (node.y - height / 2) * scale_factor, 0])

            start_dot = Dot(to_coord_local(start), color=GREEN).scale(1.2)
            goal_dot = Dot(to_coord_local(goal), color=RED).scale(1.2)
            map_group = VGroup(bg_rect, grid, obs_group, start_dot, goal_dot)

            # Use map_group center so coordinates stay correct after moving the map on screen.
            def to_coord(node):
                return map_group.get_center() + to_coord_local(node)

            return map_group, to_coord, start_dot, goal_dot

        # ---------------------------------------------------------
        # INTRO SLIDE A: Path planning fundamentals
        # ---------------------------------------------------------
        intro_a_title = Text("What Is Path Planning?", font_size=52).to_edge(UP)
        intro_a_subtitle = Text(
            "Find a collision-free route from start A to goal B in free space.",
            font_size=28,
            color=LIGHT_GRAY,
        ).next_to(intro_a_title, DOWN, buff=0.3)

        intro_a_world = Rectangle(width=8.8, height=4.9)
        intro_a_world.set_fill(DARK_GRAY, opacity=1.0)
        intro_a_world.set_stroke(WHITE, width=2)
        intro_a_world.shift(DOWN * 0.35)

        intro_a_start = Dot(intro_a_world.get_left() + RIGHT * 1.0 + DOWN * 1.4, color=GREEN).scale(1.2)
        intro_a_goal = Dot(intro_a_world.get_right() + LEFT * 1.0 + UP * 1.4, color=RED).scale(1.2)
        intro_a_start_label = Text("A", font_size=24, color=GREEN).next_to(intro_a_start, DOWN, buff=0.08)
        intro_a_goal_label = Text("B", font_size=24, color=RED).next_to(intro_a_goal, DOWN, buff=0.08)
        intro_a_line = Line(intro_a_start.get_center(), intro_a_goal.get_center(), color=YELLOW, stroke_width=5)

        intro_a_caption = Text(
            "Without obstacles, a direct route is usually feasible.",
            font_size=22,
            color=LIGHT_GRAY,
        ).to_edge(DOWN).shift(UP * 0.1)

        self.play(FadeIn(intro_a_title), FadeIn(intro_a_subtitle), FadeIn(intro_a_world), run_time=1.1)
        self.play(
            FadeIn(intro_a_start),
            FadeIn(intro_a_goal),
            FadeIn(intro_a_start_label),
            FadeIn(intro_a_goal_label),
            run_time=0.9,
        )
        self.play(Create(intro_a_line), FadeIn(intro_a_caption), run_time=1.0)
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # INTRO SLIDE B: Obstacles break the straight-line solution
        # ---------------------------------------------------------
        intro_b_title = Text("Now Add Obstacles", font_size=50, color=WHITE).to_edge(UP)
        intro_b_subtitle = Text(
            "A naive straight line can collide with forbidden space.",
            font_size=28,
            color=LIGHT_GRAY,
        ).next_to(intro_b_title, DOWN, buff=0.3)

        intro_b_world = Rectangle(width=8.8, height=4.9)
        intro_b_world.set_fill(DARK_GRAY, opacity=1.0)
        intro_b_world.set_stroke(WHITE, width=2)
        intro_b_world.shift(DOWN * 0.35)

        intro_b_start = Dot(intro_b_world.get_left() + RIGHT * 1.0 + DOWN * 1.4, color=GREEN).scale(1.2)
        intro_b_goal = Dot(intro_b_world.get_right() + LEFT * 1.0 + UP * 1.4, color=RED).scale(1.2)
        intro_b_start_label = Text("A", font_size=24, color=GREEN).next_to(intro_b_start, DOWN, buff=0.08)
        intro_b_goal_label = Text("B", font_size=24, color=RED).next_to(intro_b_goal, DOWN, buff=0.08)

        intro_b_obstacle_1 = Rectangle(width=1.6, height=2.6)
        intro_b_obstacle_1.set_fill(WHITE, opacity=1.0)
        intro_b_obstacle_1.set_stroke(WHITE, opacity=1.0)
        intro_b_obstacle_1.move_to(intro_b_world.get_center() + LEFT * 0.8 + DOWN * 0.1)

        intro_b_obstacle_2 = Rectangle(width=1.8, height=1.6)
        intro_b_obstacle_2.set_fill(WHITE, opacity=1.0)
        intro_b_obstacle_2.set_stroke(WHITE, opacity=1.0)
        intro_b_obstacle_2.move_to(intro_b_world.get_center() + RIGHT * 1.4 + UP * 0.95)

        intro_b_blocked_line = Line(intro_b_start.get_center(), intro_b_goal.get_center(), color=RED, stroke_width=5)
        intro_b_mid = intro_b_blocked_line.point_from_proportion(0.52)
        intro_b_collision_mark = VGroup(
            Line(intro_b_mid + LEFT * 0.28 + UP * 0.28, intro_b_mid + RIGHT * 0.28 + DOWN * 0.28, color=RED, stroke_width=6),
            Line(intro_b_mid + LEFT * 0.28 + DOWN * 0.28, intro_b_mid + RIGHT * 0.28 + UP * 0.28, color=RED, stroke_width=6),
        )

        # Route below both obstacles, then climb on the far right to stay collision-free.
        intro_b_waypoint_1 = intro_b_start.get_center() + RIGHT * 0.7 + DOWN * 0.55
        intro_b_waypoint_2 = intro_b_world.get_right() + LEFT * 1.0 + DOWN * 1.95
        intro_b_waypoint_3 = intro_b_world.get_right() + LEFT * 1.0 + UP * 1.1
        intro_b_detour = VGroup(
            Line(intro_b_start.get_center(), intro_b_waypoint_1, color=BLUE, stroke_width=5),
            Line(intro_b_waypoint_1, intro_b_waypoint_2, color=BLUE, stroke_width=5),
            Line(intro_b_waypoint_2, intro_b_waypoint_3, color=BLUE, stroke_width=5),
            Line(intro_b_waypoint_3, intro_b_goal.get_center(), color=BLUE, stroke_width=5),
        )

        intro_b_caption = Text(
            "This is why autonomous robots need search-based planners.",
            font_size=22,
            color=LIGHT_GRAY,
        ).to_edge(DOWN).shift(UP * 0.1)

        self.play(FadeIn(intro_b_title), FadeIn(intro_b_subtitle), FadeIn(intro_b_world), run_time=1.0)
        self.play(
            FadeIn(intro_b_start),
            FadeIn(intro_b_goal),
            FadeIn(intro_b_start_label),
            FadeIn(intro_b_goal_label),
            FadeIn(intro_b_obstacle_1),
            FadeIn(intro_b_obstacle_2),
            run_time=1.0,
        )
        self.play(Create(intro_b_blocked_line), Create(intro_b_collision_mark), run_time=0.9)
        self.play(Create(intro_b_detour), FadeIn(intro_b_caption), run_time=1.2)
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # INTRO SLIDE C: Why this problem matters (image-driven)
        # ---------------------------------------------------------
        intro_c_title = Text("Why Path Planning Matters", font_size=50).to_edge(UP)

        def make_bullet(text, font_size=20, color=LIGHT_GRAY):
            dot = Dot(radius=0.055, color=WHITE)
            line = Text(text, font_size=font_size, color=color)
            line.scale_to_fit_width(5.6)
            return VGroup(dot, line).arrange(RIGHT, buff=0.16, aligned_edge=UP)

        intro_c_points_title = Text("Why this matters:", font_size=30, color=WHITE)
        intro_c_importance = VGroup(
            make_bullet("Safety: avoid collisions around people and assets."),
            make_bullet("Efficiency: reduce path length, time, and battery use."),
            make_bullet("Reliability: maintain performance in cluttered scenes."),
            make_bullet("Path planning is a core capability for autonomy.", color=YELLOW),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.18)
        intro_c_points_block = VGroup(intro_c_points_title, intro_c_importance).arrange(DOWN, aligned_edge=LEFT, buff=0.2)

        def make_image_card(image_path, app_label, ref_text, max_width=4.0, max_height=1.95):
            image = ImageMobject(image_path)
            image.set_z_index(2)
            image.scale_to_fit_width(max_width)
            if image.height > max_height:
                image.scale_to_fit_height(max_height)

            frame = RoundedRectangle(
                width=image.width + 0.22,
                height=image.height + 0.22,
                corner_radius=0.08,
            )
            frame.set_fill(BLACK, opacity=0.2)
            frame.set_stroke(GRAY, width=2)
            frame.move_to(image.get_center())

            caption_title = Text(app_label, font_size=16, color=WHITE)
            caption_ref = Text(ref_text, font_size=11, color=GRAY)
            caption_ref.scale_to_fit_width(frame.width)
            caption = VGroup(caption_title, caption_ref).arrange(DOWN, aligned_edge=LEFT, buff=0.02)
            caption.next_to(frame, DOWN, buff=0.08).align_to(frame, LEFT)

            return Group(frame, image, caption)

        intro_c_top_left_image = make_image_card(
            "images/realtime-robotics.jpeg",
            "Autonomous cars",
            "(Solving the Autonomous Vehicles Motion Planning Conundrum, 2019)",
        )
        intro_c_top_right_image = make_image_card(
            "images/robot-arm-planning.jpeg",
            "Industrial robotics",
            "(Paes et al., 2014)",
        )
        intro_c_bottom_right_image = make_image_card(
            "images/racing-drones.jpeg",
            "Autonomous drones",
            "(IEEE Spectrum, 2023)",
        )

        # Keep each visual centered within its intended quadrant.
        intro_c_top_left_image.move_to(LEFT * 3.45 + UP * 1.45)
        intro_c_top_right_image.move_to(RIGHT * 3.45 + UP * 1.45)
        intro_c_bottom_right_image.move_to(RIGHT * 3.45 + DOWN * 1.85)

        intro_c_points_block.move_to(LEFT * 3.45 + DOWN * 1.95)

        self.play(FadeIn(intro_c_title), run_time=0.8)
        self.play(
            FadeIn(intro_c_top_left_image),
            FadeIn(intro_c_top_right_image),
            FadeIn(intro_c_bottom_right_image),
            FadeIn(intro_c_points_block),
            run_time=1.6,
        )
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # INTRO SLIDE D: Why this is hard to solve
        # ---------------------------------------------------------
        intro_d_title = Text("Why It Is Hard to Solve", font_size=48).to_edge(UP)

        def make_challenge_bullet(text, color=LIGHT_GRAY):
            dot = Dot(radius=0.065, color=WHITE)
            line = Text(text, font_size=27, color=color)
            if line.width > 6.15:
                line.scale_to_fit_width(6.15)
            return VGroup(dot, line).arrange(RIGHT, buff=0.18, aligned_edge=UP)

        intro_d_left_title = Text("Key complexities:", font_size=36, color=WHITE)

        intro_d_points = VGroup(
            make_challenge_bullet("Configuration space grows rapidly with DOF."),
            make_challenge_bullet("Obstacles create narrow valid corridors."),
            make_challenge_bullet("Nonholonomic robots cannot move arbitrarily."),
            make_challenge_bullet("We need speed first, then better path quality.", color=YELLOW),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.24)
        intro_d_left_block = VGroup(intro_d_left_title, intro_d_points).arrange(DOWN, aligned_edge=LEFT, buff=0.28)
        intro_d_left_block.next_to(intro_d_title, DOWN, buff=0.54).to_edge(LEFT, buff=0.62)

        intro_d_visual = VGroup(
            VGroup(
                Text("Search space grows roughly as", font_size=30, color=YELLOW),
                MathTex(r"k^d", font_size=54, color=YELLOW),
            ).arrange(RIGHT, buff=0.12, aligned_edge=DOWN),
            Text("Higher dimension means many more states to evaluate.", font_size=24, color=LIGHT_GRAY),
            Arrow(LEFT * 2.2, RIGHT * 2.2, buff=0.0, color=BLUE),
            Text("Need scalable planning methods", font_size=28, color=WHITE),
        ).arrange(DOWN, buff=0.23)
        intro_d_visual.to_corner(DR, buff=0.72).shift(DOWN * 0.22)

        self.play(FadeIn(intro_d_title), run_time=0.8)
        self.play(FadeIn(intro_d_left_block), FadeIn(intro_d_visual), run_time=1.5)
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # SLIDE 1: Title and Context
        # ---------------------------------------------------------
        title1 = Text("Path Planning in Robotics", font_size=50).shift(UP * 1.1)
        title2 = Text("RRT* for Optimised Path Planning", font_size=52, color=BLUE).next_to(title1, DOWN, buff=0.25)
        subtitle = Text("Fast Exploration vs Optimal Navigation", font_size=30, color=LIGHT_GREY).next_to(title2, DOWN, buff=0.4)

        refs = VGroup(
            Text("[1] LaValle (1998): Rapidly-exploring Random Trees", font_size=22),
            Text("[2] Karaman & Frazzoli (2011): Asymptotic Optimality", font_size=22),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.15).next_to(subtitle, DOWN, buff=0.8)

        self.play(Write(title1), Write(title2), run_time=1.4)
        self.play(FadeIn(subtitle), FadeIn(refs), run_time=1.0)
        self.next_slide()
        self.play(FadeOut(title1), FadeOut(title2), FadeOut(subtitle), FadeOut(refs))

        # ---------------------------------------------------------
        # Shared deterministic setup for granular slides
        # ---------------------------------------------------------
        small_width, small_height = 12, 8
        small_scale = 0.58
        small_start = Node(1.2, 1.0)
        small_goal = Node(10.5, 6.8)
        small_obstacles = generate_obstacles(
            6,
            small_width,
            small_height,
            small_start,
            small_goal,
            clear_radius=1.0,
            rng=random.Random(101),
        )

        rrt_detail = run_rrt_trace(
            small_start,
            small_goal,
            small_obstacles,
            small_width,
            small_height,
            max_iter=250,
            step_size=0.85,
            star=False,
            rng=random.Random(202),
            goal_sample_rate=0.08,
            trace_limit=160,
            terminate_on_goal=False,
        )

        rrt_star_detail = run_rrt_trace(
            small_start,
            small_goal,
            small_obstacles,
            small_width,
            small_height,
            max_iter=320,
            step_size=0.85,
            star=True,
            rng=random.Random(303),
            goal_sample_rate=0.08,
            radius=2.6,
            trace_limit=220,
            terminate_on_goal=False,
        )

        # ---------------------------------------------------------
        # SLIDE 2: Hyper-detailed RRT Iterations
        # ---------------------------------------------------------
        rrt_title = Tex(r"\textbf{RRT: How does it work?}", font_size=40, color=BLUE).to_corner(UL)
        self.play(FadeIn(rrt_title))

        pseudo_code_rrt = VGroup(
            Tex(r"$x_{rand} \leftarrow \text{SampleFree}(X_{free})$", font_size=24),
            Tex(r"$x_{nearest} \leftarrow \text{Nearest}(\mathcal{T}, x_{rand})$", font_size=24),
            Tex(r"$x_{new} \leftarrow \text{Steer}(x_{nearest}, x_{rand}, \Delta t)$", font_size=24),
            Tex(r"\textbf{if} $\text{ObstacleFree}(x_{nearest}, x_{new})$ \textbf{then}", font_size=24),
            Tex(r"$\mathcal{T}.\text{add\_vertex}(x_{new})$", font_size=24),
            Tex(r"$\mathcal{T}.\text{add\_edge}(x_{nearest}, x_{new})$", font_size=24),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.2).next_to(rrt_title, DOWN, buff=0.35).align_to(rrt_title, LEFT)
        pseudo_code_rrt[4].shift(RIGHT * 0.45)
        pseudo_code_rrt[5].shift(RIGHT * 0.45)
        self.play(Write(pseudo_code_rrt), run_time=1.5)

        small_map_rrt, to_coord_small, _, _ = build_map(
            small_width,
            small_height,
            small_scale,
            rrt_detail["start"],
            rrt_detail["goal"],
            small_obstacles,
        )
        small_map_rrt.to_edge(RIGHT, buff=0.55).shift(RIGHT * 0.1 + DOWN * 0.35)
        self.play(FadeIn(small_map_rrt))

        iter_label_rrt = Tex(r"\textbf{Iteration: }0", font_size=28, color=WHITE).to_corner(UR).shift(DOWN * 0.8)
        caption_rrt = Text("We now run RRT line-by-line using seeded random samples.", font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)
        footnote_rrt = Text("[1] LaValle, S. M. (1998). Rapidly-exploring random trees", font_size=16, color=GRAY).to_corner(DL).shift(RIGHT * 0.1 + UP * 0.05)
        self.play(FadeIn(iter_label_rrt), FadeIn(caption_rrt), FadeIn(footnote_rrt))
        self.next_slide()

        rrt_tree_dots = {rrt_detail["start"]: small_map_rrt[-2]}
        rrt_tree_edges = {}
        rrt_steps_to_show = 10 # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Changable number of iterations to show in detail
        trace_rrt = rrt_detail["trace"][:rrt_steps_to_show]

        for step in trace_rrt:
            new_iter_label = Tex(rf"\textbf{{Iteration: }}{step['iteration']}", font_size=28, color=WHITE).to_corner(UR).shift(DOWN * 0.8)
            self.play(Transform(iter_label_rrt, new_iter_label), run_time=0.4)

            highlight_code(pseudo_code_rrt, [0])
            set_caption(caption_rrt, "Step 1: We randomly sample a target state in free space.")
            rand_dot = Dot(to_coord_small(step["rand_node"]), color=YELLOW).scale(0.7)
            rand_label = Tex(r"$x_{rand}$", font_size=20, color=YELLOW).next_to(rand_dot, UP, buff=0.05)
            self.play(FadeIn(rand_dot), FadeIn(rand_label), run_time=0.9)
            self.next_slide()

            highlight_code(pseudo_code_rrt, [1])
            set_caption(caption_rrt, "Step 2: We find the existing tree node closest to the sample.")
            nearest_node = step["nearest"]
            nearest_dot = rrt_tree_dots[nearest_node]
            nearest_label = Tex(r"$x_{nearest}$", font_size=20, color=GREEN).next_to(nearest_dot, LEFT, buff=0.05)
            guide_to_rand = DashedLine(nearest_dot.get_center(), rand_dot.get_center(), color=GRAY, dashed_ratio=0.5)
            self.play(Create(guide_to_rand), FadeIn(nearest_label), nearest_dot.animate.set_color(GREEN), run_time=1.1)
            self.next_slide()

            highlight_code(pseudo_code_rrt, [2])
            set_caption(caption_rrt, "Step 3: We steer only one step toward the random sample.")
            new_temp = Dot(to_coord_small(step["new_node"]), color=RED).scale(0.75)
            new_label = Tex(r"$x_{new}$", font_size=20, color=RED).next_to(new_temp, DOWN, buff=0.05)
            steer_line = DashedLine(nearest_dot.get_center(), new_temp.get_center(), color=YELLOW, dashed_ratio=0.55)
            self.play(FadeIn(new_temp), FadeIn(new_label), Create(steer_line), run_time=1.0)
            self.next_slide()

            highlight_code(pseudo_code_rrt, [3])
            set_caption(caption_rrt, "Step 4: Collision check decides if we can safely add this node.")
            collision_color = RED if step["collision"] else GREEN
            collision_text = "Collision detected: reject this sample." if step["collision"] else "No collision: we can add this node to the tree."
            collision_tag = Text(collision_text, font_size=18, color=collision_color).next_to(caption_rrt, UP, buff=0.1)
            self.play(FadeIn(collision_tag), run_time=0.8)
            self.next_slide()

            if step["collision"]:
                set_caption(caption_rrt, "The candidate intersects an obstacle, so this iteration ends.")
                self.play(
                    FadeOut(collision_tag),
                    FadeOut(steer_line),
                    FadeOut(guide_to_rand),
                    FadeOut(new_temp),
                    FadeOut(new_label),
                    FadeOut(rand_dot),
                    FadeOut(rand_label),
                    FadeOut(nearest_label),
                    nearest_dot.animate.set_color(BLUE),
                    run_time=1.0,
                )
                self.next_slide()
                continue

            highlight_code(pseudo_code_rrt, [4, 5])
            set_caption(caption_rrt, "Step 5 and 6: Add the new node and connect it with an edge.")
            parent = step["parent"]
            parent_dot = rrt_tree_dots[parent]
            new_edge = Line(parent_dot.get_center(), new_temp.get_center(), color=BLUE, stroke_width=2)
            self.play(FadeOut(steer_line), FadeOut(guide_to_rand), Create(new_edge), new_temp.animate.set_color(BLUE), run_time=1.2)
            rrt_tree_dots[step["new_node"]] = new_temp
            rrt_tree_edges[(parent, step["new_node"])] = new_edge

            self.play(
                FadeOut(collision_tag),
                FadeOut(rand_dot),
                FadeOut(rand_label),
                FadeOut(nearest_label),
                FadeOut(new_label),
                parent_dot.animate.set_color(BLUE),
                run_time=0.8,
            )
            self.next_slide()

        highlight_code(pseudo_code_rrt, [])
        set_caption(caption_rrt, "After many repeated iterations, RRT quickly explores free space.")
        self.next_slide()

        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # SLIDE 3: Full-length RRT Growth (slower)
        # ---------------------------------------------------------
        full_rrt_title = Tex(r"\textbf{RRT Full Demonstration}", font_size=40, color=BLUE).to_edge(UP)
        self.play(FadeIn(full_rrt_title))

        full_width, full_height = 21, 13
        full_scale = 0.48
        full_start = Node(2, 2)
        full_goal = Node(full_width - 3, full_height - 3)
        full_obstacles = generate_obstacles(14, full_width, full_height, full_start, full_goal, rng=random.Random(501))

        full_rrt_result = run_rrt_trace(
            full_start,
            full_goal,
            full_obstacles,
            full_width,
            full_height,
            max_iter=1400,
            step_size=0.95,
            star=False,
            rng=random.Random(777),
            goal_sample_rate=0.08,
            terminate_on_goal=True,
        )

        map_full_rrt, to_coord_full, _, _ = build_map(
            full_width,
            full_height,
            full_scale,
            full_rrt_result["start"],
            full_rrt_result["goal"],
            full_obstacles,
        )
        map_full_rrt.to_edge(DOWN, buff=0.45)
        self.play(FadeIn(map_full_rrt))

        caption_full_rrt = Text("RRT grows quickly, but path quality is usually jagged.", font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)
        self.play(FadeIn(caption_full_rrt))
        self.next_slide()

        rrt_full_lines = []
        for op in full_rrt_result["edits"]:
            if op[0] == "add":
                rrt_full_lines.append(Line(to_coord_full(op[1]), to_coord_full(op[2]), color=BLUE, stroke_width=2))

        batch_size = 30
        for idx in range(0, len(rrt_full_lines), batch_size):
            chunk = rrt_full_lines[idx:idx + batch_size]
            if not chunk:
                continue
            self.play(LaggedStart(*[Create(line) for line in chunk], lag_ratio=0.08), run_time=2.8)
            if idx % (batch_size * 2) == 0:
                self.next_slide()

        rrt_full_path = VGroup(
            *[
                Line(to_coord_full(full_rrt_result["path"][i]), to_coord_full(full_rrt_result["path"][i + 1]), color=YELLOW, stroke_width=5)
                for i in range(max(0, len(full_rrt_result["path"]) - 1))
            ]
        )
        self.play(Create(rrt_full_path), run_time=2.5)
        set_caption(caption_full_rrt, "This is a valid path, but it is not globally optimal.")
        self.next_slide()

        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # SLIDE 4: Hyper-detailed RRT* Iterations
        # ---------------------------------------------------------
        star_title = Tex(r"\textbf{RRT* Line-by-Line Optimization}", font_size=40, color=ORANGE).to_corner(UL)
        self.play(FadeIn(star_title))

        pseudo_code_star = VGroup(
            Tex(r"$x_{rand} \leftarrow \text{SampleFree}(X_{free})$", font_size=20),
            Tex(r"$x_{nearest} \leftarrow \text{Nearest}(\mathcal{T}, x_{rand})$", font_size=20),
            Tex(r"$x_{new} \leftarrow \text{Steer}(x_{nearest}, x_{rand}, \Delta t)$", font_size=20),
            Tex(r"\textbf{if} $\text{ObstacleFree}(x_{nearest}, x_{new})$ \textbf{then}", font_size=20),
            Tex(r"$X_{near} \leftarrow \text{Near}(\mathcal{T}, x_{new}, r_n)$", font_size=20),
            Tex(r"$x_{min} \leftarrow x_{nearest}, \; c_{min} \leftarrow \text{Cost}(x_{nearest}) + c(\text{Line}(x_{nearest}, x_{new}))$", font_size=20),
            Tex(r"\textbf{for each} $x_{near} \in X_{near}$", font_size=20),
            Tex(r"\textbf{if} $\text{ObstacleFree}(x_{near}, x_{new})$ and cheaper: update $x_{min}$", font_size=20),
            Tex(r"$\mathcal{T}.\text{add\_edge}(x_{min}, x_{new})$", font_size=20),
            Tex(r"\textbf{for each} $x_{near} \in X_{near} \setminus \{x_{min}\}$", font_size=20),
            Tex(r"\textbf{if} $\text{ObstacleFree}(x_{new}, x_{near})$ and cheaper: \textbf{rewire}", font_size=20),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.14).next_to(star_title, DOWN, buff=0.28).align_to(star_title, LEFT)
        for idx in [4, 5, 6, 8, 9]:
            pseudo_code_star[idx].shift(RIGHT * 0.45)
        for idx in [7, 10]:
            pseudo_code_star[idx].shift(RIGHT * 0.90)
        self.play(Write(pseudo_code_star), run_time=1.6)

        small_map_star, to_coord_small_star, _, _ = build_map(
            small_width,
            small_height,
            small_scale,
            rrt_star_detail["start"],
            rrt_star_detail["goal"],
            small_obstacles,
        )
        small_map_star.to_edge(RIGHT, buff=0.55).shift(RIGHT * 0.1 + DOWN * 0.35)
        self.play(FadeIn(small_map_star))

        iter_label_star = Tex(r"\textbf{Accepted Node: }0", font_size=28, color=WHITE).to_corner(UR).shift(DOWN * 0.8)
        caption_star = Text("RRT* uses the same samples, then optimizes local connections.", font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)
        footnote_star = Text("[2] Karaman & Frazzoli (2011). Sampling-based algorithms for optimal motion planning.", font_size=16, color=GRAY).to_corner(DL).shift(RIGHT * 0.1 + UP * 0.05)
        
        self.play(FadeIn(iter_label_star), FadeIn(caption_star), FadeIn(footnote_star))
        self.next_slide()

        star_tree_dots = {rrt_star_detail["start"]: small_map_star[-2]}
        star_tree_edges = {}

        accepted_star_trace = [t for t in rrt_star_detail["trace"] if t["accepted"]]
        fast_forward_count = 12 # Number to fast forward
        detailed_count = 5 # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Changable number of iterations to show in detail
        fast_forward_trace = accepted_star_trace[:fast_forward_count]
        selected_star_trace = accepted_star_trace[fast_forward_count:fast_forward_count + detailed_count]

        highlight_code(pseudo_code_star, [])
        set_caption(caption_star, f"Fast-forward: first {len(fast_forward_trace)} accepted RRT* nodes.")
        for step in fast_forward_trace:
            parent = step["parent"]
            if parent not in star_tree_dots:
                continue

            parent_dot = star_tree_dots[parent]
            new_dot = Dot(to_coord_small_star(step["new_node"]), color=ORANGE).scale(0.75)
            new_edge = Line(parent_dot.get_center(), new_dot.get_center(), color=ORANGE, stroke_width=2.4)
            self.play(Create(new_edge), FadeIn(new_dot), run_time=0.16)
            star_tree_dots[step["new_node"]] = new_dot
            star_tree_edges[(parent, step["new_node"])] = new_edge

            for rewired_node, old_parent, new_parent in step["rewired"]:
                if rewired_node not in star_tree_dots or new_parent not in star_tree_dots:
                    continue

                rewired_dot = star_tree_dots[rewired_node]
                old_edge_key = (old_parent, rewired_node)
                rewire_anims = []
                if old_edge_key in star_tree_edges:
                    rewire_anims.append(FadeOut(star_tree_edges[old_edge_key]))
                    del star_tree_edges[old_edge_key]

                new_edge_rewire = Line(
                    star_tree_dots[new_parent].get_center(),
                    rewired_dot.get_center(),
                    color=PURPLE,
                    stroke_width=2.6,
                )
                star_tree_edges[(new_parent, rewired_node)] = new_edge_rewire
                rewire_anims.extend([Create(new_edge_rewire), rewired_dot.animate.set_color(PURPLE)])
                self.play(*rewire_anims, run_time=0.14)

        if fast_forward_trace:
            last_fast_forward_iter = len(fast_forward_trace)
            ff_iter_label = Tex(
                rf"\textbf{{Accepted Node: }}{last_fast_forward_iter}",
                font_size=28,
                color=WHITE,
            ).to_corner(UR).shift(DOWN * 0.8)
            self.play(Transform(iter_label_star, ff_iter_label), run_time=0.35)

        set_caption(
            caption_star,
            f"Now slow mode: line-by-line walkthrough for the next {len(selected_star_trace)} accepted iterations.",
        )
        self.next_slide()

        for accepted_idx, step in enumerate(selected_star_trace, start=len(fast_forward_trace) + 1):
            new_iter_label = Tex(rf"\textbf{{Accepted Node: }}{accepted_idx}", font_size=28, color=WHITE).to_corner(UR).shift(DOWN * 0.8)
            self.play(Transform(iter_label_star, new_iter_label), run_time=0.4)

            highlight_code(pseudo_code_star, [0])
            set_caption(caption_star, "Sample a random target state x_rand.")
            rand_dot = Dot(to_coord_small_star(step["rand_node"]), color=YELLOW).scale(0.7)
            rand_label = Tex(r"$x_{rand}$", font_size=18, color=YELLOW).next_to(rand_dot, UP, buff=0.05)
            self.play(FadeIn(rand_dot), FadeIn(rand_label), run_time=0.8)
            self.next_slide()

            highlight_code(pseudo_code_star, [1])
            set_caption(caption_star, "Find x_nearest in the current tree.")
            nearest_node = step["nearest"]
            nearest_dot = star_tree_dots[nearest_node]
            nearest_label = Tex(r"$x_{nearest}$", font_size=18, color=GREEN).next_to(nearest_dot, LEFT, buff=0.05)
            line_to_rand = DashedLine(nearest_dot.get_center(), rand_dot.get_center(), color=GRAY, dashed_ratio=0.5)
            self.play(Create(line_to_rand), FadeIn(nearest_label), nearest_dot.animate.set_color(GREEN), run_time=1.0)
            self.next_slide()

            highlight_code(pseudo_code_star, [2])
            set_caption(caption_star, "Steer one step from x_nearest toward x_rand.")
            new_temp = Dot(to_coord_small_star(step["new_node"]), color=RED).scale(0.75)
            new_label = Tex(r"$x_{new}$", font_size=18, color=RED).next_to(new_temp, DOWN, buff=0.05)
            steer_line = DashedLine(nearest_dot.get_center(), new_temp.get_center(), color=YELLOW, dashed_ratio=0.55)
            self.play(FadeIn(new_temp), FadeIn(new_label), Create(steer_line), run_time=1.0)
            self.next_slide()

            highlight_code(pseudo_code_star, [3])
            set_caption(caption_star, "Collision check: only safe candidates proceed.")
            if step["collision"]:
                reject_text = Text("Blocked by obstacle, skip this sample.", font_size=18, color=RED).next_to(caption_star, UP, buff=0.1)
                self.play(FadeIn(reject_text), run_time=0.8)
                self.next_slide()
                self.play(
                    FadeOut(reject_text),
                    FadeOut(rand_dot),
                    FadeOut(rand_label),
                    FadeOut(nearest_label),
                    FadeOut(line_to_rand),
                    FadeOut(steer_line),
                    FadeOut(new_label),
                    FadeOut(new_temp),
                    nearest_dot.animate.set_color(ORANGE),
                    run_time=1.0,
                )
                self.next_slide()
                continue

            highlight_code(pseudo_code_star, [4])
            set_caption(caption_star, "Compute X_near: nearby nodes around x_new.")
            near_circle = Circle(radius=2.6 * small_scale, color=ORANGE, stroke_width=2).move_to(new_temp.get_center())
            near_labels = VGroup()
            for near_node in step["near_nodes"]:
                if near_node not in star_tree_dots:
                    continue
                near_dot = star_tree_dots[near_node]
                near_labels.add(Tex(r"$X_{near}$", font_size=14, color=ORANGE).next_to(near_dot, UP, buff=0.02))
            self.play(Create(near_circle), FadeIn(near_labels), run_time=1.1)
            self.next_slide()

            highlight_code(pseudo_code_star, [5, 6, 7])
            set_caption(caption_star, "Evaluate all candidate parents and pick the cheapest x_min.")
            candidate_lines = VGroup()
            for near_node in step["near_nodes"]:
                if near_node in star_tree_dots:
                    candidate_lines.add(DashedLine(star_tree_dots[near_node].get_center(), new_temp.get_center(), color=GRAY, dashed_ratio=0.5))
            if step["nearest"] in star_tree_dots:
                candidate_lines.add(DashedLine(star_tree_dots[step["nearest"]].get_center(), new_temp.get_center(), color=GRAY, dashed_ratio=0.5))
            self.play(Create(candidate_lines), run_time=1.0)

            parent = step["parent"]
            parent_dot = star_tree_dots[parent]
            x_min_label = Tex(r"$x_{min}$", font_size=18, color=ORANGE).next_to(parent_dot, UP, buff=0.05)
            self.play(parent_dot.animate.set_color(ORANGE), FadeIn(x_min_label), run_time=0.8)
            self.next_slide()

            highlight_code(pseudo_code_star, [8])
            set_caption(caption_star, "Attach x_new using the selected parent x_min.")
            new_edge = Line(parent_dot.get_center(), new_temp.get_center(), color=ORANGE, stroke_width=2.4)
            self.play(Create(new_edge), new_temp.animate.set_color(ORANGE), run_time=1.0)
            star_tree_dots[step["new_node"]] = new_temp
            star_tree_edges[(parent, step["new_node"])] = new_edge
            self.next_slide()

            highlight_code(pseudo_code_star, [9, 10])
            if len(step["rewired"]) == 0:
                set_caption(caption_star, "Rewire check found no cheaper local alternatives this round.")
                self.play(FadeOut(candidate_lines), run_time=0.6)
            else:
                set_caption(caption_star, "Rewire: some nearby nodes get cheaper paths through x_new.")
                for rewired_node, old_parent, new_parent in step["rewired"]:
                    if rewired_node not in star_tree_dots:
                        continue
                    rewired_dot = star_tree_dots[rewired_node]
                    old_edge_key = (old_parent, rewired_node)
                    if old_edge_key in star_tree_edges:
                        self.play(FadeOut(star_tree_edges[old_edge_key]), run_time=0.35)
                        del star_tree_edges[old_edge_key]
                    new_edge_rewire = Line(star_tree_dots[new_parent].get_center(), rewired_dot.get_center(), color=PURPLE, stroke_width=2.6)
                    self.play(Create(new_edge_rewire), rewired_dot.animate.set_color(PURPLE), run_time=0.7)
                    star_tree_edges[(new_parent, rewired_node)] = new_edge_rewire
                self.play(FadeOut(candidate_lines), run_time=0.4)

            self.play(
                FadeOut(rand_dot),
                FadeOut(rand_label),
                FadeOut(nearest_label),
                FadeOut(line_to_rand),
                FadeOut(steer_line),
                FadeOut(new_label),
                FadeOut(near_circle),
                FadeOut(near_labels),
                FadeOut(x_min_label),
                parent_dot.animate.set_color(ORANGE),
                run_time=0.9,
            )
            self.next_slide()

        highlight_code(pseudo_code_star, [])
        set_caption(caption_star, "RRT* repeats these local optimizations to improve path quality over time.")
        self.next_slide()

        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # SLIDE 5: Full-length RRT* Growth (slower)
        # ---------------------------------------------------------
        full_star_title = Tex(r"\textbf{RRT* Full Demonstration}", font_size=40, color=ORANGE).to_edge(UP)
        self.play(FadeIn(full_star_title))

        full_star_result = run_rrt_trace(
            full_start,
            full_goal,
            full_obstacles,
            full_width,
            full_height,
            max_iter=1600,
            step_size=0.95,
            star=True,
            rng=random.Random(777),
            goal_sample_rate=0.08,
            radius=2.7,
            terminate_on_goal=True,
        )

        map_full_star, to_coord_full_star, _, _ = build_map(
            full_width,
            full_height,
            full_scale,
            full_star_result["start"],
            full_star_result["goal"],
            full_obstacles,
        )
        map_full_star.to_edge(DOWN, buff=0.45)
        self.play(FadeIn(map_full_star))

        caption_full_star = Text("RRT* grows and rewires, spending more time to reduce total path cost.", font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)
        self.play(FadeIn(caption_full_star))
        self.next_slide()

        star_full_lines = []
        for op in full_star_result["edits"]:
            if op[0] == "add":
                star_full_lines.append(Line(to_coord_full_star(op[1]), to_coord_full_star(op[2]), color=ORANGE, stroke_width=2))
            elif op[0] == "rewire":
                star_full_lines.append(Line(to_coord_full_star(op[3]), to_coord_full_star(op[1]), color=PURPLE, stroke_width=2.3))

        batch_size_star = 30
        for idx in range(0, len(star_full_lines), batch_size_star):
            chunk = star_full_lines[idx:idx + batch_size_star]
            if not chunk:
                continue
            self.play(LaggedStart(*[Create(line) for line in chunk], lag_ratio=0.08), run_time=2.9)
            if idx % (batch_size_star * 2) == 0:
                self.next_slide()

        star_full_path = VGroup(
            *[
                Line(to_coord_full_star(full_star_result["path"][i]), to_coord_full_star(full_star_result["path"][i + 1]), color=PURPLE, stroke_width=6)
                for i in range(max(0, len(full_star_result["path"]) - 1))
            ]
        )
        self.play(Create(star_full_path), run_time=2.7)
        set_caption(caption_full_star, "The final route is usually smoother and lower cost than plain RRT.")
        self.next_slide()

        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # SLIDE 6: Side-by-side comparison and conclusion
        # ---------------------------------------------------------
        compare_title = Tex(r"\textbf{RRT vs RRT* Outcome}", font_size=40, color=WHITE).to_edge(UP)
        self.play(FadeIn(compare_title))

        map_compare, to_coord_compare, _, _ = build_map(
            full_width,
            full_height,
            full_scale,
            full_rrt_result["start"],
            full_rrt_result["goal"],
            full_obstacles,
        )
        map_compare.to_edge(DOWN, buff=0.45)
        self.play(FadeIn(map_compare))

        compare_rrt_path = VGroup(
            *[
                Line(to_coord_compare(full_rrt_result["path"][i]), to_coord_compare(full_rrt_result["path"][i + 1]), color=YELLOW, stroke_width=4)
                for i in range(max(0, len(full_rrt_result["path"]) - 1))
            ]
        )
        compare_star_path = VGroup(
            *[
                Line(to_coord_compare(full_star_result["path"][i]), to_coord_compare(full_star_result["path"][i + 1]), color=PURPLE, stroke_width=6)
                for i in range(max(0, len(full_star_result["path"]) - 1))
            ]
        )

        legend_rrt = Text("RRT: fast exploration, suboptimal path", font_size=24, color=YELLOW).to_corner(UL).shift(DOWN * 0.95)
        legend_star = Text("RRT*: rewiring improves path cost", font_size=24, color=PURPLE).next_to(legend_rrt, DOWN, aligned_edge=LEFT)
        conclusion_caption = Text("RRT* adds local optimization, which makes it better for real robot deployment.", font_size=21, color=LIGHT_GRAY).to_edge(DOWN).shift(UP * 0.1)

        self.play(Create(compare_rrt_path), FadeIn(legend_rrt), run_time=2.0)
        self.play(Create(compare_star_path), FadeIn(legend_star), FadeIn(conclusion_caption), run_time=2.0)
        self.next_slide()

        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # OUTRO SLIDE A: Strengths and drawbacks
        # ---------------------------------------------------------
        summary_title = Text("RRT and RRT* Strengths and Drawbacks", font_size=42).to_edge(UP)

        rrt_box = RoundedRectangle(width=6.0, height=4.9, corner_radius=0.14)
        rrt_box.set_fill(DARK_GRAY, opacity=0.55)
        rrt_box.set_stroke(YELLOW, width=2)
        rrt_box.to_corner(UL).shift(DOWN * 1.0 + RIGHT * 0.2)

        star_box = RoundedRectangle(width=6.0, height=4.9, corner_radius=0.14)
        star_box.set_fill(DARK_GRAY, opacity=0.55)
        star_box.set_stroke(PURPLE, width=2)
        star_box.to_corner(UR).shift(DOWN * 1.0 + LEFT * 0.2)

        rrt_points = VGroup(
            Text("RRT", font_size=31, color=YELLOW),
            Text("+ Fast first feasible path", font_size=22),
            Text("+ Probabilistically complete", font_size=22),
            Text("   (with infinite samples)", font_size=22),
            Text("+ Handles constrained motion", font_size=22),
            Text("- Suboptimal, jagged routes", font_size=22, color=RED),
            Text("- Chaotic tree growth", font_size=22, color=RED),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
        rrt_points.move_to(rrt_box.get_center()).align_to(rrt_box, LEFT).shift(RIGHT * 0.35)

        star_points = VGroup(
            Text("RRT*", font_size=31, color=PURPLE),
            Text("+ Asymptotically optimal", font_size=22),
            Text("   (with infinite samples)", font_size=22),
            Text("+ Anytime path improvement", font_size=22),
            Text("+ Better final path quality", font_size=22),
            Text("- Slower first solution", font_size=22, color=RED),
            Text("- Extra rewiring overhead", font_size=22, color=RED),
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
        star_points.move_to(star_box.get_center()).align_to(star_box, LEFT).shift(RIGHT * 0.35)

        summary_caption = Text(
            "Use RRT for speed, and RRT* when long-run path quality matters.",
            font_size=22,
            color=LIGHT_GRAY,
        ).to_edge(DOWN).shift(UP * 0.1)

        self.play(FadeIn(summary_title), FadeIn(rrt_box), FadeIn(star_box), run_time=1.0)
        self.play(FadeIn(rrt_points), FadeIn(star_points), FadeIn(summary_caption), run_time=1.2)
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # OUTRO SLIDE B: Relevance and Extended Impact
        # ---------------------------------------------------------
        relevance_title = Text("Significance & Extended Impact", font_size=44).to_edge(UP)

        # Real-world impact box
        impact_box = RoundedRectangle(width=5.5, height=3.5, corner_radius=0.14)
        impact_box.set_fill(DARK_GRAY, opacity=0.55)
        impact_box.set_stroke(BLUE, width=2)
        impact_box.to_corner(UL).shift(DOWN * 1.5 + RIGHT * 0.5)

        impact_title = Text("Real-World Deployment", font_size=28, color=BLUE)
        impact_bullet1 = Text("• Reduces robot battery waste", font_size=20)
        impact_bullet2 = Text("• Minimises actuator wear", font_size=20)
        impact_bullet3 = Text("• Ideal for UAVs & car-like robots", font_size=20)
        impact_points = VGroup(impact_title, impact_bullet1, impact_bullet2, impact_bullet3).arrange(DOWN, aligned_edge=LEFT, buff=0.2)
        impact_points.move_to(impact_box.get_center())

        # Family of Algorithms box
        family_box = RoundedRectangle(width=5.5, height=3.5, corner_radius=0.14)
        family_box.set_fill(DARK_GRAY, opacity=0.55)
        family_box.set_stroke(GREEN, width=2)
        family_box.to_corner(UR).shift(DOWN * 1.5 + LEFT * 0.5)

        family_title = Text("Algorithm Extensions", font_size=28, color=GREEN)
        family_bullet1 = Text("• Informed RRT*: Elliptical subset search", font_size=20)
        family_bullet2 = Text("• Anytime RRT*: Real-time trajectory execution", font_size=20)
        family_bullet3 = Text("• Bridges gap between speed & optimality", font_size=20)
        family_points = VGroup(family_title, family_bullet1, family_bullet2, family_bullet3).arrange(DOWN, aligned_edge=LEFT, buff=0.2)
        family_points.move_to(family_box.get_center())

        relevance_footnote = Text("[3] Noreen et al. (2016) | [4] Gammell et al. (2014) | [5] Karaman et al. (2011) | [6] Elbanhawi & Simic (2014)", font_size=16, color=GRAY).to_edge(DOWN).shift(UP * 0.5)

        relevance_caption = Text(
            "RRT* paved the way for robust, optimal trajectory planners in modern robotics.",
            font_size=22,
            color=LIGHT_GRAY,
        ).next_to(relevance_footnote, UP, buff=0.2)

        self.play(FadeIn(relevance_title), run_time=0.8)
        self.play(FadeIn(impact_box), FadeIn(impact_points), run_time=1.0)
        self.play(FadeIn(family_box), FadeIn(family_points), run_time=1.0)
        self.play(FadeIn(relevance_footnote), FadeIn(relevance_caption), run_time=0.8)
        self.next_slide()
        self.play(*[FadeOut(mob) for mob in list(self.mobjects)])

        # ---------------------------------------------------------
        # OUTRO SLIDE C: References (final slide)
        # ---------------------------------------------------------
        refs_title_final = Text("References", font_size=56).to_edge(UP)

        ref_1 = Text(
            "[1] LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for\n"
            "     path planning. Technical Report, Iowa State University.",
            font_size=18,
            color=WHITE,
        )
        ref_2 = Text(
            "[2] Karaman, S., and Frazzoli, E. (2011). Sampling-based algorithms\n"
            "     for optimal motion planning. International Journal of Robotics\n"
            "     Research, 30(7), 846-894.",
            font_size=18,
            color=WHITE,
        )
        ref_3 = Text(
            "[3] Realtime Robotics (2019). Solving the Autonomous Vehicles Motion\n"
            "     Planning Conundrum. https://rtr.ai/resources/solving-the-autonomous-\n"
            "     vehicles-motion-planning-conundrum/",
            font_size=18,
            color=WHITE,
        )
        ref_4 = Text(
            "[4] Paes, K., Dewulf, W., Elst, K., Kellens, K., and Slaets, P. (2014).\n"
            "     Energy Efficient Trajectories for an Industrial ABB Robot.\n"
            "     Procedia CIRP, 15, 105-110. doi:10.1016/j.procir.2014.06.043",
            font_size=18,
            color=WHITE,
        )
        ref_5 = Text(
            "[5] IEEE Spectrum (2023). Superhuman Speed: How Autonomous Drones\n"
            "     Beat the Best Human Racers. https://spectrum.ieee.org/ai-drone-racing",
            font_size=18,
            color=WHITE,
        )
        
        ref_6 = Text(
            "[6] Elbanhawi, M., & Simic, M. (2014). Sampling-Based Robot Motion Planning: A Review. IEEE Access.",
            font_size=18,
            color=WHITE,
        )
        
        ref_7 = Text(
            "[7] Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). Informed RRT*: Optimal sampling-based path planning focused via direct sampling\n"
            "     of an admissible ellipsoidal heuristic. IROS.",
            font_size=18,
            color=WHITE,
        )
        
        ref_8 = Text(
            "[8] Noreen, I., Khan, A., & Habib, Z. (2016). Optimal Path Planning using RRT* based Approaches: A Survey and Future Directions. IJACSA.",
            font_size=18,
            color=WHITE,
        )
        
        ref_9 = Text(
            "[9] Karaman, S., Walter, M. R., Perez, A., Frazzoli, E., & Teller, S. (2011). Anytime Motion Planning using the RRT*. ICRA.",
            font_size=18,
            color=WHITE,
        )
        
        refs_group_final = VGroup(ref_1, ref_2, ref_3, ref_4, ref_5, ref_6, ref_7, ref_8, ref_9).arrange(DOWN, aligned_edge=LEFT, buff=0.23)
        refs_group_final.next_to(refs_title_final, DOWN, buff=0.42).to_edge(LEFT, buff=0.45)

        refs_caption_final = Text(
            "Parts of these animations were rendered from code generated from Generative AI tools (LLMs), largely GPT-5.3-Codex.",
            font_size=20,
            color=LIGHT_GRAY,
        ).to_edge(DOWN).shift(UP * 0.2)

        self.play(FadeIn(refs_title_final), FadeIn(refs_group_final), FadeIn(refs_caption_final), run_time=1.4)
        self.next_slide()


def main():
    print("Hello from rrt-par-presentation!")

if __name__ == "__main__":
    main()
