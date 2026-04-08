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

def get_random_node(width, height, obstacles):
    while True:
        x = random.uniform(0, width)
        y = random.uniform(0, height)
        in_obs = False
        for (xmin, ymin, xmax, ymax) in obstacles:
            if xmin <= x <= xmax and ymin <= y <= ymax:
                in_obs = True
                break
        if not in_obs:
            return Node(x, y)

def generate_obstacles(num_obstacles, width, height, start, goal, clear_radius=1.5):
    obstacles = []
    for _ in range(num_obstacles):
        w = random.uniform(1.5, 4.0)
        h = random.uniform(1.5, 4.0)
        # Limit the maximum y-coordinate to prevent obstacles from overlapping the title at the top
        x = random.uniform(0, width - w)
        y = random.uniform(0, height - h - 2.0)
        xmin, ymin, xmax, ymax = x, y, x + w, y + h
        
        # Ensure start and goal are not inside or too close to the obstacle
        def overlaps(px, py):
            return (xmin - clear_radius <= px <= xmax + clear_radius and 
                    ymin - clear_radius <= py <= ymax + clear_radius)
                    
        if overlaps(start.x, start.y) or overlaps(goal.x, goal.y):
            continue
            
        obstacles.append((xmin, ymin, xmax, ymax))
    return obstacles

def run_rrt(start, goal, obstacles, width, height, max_iter=500, step_size=1.0, star=False):
    nodes = [start]
    edges_to_animate = []
    
    for _ in range(max_iter):
        if random.random() < 0.1:
            rand_node = Node(goal.x, goal.y)
        else:
            rand_node = get_random_node(width, height, obstacles)
            
        nearest = min(nodes, key=lambda n: distance(n, rand_node))
        
        dist = distance(nearest, rand_node)
        if dist == 0:
            continue
            
        theta = math.atan2(rand_node.y - nearest.y, rand_node.x - nearest.x)
        current_step = min(step_size, dist)
        new_x = nearest.x + current_step * math.cos(theta)
        new_y = nearest.y + current_step * math.sin(theta)
        
        new_node = Node(new_x, new_y)
        
        if not check_collision(obstacles, nearest, new_node, width, height):
            if not star:
                new_node.parent = nearest
                new_node.cost = nearest.cost + distance(nearest, new_node)
                nodes.append(new_node)
                edges_to_animate.append(('add', nearest, new_node))
                
                if distance(new_node, goal) < step_size:
                    if not check_collision(obstacles, new_node, goal, width, height):
                        goal.parent = new_node
                        goal.cost = new_node.cost + distance(new_node, goal)
                        nodes.append(goal)
                        edges_to_animate.append(('add', new_node, goal))
                        break
            else:
                # RRT* implementation
                radius = 3.0
                near_nodes = [n for n in nodes if distance(n, new_node) <= radius and not check_collision(obstacles, n, new_node, width, height)]
                
                min_cost_node = nearest
                min_cost = nearest.cost + distance(nearest, new_node)
                
                for n in near_nodes:
                    if n.cost + distance(n, new_node) < min_cost:
                        min_cost_node = n
                        min_cost = n.cost + distance(n, new_node)
                
                new_node.parent = min_cost_node
                new_node.cost = min_cost
                nodes.append(new_node)
                edges_to_animate.append(('add', min_cost_node, new_node))
                
                # Rewiring logic
                for n in near_nodes:
                    if new_node.cost + distance(new_node, n) < n.cost:
                        old_parent = n.parent
                        n.parent = new_node
                        n.cost = new_node.cost + distance(new_node, n)
                        edges_to_animate.append(('rewire', n, old_parent, new_node))
                        
                if distance(new_node, goal) < step_size:
                    if not check_collision(obstacles, new_node, goal, width, height):
                        goal_copy = Node(goal.x, goal.y)
                        goal_copy.parent = new_node
                        goal_copy.cost = new_node.cost + distance(new_node, goal_copy)
                        nodes.append(goal_copy)
                        edges_to_animate.append(('add', new_node, goal_copy))
                        goal = goal_copy
                        break
                    
    path = []
    curr = nodes[-1] if distance(nodes[-1], goal) < 0.1 else None
    
    if curr is None:
        min_dist = float('inf')
        for n in nodes:
             if distance(n, goal) < min_dist:
                 min_dist = distance(n, goal)
                 curr = n
                 
    while curr is not None:
        path.append(curr)
        curr = curr.parent
    path.reverse()
    
    return nodes, edges_to_animate, path

class MazePathPlanning(Slide):
    skip_reversing = True
    
    def construct(self):
        # Set a fixed random seed so that Manim caches the generated layout and partial videos.
        random.seed(42)
        
        width, height = 21, 13
        scale_factor = 0.6
        
        # SLIDE 1: Displaying the workspace (Background and Grid)
        # Dark grey background
        bg_rect = Rectangle(width=width * scale_factor, height=height * scale_factor)
        bg_rect.set_fill(DARK_GRAY, opacity=1.0)
        bg_rect.set_stroke(WHITE, width=2)
        self.play(FadeIn(bg_rect))
        
        # Grid layout
        grid = VGroup()
        for x in range(width + 1):
            x_coord = (x - width / 2) * scale_factor
            grid.add(Line([x_coord, -height/2 * scale_factor, 0], [x_coord, height/2 * scale_factor, 0], color=GRAY, stroke_width=1))
        for y in range(height + 1):
            y_coord = (y - height / 2) * scale_factor
            grid.add(Line([-width/2 * scale_factor, y_coord, 0], [width/2 * scale_factor, y_coord, 0], color=GRAY, stroke_width=1))
        self.play(Create(grid))
        
        self.next_slide()
        
        # SLIDE 2: Generate and display mapping obstacles
        start = Node(2, 2)
        goal = Node(width - 3, height - 3)
        obstacles = generate_obstacles(15, width, height, start, goal)
        
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
        
        self.play(FadeIn(obs_group))
        
        self.next_slide()
        
        # SLIDE 3: Show Start & Goal points
        def to_coord(node):
            return np.array([(node.x - width / 2) * scale_factor, (node.y - height / 2) * scale_factor, 0])
            
        start_dot = Dot(to_coord(start), color=GREEN).scale(1.5)
        goal_dot = Dot(to_coord(goal), color=RED).scale(1.5)
        self.play(FadeIn(start_dot), FadeIn(goal_dot))
        
        self.next_slide()
        
        # SLIDE 4: Standard RRT Animation
        label_rrt = Text("Rapidly-exploring Random Tree (RRT)", font_size=36).to_edge(UP)
        self.play(Write(label_rrt))
        
        nodes, edits, path = run_rrt(start, goal, obstacles, width, height, max_iter=800, step_size=1.0, star=False)
        
        lines_rrt = VGroup()
        for edit in edits:
            if edit[0] == 'add':
                line = Line(to_coord(edit[1]), to_coord(edit[2]), color=BLUE, stroke_width=2)
                lines_rrt.add(line)
        self.play(Create(lines_rrt, run_time=3))
        
        path_lines_rrt = VGroup()
        for i in range(len(path)-1):
            line = Line(to_coord(path[i]), to_coord(path[i+1]), color=YELLOW, stroke_width=5)
            path_lines_rrt.add(line)
        if len(path_lines_rrt) > 0:
            self.play(Create(path_lines_rrt, run_time=1.5))
        
        self.next_slide()
        
        # Hide original RRT structure so we can clearly view RRT* 
        self.play(FadeOut(lines_rrt), FadeOut(path_lines_rrt), FadeOut(label_rrt))
        
        # SLIDE 5: RRT* (Optimal RRT) Animation
        label_rrt_star = Text("RRT*", font_size=36).to_edge(UP)
        self.play(Write(label_rrt_star))
        
        start_star = Node(start.x, start.y)
        goal_star = Node(goal.x, goal.y)
        
        nodes_star, edits_star, path_star = run_rrt(start_star, goal_star, obstacles, width, height, max_iter=800, step_size=1.0, star=True)
        
        lines_star_group = VGroup()
        self.add(lines_star_group)
        
        lines_cache = {}
        def get_line(p1, p2):
            if (p1, p2) not in lines_cache:
                lines_cache[(p1, p2)] = Line(to_coord(p1), to_coord(p2), color=ORANGE, stroke_width=2)
            return lines_cache[(p1, p2)]
            
        tracker = ValueTracker(0)
        
        def update_star(mob):
            idx = int(tracker.get_value())
            mob.submobjects = []
            
            active_lines = {}
            for i in range(min(idx, len(edits_star))):
                edit = edits_star[i]
                if edit[0] == 'add':
                    n1, n2 = edit[1], edit[2]
                    active_lines[(n1, n2)] = True
                elif edit[0] == 'rewire':
                    n, old_p, new_p = edit[1], edit[2], edit[3]
                    if (old_p, n) in active_lines:
                        del active_lines[(old_p, n)]
                    active_lines[(new_p, n)] = True
            
            for (p1, p2) in active_lines:
                mob.add(get_line(p1, p2))
                
        lines_star_group.add_updater(update_star)
        self.play(tracker.animate.set_value(len(edits_star)), run_time=8.0, rate_func=linear)
        lines_star_group.remove_updater(update_star)
                        
        path_lines_star = VGroup()
        for i in range(len(path_star)-1):
            line = Line(to_coord(path_star[i]), to_coord(path_star[i+1]), color=PURPLE, stroke_width=5)
            path_lines_star.add(line)
        if len(path_lines_star) > 0:
            self.play(Create(path_lines_star, run_time=1.5))
            
        self.next_slide()
        
        # SLIDE 6: Comparative Overlay of RRT and RRT*
        label_overlay = Text("Comparison: RRT vs RRT*", font_size=36).to_edge(UP)
        self.play(Transform(label_rrt_star, label_overlay))
        
        # Since we used different colors (BLUE vs ORANGE) & (YELLOW vs PURPLE)
        # we can just bring back the initial RRT layout, putting it on top or side-by-side. 
        self.play(FadeIn(lines_rrt), FadeIn(path_lines_rrt))
        
        self.next_slide()

def main():
    print("Hello from rrt-par-presentation!")

if __name__ == "__main__":
    main()
