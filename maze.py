import os
import heapq

class Node():
    def __init__(self, state, parent, action, g, f):
        self.state = state
        self.parent = parent
        self.action = action
        self.g = g #Stores cost from start to n
        self.f = f #Stores f(node)

    # Lesser than overide to tiebreak f(n) comparisons
    def __lt__(self, other):
        return self.f < other.f


class StackFrontier():
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    # Heap appends to tuple correctly
    def add_to_heap(self, f, node):
        self.frontier.append((f, node))

    # Check if state already exists in the heap
    def contains_state_heap(self, state):
        return any(n.state == state for f, n in self.frontier)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node

class QueueFrontier(StackFrontier):

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node

class Maze():

    def __init__(self, filename):

        # Read file and set height and width of maze
        with open(filename) as f:
            contents = f.read()

        # Validate start and goal
        if contents.count("A") != 1:
            raise Exception("maze must have exactly one start point")
        if contents.count("B") != 1:
            raise Exception("maze must have exactly one goal")

        # Determine height and width of maze
        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        # Keep track of walls
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                try:
                    if contents[i][j] == "A":
                        self.start = (i, j)
                        row.append(False)
                    elif contents[i][j] == "B":
                        self.goal = (i, j)
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None
        self.path_cost = None


    def print(self):
        solution = self.solution[1] if self.solution is not None else None
        print()
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):
                if col:
                    print("█", end="")
                elif (i, j) == self.start:
                    print("A", end="")
                elif (i, j) == self.goal:
                    print("B", end="")
                elif solution is not None and (i, j) in solution:
                    print("*", end="")
                else:
                    print(" ", end="")
            print()
        print()


    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

    def reconstruct(self, node):
        actions = []
        cells = []
        while node.parent is not None:
            actions.append(node.action)
            cells.append(node.state)
            node = node.parent
        actions.reverse()
        cells.reverse()
        self.solution = (actions, cells)
        self.path_cost = len(actions) #Calculate path cost
        return

    def solve(self):
        """Finds a solution to maze, if one exists."""

        # Keep track of number of states explored
        self.num_explored = 0

        # Initialize frontier to just the starting position
        start = Node(state=self.start, parent=None, action=None, g=None, f=None)
        frontier = StackFrontier()
        frontier.add(start)

        # Initialize an empty explored set
        self.explored = set()

        # Keep looping until solution found
        while True:

            # If nothing left in frontier, then no path
            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove()
            self.num_explored += 1 # Used for search data

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                self.path_cost = len(actions) #Calculate path cost
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action, g=None, f=None)
                    frontier.add(child)
    
    # Depth-Limited DFS
    def depthLimitedSearch(self, node, limit):
        # Count node
        self.num_explored += 1 #Used for search data
        self.explored.add(node.state) 

        # Check if node is the goal
        if node.state == self.goal:
            self.reconstruct(node)
            return True
        
        elif limit == 0:
            return False
        
        else:
            for action, state in self.neighbors(node.state):
                if node.parent is not None and state == node.parent.state:
                    continue
                child = Node(state=state, parent=node, action=action, g=None, f=None)
                if self.depthLimitedSearch(child, limit - 1): #Uses recursion to search the limited tree
                    return True
        return False

    def iterativeDeepeningDepthFirstSearch(self):
        
        self.num_explored = 0 #Used for search data
        self.explored = set()

        # self.solution = None
        max_depth = self.height * self.width  # finite upper bound for this grid

        for depth in range(max_depth + 1):
            self.explored = set()
            start = Node(state=self.start, parent=None, action=None, g=None, f=None)
            if self.depthLimitedSearch(start, depth):  #Will return true if a solution was found
                return
        
        raise Exception("no solution")

    def heuristic(self, state):    
        (r1, c1) = state
        (r2, c2) = self.goal
        return abs(r1 - r2) + abs(c1 - c2)


    def aStarSearch(self):
        self.explored = set()
        self.num_explored = 0

        # Initialize heap to just the starting position
        start = Node(state=self.start, parent=None, action=None, g=0, f=None)
        #Find f, root node has no g(n) to calculate
        start.f = self.heuristic(start.state)
        heap = StackFrontier()
        heap.add_to_heap(start.f, start)

        while True:

            # If nothing left in heap, then no path
            if not heap.frontier:
                raise Exception("no solution")

            #Pop the next node in the stack, has to get the second tuple value
            fNode = heapq.heappop(heap.frontier)
            node = fNode[1]

            self.num_explored += 1
            
            # If node is the goal, then we have a solution
            if node.state == self.goal:
                self.reconstruct(node)
                return
            
            self.explored.add(node.state)
            
            # Add neighbors to state    
            for action, state in self.neighbors(node.state):
                if not heap.contains_state_heap(state) and state not in self.explored:
                    #Calculate f(n) for this child
                    g = node.g + 1
                    h = self.heuristic(state)
                    f = g + h

                    # Create child node
                    child = Node(state=state, parent=node, action=action, g=g, f=f)
                    heapq.heappush(heap.frontier, (f, child))

    def solution_moves(self):
        if self.solution is None:
            return None
        map_dir = {"left": "L", "right": "R", "up": "U", "down": "D"}
        actions = self.solution[0]
        return "-".join(map_dir[action] for action in actions)

    def output_image(self, filename, show_solution=True, show_explored=False):
        from PIL import Image, ImageDraw
        cell_size = 50
        cell_border = 2

        # Create a blank canvas
        img = Image.new(
            "RGBA",
            (self.width * cell_size, self.height * cell_size),
            "black"
        )
        draw = ImageDraw.Draw(img)

        solution = self.solution[1] if self.solution is not None else None
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):

                # Walls
                if col:
                    fill = (40, 40, 40)

                # Start
                elif (i, j) == self.start:
                    fill = (255, 0, 0)

                # Goal
                elif (i, j) == self.goal:
                    fill = (0, 171, 28)

                # Solution
                elif solution is not None and show_solution and (i, j) in solution:
                    fill = (220, 235, 113)

                # Explored
                elif solution is not None and show_explored and (i, j) in self.explored:
                    fill = (212, 97, 85)

                # Empty cell
                else:
                    fill = (237, 240, 252)

                # Draw cell
                draw.rectangle(
                    ([(j * cell_size + cell_border, i * cell_size + cell_border),
                      ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)]),
                    fill=fill
                )

        img.save(filename)


maze_files = [file for file in os.listdir('.') if file.endswith('.txt')]
print("Available maze files:")
for i, file in enumerate(maze_files, 1):
    print(f"{i}. {file}")

# Choose file
file_choice = int(input("Select maze file number: ")) - 1
filename = maze_files[file_choice]

# Choose algorithm
print("\nAlgorithms:")
print("1. DFS")
print("2. IDDFS")
print("3. A*")
algorithm_choice = int(input("Select algorithm number: "))

# Run chosen algorithm
maze = Maze(filename)
print("\nMaze:")
maze.print()

if algorithm_choice == 1:
    print("Running DFS Search...")
    algorithm_choice = "DFS"
    maze.solve()
elif algorithm_choice == 2:
    print("Running Iterative Deepening DFS...")
    algorithm_choice = "IDDFS"
    maze.iterativeDeepeningDepthFirstSearch()
elif algorithm_choice == 3:
    print("Running A* Search...")
    algorithm_choice = "A*"
    maze.aStarSearch()
else:
    print("Invalid choice.")
    exit(1)

# Print results
print("States Explored:", maze.num_explored)
print("Path Cost:", maze.path_cost)
print("Moves:", maze.solution_moves())

# Output image
maze.output_image(f"{algorithm_choice}-{filename}-solution.png", show_explored=True)
print(f"Saved solution image")