import random
import time
from queue import Queue
from single_agent_planner import simple_single_agent_astar

class Tug:
    def __init__(self, id, start, goal, status, speed=0.0):
        self.id = id
        self.start = start
        self.goal = goal
        self.status = status
        self.speed = speed
        self.position = (0, 0)  # Default starting position (will be updated)
        self.path_to_goal = []
        self.from_to = []
        self.current_aircraft = None  # Assuming the tug may carry an aircraft

    def move(self, dt, t, nodes_dict, edges_dict, heuristics):
        """
        Move the tug (and attached aircraft) along the planned path.
        """
        # Check if the position is valid (tuple with two elements)
        if not isinstance(self.position, tuple) or len(self.position) != 2:
            raise TypeError(f"Tug position is not a valid tuple. Current position: {self.position}")
        
        if self.status == "taxiing" and not self.path_to_goal:
            self.plan_independent(nodes_dict, edges_dict, heuristics, t)

        # If the tug is taxiing and has a planned path
        if self.status == "taxiing" and self.path_to_goal:
            if self.position == self.path_to_goal[0]:
                # Move to the next node in the path if we've reached the current one
                self.path_to_goal.pop(0)
                if self.path_to_goal:
                    self.position = self.path_to_goal[0]
            else:
                # Update position along the path, based on speed and time
                next_node = self.path_to_goal[0]
                # Move towards the next node in the path
                self.position = (self.position[0] + self.speed * dt, self.position[1])  # Adjust as needed

        # Update aircraft position, if there is any attached
        if self.current_aircraft:
            self.current_aircraft.position = self.position

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing the tug to its goal, assuming it knows the entire layout.
        Other traffic is not taken into account.
        """
        
        if self.status == "taxiing":
            start_node = self.start  # Starting node for path planning
            goal_node = self.goal  # Goal node for path planning

            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]  # Remove the start node, as we already know it
                next_node_id = self.path_to_goal[0][0]  # Next node is the first in the path
                self.from_to = [path[0][0], next_node_id]
                print("Path for Tug", self.id, ":", path)
            else:
                raise Exception("No solution found for Tug", self.id)
            
            # Ensure the timing is correct for path planning
            if path[0][1] != t:
                raise Exception("Timing issue in path planning for Tug", self.id)



class Depot:
    """Depot class to manage tugs and task queue."""
    def __init__(self, depot_id, position):
        self.id = depot_id  # Depot ID
        self.position = position  # Depot position (node ID)
        self.tugs = Queue()  # FIFO queue for idle tugs
        self.tasks = Queue()  # FIFO queue for tasks

    def add_tug(self, tug):
        """Add a tug to the depot."""
        self.tugs.put(tug)

    def add_task(self, task):
        """Add a task to the depot queue."""
        self.tasks.put(task)

    def match_task(self):
        """Assign tasks to available tugs."""
        if not self.tugs.empty() and not self.tasks.empty():
            tug = self.tugs.get()
            task = self.tasks.get()
            tug.assign_task(task)
            tug.request_path(task)

class FlightTask:
    """Represents a flight task."""
    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time):
        self.flight_id = flight_id
        self.type = a_d  # Arrival (A) or Departure (D)
        self.start_node = start_node
        self.goal_node = goal_node
        self.spawn_time = spawn_time



def generate_flight_task(flight_id):
    """Generates a flight task at a certain frequency."""
    a_d = random.choice(["A", "D"])  # Arrival or Departure
    
    if a_d == "A":
        start_node = random.choice([37, 38])  # Arrivals spawn at node 37 or 38
        goal_node = random.choice([97, 34, 35, 36, 98])  # Arrivals go to one of these nodes
    else:
        start_node = random.choice([97, 34, 35, 36, 98])  # Departures spawn at these nodes
        goal_node = random.choice([1, 2])  # Departures go to either node 1 or 2
    
    spawn_time = time.time()
    return FlightTask(flight_id, a_d, start_node, goal_node, spawn_time)

# Creating two depots
departure_depot = Depot(1, position=20)  # Departure depot at node 20
arrival_depot = Depot(2, position=17)  # Arrival depot at node 17


# Creating tugs for each depot

tug1 = Tug(1, position=(2, 5), depot=departure_depot)  # Position as a tuple
tug2 = Tug(2, position=(2, 4), depot=arrival_depot)    # Position as a tuple

departure_depot.add_tug(tug1)
arrival_depot.add_tug(tug2)

# Generating a flight task
task1 = generate_flight_task(1) #Voor nu 1 task en 1 tug per depot om te checken

# Assigning tasks to the appropriate depot
if task1.type == "A":
    arrival_depot.add_task(task1)
    arrival_depot.match_task()
else:
    departure_depot.add_task(task1)
    departure_depot.match_task()


# Als je runt krijg je:
# Tug 2 assigned to task 1
# Tug 2 requesting A* path from 17 to 35