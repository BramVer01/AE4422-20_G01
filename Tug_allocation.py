import random
import time
from queue import Queue

class Tug:
    """Tug class representing a tug agent."""
    def __init__(self, tug_id, position, depot):
        self.id = tug_id  # Unique tug ID
        self.status = "idle"  # Tug status (idle, assigned, busy)
        self.position = position  # Current position (node ID)
        self.path = []  # Path assigned to the tug
        self.distance_traveled = 0  # Total distance traveled; KPI voor later
        self.depot = depot  # Home depot
    
    def assign_task(self, task):
        """Assigns a task to the tug."""
        self.status = "assigned"
        self.path = []  # Will be determined by A*
        print(f"Tug {self.id} assigned to task {task.flight_id}")
        return task

    def request_path(self, task):
        """Request A* algorithm to calculate the path."""
        print(f"Tug {self.id} requesting A* path from {self.position} to {task.goal_node}")
        # This should call the A* function from another file
        return []  # Placeholder for A* path

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
# NOTE Miguel: Deze nodes (node 20 and node 17) gaan we nog veranderen, maar eerst moeten we even de nodes toevoegen aan de Excel file hiervoor
departure_depot = Depot(1, position=20)  # Departure depot at node 20
arrival_depot = Depot(2, position=17)  # Arrival depot at node 17

# Creating tugs for each depot
tug1 = Tug(1, position=20, depot=departure_depot)  
tug2 = Tug(2, position=17, depot=arrival_depot)  

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