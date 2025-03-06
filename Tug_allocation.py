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
        self.distance_traveled = 0  # Total distance traveled
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
    start_node = random.randint(1, 10)  # Random start node (example range)
    goal_node = random.randint(1, 10)  # Random goal node (example range)
    spawn_time = time.time()
    return FlightTask(flight_id, a_d, start_node, goal_node, spawn_time)

# Example setup
depot = Depot(1, position=5)  # Creating a depot at node 5

tug1 = Tug(1, position=5, depot=depot)  # Tug at depot

depot.add_tug(tug1)  # Adding tug to depot

task1 = generate_flight_task(1)  # Generating a flight task

depot.add_task(task1)  # Adding task to depot

depot.match_task()  # Matching a task to a tug
