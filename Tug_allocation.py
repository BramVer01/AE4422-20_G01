import random
import time
from queue import Queue
import numpy as np
import math

class Tug:
    def __init__(self, tug_id, position, depot):
        self.id = tug_id
        self.position = position  # Initial position of the tug as a tuple (x, y)
        self.depot = depot
        self.current_aircraft = None  # Initially no aircraft attached
        self.speed = 5    # Define the tug's speed
        self.heading = 0

    def assign_task(self, task):
        self.task = task
        print(f"Task '{task}' assigned to Tug {self.id}")

    def move(self, dt, t, nodes_dict):
        """Move the tug (and attached aircraft)"""
        if self.current_aircraft:
            # Move the aircraft along with the tug
            self.current_aircraft.position = (self.position[0], self.position[1])
        
        # Update the tug's position, make sure it stays as a tuple
        x = self.position[0] + self.speed * dt * np.cos(self.heading)
        y = self.position[1] + self.speed * dt * np.sin(self.heading)
        self.position = (x,y)  # Update position of tug

    def move_tug_with_aircraft(self, dt, t):
        """
        Moves the tug and the attached aircraft together.
        """
        if self.current_aircraft:
            ac_position = self.current_aircraft.position
            tug_position = self.position

            # Calculate direction from tug to aircraft
            direction = (ac_position[0] - tug_position[0], ac_position[1] - tug_position[1])

            # Normalize direction to ensure consistent speed
            distance = math.sqrt(direction[0]**2 + direction[1]**2)
            if distance > 0:
                direction = (direction[0] / distance, direction[1] / distance)

            # Move the tug and the aircraft towards each other
            self.position = (tug_position[0] + direction[0] * self.speed * dt,
                             tug_position[1] + direction[1] * self.speed * dt)

            # Move the aircraft with the tug
            self.current_aircraft.position = (ac_position[0] + direction[0] * self.speed * dt,
                                              ac_position[1] + direction[1] * self.speed * dt)

            # Check if the tug and aircraft have reached the goal and detach
            if self.position == self.current_aircraft.position:
                self.current_aircraft.status = "taxiing"
                self.current_aircraft = None

    def request_path(self, task):
        """
        This method could be used to request a path for the tug to follow.
        You could have the tug request a path from a pathfinding algorithm
        or some other process based on the task.
        """
        # Example of what this method might look like
        print(f"Tug {self.id} is requesting a path for task {task}")
        # Implement pathfinding or other logic as needed


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
departure_depot = Depot(1, position=112)  # Departure depot at node 20
arrival_depot = Depot(2, position=113)  # Arrival depot at node 17

# Creating tugs for each depot

tug1 = Tug(1, position=(2, 5), depot=departure_depot)  # Use a tuple for position
tug2 = Tug(2, position=(2, 4), depot=arrival_depot)    # Use a tuple for position

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