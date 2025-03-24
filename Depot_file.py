import random
import time
from queue import Queue


from queue import Queue

class Depot:
    """Depot class to manage tugs and task queue."""
    
    def __init__(self, depot_id, position):
        self.id = depot_id       # Depot ID
        self.position = position # Depot position (node ID)
        self.tugs = []      # List of idle tugs in depot
        self.tasks = []     # List of flight tasks

    def add_task(self, task):
        """Add a flight task to the depot queue."""
        self.tasks.append(task)
        print(f"Task {task.flight_id} added to depot {self.id}'s task queue.")

    def charging(self, dt):
        for tug in self.tugs:
            if tug.status == "idle" and tug.bat_perc < 100:
                tug.bat_state += tug.bat_charge * dt
                tug.bat_state = min(tug.bat_state, tug.bat_cap)  # Don't exceed max capacity
                tug.bat_perc = (tug.bat_state / tug.bat_cap) * 100
                if tug.bat_perc >= 99.9:
                    print(f"Tug {tug.id} is now fully charged")
'''
    def match_task(self, t):
        """Assign a flight task to an idle tug if available."""
        if not self.tugs.empty() and not self.tasks.empty():
            tug = self.tugs.get()
            task = self.tasks.get()
            print(f"At time: {t} - Tug {tug.id} is matched with {task.type} task {task.flight_id}")
            tug.assign_task(task)
'''
# class Depot:
#     """Depot class to manage tugs and task queue."""
#     def __init__(self, depot_id, position):
#         self.id = depot_id  # Depot ID
#         self.position = position  # Depot position (node ID)
#         self.tugs = Queue()  # FIFO queue for idle tugs
#         self.tasks = Queue()  # FIFO queue for tasks


#     def add_task(self, task):
#         """Add a task to the depot queue."""
#         self.tasks.put(task)


#     def match_task(self, t):
#         """Assign tasks to available tugs."""
#         if not self.tugs.empty() and not self.tasks.empty():
#             tug = self.tugs.get()
#             task = self.tasks.get()
            
#             print(f'At time: {t} tug {tug.id} is mached with type {task.type} task {task.flight_id}')

#             tug.assign_task(task)
#             tug.request_path(task)

class FlightTask:
    """Represents a flight task."""
    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time):
        self.flight_id = flight_id
        self.type = a_d  # Arrival (A) or Departure (D)
        self.start_node = start_node
        self.goal_node = goal_node
        self.spawn_time = spawn_time








# Als je runt krijg je:
# Tug 2 assigned to task 1
# Tug 2 requesting A* path from 17 to 35
