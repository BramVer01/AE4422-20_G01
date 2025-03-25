import numpy as np
import math
from single_agent_planner import simple_single_agent_astar, simple_single_agent_astar_prioritized

class Tug(object):
    """Aircraft class, used to create new tugs for airport operations."""

    # Default configuration parameters
    DEFAULT_CONFIG = {
        "speed": 1,              # Distance units per time unit
        "battery_capacity": 300, # Battery capacity in Ah
        "battery_discharge": 4,  # Battery discharge in Ah per unit of distance
        "battery_charge": 20,    # Battery charge in Ah per second
        "battery_min": 40,       # Minimum battery percentage threshold
        "safety_margin": 1.5,    # Safety margin for route distance estimation
    }

    def __init__(self, tug_id, a_d, start_node, spawn_time, nodes_dict, config=None):
        """
        Initialization of tug object.
        
        Parameters:
            - tug_id: [int] unique id for this tug
            - a_d: [str] "a" for arrival tug and "d" for departure tug
            - start_node: node_id of start node
            - spawn_time: spawn_time of tug
            - nodes_dict: copy of the nodes_dict
            - config: optional configuration parameters to override defaults
        """
        # Initialize configuration
        self.config = self.DEFAULT_CONFIG.copy()
        if config:
            self.config.update(config)
        
        # Fixed parameters
        self.speed = self.config["speed"]
        self.bat_cap = self.config["battery_capacity"]
        self.bat_disc = self.config["battery_discharge"]
        self.bat_charge = self.config["battery_charge"]
        self.bat_state = self.bat_cap  # Initially fully charged
        self.bat_perc = 100            # Battery charge in %
        self.bat_min = self.config["battery_min"]
        
        # Identification parameters
        self.id = tug_id
        self.type = a_d  # arrival or departure (A/D), also defines to which depot it will return initially
        self.spawntime = spawn_time
        
        # Location parameters
        self.start = start_node
        self.goal = None
        self.nodes_dict = nodes_dict
        self.coupled = start_node  # coupled flight/depot
        
        # Task related parameters
        self.current_task = None
        self.final_goal = None
        self.wait = None
        self.constraining_tug = None
        
        # Route related parameters
        self.status = 'idle'  # idle, moving_to_task, executing, to_depot
        self.path_to_goal = []  # planned path left from current location
        self.from_to = [0, 0]
        
        # State related parameters
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"]  # Initialize position to the start node's position

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of a tug based on a start and end xy position.
        
        Parameters:
            - xy_start: tuple with (x,y) position of start node
            - xy_next: tuple with (x,y) position of next node
            
        Returns:
            - heading: heading of tug in degrees
        """
        if xy_start[0] == xy_next[0]:  # moving up or down
            if xy_start[1] > xy_next[1]:  # moving down
                heading = 180
            elif xy_start[1] < xy_next[1]:  # moving up
                heading = 0
            else:
                heading = self.heading
        elif xy_start[1] == xy_next[1]:  # moving right or left
            if xy_start[0] > xy_next[0]:  # moving left
                heading = 90
            elif xy_start[0] < xy_next[0]:  # moving right
                heading = 270
            else:
                heading = self.heading
        else:
            raise Exception("Invalid movement")
    
        self.heading = heading
      
    def move(self, dt, t):   
        """
        Moves a tug between from_node and to_node and checks if to_node or goal is reached.
        
        Parameters:
            - dt: time step duration
            - t: current time
        """
        # Determine nodes between which the tug is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        if from_node == 0 or to_node == 0 or not self.path_to_goal:
            self.position = self.position
            self.wait = True
            return
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep
  
        # Update position with rounded values
        if xy_from == xy_to or self.wait:
            # Waiting: position remains the same
            pass
        else:
            x = xy_to[0] - xy_from[0]
            y = xy_to[1] - xy_from[1]
            distance = math.sqrt(x**2 + y**2)
        
            if distance == 0:
                x_normalized, y_normalized = 0, 0
            else:    
                x_normalized = x / distance
                y_normalized = y / distance

            posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
            posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
             
            # Battery Discharge
            dist_bat = np.sqrt((posx - self.position[0])**2 + (posy - self.position[1])**2)
            self.bat_state -= dist_bat * self.bat_disc
            self.bat_perc = (self.bat_state / self.bat_cap) * 100

            self.position = (posx, posy) 

        self.get_heading(xy_from, xy_to)	

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == round(t+dt, 1): #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.wait = True
                if self.status == "moving_to_task":
                    # Reached pickup location; now switch to executing.
                    print(f"Tug {self.id} reached pickup location {self.goal}. Switching status to executing.")
                    self.status = "executing"
                    # Set new goal to aircraft's destination.
                    self.start = self.goal
                    self.goal = self.final_goal
                    self.path_to_goal = []
                elif self.status == "executing":
                    # Reached the aircraft's destination.
                    print(f"Tug {self.id} delivered task. Switching status to to_depot.")
                    self.status = "to_depot"
                    depot_node = 112 if self.type == "D" else 113
                    self.start = self.goal
                    self.goal = depot_node
                    self.path_to_goal = []
                elif self.status == "to_depot":
                    # Reached the depot.
                    print(f"Tug {self.id} returned to depot at node {to_node}. Setting status to idle.")
                    self.status = "idle"
                    self.coupled = to_node
                    self.current_task = None
            else:  # current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1]  # new from node
                new_next_id = self.path_to_goal[0][0]  # new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id]  # update new from and to node

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing tug assuming that it knows the entire layout.
        Other traffic is not taken into account.
        
        Parameters:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
            - heuristics: heuristics for path planning
            - t: current time
        """
        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done
            
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path tug", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)
            
            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
    
    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t, delta_t, constraints):
        """
        Plans a path for the tug using prioritized planning that takes constraints into account.
        
        Parameters:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
            - heuristics: heuristics for path planning
            - t: current time
            - delta_t: time step
            - constraints: constraints from higher priority agents
        """
        # Allow planning when the tug is in the "moving_to_task" or "executing" state.
        if self.status in ["moving_to_task", "executing", "to_depot"] and not self.path_to_goal:
            start_node = self.start
            goal_node = self.goal
            # Call the prioritized A* function with the extra parameters.
            success, path_agent = simple_single_agent_astar_prioritized(
                nodes_dict, start_node, goal_node, heuristics, t, delta_t, self, constraints
            )
            if success:
                self.path_to_goal = path_agent[1:]
                next_node_id = self.path_to_goal[0][0]
                self.from_to = [path_agent[0][0], next_node_id]
                self.wait = False
                print("Path (prioritized) for tug", self.id, ":", path_agent)
            else:
                print("No solution found for tug", self.id, "start to switch priority")
                self.wait = True
                self.constraining_tug = path_agent
                path_agent.path_to_goal = []
                path_agent.wait = True
                path_agent.start = path_agent.from_to[0]

    def assign_task(self, task):
        """
        Assign a flight task to this tug.
        
        Parameters:
            - task: task to be assigned
            
        Returns:
            - bool: True if task can be assigned, False otherwise
        """
        # First check if the tug has enough battery to handle this task
        depot_node = 112 if self.type == "D" else 113

        # Estimate the distances for the task: to task, to goal, and return to depot
        dist_to_task = np.sqrt(
            (self.nodes_dict[task.start_node]["xy_pos"][0] - self.position[0]) ** 2 +
            (self.nodes_dict[task.start_node]["xy_pos"][1] - self.position[1]) ** 2
        )

        dist_for_task = np.sqrt(
            (self.nodes_dict[task.goal_node]["xy_pos"][0] - self.nodes_dict[task.start_node]["xy_pos"][0]) ** 2 +
            (self.nodes_dict[task.goal_node]["xy_pos"][1] - self.nodes_dict[task.start_node]["xy_pos"][1]) ** 2
        )

        dist_to_depot = np.sqrt(
            (self.nodes_dict[depot_node]["xy_pos"][0] - self.nodes_dict[task.goal_node]["xy_pos"][0]) ** 2 +
            (self.nodes_dict[depot_node]["xy_pos"][1] - self.nodes_dict[task.goal_node]["xy_pos"][1]) ** 2
        )

        # Total estimated distance (with safety margin for unforeseen circumstances)
        total_est_distance = (dist_to_task + dist_for_task + dist_to_depot) * self.config["safety_margin"]

        # Calculate the battery required for the full trip
        required_battery = total_est_distance * self.bat_disc

        # Check if the tug has enough battery for the entire task
        if self.bat_state < required_battery:
            return False

        print(f"Assigning task {task.flight_id} to tug {self.id}. Current status: {self.status}")
        self.current_task = task.flight_id
        self.goal = task.start_node
        self.final_goal = task.goal_node
        if self.start != depot_node:
            self.start = self.from_to[0]
        self.status = "moving_to_task"
        self.wait = True
        self.path_to_goal = []  # Reset path_to_goal to force replanning
        print(f"Tug {self.id} status updated to {self.status}. Goal set to pickup location: {self.goal}")
        return True

    def set_init_tug_params(self, tug_id, a_d, start_node, nodes_dict):
        """
        Reset tug parameters to initial values.
        
        Parameters:
            - tug_id: unique id for this tug
            - a_d: "a" for arrival tug and "d" for departure tug
            - start_node: node_id of start node
            - nodes_dict: copy of the nodes_dict
        """
        # Fixed parameters
        self.speed = self.config["speed"]
        self.id = tug_id
        self.type = a_d
        self.start = start_node
        self.goal = None
        self.coupled = start_node
        
        # Task related parameters
        self.current_task = None
        self.final_goal = None
        self.wait = None
        
        # Route related parameters
        self.status = 'idle'
        self.path_to_goal = []
        self.from_to = [0, 0]
        
        # State related parameters
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"]

    def bidders_value(self, task, nodes_dict, heuristics, t, depots, gamma=1, alpha=1, beta=1, eta=1):
        """
        Returns the max price to pay at the auction (tug allocation).
        
        Parameters:
            - task: task to be auctioned
            - nodes_dict: copy of the nodes_dict
            - heuristics: heuristics for path planning
            - t: current time
            - depots: list of depot nodes
            - gamma: exponent for delay factor
            - alpha: weight for delay factor
            - beta: weight for proximity factor
            - eta: weight for battery factor
            
        Returns:
            - bid_value: calculated bid value
            - suitable: whether the tug is suitable for the task
        """
        current_xy = self.position
        task_start_xy = nodes_dict[task.start_node]["xy_pos"]
        
        # Estimate distance to task
        dist_to_task = ((current_xy[0] - task_start_xy[0])**2 + (current_xy[1] - task_start_xy[1])**2)**0.5
        
        # Battery considerations
        # Calculate a battery efficiency factor - prioritize tasks when battery is 30-80%
        battery_factor = 1.0
        if self.bat_perc < 30:
            battery_factor = 0.05  # Strongly discourage taking tasks when battery is low
        elif self.bat_perc < 50:
            battery_factor = 0.1  # Somewhat discourage taking tasks when battery is getting low
        elif self.bat_perc > 80:
            battery_factor = 1.5  # Encourage taking tasks when battery is high
        
        # Calculate delay penalty - older tasks get higher priority
        delay = max(0, t - task.spawn_time)
        delay_factor = alpha * (delay)**gamma
        
        # Calculate distance penalty - closer tasks get higher priority
        proximity_factor = beta / (dist_to_task + 1)
        
        # Calculate battery bonus - tugs with more battery get higher priority
        battery_bonus = eta * self.bat_perc * battery_factor
        
        # Check if task is suitable based on battery
        suitable = self.bat_perc >= self.bat_min
        
        # Calculate final bid value
        bid_value = delay_factor + proximity_factor + battery_bonus
        
        return bid_value, suitable