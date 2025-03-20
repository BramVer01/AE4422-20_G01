import numpy as np

from single_agent_planner import simple_single_agent_astar, simple_single_agent_astar_prioritized
import math

class Tug(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, tug_id, a_d, start_node, spawn_time, nodes_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this tug
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
            - heuristics: heuristics for path planning
        """
        
        #Fixed parameters
        self.speed = 1         #how much tug moves per unit of t
        self.bat_cap = 300     # Battery capacity in Ah
        self.bat_disc = 4     # Battery discharge in Ah per unit of distance
        self.bat_charge = 20    # Battery charge in Ah per second
        self.bat_state = 300   # Current state of the battery in Ah (initially fully charged)
        self.bat_perc = 100    # Battery charge in %
        self.id = tug_id       #tug_id
        self.type = a_d           #arrival or departure (A/D), also defines to which depot it will return initially
        self.spawntime = spawn_time #spawntime
        self.start = start_node   #start_node_id
        self.goal = None     #goal_node_id
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        self.coupled = start_node #coupled flight /depot 
        # Add the new fields
        self.current_task = None
        self.final_goal = None
        self.wait = None
        self.constraining_tug = None
        
        #Route related
        self.status = 'idle' # idle, moving_to_task, executing, to_depot
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]

        #State related
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"] # Initialize position to the start node's position

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading
      
    def move(self, dt, t):   
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = time step
            - t = current time
        """
        
        # Check if path_to_goal is empty
        if not self.path_to_goal:
            # Nothing to move, tug is waiting
            # Charge battery when at depot
            if self.status == "idle" and (self.coupled == 112 or self.coupled == 113):
                if self.bat_perc < 100:
                    # Charge battery
                    self.bat_state += self.bat_charge * dt
                    if self.bat_state > self.bat_cap:
                        self.bat_state = self.bat_cap
                    self.bat_perc = (self.bat_state / self.bat_cap) * 100
                    if t % 5 == 0:  # Print charging status every 5 time units to avoid console spam
                        print(f"Tug {self.id} charging at depot, battery at {self.bat_perc:.1f}%")
            return
        
        # Check if battery is too low to move
        if self.bat_perc <= 0:
            self.wait = True
            print(f"Tug {self.id} has no battery left and cannot move.")
            # Force return to depot if not already on the way
            if self.status != "to_depot":
                self.status = "to_depot"
                depot_node = 112 if self.type == "D" else 113
                self.start = self.from_to[0] if self.from_to[0] != 0 else self.coupled
                self.goal = depot_node
                self.path_to_goal = []
            return
            
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        
        # Safety check for valid node IDs
        if from_node == 0 or to_node == 0 or from_node not in self.nodes_dict or to_node not in self.nodes_dict:
            print(f"Warning: Invalid node ID - from_node: {from_node}, to_node: {to_node}")
            self.wait = True
            return
        
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep

        #Update position with rounded values
        if xy_from == xy_to or self.wait:
            # Waiting: position remains the same.
            pass
        else:
            x = xy_to[0]-xy_from[0]
            y = xy_to[1]-xy_from[1]
            distance = math.sqrt(x**2+y**2)
        
            if distance == 0:
                x_normalized = 0
                y_normalized = 0
            else:    
                x_normalized = x / distance
                y_normalized = y / distance

            posx = round(self.position[0] + x_normalized * distance_to_move, 2) #round to prevent errors
            posy = round(self.position[1] + y_normalized * distance_to_move, 2) #round to prevent errors

            # Battery Discharge
            dist_bat = np.sqrt((posx-self.position[0])**2+(posy-self.position[1])**2)
            self.bat_state -= dist_bat * self.bat_disc
            self.bat_perc = (self.bat_state / self.bat_cap) * 100

            self.position = (posx, posy)

        self.get_heading(xy_from, xy_to)    

        # Check if we've reached a node
        if self.position == xy_to and len(self.path_to_goal) > 0 and self.path_to_goal[0][1] == t+dt: 
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

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                if not self.path_to_goal:  # If we've exhausted the path
                    self.wait = True
                    return
                    
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        
        if self.status == "taxiing":
            start_node = self.start #node from which planning should be done
            goal_node = self.goal #node to which planning should be done
            
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path tug", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)
            
            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
    
    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t, delta_t, constraints):
        if self.status in ["moving_to_task", "executing", "to_depot"] and not self.path_to_goal:
            start_node = self.start
            goal_node = self.goal
            
            # Check if the battery is too low to continue
            if self.bat_perc < 15:  # 15% battery threshold
                print(f"Tug {self.id} battery too low ({self.bat_perc:.1f}%). Returning to depot.")
                # Force return to depot
                if self.status != "to_depot":
                    self.status = "to_depot"
                    depot_node = 112 if self.type == "D" else 113
                    self.start = self.from_to[0] if hasattr(self, 'from_to') and self.from_to[0] != 0 else start_node
                    self.goal = depot_node
                    goal_node = depot_node  # Update the goal for path planning
            
            # Make sure we have valid nodes for planning
            if start_node == 0 or goal_node == 0 or start_node not in nodes_dict or goal_node not in nodes_dict:
                print(f"Warning: Invalid nodes for planning. start_node: {start_node}, goal_node: {goal_node}")
                self.wait = True
                return
                
            # Call the prioritized A* function
            success, path_agent = simple_single_agent_astar_prioritized(
                nodes_dict, start_node, goal_node, heuristics, t, delta_t, self, constraints
            )
            
            if success:
                if not path_agent or len(path_agent) < 2:  # Path too short or empty
                    print(f"Path found but invalid for tug {self.id}, waiting")
                    self.wait = True
                    return
                    
                self.path_to_goal = path_agent[1:]
                if len(self.path_to_goal) > 0:  # Check if path is not empty
                    next_node_id = self.path_to_goal[0][0]
                    self.from_to = [path_agent[0][0], next_node_id]
                    self.wait = False
                    print(f"Path found for (prioritized) for tug {self.id}")
                else:
                    # Handle empty path case
                    self.wait = True
                    print(f"Path found but empty for tug {self.id}, waiting")
            else:
                print(f"No solution found for tug {self.id}, will wait")
                self.wait = True
                if path_agent:  # Make sure path_agent is not None
                    self.constraining_tug = path_agent
                    if hasattr(path_agent, 'path_to_goal'):
                        path_agent.path_to_goal = []
                        path_agent.wait = True
                        path_agent.start = path_agent.from_to[0] if hasattr(path_agent, 'from_to') and path_agent.from_to[0] != 0 else start_node
                # # this is new to prevent the simulation from stopping when no path is found using prio A*
                # success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)

                # if success:
                #     self.path_to_goal = path[1:]
                #     next_node_id = self.path_to_goal[0][0]
                #     self.from_to = [path[0][0], next_node_id]
                #     self.wait = False
                #     # print("Path (prioritized) for tug", self.id, ":", path)
                
                # else:
                #     raise Exception("No solution found for tug", self.id)
            
            # Optionally, verify that the planning start time matches.
            # if path_agent[0][1] != t:
            #     raise Exception("Timing error in path planning for tug", self.id)

    def assign_task(self, task):
        """Assign a flight task to this tug."""
        # First check if the tug has enough battery to handle this task
        closest_node = min(
            self.nodes_dict.keys(),
            key=lambda n: (self.nodes_dict[n]["xy_pos"][0] - self.position[0])**2 + 
                        (self.nodes_dict[n]["xy_pos"][1] - self.position[1])**2
        )
        
        # Calculate if there's enough battery for the full journey
        # (to task start + execute task + return to depot)
        depot_node = 112 if self.type == "D" else 113
        
        # Simple distance estimation (could be improved)
        est_distance_to_task = ((self.nodes_dict[task.start_node]["xy_pos"][0] - self.position[0])**2 + 
                            (self.nodes_dict[task.start_node]["xy_pos"][1] - self.position[1])**2)**0.5
        
        est_distance_for_task = ((self.nodes_dict[task.goal_node]["xy_pos"][0] - self.nodes_dict[task.start_node]["xy_pos"][0])**2 + 
                                (self.nodes_dict[task.goal_node]["xy_pos"][1] - self.nodes_dict[task.start_node]["xy_pos"][1])**2)**0.5
        
        est_distance_to_depot = ((self.nodes_dict[depot_node]["xy_pos"][0] - self.nodes_dict[task.goal_node]["xy_pos"][0])**2 + 
                                (self.nodes_dict[depot_node]["xy_pos"][1] - self.nodes_dict[task.goal_node]["xy_pos"][1])**2)**0.5
        
        total_est_distance = (est_distance_to_task + est_distance_for_task + est_distance_to_depot) * 1.5  # Safety factor
        required_battery = total_est_distance * self.bat_disc
        
        if self.bat_state < required_battery:
            print(f"Tug {self.id} cannot accept task {task.flight_id} due to insufficient battery ({self.bat_perc:.1f}%)")
            return False
        
        print(f"Assigning task {task.flight_id} to tug {self.id}. Current status: {self.status}")
        self.current_task = task.flight_id
        self.goal = task.start_node  
        self.final_goal = task.goal_node  
        self.status = "moving_to_task"
        self.wait = True
        self.path_to_goal = []  # Reset path_to_goal to force replanning
        print(f"Tug {self.id} status updated to {self.status}. Goal set to pickup location: {self.goal}")
        return True
    

    def set_init_tug_params(self, tug_id, a_d, start_node, nodes_dict):
                #Fixed parameters
        self.speed = 1         #how much tug moves per unit of t
        self.id = tug_id       #tug_id
        self.type = a_d           #arrival or departure (A/D), also defines to which depot it will return initially
        self.start = start_node   #start_node_id
        self.goal = None     #goal_node_id
        self.coupled = start_node #coupled flight /depot 
        # Add the new fields
        self.current_task = None
        self.final_goal = None
        self.wait = None
        
        #Route related
        self.status = 'idle' # idle, moving_to_task, executing, to_depot
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]

        #State related
        self.heading = 0
        self.position = nodes_dict[start_node]["xy_pos"] # Initialize position to the start node's position

    def bidders_value(self, task, nodes_dict, heuristics, t, depots, gamma=1, alpha=1, beta=1, eta=1):
        '''returns the max price to pay at the auction (tug allocation)'''
        suitable = True
        # Determine the closest node in nodes_dict to the tug's current position.
        current_xy = self.position
        closest_node = min(
            nodes_dict.keys(),
            key=lambda n: (nodes_dict[n]["xy_pos"][0] - current_xy[0])**2 + (nodes_dict[n]["xy_pos"][1] - current_xy[1])**2
        )

        if self.type == 'D':
            depot_node = depots[0].position
        else:
            depot_node = depots[1].position
        
        # Plan path from the closest node to the task's start location.
        # TODO: path length is currently calculated using A*, but we could use a Heuristic in order to reduce computation time
        success, path = simple_single_agent_astar(nodes_dict, closest_node, task.start_node, heuristics, t)
        success_depot, path_depot = simple_single_agent_astar(nodes_dict, task.goal_node, depot_node, heuristics, t)
        
        if success:
            path_length = len(path)  # number of nodes in the path
        else:
            print('no path can be found')
            suitable = False
        
        delay = (t - task.spawn_time)
        # print(f'Tug {self.id}: task.spawn_time = {task.spawn_time}, t = {t}')
        # print(f'Tug {self.id}: path_length = {path_length}, delay = {delay}')

        # Battery check for task
        if success_depot:
            path_depot = len(path_depot)
        else:
            print('no path can be found')
            suitable = False

        bat_exp = self.bat_state - ((path_length + path_depot) * 1.5) * self.bat_disc   # Safety factor of 1.5 taken into account
        if bat_exp < 0:
            suitable = False

        return alpha * (delay)**gamma + beta / path_length + eta * self.bat_perc, suitable

        

