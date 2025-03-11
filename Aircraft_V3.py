from single_agent_planner import simple_single_agent_astar, simple_single_agent_astar_prioritized
import math

class Tug(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, tug_id, a_d, start_node, spawn_time, nodes_dict, heuristics=None):
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
        self.speed = 5         # Set a default speed (was 0)
        self.id = tug_id       # tug_id
        self.type = a_d        # arrival or departure (A/D), also defines to which depot it will return initially
        self.spawntime = spawn_time # spawntime
        self.start = start_node   # start_node_id
        self.goal = None     # goal_node_id
        self.nodes_dict = nodes_dict # keep copy of nodes dict
        self.coupled = start_node # coupled flight /depot 
        
        # Add the new fields
        self.heuristics = heuristics
        self.current_task = None
        self.final_goal = None
        
        # Route related


        self.status = 'idle' # idle, moving_to_task, executing, to_depod
        self.path_to_goal = [] # planned path left from current location
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
      
    # def move(self, dt, t):
    #     """
    #     Moves a tug between from_node and to_node and manages task progression.
    #     INPUT:
    #         - dt = time step increment
    #         - t = current simulation time
    #     """
    #     # Only move if the tug is taxiing or returning
    #     if self.status not in ["taxiing", "returning"]:
    #         return
            
    #     # Determine nodes between which the tug is moving
    #     from_node = self.from_to[0]
    #     to_node = self.from_to[1]
    #     xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
    #     xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
    #     distance_to_move = self.speed * dt  # distance to move in this timestep
    
    #     # Update position with rounded values
    #     x = xy_to[0] - xy_from[0]
    #     y = xy_to[1] - xy_from[1]
        
    #     # Avoid division by zero
    #     if x == 0 and y == 0:
    #         # Already at destination, no movement needed
    #         return
            
    #     distance = math.sqrt(x**2 + y**2)
    #     x_normalized = x / distance
    #     y_normalized = y / distance
        
    #     # Calculate new position
    #     posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
    #     posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
        
    #     # Check if we would overshoot the target
    #     target_distance = math.sqrt((xy_to[0] - self.position[0])**2 + (xy_to[1] - self.position[1])**2)
    #     if distance_to_move >= target_distance:
    #         # We would reach or overshoot the target, just set position to target
    #         self.position = xy_to
    #     else:
    #         self.position = (posx, posy)
            
    #     # Update heading
    #     self.get_heading(xy_from, xy_to)

    #     # Check if to_node is reached
    #     if self.position == xy_to and len(self.path_to_goal) > 0 and self.path_to_goal[0][1] == t+dt:
    #         # Current to_node is reached
            
    #         # Check if it's the final goal
    #         if self.position == self.nodes_dict[self.goal]["xy_pos"]:
    #             # Handle what happens when goal is reached
                
    #             if hasattr(self, 'current_task') and self.current_task is not None:
    #                 if self.goal == self.current_task.start_node and hasattr(self, 'final_goal') and self.final_goal is not None:
    #                     # Reached aircraft, now take it to its destination
    #                     print(f"Tug {self.id} reached aircraft at node {self.goal}, moving to {self.final_goal}")
    #                     self.start = self.goal
    #                     self.goal = self.final_goal
    #                     self.final_goal = None
                        
    #                     # Plan new path to final destination
    #                     success, path = simple_single_agent_astar(self.nodes_dict, self.start, self.goal, self.heuristics, t)
    #                     if success:
    #                         self.path_to_goal = path[1:]
    #                         next_node_id = self.path_to_goal[0][0]
    #                         self.from_to = [path[0][0], next_node_id]
    #                     else:
    #                         print(f"No path found for tug {self.id} to destination {self.goal}")
    #                         self.status = "waiting"
                    
    #                 elif hasattr(self, 'final_goal') and self.final_goal is None:
    #                     # Task completed, return to depot
    #                     print(f"Tug {self.id} completed task, returning to depot")
    #                     self.status = "returning"
                        
    #                     # Determine which depot to return to
    #                     depot_node = 112 if self.type == "D" else 113
    #                     self.start = self.goal
    #                     self.goal = depot_node
                        
    #                     # Plan path back to depot
    #                     success, path = simple_single_agent_astar(self.nodes_dict, self.start, self.goal, self.heuristics, t)
    #                     if success:
    #                         self.path_to_goal = path[1:]
    #                         next_node_id = self.path_to_goal[0][0]
    #                         self.from_to = [path[0][0], next_node_id]
    #                     else:
    #                         print(f"No path found for tug {self.id} to return to depot {depot_node}")
    #                         self.status = "stranded"  # A new status for tugs that can't find their way back
                
    #             # Check if tug has returned to depot
    #             if self.status == "returning" and self.goal in [112, 113]:
    #                 print(f"Tug {self.id} returned to depot {self.goal}")
    #                 self.status = "waiting"
    #                 self.coupled = self.goal
    #                 self.current_task = None
                    
    #                 # Add tug back to appropriate depot queue - need access to depot objects
    #                 # This part might need to be handled in the main simulation loop
    #                 # or by adding depot references to the Tug class
                    
    #         else:
    #             # Current to_node is reached but it's not the final goal
    #             # Update the remaining path
    #             remaining_path = self.path_to_goal
    #             self.path_to_goal = remaining_path[1:]
                
    #             if not self.path_to_goal:
    #                 # No more nodes in path but we haven't reached goal
    #                 # This shouldn't happen with proper planning, but handle it gracefully
    #                 print(f"Warning: Tug {self.id} ran out of path nodes before reaching goal {self.goal}")
    #                 self.status = "waiting"
    #                 return
                    
    #             new_from_id = self.from_to[1]  # new from node
    #             new_next_id = self.path_to_goal[0][0]  # new to node
                
    #             if new_from_id != self.from_to[0]:
    #                 self.last_node = self.from_to[0]
                
    #             self.from_to = [new_from_id, new_next_id]  # update new from and to node

    def move(self, dt, t):
        """
        Moves a tug between nodes and manages state transitions.
        """
        if self.status not in ["moving_to_task", "executing", "to_depod"]:
            print(f"Tug {self.id} is not moving because status is {self.status}")
            return

        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"]
        xy_to = self.nodes_dict[to_node]["xy_pos"]
        distance_to_move = self.speed * dt

        # Compute movement vector.
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        
        if x == 0 and y == 0:
            print(f"Tug {self.id} is already at destination node {to_node}")
            return
            
        distance = math.sqrt(x**2 + y**2)
        x_normalized = x / distance
        y_normalized = y / distance

        # New position computation.
        posx = round(self.position[0] + x_normalized * distance_to_move, 2)
        posy = round(self.position[1] + y_normalized * distance_to_move, 2)
        target_distance = math.sqrt((xy_to[0] - self.position[0])**2 + (xy_to[1] - self.position[1])**2)
        
        if distance_to_move >= target_distance:
            self.position = xy_to
        else:
            self.position = (posx, posy)

        self.get_heading(xy_from, xy_to)
        # print(f"Tug {self.id} moved to {self.position} with heading {self.heading}")

        # Check if the node has been reached (simple check by position equality).
        if self.position == xy_to:
            # print(f"Tug {self.id} reached node {to_node}")
            # Update path: remove the reached node.
            if self.path_to_goal:
                self.path_to_goal = self.path_to_goal[1:]
            if self.path_to_goal:
                new_next = self.path_to_goal[0][0]
                self.from_to = [to_node, new_next]
                # print(f"Tug {self.id} updated path: {self.path_to_goal}, new from_to: {self.from_to}")
            else:
                # Path is complete: decide the next state transition.
                if self.status == "moving_to_task":
                    # Reached pickup location; now switch to executing.
                    # print(f"Tug {self.id} reached pickup location {self.goal}. Switching status to executing.")
                    self.status = "executing"
                    # Set new goal to aircraft's destination.
                    self.goal = self.final_goal
                    self.coupled = to_node  # update current node
                    self.request_path()
                elif self.status == "executing":
                    # Reached the aircraft's destination.
                    print(f"Tug {self.id} delivered task. Switching status to to_depod.")
                    self.status = "to_depod"
                    depot_node = 112 if self.type == "D" else 113
                    self.goal = depot_node
                    self.coupled = to_node
                    self.request_path()
                elif self.status == "to_depod":
                    # Reached the depot.
                    # print(f"Tug {self.id} returned to depot at node {to_node}. Setting status to idle.")
                    self.status = "idle"
                    self.coupled = to_node
                    self.current_task = None

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

    
    # def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t):
    #     """
    #     Plans a path for taxiing aircraft assuming that it knows the entire layout.
    #     Other traffic is not taken into account.
    #     INPUT:
    #         - nodes_dict: copy of the nodes_dict
    #         - edges_dict: edges_dict with current edge weights
    #     """
        
    #     if self.status == "taxiing":
    #         start_node = self.start #node from which planning should be done
    #         goal_node = self.goal #node to which planning should be done
            
    #         success, path = simple_single_agent_astar_prioritized(nodes_dict, start_node, goal_node, heuristics, t)
    #         if success:
    #             self.path_to_goal = path[1:]
    #             next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
    #             self.from_to = [path[0][0], next_node_id]
    #             print("Path tug", self.id, ":", path)
    #         else:
    #             raise Exception("No solution found for", self.id)
            
    #         #Check the path
    #         if path[0][1] != t:
    #             raise Exception("Something is wrong with the timing of the path planning")

    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t, delta_t=0.1, constraints=[]):
        # Allow planning when the tug is in the "moving_to_task" or "executing" state.
        if self.status in ["moving_to_task", "executing"] and not self.path_to_goal:
            start_node = self.start
            goal_node = self.goal
            # Call the prioritized A* function with the extra parameters.
            success, path = simple_single_agent_astar_prioritized(
                nodes_dict, start_node, goal_node, heuristics, t, delta_t, self, constraints
            )
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]
                self.from_to = [path[0][0], next_node_id]
                print("Path (prioritized) for tug", self.id, ":", path)
            else:
                raise Exception("No solution found for tug", self.id)
            
            # Optionally, verify that the planning start time matches.
            if path[0][1] != t:
                raise Exception("Timing error in path planning for tug", self.id)


    # def assign_task(self, task):
    #     """Assign a flight task to this tug."""

    #     print('assinging task to tug and updateing tug status...')

    #     self.current_task = task
    #     self.goal = task.start_node  # First go to the aircraft
    #     self.final_goal = task.goal_node  # Then take aircraft to its destination
    #     self.status = "moving_to_task"  # Start moving

    def assign_task(self, task):
        """Assign a flight task to this tug."""
        print(f"Assigning task {task.flight_id} to tug {self.id}. Current status: {self.status}")
        self.current_task = task
        # Set pickup location (aircraft location) as the immediate goal.
        self.goal = task.start_node  
        self.final_goal = task.goal_node  
        self.status = "moving_to_task"
        print(f"Tug {self.id} status updated to {self.status}. Goal set to pickup location: {self.goal}")
        # Request a path to the pickup location.
        self.request_path()

    def request_path(self):
        """Request a path based on the current goal."""
        print(f"Tug {self.id} is requesting path from {self.coupled} to {self.goal}")
        self.start = self.coupled  # use current position as starting node
        success, path = simple_single_agent_astar(self.nodes_dict, self.start, self.goal, self.heuristics, self.spawntime)
        if success:
            self.path_to_goal = path[1:]
            next_node_id = self.path_to_goal[0][0]
            self.from_to = [path[0][0], next_node_id]
            self.position = self.nodes_dict[self.start]["xy_pos"]
            # print(f"Tug {self.id} obtained path: {self.path_to_goal} with from_to: {self.from_to}")
        else:
            print(f"No path found for tug {self.id} to goal {self.goal}")
            self.status = "idle"

    # def request_path(self, task):
    #     """Request a path to the task start location."""
    #     print('requesting path...')

    #     self.start = self.coupled  # Current position
    #     success, path = simple_single_agent_astar(self.nodes_dict, self.start, self.goal, self.heuristics, self.spawntime)
    #     if success:
    #         self.path_to_goal = path[1:]
    #         next_node_id = self.path_to_goal[0][0]
    #         self.from_to = [path[0][0], next_node_id]
    #         self.position = self.nodes_dict[self.start]["xy_pos"]  # Set initial position
    #         print(f"Tug {self.id} status {self.status}, heading to node {self.goal}")
    #         print(f"Tug {self.id} has the path {self.path_to_goal} to the goal")
    #     else:
    #         print(f"No path found for tug {self.id} to task at {self.goal}")
    #         self.status = "idle"
        