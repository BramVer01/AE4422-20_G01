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
            - dt = 
            - t = 
        """
        
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        if from_node == 0 or to_node == 0 or not self.path_to_goal:
            self.position = self.position
            self.wait = True
            return
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep
  
        #Update position with rounded values
        if xy_from == xy_to or self.wait:
            # Waiting: position remains the same.
            self.position = self.position
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

            posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
            posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
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

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
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
        print(f"Assigning task {task.flight_id} to tug {self.id}. Current status: {self.status}")
        self.current_task = task.flight_id
        # Set pickup location (aircraft location) as the immediate goal.
        self.goal = task.start_node  
        self.final_goal = task.goal_node  
        self.status = "moving_to_task"
        self.wait = True
        print(f"Tug {self.id} status updated to {self.status}. Goal set to pickup location: {self.goal}")

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
