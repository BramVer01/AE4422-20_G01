"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import random  # Add this for the random choice in generating flight tasks
import time    # Add this for using current time in flight tasks
from single_agent_planner import calc_heuristics
from visualization_V3 import map_initialization, map_running
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
from Depot_file import Depot, FlightTask  # Import specific classes
from Aircraft_V3 import Tug  # Import the Tug class


#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 100
planner = "Prioritized" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)


# Initialize list to track all aircraft and tugs
aircraft_lst = []

# Initialize depots
departure_depot = Depot(1, position=112)
arrival_depot = Depot(2, position=113)

# Initialize tugs and add them to depots
tug1 = Tug(tug_id=1, a_d="D", start_node=112, spawn_time=0, nodes_dict=nodes_dict, heuristics=heuristics)
tug2 = Tug(tug_id=2, a_d="A", start_node=113, spawn_time=0, nodes_dict=nodes_dict, heuristics=heuristics)

# Add tugs to their respective depots
departure_depot.tugs.put(tug1)
arrival_depot.tugs.put(tug2)

# Add tugs to the aircraft_lst for tracking and visualization
aircraft_lst.append(tug1)
aircraft_lst.append(tug2)


def generate_flight_task(flight_id): #NIEUW
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

aircraft_lst = []   #List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0
task_counter = 0


print("Simulation Started")
while running:
    t = round(t, 2)    
    
    # Generate new tasks randomly with some probability
    if random.random() < 0.1:  # 10% chance per time step to generate a new task
        task_counter += 1
        new_task_id = task_counter
        
        # Generate a flight task
        task = generate_flight_task(new_task_id)
        
        # Assign to appropriate depot
        if task.type == "A":
            arrival_depot.add_task(task)
            print(f"Time {t}: New arrival task {task.flight_id} added to arrival depot (from {task.start_node} to {task.goal_node})")
        else:
            departure_depot.add_task(task)
            print(f"Time {t}: New departure task {task.flight_id} added to departure depot (from {task.start_node} to {task.goal_node})")
    
    # Try to match tasks with available tugs
    arrival_depot.match_task()
    departure_depot.match_task()

    # Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    # Visualization: Update map if visualization is true
    if visualization:
        current_states = {} # Collect current states of all vehicles
        for ac in aircraft_lst:
            if ac.status == "taxiing" or ac.status == "returning":
                # Check if this is a tug and whether it has a flight assigned
                has_flight = hasattr(ac, 'current_task') and ac.current_task is not None
                
                current_states[ac.id] = {
                    "ac_id": ac.id,
                    "xy_pos": ac.position,
                    "heading": ac.heading,
                    "has_flight": has_flight  # This tells the visualization whether to show a tug or aircraft
                }
                
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed)
      
    # Do planning 
    if planner == "Independent":     
        for tug in aircraft_lst:
            if tug.status == "taxiing" and not tug.path_to_goal:
                tug.plan_independent(nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        for tug in aircraft_lst:
            if tug.status == "taxiing" and not tug.path_to_goal:
                tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t)
    elif planner == "CBS":
        run_CBS()
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    # Move the tugs that are taxiing or returning
    for ac in aircraft_lst: 
        if ac.status in ["taxiing", "returning"]: 
            ac.move(dt, t)
                           
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
