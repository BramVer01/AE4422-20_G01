"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import random
import time
from single_agent_planner import calc_heuristics
from visualization_V3 import map_initialization, map_running
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
from Depot_file import Depot, FlightTask
from Aircraft_V4 import Tug

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 50
planner = "Prioritized" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

task_interval = 5    # New: generate a task every 5 seconds
total_tugs = 8       # New: total number of tugs (will be split evenly between depots)


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
    
    df_nodes = pd.read_excel(os.path.join(os.getcwd(), nodes_file))
    df_edges = pd.read_excel(os.path.join(os.getcwd(), edges_file))
    
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
tug_list = []   #List which can contain tug agents

# Initialize depots
departure_depot = Depot(1, position=112)
arrival_depot = Depot(2, position=113)

# Initialize tugs and add them to depots
# Initialize tugs and add them to depots based on total_tugs parameter
for i in range(total_tugs):
    tug_id = i + 1
    if i < total_tugs // 2:
        # First half: departure tugs (type "D")
        tug = Tug(tug_id=tug_id, a_d="D", start_node=departure_depot.position, spawn_time=0, nodes_dict=nodes_dict)
        departure_depot.tugs.put(tug)
    else:
        # Second half: arrival tugs (type "A")
        tug = Tug(tug_id=tug_id, a_d="A", start_node=arrival_depot.position, spawn_time=0, nodes_dict=nodes_dict)
        arrival_depot.tugs.put(tug)
    tug_list.append(tug)


def generate_flight_task(flight_id):
    a_d = random.choice(["A", "D"])
    if a_d == "A":
        start_node = random.choice([37, 38])
        goal_node = random.choice([97, 34, 35, 36, 98])
    else:
        start_node = random.choice([97, 34, 35, 36, 98])
        goal_node = random.choice([1, 2])
    spawn_time = time.time()
    return FlightTask(flight_id, a_d, start_node, goal_node, spawn_time)

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
delta_t = 0.5
constraints = []

print("Simulation Started")
while running:
    t= round(t,2)    
    # Create task at t = 0 and every task_interval seconds.
    if abs(t - round(t)) < 1e-9 and (round(t) % task_interval == 0):
        task_counter += 1
        new_task_id = task_counter
        task = generate_flight_task(new_task_id)
        if task.type == "A":
            arrival_depot.add_task(task)
            print(f"Time {t}: New arrival task {task.flight_id} added to arrival depot (from {task.start_node} to {task.goal_node})")
        else:
            departure_depot.add_task(task)
            print(f"Time {t}: New departure task {task.flight_id} added to departure depot (from {task.start_node} to {task.goal_node})")

    
    # print status' of the depots every 10 time steps 
    if t == 0 or (t % 10 == 0):  
        dep_tugs_ids = [tug.id for tug in departure_depot.tugs.queue]
        dep_tasks_ids = [task.flight_id for task in departure_depot.tasks.queue]
        arr_tugs_ids = [tug.id for tug in arrival_depot.tugs.queue]
        arr_tasks_ids = [task.flight_id for task in arrival_depot.tasks.queue]
        
        print(f"\n--- Time {t} Depot Queues ---")
        print("Departure Depot Tugs:", dep_tugs_ids)
        print("Departure Depot Tasks:", dep_tasks_ids)
        print("Arrival Depot Tugs:", arr_tugs_ids)
        print("Arrival Depot Tasks:", arr_tasks_ids)
        print("-------------------------------\n")

    arrival_depot.match_task(t)
    departure_depot.match_task(t)
       
    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
        for tug in tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                has_flight = hasattr(tug, 'current_task') and tug.current_task is not None
                current_states[tug.id] = {"tug_id": tug.id,
                                          "xy_pos": tug.position,
                                          "heading": tug.heading,
                                          "has_flight": has_flight,
                                          "status": tug.status}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed) 
      
    #Spawn aircraft for this timestep (use for example a random process)
    # if t == 1:    
    #     ac = Aircraft(1, 'A', 37,36,t, nodes_dict) #As an example we will create one aicraft arriving at node 37 with the goal of reaching node 36
    #     ac1 = Aircraft(2, 'D', 36,37,t, nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
    #     aircraft_lst.append(ac)
    #     aircraft_lst.append(ac1)
        
    #Do planning 
    if planner == "Independent":     
        for tug in tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                run_independent_planner(tug, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        for tug in tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                constraints = run_prioritized_planner(tug_list, tug, nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    elif planner == "CBS":
        run_CBS()
    #elif planner == -> you may introduce other planners here
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    #Move the tugs that are in use
    for tug in tug_list:
        if tug.status in ["moving_to_task", "executing", "to_depot"]:
            tug.move(dt, t)

    # Check for idle tugs that have reached the depot and update depot queues
    for tug in tug_list:
        if tug.status == "idle":
            if tug.type == "D" and tug.coupled == departure_depot.position:
                if tug not in departure_depot.tugs.queue:
                    departure_depot.tugs.put(tug)
                    print(f"Tug {tug.id} has returned to the departure depot.")
            elif tug.type == "A" and tug.coupled == arrival_depot.position:
                if tug not in arrival_depot.tugs.queue:
                    arrival_depot.tugs.put(tug)
                    print(f"Tug {tug.id} has returned to the arrival depot.")                           
                           
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
