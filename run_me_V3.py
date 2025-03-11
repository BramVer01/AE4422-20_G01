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
from Aircraft_V3 import Tug

#%% SET SIMULATION PARAMETERS
nodes_file = "nodes.xlsx"
edges_file = "edges.xlsx"
simulation_time = 100
planner = "Prioritized"
plot_graph = False
visualization = True
visualization_speed = 0.1

#%%Function definitions
def import_layout(nodes_file, edges_file):
    gates_xy = []
    rwy_dep_xy = []
    rwy_arr_xy = []
    
    df_nodes = pd.read_excel(os.path.join(os.getcwd(), nodes_file))
    df_edges = pd.read_excel(os.path.join(os.getcwd(), edges_file))
    
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"], row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()}
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"], row["y_pos"]))
            
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        start_end_pos = (nodes_dict[row["from"]]["xy_pos"], nodes_dict[row["to"]]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos}
        edges_dict[edge_id] = edge_properties
   
    for edge in edges_dict:
        from_node = edge[0]
        nodes_dict[from_node]["neighbors"].add(edge[1])
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph=True):
    graph = nx.DiGraph()
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id=nodes_dict[node]["id"],
                       xy_pos=nodes_dict[node]["xy_pos"],
                       node_type=nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id=edge,
                       from_node=edges_dict[edge]["from"],
                       to_node=edges_dict[edge]["to"],
                       weight=edges_dict[edge]["length"])
    
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

#%% RUN SIMULATION
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

# Initialize list to track all aircraft and tugs
tug_list = []

# Initialize depots
departure_depot = Depot(1, position=112)
arrival_depot = Depot(2, position=113)

# Initialize tugs and add them to depots
tug1 = Tug(tug_id=1, a_d="D", start_node=112, spawn_time=0, nodes_dict=nodes_dict, heuristics=heuristics)
tug2 = Tug(tug_id=2, a_d="A", start_node=113, spawn_time=0, nodes_dict=nodes_dict, heuristics=heuristics)
departure_depot.tugs.put(tug1)
arrival_depot.tugs.put(tug2)
tug_list.append(tug1)
tug_list.append(tug2)

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
    map_properties = map_initialization(nodes_dict, edges_dict)

running = True
escape_pressed = False
time_end = simulation_time
dt = 0.1
t = 0
task_counter = 0

print("Simulation Started")
while running:
    t = round(t, 2)    

    # create task at t = 0 and every 3 time steps 
    if t == 0 or (t % 3 == 0):  
        task_counter += 1
        new_task_id = task_counter
        task = generate_flight_task(new_task_id)
        if task.type == "A":
            arrival_depot.add_task(task)
            print(f"Time {t}: New arrival task {task.flight_id} added to arrival depot (from {task.start_node} to {task.goal_node})")
        else:
            departure_depot.add_task(task)
            print(f"Time {t}: New departure task {task.flight_id} added to departure depot (from {task.start_node} to {task.goal_node})")
    
    # print status' of the depods every 10 time steps 
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

    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    if visualization:
        current_states = {}
        # for ac in tug_list:
        #     # print(f'tug {ac.id} has current location (x, y): ({ac.position})'
        #     if ac.status == "moving_to_task" or ac.status == "executing" or ac.status == 'to_depod':
        #         has_flight = hasattr(ac, 'current_task') and ac.current_task is not None
        #         current_states[ac.id] = {
        #             "ac_id": ac.id,
        #             "xy_pos": ac.position,
        #             "heading": ac.heading,
        #             "has_flight": has_flight
        #         }
        for ac in tug_list:
            if ac.status in ["moving_to_task", "executing", "to_depod"]:
                has_flight = hasattr(ac, 'current_task') and ac.current_task is not None
                current_states[ac.id] = {
                    "ac_id": ac.id,
                    "xy_pos": ac.position,
                    "heading": ac.heading,
                    "has_flight": has_flight,
                    "status": ac.status
                }

        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed)
      
    # if planner == "Independent":     
    #     for tug in tug_list:
    #         if tug.status == "taxiing" and not tug.path_to_goal:
    #             tug.plan_independent(nodes_dict, edges_dict, heuristics, t)

    # elif planner == "Prioritized":
    #     for tug in tug_list:
    #         if tug.status == "taxiing" and not tug.path_to_goal:
    #             tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t)

    if planner == "Prioritized":     
        for tug in tug_list:
            if tug.status == "moving_to_task" and not tug.path_to_goal:
                tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t)

    # elif planner == "CBS":
    #     run_CBS()
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    for tug in tug_list:
        if tug.status in ["moving_to_task", "executing", "to_depod", "returning"]:
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
