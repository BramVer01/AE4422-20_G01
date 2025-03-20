"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
from time import sleep

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
from Auctioneer_file import Auctioneer
import matplotlib.pyplot as plt


#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
dubai = False  # Whether to use Dubai or baseline network
if dubai:
    nodes_file = "nodes_DXB.xlsx"  # xlsx file with for each node: id, x_pos, y_pos, type
    edges_file = "edges_DXB.xlsx"  # xlsx file with for each edge: from  (node), to (node), length
else:
    nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
    edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 100
planner = "Prioritized" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

task_interval = 5    # New: generate a task every 5 seconds
total_tugs = 4       # New: total number of tugs (will be split evenly between depots)


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

def run_simulation(visualization_speed, task_interval, total_tugs, simulation_time):
    # --- KPI Global Variables ---
    total_collisions = 0
    task_completion_times = []         # Execution time: from "moving_to_task"->"executing" to "executing"->"to_depot"
    total_task_completion_times = []   # Total task time: from "idle"->"moving_to_task" to "to_depot"->"idle"
    task_distances = []                # Number of node transitions during execution (only computed for execution phase)
    total_tasks_completed = 0          # Total number of tasks completed

    total_task_start_times = {}        # Record time when a tug is assigned a task (idle -> moving_to_task)
    execution_start_times = {}         # Record time when a tug starts executing (moving_to_task -> executing)
    task_nodes = {}                    # List of nodes traversed during execution (for distance metric)
    prev_status = {}                   # Previous status for each tug
    # ----------------------------------------------------------

    nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    # Initialize list to track all tug agents
    tug_list = []

    # Initialize depots
    departure_depot = Depot(1, position=112)
    arrival_depot = Depot(2, position=113)

    # Create tugs and add to corresponding depot queue
    for i in range(total_tugs):
        tug_id = i + 1
        if i < total_tugs // 2:
            tug = Tug(tug_id=tug_id, a_d="D", start_node=departure_depot.position, spawn_time=0, nodes_dict=nodes_dict)
            departure_depot.tugs.append(tug)
        else:
            tug = Tug(tug_id=tug_id, a_d="A", start_node=arrival_depot.position, spawn_time=0, nodes_dict=nodes_dict)
            arrival_depot.tugs.append(tug)
        tug_list.append(tug)
        prev_status[tug.id] = tug.status

    # Initialize Auctioneer
    auctioneer = Auctioneer(tug_list)

    # Flight task generator function
    def generate_flight_task(flight_id, t):
        a_d = random.choice(["A", "D"])
        if a_d == "A":
            start_node = random.choice([37, 38])
            available_gates = [gate for gate in [97, 34, 35, 36, 98] if gate not in gate_status]
            if available_gates:
                goal_node = random.choice(available_gates)
                gate_status[goal_node] = {"release_time": t + 10, "flight_id": flight_id}
                print(f"Time {t}: Aircraft {flight_id} arriving at gate {goal_node}, scheduled to depart at {t+10}")
            else:
                goal_node = "waiting"
                print(f"Time {t}: Aircraft {flight_id} is waiting for a free gate.")
        else:
            ready_flights = [gate for gate, info in gate_status.items() if info["release_time"] <= t]
            if not ready_flights:
                return None
            start_node = random.choice(ready_flights)
            goal_node = random.choice([1, 2])
            departing_flight_id = gate_status[start_node]["flight_id"]
            del gate_status[start_node]
            print(f"Time {t}: Aircraft {departing_flight_id} departing from gate {start_node} to runway {goal_node}")
            return FlightTask(departing_flight_id, "D", start_node, goal_node, t)
        return FlightTask(flight_id, a_d, start_node, goal_node, t)

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict)

    running = True
    escape_pressed = False
    time_end = simulation_time
    dt = 0.1
    t = 0
    task_counter = 0
    delta_t = 0.5
    constraints = []
    gate_status = {}

    print("Simulation Started")
    while running:
        t = round(t, 2)
        # --- Task Creation ---
        for gate, info in list(gate_status.items()):
            if info["release_time"] <= t:
                print(f"Time {t}: Aircraft {info['flight_id']} at gate {gate} is now ready for departure.")
                task = FlightTask(info["flight_id"], "D", gate, random.choice([1, 2]), t)
                if task:
                    departure_depot.add_task(task)
                    print(f"Time {t}: Departure task for Aircraft {info['flight_id']} created (from {gate} to runway).")
                del gate_status[gate]
                if "waiting_aircraft" in globals() and waiting_aircraft:
                    next_aircraft = waiting_aircraft.pop(0)
                    gate_status[gate] = {"release_time": t + 10, "flight_id": next_aircraft.flight_id}
                    next_aircraft.goal_node = gate
                    print(f"Time {t}: Waiting aircraft {next_aircraft.flight_id} is now assigned to gate {gate}.")

        if abs(t - round(t)) < 1e-9 and (round(t) % task_interval == 0):
            task_counter += 1
            task_id = task_counter
            task = generate_flight_task(task_id, t)
            if task:
                if task.goal_node == "waiting":
                    if "waiting_aircraft" not in globals():
                        waiting_aircraft = []
                    waiting_aircraft.append(task)
                    print(f"Time {t}: Aircraft {task.flight_id} is waiting for a free gate.")
                elif task.type == "A":
                    arrival_depot.add_task(task)
                    print(f"Time {t}: New arrival task {task.flight_id} added to arrival depot (from {task.start_node} to {task.goal_node})")
                else:
                    departure_depot.add_task(task)
                    print(f"Time {t}: New departure task {task.flight_id} added to departure depot (from {task.start_node} to {task.goal_node})")
        
        # --- Print Status Every 10 Seconds ---
        if t == 0 or (t % 10 == 0):
            dep_tugs_ids = [tug.id for tug in departure_depot.tugs]
            dep_tasks_ids = [task.flight_id for task in departure_depot.tasks]
            arr_tugs_ids = [tug.id for tug in arrival_depot.tugs]
            arr_tasks_ids = [task.flight_id for task in arrival_depot.tasks]
            print(f"\n--- Time {t} Depot Queues ---")
            print("Departure Depot Tugs:", dep_tugs_ids)
            print("Departure Depot Tasks:", dep_tasks_ids)
            print("Arrival Depot Tugs:", arr_tugs_ids)
            print("Arrival Depot Tasks:", arr_tasks_ids)
            print("-------------------------------\n")
            for tug in tug_list:
                print(f'Tug {tug.id} is charged at: {tug.bat_perc} %')
                for task in departure_depot.tasks:
                    bidders_value_price = tug.bidders_value(task, nodes_dict, heuristics, t, [departure_depot,arrival_depot], gamma=1, alpha=1, beta=1)
                    #print(f"Tug {tug.id}: price for task {task.flight_id} = {bidders_value_price}")
                
                for task in arrival_depot.tasks:
                    bidders_value_price = tug.bidders_value(task, nodes_dict, heuristics, t, [departure_depot,arrival_depot], gamma=1, alpha=1, beta=1)
                    #print(f"Tug {tug.id}: price for task {task.flight_id} = {bidders_value_price}")
                
                print(f"Tug {tug.id}: status = {tug.status}, coupled = {tug.coupled}, position = {tug.position}")
                if hasattr(tug, 'path_to_goal'):
                    print(f"  Current path: {tug.path_to_goal}")
                    print(f"  Current goal: {tug.goal}")
                else:
                    print("  Current path: None")
        
        # --- Collision Detection KPI (always computed) ---
        current_states = {}
        for tug in tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                has_flight = hasattr(tug, 'current_task') and tug.current_task is not None
                current_states[tug.id] = {
                    "tug_id": tug.id,
                    "xy_pos": tug.position,
                    "heading": tug.heading,
                    "has_flight": has_flight,
                    "status": tug.status,
                    "bat_perc": tug.bat_perc  # Add battery percentage
                }
        if visualization:
            escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed)
        
        for id1 in current_states:
            for id2 in current_states:
                if id1 < id2:
                    if current_states[id1]["xy_pos"] == current_states[id2]["xy_pos"]:
                        total_collisions += 1
                        print(f"Collision detected between Tug {id1} and Tug {id2} at time {t}")

        # Tasks Assignment
        tasks_available = departure_depot.tasks + arrival_depot.tasks
        if len(tasks_available) > 0:
            auctioneer.tug_availability(tug_list)
            auctioneer.ask_price(tasks_available,nodes_dict,heuristics,t,[departure_depot,arrival_depot])
            auctioneer.decision(departure_depot,arrival_depot)

        # Tugs Charging
        departure_depot.charging(dt)
        arrival_depot.charging(dt)

        if t >= time_end or escape_pressed:
            running = False
            pg.quit()
            print("Simulation Stopped")
            break

        # --- Run Planning ---
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
        else:
            raise Exception("Planner:", planner, "is not defined.")

        for tug in tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                tug.move(dt, t)
        
        # --- KPI: Detect Task Completion & Record Metrics ---
        # We record two time metrics:
        #   1. Execution Time (old KPI): from when a tug enters "executing" until it transitions to "to_depot".
        #   2. Total Task Time (new KPI): from when a tug is assigned a task (idle -> moving_to_task) until it returns to "idle".
        for tug in tug_list:
            current_node = getattr(tug, 'current_node', tug.start)
            # Record total task start time when a tug is assigned a task:
            if prev_status[tug.id] == "idle" and tug.status == "moving_to_task":
                total_task_start_times[tug.id] = t

            # Record execution start time when a tug transitions from moving_to_task to executing:
            if prev_status[tug.id] == "moving_to_task" and tug.status == "executing":
                execution_start_times[tug.id] = t
                task_nodes[tug.id] = [current_node]

            # Update node list while task is active:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                if tug.id in task_nodes:
                    if task_nodes[tug.id][-1] != current_node:
                        task_nodes[tug.id].append(current_node)
                else:
                    task_nodes[tug.id] = [current_node]

            # When execution completes (executing -> to_depot), record execution time:
            if prev_status[tug.id] == "executing" and tug.status == "to_depot":
                if tug.id in execution_start_times and tug.id in task_nodes:
                    exec_duration = t - execution_start_times[tug.id]
                    task_completion_times.append(exec_duration)
                    traveled_distance = len(task_nodes[tug.id]) - 1
                    task_distances.append(traveled_distance)
                    total_tasks_completed += 1
                    print(f"Tug {tug.id} executed task in {exec_duration} sec, nodes traversed: {traveled_distance}")
                    del execution_start_times[tug.id]
                    del task_nodes[tug.id]

            # When total task completes (to_depot -> idle), record total task time:
            if prev_status[tug.id] == "to_depot" and tug.status == "idle":
                if tug.id in total_task_start_times:
                    total_duration = t - total_task_start_times[tug.id]
                    # Append to new KPI list for total task time.
                    total_task_completion_times.append(total_duration)
                    print(f"Tug {tug.id} completed total task in {total_duration} sec")
                    del total_task_start_times[tug.id]

            prev_status[tug.id] = tug.status

        # --- Update Depot Queues for Idle Tugs ---
        for tug in tug_list:
            if tug.status == "idle":
                if tug.type == "D" and tug.coupled == departure_depot.position:
                    if tug not in departure_depot.tugs:
                        departure_depot.tugs.append(tug)
                        tug.set_init_tug_params(tug.id, "D", departure_depot.position, nodes_dict)
                        print(f"Tug {tug.id} has returned to the departure depot.")
                elif tug.type == "A" and tug.coupled == arrival_depot.position:
                    if tug not in arrival_depot.tugs:
                        arrival_depot.tugs.append(tug)
                        tug.set_init_tug_params(tug.id, "A", arrival_depot.position, nodes_dict)
                        print(f"Tug {tug.id} has returned to the arrival depot.")


        t = t + dt

    # --- Compute Average KPI Values ---
    if task_completion_times:
        avg_execution_time = sum(task_completion_times) / len(task_completion_times)
    else:
        avg_execution_time = 0
    if total_task_completion_times:
        avg_total_time = sum(total_task_completion_times) / len(total_task_completion_times)
    else:
        avg_total_time = 0
    if task_distances:
        avg_distance = sum(task_distances) / len(task_distances)
    else:
        avg_distance = 0

    print("\n----- KPI SUMMARY -----")
    print("Total collisions detected:", total_collisions)
    print("Total tasks completed:", total_tasks_completed)
    print("Average execution time (old KPI):", avg_execution_time)
    print("Average total task time (new KPI):", avg_total_time)
    print("Average task distance (in nodes traversed):", avg_distance)
    print("-----------------------")

    return {"collisions": total_collisions,
            "tasks_completed": total_tasks_completed,
            "avg_execution_time": avg_execution_time,
            "avg_total_time": avg_total_time,
            "avg_distance": avg_distance}

# To run the simulation standalone:
if __name__ == "__main__":
    visualization_speed = 0.001
    total_tugs = 3
    task_interval = 0.5
    run_simulation(visualization_speed, task_interval, total_tugs, simulation_time)


# if __name__ == "__main__":
#     import matplotlib.pyplot as plt

#     visualization = False
#     simulation_time = 50
#     visualization_speed = 0.00001  # Speed up simulation
#     task_interval = 3             # Fixed task interval for this experiment
#     tug_counts = list(range(1, 14))  # From 1 to 8 tugs
#     num_executions = 15

#     # Containers for averaged KPI values
#     collisions_data = []         # average collisions for each tug_count
#     avg_execution_time_data = []   # average execution time (old KPI) per tug_count
#     avg_total_time_data = []       # average total task time (new KPI) per tug_count
#     tasks_completed_data = []      # average total tasks completed per tug_count

#     # Containers for individual run data (for scatter plotting)
#     collisions_individual = {}     # mapping tug_count -> list of collisions (one per run)
#     exec_time_individual = {}        # mapping tug_count -> list of execution times (one per run)
#     total_time_individual = {}       # mapping tug_count -> list of total task times (one per run)
#     tasks_individual = {}          # mapping tug_count -> list of tasks completed (one per run)

#     # For each tug_count value, run the simulation multiple times and average the KPIs.
#     for tug_count in tug_counts:
#         collisions_runs = []
#         exec_time_runs = []
#         total_time_runs = []
#         tasks_runs = []
#         for run in range(num_executions):
#             kpi = run_simulation(visualization_speed, task_interval, tug_count, simulation_time)
#             collisions_runs.append(kpi["collisions"])
#             exec_time_runs.append(kpi["avg_execution_time"])
#             total_time_runs.append(kpi["avg_total_time"])
#             tasks_runs.append(kpi["tasks_completed"])
#         avg_collisions = sum(collisions_runs) / len(collisions_runs)
#         avg_exec_time = sum(exec_time_runs) / len(exec_time_runs)
#         avg_total_time = sum(total_time_runs) / len(total_time_runs)
#         avg_tasks = sum(tasks_runs) / len(tasks_runs)
#         collisions_data.append(avg_collisions)
#         avg_execution_time_data.append(avg_exec_time)
#         avg_total_time_data.append(avg_total_time)
#         tasks_completed_data.append(avg_tasks)
#         collisions_individual[tug_count] = collisions_runs
#         exec_time_individual[tug_count] = exec_time_runs
#         total_time_individual[tug_count] = total_time_runs
#         tasks_individual[tug_count] = tasks_runs
#         print(f"Tugs: {tug_count} -> Avg Collisions: {avg_collisions}, Avg Exec Time: {avg_exec_time}, Avg Total Time: {avg_total_time}, Total Tasks: {avg_tasks}")

#     # Plot everything in one graph using twin y-axes.
#     fig, ax1 = plt.subplots(figsize=(10, 6))
#     ax2 = ax1.twinx()

#     # Left y-axis: Plot average collisions (blue), average execution time (red) and average total task time (orange).
#     ax1.plot(tug_counts, collisions_data, marker='o', color='blue', label='Avg Collisions')
#     ax1.plot(tug_counts, avg_execution_time_data, marker='o', color='red', label='Avg Exec Time')
#     ax1.plot(tug_counts, avg_total_time_data, marker='o', color='orange', label='Avg Total Task Time')
#     for tc in tug_counts:
#         runs_collisions = collisions_individual[tc]
#         ax1.scatter([tc] * len(runs_collisions), runs_collisions, color='blue', alpha=0.6)
#         runs_exec = exec_time_individual[tc]
#         ax1.scatter([tc] * len(runs_exec), runs_exec, color='red', alpha=0.6)
#         runs_total = total_time_individual[tc]
#         ax1.scatter([tc] * len(runs_total), runs_total, color='orange', alpha=0.6)
#     ax1.set_xlabel("Number of Tugs")
#     ax1.set_ylabel("Avg Collisions / Task Times", color='black')
#     ax1.tick_params(axis='y', labelcolor='black')

#     # Right y-axis: Plot total tasks completed (green)
#     ax2.plot(tug_counts, tasks_completed_data, marker='o', color='green', label='Total Tasks Completed')
#     for tc in tug_counts:
#         runs_tasks = tasks_individual[tc]
#         ax2.scatter([tc] * len(runs_tasks), runs_tasks, color='green', alpha=0.6)
#     ax2.set_ylabel("Total Tasks Completed", color='green')
#     ax2.tick_params(axis='y', labelcolor='green')

#     plt.title("KPIs vs Number of Tugs")
#     # Combine legends from both axes.
#     lines1, labels1 = ax1.get_legend_handles_labels()
#     lines2, labels2 = ax2.get_legend_handles_labels()
#     ax1.legend(lines1 + lines2, labels1 + labels2, loc='best')

#     plt.tight_layout()
#     plt.show()


# '''TASK INVERVAL OMHOOG'''
# if __name__ == "__main__":
#     import matplotlib.pyplot as plt

#     visualization = False
#     simulation_time = 100
#     visualization_speed = 0.00001  # Speed up simulation
#     total_tugs = 4

#     # Vary task_interval from 10 down to 1 (here using step 2; adjust if desired)
#     task_intervals = list(range(10, 0, -2))
#     collisions_data = []   # average collisions per task_interval
#     avg_time_data = []     # average task completion time per task_interval
#     tasks_completed_data = []  # average total tasks completed per task_interval

#     # For scatter plots, store individual run data
#     collisions_all = []  # list of lists: collisions for each run at a given task_interval
#     times_all = []       # list of lists: task times for each run at a given task_interval
#     tasks_all = []       # list of lists: tasks completed for each run at a given task_interval

#     # For each task_interval, run the simulation 5 times and average the KPIs.
#     for ti in task_intervals:
#         collisions_runs = []
#         time_runs = []
#         tasks_runs = []
#         for run in range(15):
#             kpi = run_simulation(visualization_speed, ti, total_tugs, simulation_time)
#             collisions_runs.append(kpi["collisions"])
#             time_runs.append(kpi["avg_time"])
#             tasks_runs.append(kpi["tasks_completed"])
#         avg_collisions = sum(collisions_runs) / len(collisions_runs)
#         avg_time = sum(time_runs) / len(time_runs)
#         avg_tasks = sum(tasks_runs) / len(tasks_runs)

#         collisions_data.append(avg_collisions)
#         avg_time_data.append(avg_time)
#         tasks_completed_data.append(avg_tasks)

#         collisions_all.append(collisions_runs)
#         times_all.append(time_runs)
#         tasks_all.append(tasks_runs)

#         print(f"Task Interval: {ti} -> Avg Collisions: {avg_collisions}, Avg Time: {avg_time}, Avg Tasks Completed: {avg_tasks}")

#     # Plot all three KPIs in the same figure
#     plt.figure()
#     # Plot average curves
#     plt.plot(task_intervals, collisions_data, marker='o', color='blue', label='Avg Collisions')
#     plt.plot(task_intervals, avg_time_data, marker='o', color='red', label='Avg Task Completion Time')
#     plt.plot(task_intervals, tasks_completed_data, marker='o', color='green', label='Avg Tasks Completed')
#     # Plot individual datapoints
#     for i, ti in enumerate(task_intervals):
#         plt.scatter([ti] * len(collisions_all[i]), collisions_all[i], color='blue', alpha=0.6)
#         plt.scatter([ti] * len(times_all[i]), times_all[i], color='red', alpha=0.6)
#         plt.scatter([ti] * len(tasks_all[i]), tasks_all[i], color='green', alpha=0.6)
#     plt.xlabel("Task Interval")
#     plt.ylabel("KPI Value")
#     plt.title("KPIs vs Task Interval")
#     plt.legend()
#     plt.show()
