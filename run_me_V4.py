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
from single_agent_planner import calc_heuristics, simple_single_agent_astar
from visualization_V3 import map_initialization, map_running
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import run_CBS
from Depot_file import Depot, FlightTask
from Aircraft_V4 import Tug
from Auctioneer_file import Auctioneer
from ATC import ATC

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import normaltest, norm

def test_normality(data, name, alpha=0.05):
    stat, p = normaltest(data)
    print(f"{name}: stat={stat:.4f}, p={p:.4f}")
    if p < alpha:
        print(f"Result: {name} data is not normally distributed (reject H0).")
    else:
        print(f"Result: {name} data appears normally distributed (fail to reject H0).")
    print("-----")

def plot_distribution(data, name):
    plt.figure()
    # Plot histogram with density normalization
    plt.hist(data, bins=10, density=True, alpha=0.6, edgecolor='black')
    mu, std = np.mean(data), np.std(data)
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    plt.title(f"{name} Distribution\nMean: {mu:.2f}, Std: {std:.2f}")
    plt.xlabel(name)
    plt.ylabel("Probability Density")
    plt.grid(True)
    # plt.savefig(f'figure_normal_dis_{name}')
    plt.show()

def get_flight_task_by_id(flight_id, departure_depot, arrival_depot):
    """
    Searches for and returns the FlightTask object with the given flight_id from the depot task lists.
    Returns None if not found.
    """
    for task in departure_depot.tasks + arrival_depot.tasks:
        if task.flight_id == flight_id:
            return task
    return None



#%% SIMULATION PARAMETERS
# Layout parameters
LFPG_LAYOUT = True  # Whether to use LFPG or baseline network
WIND_WEST = True    # Whether to use west or east wind for LFPG layout
NODES_FILE = "nodes_LFPG.xlsx" if LFPG_LAYOUT else "nodes.xlsx"
EDGES_FILE = "edges_LFPG.xlsx" if LFPG_LAYOUT else "edges.xlsx"

# Simulation settings
SIMULATION_TIME = 300
PLANNER = "Prioritized"  # Choose which planner to use (Independent, Prioritized, CBS)
DELTA_T = 0.5  # Time step for planning
DT = 0.1  # Time step for movement

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = False        #pygame visualization
visualization_speed = 0.1 #set at 0.1 as default

task_interval = 2    # New: generate a task every x seconds
total_tugs = 8       # New: total number of tugs (will be split evenly between depots)

# Node IDs 
if LFPG_LAYOUT:
    if WIND_WEST:
        ARRIVAL_RUNWAY_NODES = [69.0, 70.0]
        DEPARTURE_RUNWAY_NODES = [58.0, 59.0]
        DEPARTURE_DEPOT_POSITION = 200.0
        ARRIVAL_DEPOT_POSITION = 201.0
    else:
        ARRIVAL_RUNWAY_NODES = [71.0, 72.0]
        DEPARTURE_RUNWAY_NODES = [53.0, 54.0]
        DEPARTURE_DEPOT_POSITION = 200.0
        ARRIVAL_DEPOT_POSITION = 201.0
    GATE_NODES = [100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124]
else:
    DEPARTURE_DEPOT_POSITION = 112.0
    ARRIVAL_DEPOT_POSITION = 113.0
    ARRIVAL_RUNWAY_NODES = [37.0, 38.0]
    GATE_NODES = [97.0, 34.0, 35.0, 36.0, 98.0]
    DEPARTURE_RUNWAY_NODES = [1.0, 2.0]

START_NODES = [ARRIVAL_DEPOT_POSITION] + [DEPARTURE_DEPOT_POSITION] + ARRIVAL_RUNWAY_NODES + DEPARTURE_RUNWAY_NODES + GATE_NODES
GATE_HOLDING_TIME = 10  # Time an aircraft stays at a gate before being ready for departure

# Bidding parameters Ye
GAMMA = 1
ALPHA = 1
BETA = 1

#%% Function definitions
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
    gates_xy = []   # List with (x,y) positions of gates
    rwy_dep_xy = [] # List with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] # List with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.path.join(os.getcwd(), nodes_file))
    df_edges = pd.read_excel(os.path.join(os.getcwd(), edges_file))
    
    # Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {
            "id": row["id"],
            "x_pos": row["x_pos"],
            "y_pos": row["y_pos"],
            "xy_pos": (row["x_pos"], row["y_pos"]),
            "type": row["type"],
            "neighbors": set()
        }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        # Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"], row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"], row["y_pos"]))

    # Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {
        "gates": gates_xy, 
        "dep_rwy": rwy_dep_xy,
        "arr_rwy": rwy_arr_xy
    }
    
    # Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        from_node = edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {
            "id": edge_id,
            "from": row["from"],
            "to": row["to"],
            "length": row["length"],
            "weight": row["length"],
            "start_end_pos": start_end_pos
        }
        edges_dict[edge_id] = edge_properties
   
    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations


def create_graph(nodes_dict, edges_dict, plot_graph=True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    graph = nx.DiGraph() # Create directed graph in NetworkX
    
    # Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(
            node, 
            node_id=nodes_dict[node]["id"],
            xy_pos=nodes_dict[node]["xy_pos"],
            node_type=nodes_dict[node]["type"]
        )
        
    for edge in edges_dict.keys():
        graph.add_edge(
            edge[0], edge[1], 
            edge_id=edge,
            from_node=edges_dict[edge]["from"],
            to_node=edges_dict[edge]["to"],
            weight=edges_dict[edge]["length"]
        )
    
    # Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph


def generate_flight_task(flight_id, t, gate_status):
    """
    Generates a new flight task (arrival or departure)
    INPUT:
        - flight_id: ID number for the flight
        - t: Current simulation time
        - gate_status: Dictionary tracking gate occupancy
    RETURNS:
        - FlightTask object or None if no task could be created
    """
    a_d = random.choice(["A", "D"])
    if a_d == "A":
        start_node = random.choice(ARRIVAL_RUNWAY_NODES)
        available_gates = [gate for gate in GATE_NODES if gate not in gate_status]
        if available_gates:
            goal_node = random.choice(available_gates)
            release_time = t + GATE_HOLDING_TIME + random.choice([0, 0, 2, 4])
            gate_status[goal_node] = {"release_time": release_time, "flight_id": flight_id}
            print(f"Time {t}: Aircraft {flight_id} arriving at gate {goal_node}, scheduled to depart at {release_time}")
        else:
            goal_node = "waiting"
            print(f"Time {t}: Aircraft {flight_id} is waiting for a free gate.")
    else:
        ready_flights = [gate for gate, info in gate_status.items() if info["release_time"] <= t]
        if not ready_flights:
            return None
        start_node = random.choice(ready_flights)
        goal_node = random.choice(DEPARTURE_RUNWAY_NODES)
        departing_flight_id = gate_status[start_node]["flight_id"]
        del gate_status[start_node]
        print(f"Time {t}: Aircraft {departing_flight_id} departing from gate {start_node} to runway {goal_node}")
        return FlightTask(departing_flight_id, "D", start_node, goal_node, t)
    return FlightTask(flight_id, a_d, start_node, goal_node, t)

#%% MAIN SIMULATION FUNCTION (with heat map tracking)
def run_simulation(visualization_speed, task_interval, total_tugs, simulation_time):
    # --- KPI Global Variables ---
    total_collisions = 0
    task_completion_times = []         
    total_task_completion_times = []   
    task_distances = []                
    total_tasks_completed = 0          
    delays = []                        
    total_task_start_times = {}        
    execution_start_times = {}         
    task_nodes = {}                    
    prev_status = {}                   
    waiting_aircraft = []              
    task_details = {}
    idle_times = {}            
    idle_time_history = {}     
    battery_history = {}       
    time_history = []          

    # Initialize node activity tracking
    nodes_dict, edges_dict, _ = import_layout(NODES_FILE, EDGES_FILE)
    node_activity = {node_id: 0 for node_id in nodes_dict}

    # Initialize layout and graph
    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    # Initialize ATC and depots
    atc = ATC()
    departure_depot = Depot(1, position=DEPARTURE_DEPOT_POSITION)
    arrival_depot = Depot(2, position=ARRIVAL_DEPOT_POSITION)

    # Create tugs and initialize tracking
    for i in range(total_tugs):
        tug_id = i + 1
        depot = departure_depot if i < total_tugs//2 else arrival_depot
        tug = Tug(tug_id, "D" if i < total_tugs//2 else "A", 
                 depot.position, 0, nodes_dict)
        depot.tugs.append(tug)
        atc.tug_list.append(tug)
        prev_status[tug.id] = tug.status
        idle_times[tug.id] = 0
        idle_time_history[tug.id] = []
        battery_history[tug.id] = []

    # Initialize state time tracking
    tug_state_times = {tug.id: {"idle":0, "moving_to_task":0, 
                              "executing":0, "to_depot":0} 
                     for tug in atc.tug_list}

    # Initialize visualization
    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict, LFPG_LAYOUT)

    # Simulation loop
    running = True
    escape_pressed = False
    t = 0
    task_counter = 0
    gate_status = {}

    while running:
        t = round(t, 2)

        # Update node activity for heat map
        for tug in atc.tug_list:
            tug.get_node_by_xy()
            if hasattr(tug, 'current_node') and tug.current_node in node_activity:
                node_activity[tug.current_node] += 1

        # --- Task Creation ---
        for gate, info in list(gate_status.items()):
            if info["release_time"] <= t:
                if "arrival_time" in info:
                    delay_val = t - info["arrival_time"] - 10
                else:
                    delay_val = "UNKNOWN"
                print(f"Time {t}: Aircraft {info['flight_id']} at gate {gate} ready for departure. Delay: {delay_val} s")
                task = FlightTask(info["flight_id"], "D", gate, random.choice(DEPARTURE_RUNWAY_NODES), t)
                if task:
                    departure_depot.add_task(task)
                    print(f"Time {t}: Departure task for Aircraft {info['flight_id']} created (from {gate} to runway).")
                del gate_status[gate]
                if waiting_aircraft:
                    next_aircraft = waiting_aircraft.pop(0)
                    gate_status[gate] = {"release_time": t + GATE_HOLDING_TIME, "flight_id": next_aircraft.flight_id}
                    next_aircraft.goal_node = gate
                    print(f"Time {t}: Waiting aircraft {next_aircraft.flight_id} assigned to gate {gate}.")

        if abs(t - round(t)) < 1e-9 and (round(t) % task_interval == 0):
            task_counter += 1
            task_id = task_counter
            task = generate_flight_task(task_id, t, gate_status)
            if task:
                if task.goal_node == "waiting":
                    waiting_aircraft.append(task)
                    print(f"Time {t}: Aircraft {task.flight_id} waiting for a free gate.")
                elif task.type == "A":
                    arrival_depot.add_task(task)
                    print(f"Time {t}: New arrival task {task.flight_id} added (from {task.start_node} to {task.goal_node}).")
                else:
                    departure_depot.add_task(task)
                    print(f"Time {t}: New departure task {task.flight_id} added (from {task.start_node} to {task.goal_node}).")
        
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
            for tug in atc.tug_list:
                print(f'Tug {tug.id} battery: {tug.bat_perc}%')
                for task in departure_depot.tasks:
                    _ = tug.bidders_value(task, nodes_dict, heuristics, t, 
                                            [departure_depot, arrival_depot], 
                                            gamma=GAMMA, alpha=ALPHA, beta=BETA)
                for task in arrival_depot.tasks:
                    _ = tug.bidders_value(task, nodes_dict, heuristics, t, 
                                            [departure_depot, arrival_depot], 
                                            gamma=GAMMA, alpha=ALPHA, beta=BETA)
                print(f"Tug {tug.id}: status = {tug.status}, coupled = {tug.coupled}, position = {tug.position}")
        
        # --- Collision Detection KPI ---
        current_states = {}
        for tug in atc.tug_list:
            tug.get_node_by_xy()
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                has_flight = hasattr(tug, 'current_task') and tug.current_task is not None
                current_states[tug.id] = {
                    "tug_id": tug.id,
                    "xy_pos": tug.position,
                    "heading": tug.heading,
                    "has_flight": has_flight,
                    "status": tug.status,
                    "bat_perc": tug.bat_perc
                }
        if visualization:
            escape_pressed = map_running(map_properties, current_states, t, departure_depot, arrival_depot, nodes_dict)
            timer.sleep(visualization_speed)
        
        for id1 in current_states:
            for id2 in current_states:
                if id1 < id2 and current_states[id1]["xy_pos"] == current_states[id2]["xy_pos"]:
                    total_collisions += 1
                    print(f"Collision detected between Tug {id1} and Tug {id2} at time {t}")

        # --- Tasks Assignment ---
        tasks_available = departure_depot.tasks + arrival_depot.tasks
        if tasks_available and (t/DELTA_T).is_integer():
            Auctioneer.tug_availability(atc.tug_list)
            Auctioneer.ask_price(tasks_available, nodes_dict, heuristics, t, [departure_depot, arrival_depot])
            Auctioneer.decision(departure_depot, arrival_depot, START_NODES)

        # --- Tugs Charging ---
        departure_depot.charging(DT)
        arrival_depot.charging(DT)

        if t >= SIMULATION_TIME or escape_pressed:
            running = False
            pg.quit()
            print("Simulation Stopped")
            break
        
        # --- Run Planning ---
        if (t/DELTA_T).is_integer():
            if PLANNER == "Independent":
                for tug in atc.tug_list:
                    if tug.status in ["moving_to_task", "executing", "to_depot"]:
                        run_independent_planner(tug, nodes_dict, edges_dict, heuristics, t)
            elif PLANNER == "Prioritized":
                for tug in atc.tug_list:
                    if tug.status in ["moving_to_task", "executing", "to_depot"]:
                        atc.constraints = run_prioritized_planner(atc.tug_list, tug, nodes_dict, edges_dict, heuristics, t, DELTA_T, atc.constraints)
            elif PLANNER == "CBS":
                run_CBS()
            else:
                raise Exception(f"Planner: {PLANNER} is not defined.")

        atc.constraints = [con for con in atc.constraints if con["timestep"] > t]
        atc.constraints_at_t = [con for con in atc.constraints if con["timestep"] == round(t+DELTA_T, 1)]
        for tug in atc.tug_list:
            if tug.status in ["moving_to_task", "executing", "to_depot"]:
                tug.move(DT, t, atc.constraints_at_t, LFPG_LAYOUT, departure_depot, arrival_depot)
        
        # --- KPI: Task Completion & Delay Calculation ---
        for tug in atc.tug_list:
            current_node = getattr(tug, 'current_node', tug.start)

            # When a tug is assigned a task
            if prev_status[tug.id] == "idle" and tug.status == "moving_to_task":
                total_task_start_times[tug.id] = t
                task_details[tug.id] = tug.current_task  # stores the flight id of the task
            
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
                    
                    print(f'task details: {task_details}')
                    flight_id = task_details[tug.id] 
                    print(f'Task ID: {flight_id}')
                    task = FlightTask.get_task_by_id(flight_id)
                    print(f'task object: {task}')

                    total_duration_passengers = t - task.spawn_time
                    success, path = simple_single_agent_astar(nodes_dict, task.start_node, task.goal_node, heuristics, task.spawn_time)
                    if success and path:
                        ideal_time = path[-1][1] - path[0][1]
                    else:
                        ideal_time = None
                    delay_value = total_duration_passengers - ideal_time
                    delays.append(delay_value)
                    print(f"Tug {tug.id} delay: actual {total_duration_passengers:.2f} s, ideal {ideal_time:.2f} s, delay {delay_value:.2f} s")

                    del execution_start_times[tug.id]
                    del task_nodes[tug.id]
            
            # When total task completes (to_depot -> idle), record total task time:
            if prev_status[tug.id] == "to_depot" and tug.status == "idle": 
                if tug.id in total_task_start_times:
                    total_duration = t - total_task_start_times[tug.id]
                    total_task_completion_times.append(total_duration)
                    del total_task_start_times[tug.id]

            prev_status[tug.id] = tug.status

        # --- Update Depot Queues for Idle Tugs ---
        for tug in atc.tug_list:
            if tug.status == "idle":
                if tug.type == "D" and tug.coupled == departure_depot.position:
                    if tug not in departure_depot.tugs:
                        departure_depot.tugs.append(tug)
                        tug.set_init_tug_params(tug.id, "D", departure_depot.position, nodes_dict)
                        print(f"Tug {tug.id} returned to the departure depot.")
                elif tug.type == "A" and tug.coupled == arrival_depot.position:
                    if tug not in arrival_depot.tugs:
                        arrival_depot.tugs.append(tug)
                        tug.set_init_tug_params(tug.id, "A", arrival_depot.position, nodes_dict)
                        print(f"Tug {tug.id} returned to the arrival depot.")
        
        # --- Accumulate Idle Time ---
        for tug in atc.tug_list:
            if tug.status == "idle":
                idle_times[tug.id] += DT
        time_history.append(t)
        for tug in atc.tug_list:
            idle_time_history[tug.id].append(idle_times[tug.id])
        
        # --- Accumulate Battery History ---
        for tug in atc.tug_list:
            battery_history[tug.id].append(tug.bat_perc)
        
        # --- Accumulate State Time for Each Tug ---
        for tug in atc.tug_list:
            if tug.status in tug_state_times[tug.id]:
                tug_state_times[tug.id][tug.status] += DT

        # End condition
        if t >= simulation_time or escape_pressed:
            running = False
            if visualization: pg.quit()

        t += DT

    # Calculate averages
    avg_exec_time = np.mean(task_completion_times) if task_completion_times else 0
    avg_total_time = np.mean(total_task_completion_times) if total_task_completion_times else 0
    avg_distance = np.mean(task_distances) if task_distances else 0

    return {
        "collisions": total_collisions,
        "tasks_completed": total_tasks_completed,
        "node_activity": node_activity,
        "nodes_dict": nodes_dict,
        "avg_exec_time": avg_exec_time,
        "avg_total_time": avg_total_time,
        "avg_distance": avg_distance,
        "delays": delays,
        "tug_state_times": tug_state_times
    }

#%% HEAT MAP VISUALIZATION FUNCTION
def plot_heatmap(simulation_results):
    """Visualizes node activity as a heat map."""
    node_activity = simulation_results["node_activity"]
    nodes_dict = simulation_results["nodes_dict"]
    
    plt.figure(figsize=(14, 10))
    
    # Extract coordinates and activity values
    x = [nodes_dict[n]["x_pos"] for n in nodes_dict]
    y = [nodes_dict[n]["y_pos"] for n in nodes_dict]
    activity = [node_activity[n] for n in nodes_dict]
    
    # Create scatter plot with color gradient
    scatter = plt.scatter(x, y, c=activity, cmap='hot_r', 
                        s=150, alpha=0.8, edgecolor='k', 
                        linewidths=0.5, zorder=2)
    
    # Add color bar
    cbar = plt.colorbar(scatter, shrink=0.8)
    cbar.set_label('Node Activity Count', fontsize=12)
    
    # Add node IDs
    for node_id in nodes_dict:
        plt.text(nodes_dict[node_id]["x_pos"], 
                 nodes_dict[node_id]["y_pos"]+2,
                 str(int(node_id)), 
                 ha='center', va='bottom', 
                 fontsize=8, color='blue', alpha=0.7)
    
    # Formatting
    plt.title("Aircraft Tug Activity Heat Map", fontsize=14)
    plt.xlabel("X Position (meters)", fontsize=12)
    plt.ylabel("Y Position (meters)", fontsize=12)
    plt.grid(True, alpha=0.3, zorder=1)
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Add legend
    handles = [plt.Line2D([0], [0], marker='o', color='w', 
              markerfacecolor='orange', markersize=10,
              label='High Activity'),
              plt.Line2D([0], [0], marker='o', color='w',
              markerfacecolor='black', markersize=10,
              label='Low Activity')]
    plt.legend(handles=handles, loc='upper right')
    
    plt.tight_layout()
    plt.show()

#%% MAIN FUNCTION
def main():
    print("Starting airport tug simulation...")
    
    # Run simulation
    results = run_simulation(
        visualization_speed=0.1,
        task_interval=2,
        total_tugs=8,
        simulation_time=300  # 5 minutes simulation
    )
    
    # Show heat map
    plot_heatmap(results)
    
    # Print summary statistics
    print("\nSimulation Summary:")
    print(f"Total tasks completed: {results['tasks_completed']}")
    print(f"Average execution time: {results['avg_exec_time']:.2f}s")
    print(f"Average total task time: {results['avg_total_time']:.2f}s")
    print(f"Total collisions detected: {results['collisions']}")
    
    # Show node activity statistics
    activity = results["node_activity"].values()
    print("\nNode Activity Statistics:")
    print(f"Most active node: {max(results['node_activity'], key=results['node_activity'].get)}")
    print(f"Total node visits: {sum(activity)}")
    print(f"Average node visits: {np.mean(list(activity)):.1f}")
    print(f"Std.dev of node visits: {np.std(list(activity)):.1f}")

if __name__ == "__main__":
    main()