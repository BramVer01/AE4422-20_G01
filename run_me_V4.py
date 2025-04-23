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
visualization_speed = 0.5 #set at 0.1 as default

task_interval = 1    # New: generate a task every x seconds
total_tugs = 20       # New: total number of tugs (will be split evenly between depots)

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
    GATE_NODES = [100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0, 111.0, 112.0, 113.0, 114.0, 115.0, 116.0, 117.0, 118.0, 119.0, 120.0, 121.0, 122.0, 123.0, 124.0]
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

def run_simulation(visualization_speed, task_interval, 
                   total_tugs, simulation_time):
    start_cpu_time = time.time()  # CPU time tracking starts immediately

    # --- KPI Global Variables ---
    total_collisions = 0
    task_completion_times = []         # Execution time (from moving_to_task -> executing to to_depot)
    total_task_completion_times = []   # Total task time (from idle -> moving_to_task to idle)
    task_distances = []                # Number of node transitions during execution
    total_tasks_completed = 0          # Total tasks completed
    delays = []                        # List to record delay values for each completed task

    total_task_start_times = {}        # When a tug is assigned a task (idle -> moving_to_task)
    execution_start_times = {}         # When a tug starts executing (moving_to_task -> executing)
    task_nodes = {}                    # List of nodes traversed during execution
    prev_status = {}                   # Previous status for each tug
    waiting_aircraft = []              # Aircraft waiting for gates

    # New dictionary to store task details when a tug is assigned a task.
    task_details = {}

    # --- New: Idle Time Tracking ---
    idle_times = {}            # Current accumulated idle time per tug
    idle_time_history = {}     # History: for each tug, a list of accumulated idle time values per timestep
    battery_history = {}       # History: for each tug, a list of battery percentages per timestep
    time_history = []          # History of simulation time
    collision_history = {}  # key = rounded time, value = list of (position, set of tug_ids)

    # Initialize layout and graph
    nodes_dict, edges_dict, start_and_goal_locations = import_layout(NODES_FILE, EDGES_FILE)
    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    # Initialize list to track all tug agents (from ATC)
    atc = ATC()

    # Initialize depots
    departure_depot = Depot(1, position=DEPARTURE_DEPOT_POSITION)
    arrival_depot = Depot(2, position=ARRIVAL_DEPOT_POSITION)

    # Create tugs and add to depot queues; initialize idle time and battery tracking.
    for i in range(total_tugs):
        tug_id = i + 1
        if i < total_tugs // 2:
            tug = Tug(tug_id=tug_id, a_d="D", start_node=departure_depot.position, spawn_time=0, nodes_dict=nodes_dict)
            departure_depot.tugs.append(tug)
        else:
            tug = Tug(tug_id=tug_id, a_d="A", start_node=arrival_depot.position, spawn_time=0, nodes_dict=nodes_dict)
            arrival_depot.tugs.append(tug)
        atc.tug_list.append(tug)
        prev_status[tug.id] = tug.status
        idle_times[tug.id] = 0
        idle_time_history[tug.id] = []
        battery_history[tug.id] = []  # Initialize battery history for each tug

    # Initialize state time tracking for each tug (for "idle", "moving_to_task", "executing", "to_depot")
    tug_state_times = {tug.id: {"idle": 0, "moving_to_task": 0, "executing": 0, "to_depot": 0} for tug in atc.tug_list}

    # Initialize Auctioneer
    auctioneer = Auctioneer(atc.tug_list)

    # Initialize visualization if enabled
    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict, LFPG_LAYOUT)

    running = True
    escape_pressed = False
    time_end = simulation_time
    t = 0
    task_counter = 0  # Counts tasks generated
    gate_status = {}

    print("Simulation Started")
    while running:
        t = round(t, 2)

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
        
        collision_tugs_by_pos = {}


        for id1 in current_states:
            for id2 in current_states:
                if id1 < id2 and current_states[id1]["xy_pos"] == current_states[id2]["xy_pos"]:
                    pos = current_states[id1]["xy_pos"]
                    collision_tugs_by_pos.setdefault(pos, set()).update({id1, id2})

        for pos, tug_ids in collision_tugs_by_pos.items():
            if len(tug_ids) >= 2:
                print(f"Collision detected between tugs {sorted(tug_ids)} at position {pos} at time {t}")
                total_collisions += 1

        collision_history[round(t, 1)] = [
            (pos, tug_ids) for pos, tug_ids in collision_tugs_by_pos.items()
        ]

        if round(t - DT, 1) in collision_history:
            prev = collision_history[round(t - DT, 1)]
            curr = collision_history[round(t, 1)]
            for (pos1, tugs1) in prev:
                for (pos2, tugs2) in curr:
                    if pos1 == pos2 and len(tugs1) >= 3 and len(tugs2) >= 3:
                        print(f"\n❌ ERROR: Unsolvable 3-tug collision at {pos1} during time {t - DT} and {t}.")
                        raise RuntimeError("Unsolvable collision detected – simulation terminated.")





        # --- Tasks Assignment ---
        tasks_available = departure_depot.tasks + arrival_depot.tasks
        if tasks_available and (t/DELTA_T).is_integer():
            auctioneer.tug_availability(atc.tug_list)
            auctioneer.ask_price(tasks_available, nodes_dict, heuristics, t, [departure_depot, arrival_depot])
            auctioneer.decision(departure_depot, arrival_depot, START_NODES)

        # --- Tugs Charging ---
        departure_depot.charging(DT)
        arrival_depot.charging(DT)

        if t >= time_end or escape_pressed:
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
        
        t = t + DT

    avg_execution_time = sum(task_completion_times) / len(task_completion_times) if task_completion_times else 0
    avg_total_time = sum(total_task_completion_times) / len(total_task_completion_times) if total_task_completion_times else 0
    avg_distance = sum(task_distances) / len(task_distances) if task_distances else 0

    print("\n----- KPI SUMMARY -----")
    print("Total collisions detected:", total_collisions)
    print("Total tasks completed:", total_tasks_completed)
    print("Average execution time:", avg_execution_time)
    print("Average total task time:", avg_total_time)
    print("Average task distance (nodes):", avg_distance)
    print("-----------------------")
    
    print("\n----- Delay Summary -----")
    if delays:
        avg_delay = sum(delays) / len(delays)
        print("Average Delay:", avg_delay)
    else:
        print("No delay data recorded.")

    # --- Calculate Difference between Tasks Generated and Tasks Completed ---
    tasks_difference = task_counter - total_tasks_completed
    print("\n----- Task Generation Difference -----")
    print("Tasks Generated:", task_counter)
    print("Tasks Completed:", total_tasks_completed)
    print("Difference (Generated - Completed):", tasks_difference)

    # --- Print Tug State Time Summary ---
    print("\n----- Tug State Time Summary -----")
    for tug_id, states in tug_state_times.items():
        print(f"Tug {tug_id} state times: {states}")
    avg_state_times = {"idle": 0, "moving_to_task": 0, "executing": 0, "to_depot": 0}
    for times in tug_state_times.values():
        for state in avg_state_times:
            avg_state_times[state] += times[state]
    num_tugs = len(tug_state_times)
    for state in avg_state_times:
        avg_state_times[state] /= num_tugs
    print("\n----- Average State Times -----")
    print(avg_state_times)

    cpu_time = time.time() - start_cpu_time  # End CPU timing

    
    
    # Return KPI metrics along with idle time, battery history, time histories, and state times.
    return {
         "collisions": total_collisions,
         "tasks_completed": total_tasks_completed,
         "tasks_difference": tasks_difference,  # New KPI: difference between tasks generated and completed
         "avg_execution_time": avg_execution_time,
         "avg_total_time": avg_total_time,
         "avg_distance": avg_distance,
         "delays": delays,
        "cpu_time": cpu_time,
         "idle_time_history": idle_time_history,
         "battery_history": battery_history,
         "time_history": time_history,
         "tug_state_times": tug_state_times,
         "avg_state_times": avg_state_times
    }


'''Testing normality of KPIs'''
def plot_all_distributions(kpi_data_dict):
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.stats import norm
    num_plots = len(kpi_data_dict)
    cols = 4
    rows = (num_plots + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
    axes = axes.flatten() if num_plots > 1 else [axes]
    for i, (name, data) in enumerate(kpi_data_dict.items()):
        ax = axes[i]
        ax.hist(data, bins=10, density=True, alpha=0.6, edgecolor='black')
        mu, std = np.mean(data), np.std(data)
        xmin, xmax = ax.get_xlim()
        # x = np.linspace(xmin, xmax, 100)
        # p = norm.pdf(x, mu, std)
        # ax.plot(x, p, 'k', linewidth=2)
        # ax.set_title(f"{name}\nMean: {mu:.2f}, Std: {std:.2f}")
        ax.set_xlabel(name)
        ax.set_ylabel("Probability Density")
        ax.grid(True)
    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # turn off pygame visualization during batch runs
    visualization = False

    num_runs = 20               # number of independent replications
    tug_count = 8                # fleet size under test
    task_interval = 2            # seconds between task generations
    SIMULATION_TIME = 300

    # prepare empty lists for each KPI
    collisions_list            = []
    tasks_completed_list       = []
    avg_execution_time_list    = []
    total_task_time_list       = []
    avg_distance_list          = []
    avg_delay_list             = []
    idle_time_list             = []
    move_to_task_time_list     = []
    cpu_time_list              = []
    error_count = 0

    for run in range(1, num_runs + 1):
        print(f"\n--- Simulation run {run}/{num_runs} ---")
        try:
            kpi = run_simulation(
                visualization_speed=visualization_speed,
                task_interval=task_interval,
                total_tugs=tug_count,
                simulation_time=SIMULATION_TIME
            )

            # collect scalar KPIs
            collisions_list.append(kpi["collisions"])
            tasks_completed_list.append(kpi["tasks_completed"])
            total_task_time_list.append(kpi["avg_total_time"])
            avg_distance_list.append(kpi["avg_distance"])

            # average delay per run
            if kpi["delays"]:
                avg_delay_list.append(sum(kpi["delays"]) / len(kpi["delays"]))
            else:
                avg_delay_list.append(0.0)

            # extract average per‑tug state times
            idle_time_list.append(kpi["avg_state_times"]["idle"])
            move_to_task_time_list.append(kpi["avg_state_times"]["moving_to_task"])
            avg_execution_time_list.append(kpi["avg_state_times"]["executing"])

            # CPU time if returned
            if "cpu_runtime" in kpi:
                cpu_time_list.append(kpi["cpu_runtime"])

        except Exception as e:
            error_count += 1
            print(f"  ERROR in run {run}: {e}")

    # overall error rate
    error_rate = error_count / num_runs
    print(f"\n=== ERROR RATE ===")
    print(f"  {error_rate*100:.2f}% ({error_count}/{num_runs}) runs failed")
    print("-----\n")

    # print mean and std for each KPI
    from statistics import mean, pstdev
    def print_stats(name, data):
        m = mean(data) if data else 0.0
        s = pstdev(data) if len(data) > 1 else 0.0
        print(f"{name:25s}: mean = {m:.3f}, std = {s:.3f}")

    print("=== KPI SUMMARY STATISTICS ===")
    print_stats("Collisions",            collisions_list)
    print_stats("Tasks Completed",       tasks_completed_list)
    print_stats("Avg Execution Time",    avg_execution_time_list)
    print_stats("Total Task Time",       total_task_time_list)
    print_stats("Avg Task Distance",     avg_distance_list)
    print_stats("Avg Delay",             avg_delay_list)
    print_stats("Idle Time (per tug)",   idle_time_list)
    print_stats("Move-to-Task Time",     move_to_task_time_list)
    if cpu_time_list:
        print_stats("CPU Runtime",       cpu_time_list)
    print()

    # perform D’Agostino–Pearson test on each list
    print("=== NORMALITY TESTS ===")
    test_normality(collisions_list,          "Collisions")
    test_normality(tasks_completed_list,    "Tasks Completed")
    test_normality(avg_execution_time_list, "Avg Execution Time")
    test_normality(total_task_time_list,    "Total Task Time")
    test_normality(avg_distance_list,       "Avg Task Distance")
    test_normality(avg_delay_list,          "Avg Delay")
    test_normality(idle_time_list,          "Idle Time (per tug)")
    test_normality(move_to_task_time_list,  "Move-to-Task Time (per tug)")
    if cpu_time_list:
        test_normality(cpu_time_list,       "CPU Runtime")

    # plot histograms
    kpi_dict = {
        "Collisions": collisions_list,
        "Tasks Completed": tasks_completed_list,
        "Avg Execution Time": avg_execution_time_list,
        "Total Task Time": total_task_time_list,
        "Avg Task Distance": avg_distance_list,
        "Avg Delay": avg_delay_list,
        "Idle Time": idle_time_list,
        "Move-to-Task Time": move_to_task_time_list
    }
    if cpu_time_list:
        kpi_dict["CPU Runtime"] = cpu_time_list

    plot_all_distributions(kpi_dict)



'''ALL SENSITIVITY PLOTS'''
# if __name__ == "__main__":
#     import numpy as np
#     import matplotlib.pyplot as plt
#     import pandas as pd
#     import time
#     from scipy import stats

#     # ---- Batch parameters ----
#     tug_counts         = [2, 4, 6, 8, 10, 12, 14]
#     task_intervals     = [1, 2, 3, 5, 7]
#     n_runs_per_combo   = 1
#     baseline_interval  = task_intervals[len(task_intervals)//2]

#     SIM_TIME = 300
#     visualization = False
#     visualization_speed = 0.0

#     def mean_ci(data, confidence=0.95):
#         a = np.array(data, dtype=float)
#         a = a[~np.isnan(a)]
#         if len(a) < 2:
#             return np.nan, np.nan
#         m  = np.mean(a)
#         se = stats.sem(a)
#         h  = se * stats.t.ppf((1+confidence)/2., len(a)-1)
#         return m, h

#     # ---- prepare containers ----
#     delay_data = {n:{"ti":[], "mean":[], "ci":[]} for n in tug_counts}
#     compl_data = {n:{"ti":[], "mean":[], "ci":[]} for n in tug_counts}
#     cpu_data   = {n:{"ti":[], "mean":[], "ci":[]} for n in tug_counts}
#     util_data  = {
#         "tugs": tug_counts,
#         "U_mean":[], "U_ci":[],
#         "I_mean":[], "I_ci":[],
#         "M_mean":[], "M_ci":[]
#     }
#     summary_records = []

#     print("Starting batch simulations...")
#     for n in tug_counts:
#         for ti in task_intervals:
#             delays, rates, cpus = [], [], []
#             print(f" Fleet={n}, interval={ti}s...", end="")
#             for _ in range(n_runs_per_combo):
#                 k = run_simulation(
#                     visualization_speed=visualization_speed,
#                     task_interval=ti,
#                     total_tugs=n,
#                     simulation_time=SIM_TIME
#                 )
#                 delays.append(np.mean(k["delays"]) if k["delays"] else np.nan)
#                 generated = k["tasks_completed"] + k.get("tasks_difference", 0)
#                 rate = (k["tasks_completed"]/generated) if generated>0 else np.nan
#                 rates.append(np.clip(rate,0,1))
#                 if "cpu_runtime" in k:
#                     cpus.append(k["cpu_runtime"])
#             md, hd = mean_ci(delays)
#             mr, hr = mean_ci(rates)
#             mc, hc = mean_ci(cpus) if cpus else (np.nan, np.nan)

#             delay_data[n]["ti"].append(ti)
#             delay_data[n]["mean"].append(md)
#             delay_data[n]["ci"].append(hd)

#             compl_data[n]["ti"].append(ti)
#             compl_data[n]["mean"].append(mr)
#             compl_data[n]["ci"].append(hr)

#             cpu_data[n]["ti"].append(ti)
#             cpu_data[n]["mean"].append(mc)
#             cpu_data[n]["ci"].append(hc)

#             summary_records.append({
#                 "Tugs":n, "Interval":ti,
#                 "Mean Delay":f"{md:.2f}±{hd:.2f}",
#                 "Completion":f"{mr:.2f}±{hr:.2f}",
#                 "CPU Time":f"{mc:.2f}±{hc:.2f}"
#             })
#             print(" done")

#         # Utilization at baseline_interval
#         U_vals, I_vals, M_vals = [], [], []
#         for _ in range(n_runs_per_combo):
#             k = run_simulation(
#                 visualization_speed=visualization_speed,
#                 task_interval=baseline_interval,
#                 total_tugs=n,
#                 simulation_time=SIM_TIME
#             )
#             totals = sum(v for v in k["tug_state_times"].values())
#             idle = sum(v["idle"] for v in k["tug_state_times"].values())
#             move = sum(v["moving_to_task"] for v in k["tug_state_times"].values())
#             exec_ = sum(v["executing"] for v in k["tug_state_times"].values())
#             tot = idle + move + exec_
#             if tot>0:
#                 U_vals.append(exec_/tot)
#                 I_vals.append(idle/tot)
#                 M_vals.append(move/tot)
#         u_m, u_h = mean_ci(U_vals)
#         i_m, i_h = mean_ci(I_vals)
#         m_m, m_h = mean_ci(M_vals)
#         util_data["U_mean"].append(u_m); util_data["U_ci"].append(u_h)
#         util_data["I_mean"].append(i_m); util_data["I_ci"].append(i_h)
#         util_data["M_mean"].append(m_m); util_data["M_ci"].append(m_h)

#     # Print summary
#     summary_df = pd.DataFrame(summary_records)
#     print("\nBatch summary:")
#     print(summary_df.to_string(index=False))

#     # Plot #1: Delay vs Interval
#     plt.figure(figsize=(10,6))
#     for n in tug_counts:
#         plt.errorbar(delay_data[n]["ti"], delay_data[n]["mean"],
#                      yerr=delay_data[n]["ci"], fmt='o-', capsize=4, label=f"{n} tugs")
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Mean Delay (s)")
#     plt.title("Delay vs Task Interval")
#     plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()

#     # Plot #2: Completion Rate vs Interval
#     plt.figure(figsize=(10,6))
#     for n in tug_counts:
#         plt.errorbar(compl_data[n]["ti"], compl_data[n]["mean"],
#                      yerr=compl_data[n]["ci"], fmt='s--', capsize=4, label=f"{n} tugs")
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Mean Completion Rate")
#     plt.title("Completion Rate vs Task Interval")
#     plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()

#     # Plot #3: CPU Time vs Interval (skip NaNs)
#     plt.figure(figsize=(10,6))
#     for n in tug_counts:
#         ti_arr, m_arr, ci_arr = [], [], []
#         for ti, m, ci in zip(cpu_data[n]["ti"], cpu_data[n]["mean"], cpu_data[n]["ci"]):
#             if not np.isnan(m):
#                 ti_arr.append(ti); m_arr.append(m); ci_arr.append(ci)
#         if ti_arr:
#             plt.errorbar(ti_arr, m_arr, yerr=ci_arr, fmt='d-.', capsize=4, label=f"{n} tugs")
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Mean CPU Time (s)")
#     plt.title("CPU Time vs Task Interval")
#     plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()

#     # Plot #4: Utilization vs Fleet Size
#     plt.figure(figsize=(10,6))
#     plt.errorbar(util_data["tugs"], util_data["U_mean"], yerr=util_data["U_ci"],
#                  fmt='o-', capsize=4, label="Execution (U)")
#     plt.errorbar(util_data["tugs"], util_data["I_mean"], yerr=util_data["I_ci"],
#                  fmt='s--', capsize=4, label="Idle (I)")
#     plt.errorbar(util_data["tugs"], util_data["M_mean"], yerr=util_data["M_ci"],
#                  fmt='d-.', capsize=4, label="Move-to-Task (M)")
#     plt.xlabel("Number of Taxi-Bots")
#     plt.ylabel("Mean Time Ratio")
#     plt.title(f"Utilization Ratios @ Interval={baseline_interval}s")
#     plt.legend(); plt.grid(True); plt.tight_layout(); plt.show()





def plot_all_distributions(kpi_data_dict):
    num_plots = len(kpi_data_dict)
    cols = 4
    rows = (num_plots + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
    axes = axes.flatten() if num_plots > 1 else [axes]

    for i, (name, data) in enumerate(kpi_data_dict.items()):
        ax = axes[i]
        ax.hist(data, bins=10, density=True, alpha=0.6, edgecolor='black')
        ax.set_xlabel(name)
        ax.set_ylabel("Probability Density")
        ax.grid(True)

    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])

    plt.tight_layout()
    plt.show()


'''BATTERY DISCHARGE GRAPH'''
# def main():
#     import numpy as np
#     import matplotlib.pyplot as plt
#     from scipy import stats

#     # Simulation settings
#     SIMULATION_TIME = 300
#     visualization = False
#     visualization_speed = 0.1
#     task_interval = 2        # seconds between task generations
#     total_tugs = 8           # fleet size
#     n_runs = 10               # number of simulation runs

#     # Containers for battery histories
#     battery_time_series = {}   # tug_id -> list of battery arrays
#     common_time_history = None

#     # Run simulations and collect battery data
#     for run in range(n_runs):
#         try: 
#             results = run_simulation(
#                 visualization_speed=visualization,
#                 task_interval=task_interval,
#                 total_tugs=total_tugs,
#                 simulation_time=SIMULATION_TIME
#             )
#             if common_time_history is None:
#                 common_time_history = np.array(results["time_history"])
#             for tug_id, batt in results["battery_history"].items():
#                 battery_time_series.setdefault(tug_id, []).append(np.array(batt))
#         except Exception as e:
#             print(f"[ERROR] Simulation run {run+1} failed: {e}")
#             continue  # Optionally continue with next run

#     # Compute mean and 95% CI for battery percentage
#     battery_time_avg = {}
#     battery_time_ci = {}
#     for tug_id, runs in battery_time_series.items():
#         data = np.vstack(runs)  # shape: (n_runs, time_steps)
#         mean_series = np.mean(data, axis=0)
#         sem = stats.sem(data, axis=0, nan_policy='omit')
#         ci = sem * stats.t.ppf((1 + 0.95) / 2., n_runs - 1)
#         battery_time_avg[tug_id] = mean_series
#         battery_time_ci[tug_id] = ci

#     # Plot battery percentage over time
#     fig, ax = plt.subplots(figsize=(12, 6))
#     for tug_id, mean_series in battery_time_avg.items():
#         ci_series = battery_time_ci[tug_id]
#         ax.plot(common_time_history, mean_series, label=f"Tug {tug_id}")
#         ax.fill_between(common_time_history,
#                         mean_series - ci_series,
#                         mean_series + ci_series,
#                         alpha=0.2)
#     ax.set_xlabel("Simulation Time (s)")
#     ax.set_ylabel("Battery Percentage (%)")
#     ax.set_title("Battery Percentage Over Time (Mean ± 95% CI)")
#     ax.legend(fontsize='small')
#     ax.grid(True)
#     plt.tight_layout()
#     plt.show()

