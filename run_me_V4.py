"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""
import numpy as np
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



def run_simulation(visualization_speed, task_interval, 
                   total_tugs, simulation_time):
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
    
    
    # Return KPI metrics along with idle time, battery history, time histories, and state times.
    return {
         "collisions": total_collisions,
         "tasks_completed": total_tasks_completed,
         "tasks_difference": tasks_difference,  # New KPI: difference between tasks generated and completed
         "avg_execution_time": avg_execution_time,
         "avg_total_time": avg_total_time,
         "avg_distance": avg_distance,
         "delays": delays,
         "idle_time_history": idle_time_history,
         "battery_history": battery_history,
         "time_history": time_history,
         "tug_state_times": tug_state_times,
         "avg_state_times": avg_state_times
    }





'''Impact of Fleet Size and Task Interval on Task Completion Rate'''
# Updated main function: Impact of Fleet Size and Task Interval on Task Completion Rate

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import stats

# if __name__ == "__main__":
#     # Simulation settings
#     SIMULATION_TIME   = 100
#     DELTA_T           = 0.5
#     DT                = 0.1

#     # Batch parameters
#     tug_counts        = [4, 6, 8, 10]
#     task_intervals    = [1, 3, 5, 7, 9]
#     n_runs_per_combo  = 10  # increase replications for stability

#     def mean_ci(data, confidence=0.95):
#         a    = np.array(data, dtype=float)
#         mean = np.nanmean(a)
#         se   = stats.sem(a, nan_policy='omit')
#         h    = se * stats.t.ppf((1 + confidence) / 2., len(a) - 1)
#         return mean, h

#     plot_data = {n: {"ti": [], "mean_rate": [], "ci_rate": []} for n in tug_counts}
#     summary   = []

#     print("Running completion-rate batch over fleet sizes and task intervals...")
#     for n in tug_counts:
#         for ti in task_intervals:
#             rates = []
#             print(f"  Testing {n} tugs, task interval={ti}s...")
#             for _ in range(n_runs_per_combo):
#                 res = run_simulation(
#                     visualization_speed=0.0,
#                     task_interval=ti,
#                     total_tugs=n,
#                     simulation_time=SIMULATION_TIME
#                 )
#                 completed = res["tasks_completed"]
#                 # Derive total tasks generated using tasks_difference
#                 generated = completed + res.get("tasks_difference", 0)
#                 # Ensure generated >= completed
#                 generated = max(generated, completed)
#                 rate = completed / generated if generated > 0 else np.nan
#                 # Clamp to [0,1]
#                 rate = np.clip(rate, 0.0, 1.0)
#                 rates.append(rate)

#             mean_rate, ci_rate = mean_ci(rates)
#             plot_data[n]["ti"].append(ti)
#             plot_data[n]["mean_rate"].append(mean_rate)
#             plot_data[n]["ci_rate"].append(ci_rate)

#             summary.append({
#                 "Tugs": n,
#                 "Task Interval": ti,
#                 "Mean Completion Rate": mean_rate,
#                 "95% CI": ci_rate
#             })
#             print(f"    → Completion Rate = {mean_rate:.3f} ± {ci_rate:.3f}")

#     # Plot completion rate vs. task interval for each fleet size
#     plt.figure(figsize=(10, 6))
#     for n in tug_counts:
#         plt.errorbar(
#             plot_data[n]["ti"],
#             plot_data[n]["mean_rate"],
#             yerr=plot_data[n]["ci_rate"],
#             fmt='o-',
#             capsize=4,
#             label=f"{n} tugs"
#         )
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Mean Completion Rate")
#     plt.title("Effect of Task Interval on Completion Rate for Different Fleet Sizes")
#     plt.legend()
#     plt.tight_layout()
#     plt.show()

#     # Summary table
#     df = pd.DataFrame(summary)
#     print("\nCompletion Rate Summary:")
#     print(df.to_string(index=False))


# Code to check coefficient of variations

if __name__ == "__main__":
    num_runs = 100  # Fixed number of iterations
    visualization = False
    visualization_speed = 0.0001 

    kpis = {
        "collisions": [],
        "tasks_completed": [],
        "avg_execution_time": [],
        "avg_total_time": [],
        "delays": []
    }

    cv_tracking = {
        key: [] for key in kpis
    }

    def compute_cv(data):
        if len(data) < 2:
            return 0
        mean = np.mean(data)
        std = np.std(data, ddof=1)
        return std / mean if mean != 0 else 0

    for i in range(1, num_runs + 1):
        print(f"\n--- Running simulation {i} ---")

        try:
            kpi = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
        except RuntimeError as e:
            print(f"❌ Simulation {i} failed: {e}")  # For deadlock cases
            continue  

        for key in kpis:
            value = kpi[key]
            if key == "delays" and isinstance(value, list):
                value = sum(value) / len(value) if value else 0  
            kpis[key].append(value)
            cv = compute_cv(kpis[key])
            cv_tracking[key].append(cv)

    # Plot Cv progression over runs
    fig, ax = plt.subplots(figsize=(10, 6))
    for key in cv_tracking:
        ax.plot(range(1, len(cv_tracking[key]) + 1), cv_tracking[key], label=f"{key} Cv")

    ax.set_title("Coefficient of Variation (Cv) vs Number of Runs")
    ax.set_xlabel("Number of Simulation Runs")
    ax.set_ylabel("Coefficient of Variation (Cv)")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()






# This is to determine number of runs
# if __name__ == "__main__":
#     num_runs = 400
#     visualization = False
#     visualization_speed = 0.0001 

#     kpis = {
#         "collisions": [],
#         "tasks_completed": [],
#         "avg_execution_time": [],
#         "avg_total_time": [],
#         "delays": []
#     }

#     cv_tracking = {
#         key: [] for key in kpis
#     }

#     def compute_cv(data):
#         if len(data) < 2:
#             return 0
#         mean = np.mean(data)
#         std = np.std(data, ddof=1)
#         return std / mean if mean != 0 else 0
 
  
#     epsilon = 0.03
#     window_size = 20
#     early_stop = False

#     for i in range(1, num_runs + 1):
#         print(f"\n--- Running simulation {i} ---")

#         try:
#             kpi = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
#         except RuntimeError as e:
#             print(f"❌ Simulation {i} failed: {e}") # For when model gets into a deadlock with 3 tugs
#             continue  

#         for key in kpis:
#             value = kpi[key]
#             if key == "delays" and isinstance(value, list):  # Fix for delays being a list
#                 value = sum(value) / len(value) if value else 0  
#             kpis[key].append(value)
#             cv = compute_cv(kpis[key])
#             cv_tracking[key].append(cv)

#         if i >= window_size:
#             stable = True
#             for key in cv_tracking:
#                 recent_vals = cv_tracking[key][-window_size:]
#                 if max(recent_vals) - min(recent_vals) > epsilon:
#                     stable = False
#                     break
#             if stable:
#                 print(f"\n✅ Cv stabilized across all KPIs after {i} runs.")
#                 early_stop = True
#                 break

#     # Plot
#     fig, ax = plt.subplots(figsize=(10, 6))
#     for key in cv_tracking:
#         ax.plot(range(1, len(cv_tracking[key]) + 1), cv_tracking[key], label=f"{key} Cv")


#     ax.set_title("Coefficient of Variation (Cv) vs Number of Runs")
#     ax.set_xlabel("Number of Simulation Runs")
#     ax.set_ylabel("Coefficient of Variation (Cv)")
#     ax.legend()
#     ax.grid(True)
#     plt.tight_layout()
#     plt.show()




# '''Baseline Model Performance'''
# if __name__ == "__main__":
#     import time
#     import numpy as np
#     import pandas as pd

#     # Baseline configuration
#     SIMULATION_TIME = 300       # timesteps
#     TASK_INTERVAL    = 3        # seconds
#     TOTAL_TUGS       = 8        # fleet size
#     N_RUNS           = 1      # replications

#     # Containers for per‐run metrics
#     collisions        = []
#     completion_rates  = []
#     mean_delays       = []
#     exec_times        = []
#     total_times       = []
#     avg_distances     = []
#     cpu_times         = []

#     print(f"Running baseline ({TOTAL_TUGS} tugs, {TASK_INTERVAL}s interval, {SIMULATION_TIME} timesteps) for {N_RUNS} runs...")
#     for run in range(1, N_RUNS+1):
#         t0 = time.time()
#         res = run_simulation(
#             visualization_speed=0.0,
#             task_interval=TASK_INTERVAL,
#             total_tugs=TOTAL_TUGS,
#             simulation_time=SIMULATION_TIME
#         )
#         cpu_times.append(time.time() - t0)

#         collisions.append(res["collisions"])
#         tasks_completed = res["tasks_completed"]
#         tasks_generated = tasks_completed + res["tasks_difference"]
#         completion_rates.append(tasks_completed / tasks_generated if tasks_generated > 0 else np.nan)

#         mean_delays.append(np.mean(res["delays"]) if res["delays"] else np.nan)
#         exec_times.append(res["avg_execution_time"])
#         total_times.append(res["avg_total_time"])
#         avg_distances.append(res["avg_distance"])

#         if run % 10 == 0:
#             print(f"  Completed run {run}/{N_RUNS}")

#     # Aggregate
#     metrics = {
#         "Collisions per run": collisions,
#         "Completion rate": completion_rates,
#         "Average delay (s)": mean_delays,
#         "Execution time (s)": exec_times,
#         "Total task time (s)": total_times,
#         "Average distance (nodes)": avg_distances,
#         "CPU time (s)": cpu_times,
#     }

#     summary = []
#     for name, vals in metrics.items():
#         arr = np.array(vals, dtype=float)
#         summary.append({
#             "Metric": name,
#             "Mean":  np.nanmean(arr),
#             "Std":   np.nanstd(arr)
#         })

#     df = pd.DataFrame(summary)
#     print("\nBaseline Model Performance Summary:")
#     print(df.to_string(index=False))



# '''Impact of Fleet Size and Task Interval on Computation Time'''
# if __name__ == "__main__":
#     import time
#     import numpy as np
#     import matplotlib.pyplot as plt
#     import pandas as pd
#     from scipy import stats

#     # Simulation settings
#     SIMULATION_TIME = 100
#     PLANNER = "Prioritized"   # (set globally in run_simulation)
#     DELTA_T = 0.5             # planning timestep
#     DT = 0.1                  # movement timestep

#     # batch‐run parameters
#     tug_counts      = [4, 6, 8, 10, 12]
#     task_intervals  = [1, 3, 5, 7, 9]
#     n_runs_per_combo = 5

#     # helper: mean + 95% CI
#     def mean_ci(data, confidence=0.95):
#         a = np.array(data)
#         mean = np.nanmean(a)
#         se   = stats.sem(a, nan_policy='omit')
#         h    = se * stats.t.ppf((1 + confidence) / 2., len(a) - 1)
#         return mean, h

#     # containers
#     summary = []
#     plot_data = { n: {"ti": [], "mean_cpu": [], "ci_cpu": []} for n in tug_counts }

#     print("Running CPU‐time batch over fleet sizes and task intervals...")
#     for n in tug_counts:
#         for ti in task_intervals:
#             cpu_times = []
#             print(f"Testing {n} tugs, interval={ti}s...")
#             for _ in range(n_runs_per_combo):
#                 t0 = time.time()
#                 run_simulation(
#                     visualization_speed=0.0,
#                     task_interval=ti,
#                     total_tugs=n,
#                     simulation_time=SIMULATION_TIME
#                 )
#                 cpu_times.append(time.time() - t0)
#             mean_cpu, ci_cpu = mean_ci(cpu_times)
#             summary.append({
#                 "Tugs": n,
#                 "Task Interval": ti,
#                 "Mean CPU Time": mean_cpu,
#                 "95% CI": ci_cpu,
#             })
#             plot_data[n]["ti"].append(ti)
#             plot_data[n]["mean_cpu"].append(mean_cpu)
#             plot_data[n]["ci_cpu"].append(ci_cpu)
#             print(f"  CPU = {mean_cpu:.2f}±{ci_cpu:.2f} s")

#     # plot CPU time vs task interval for each fleet size
#     plt.figure(figsize=(10,6))
#     for n in tug_counts:
#         plt.errorbar(
#             plot_data[n]["ti"],
#             plot_data[n]["mean_cpu"],
#             yerr=plot_data[n]["ci_cpu"],
#             fmt='o-',
#             capsize=4,
#             label=f"{n} tugs"
#         )
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Mean CPU Time (s)")
#     plt.title("Effect of Fleet Size and Task Interval on Computation Time")
#     plt.legend()
#     plt.tight_layout()
#     plt.show()

#     # summary table
#     df = pd.DataFrame(summary)
#     print("\nComputation Time Summary:")
#     print(df.to_string(index=False))



# '''Impact of Fleet Size on Resource Utilization'''
# if __name__ == "__main__":
#     import numpy as np
#     import matplotlib.pyplot as plt
#     import pandas as pd
#     from scipy import stats

#     # Simulation settings
#     SIMULATION_TIME = 100
#     PLANNER = "Prioritized"   # (set globally in run_simulation)
#     DELTA_T = 0.5             # planning timestep
#     DT = 0.1                  # movement timestep

#     # batch‐run parameters
#     tug_counts       = [4, 6, 8, 10, 12]
#     n_runs_per_size  = 10
#     utilization_target = 0.8  # for reference

#     # helper: mean + 95% CI
#     def mean_ci(data, confidence=0.95):
#         a = np.array(data)
#         mean = np.nanmean(a)
#         se   = stats.sem(a, nan_policy='omit')
#         h    = se * stats.t.ppf((1 + confidence) / 2., len(a) - 1)
#         return mean, h

#     # containers
#     summary = []
#     plot_data = {
#       "tugs": tug_counts,
#       "U_means": [], "U_cis": [],
#       "I_means": [], "I_cis": [],
#       "M_means": [], "M_cis": []
#     }

#     for n in tug_counts:
#         U_vals = []
#         I_vals = []
#         M_vals = []
#         print(f"Running {n} tugs...")
#         for _ in range(n_runs_per_size):
#             res = run_simulation(
#                 visualization_speed=0.0,
#                 task_interval=3,
#                 total_tugs=n,
#                 simulation_time=SIMULATION_TIME
#             )
#             # sum up state times
#             it = sum(res["tug_state_times"][tid]["idle"]             for tid in res["tug_state_times"])
#             mt = sum(res["tug_state_times"][tid]["moving_to_task"]   for tid in res["tug_state_times"])
#             ex = sum(res["tug_state_times"][tid]["executing"]        for tid in res["tug_state_times"])
#             tot = it + mt + ex
#             U_vals.append(ex / tot if tot>0 else np.nan)
#             I_vals.append(it / tot if tot>0 else np.nan)
#             M_vals.append(mt / tot if tot>0 else np.nan)

#         # compute statistics
#         U_mean, U_ci = mean_ci(U_vals)
#         I_mean, I_ci = mean_ci(I_vals)
#         M_mean, M_ci = mean_ci(M_vals)

#         plot_data["U_means"].append(U_mean)
#         plot_data["U_cis"].append(U_ci)
#         plot_data["I_means"].append(I_mean)
#         plot_data["I_cis"].append(I_ci)
#         plot_data["M_means"].append(M_mean)
#         plot_data["M_cis"].append(M_ci)

#         summary.append({
#             "Tugs": n,
#             "U_mean": U_mean, "U_CI": U_ci,
#             "I_mean": I_mean, "I_CI": I_ci,
#             "M_mean": M_mean, "M_CI": M_ci,
#         })

#         print(f"  U = {U_mean:.2f}±{U_ci:.2f}, I = {I_mean:.2f}±{I_ci:.2f}, M = {M_mean:.2f}±{M_ci:.2f}")

#     # Plot all three ratios vs fleet size
#     plt.figure(figsize=(10,6))
#     plt.errorbar(plot_data["tugs"], plot_data["U_means"], yerr=plot_data["U_cis"],
#                  fmt='o-', capsize=4, label="Utilization (U)")
#     plt.errorbar(plot_data["tugs"], plot_data["I_means"], yerr=plot_data["I_cis"],
#                  fmt='s--', capsize=4, label="Idle (I)")
#     plt.errorbar(plot_data["tugs"], plot_data["M_means"], yerr=plot_data["M_cis"],
#                  fmt='d-.', capsize=4, label="Move-to-Task (M)")
#     plt.axhline(y=utilization_target, color='gray', linestyle=':', label=f"{int(utilization_target*100)}% U target")
#     plt.xlabel("Number of Taxi‑Bots")
#     plt.ylabel("Mean Time Ratio")
#     plt.title("Effect of Fleet Size on Idle, Move‑to‑Task, and Execution Ratios")
#     plt.legend()
#     plt.tight_layout()
#     plt.show()

#     # print summary
#     df = pd.DataFrame(summary)
#     print("\nResource Utilization Summary:")
#     print(df.to_string(index=False))





# '''Testing normality of KPIs'''
# def plot_all_distributions(kpi_data_dict):
#     import matplotlib.pyplot as plt
#     import numpy as np
#     from scipy.stats import norm
#     num_plots = len(kpi_data_dict)
#     cols = 3
#     rows = (num_plots + cols - 1) // cols
#     fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows))
#     axes = axes.flatten() if num_plots > 1 else [axes]
#     for i, (name, data) in enumerate(kpi_data_dict.items()):
#         ax = axes[i]
#         ax.hist(data, bins=10, density=True, alpha=0.6, edgecolor='black')
#         mu, std = np.mean(data), np.std(data)
#         xmin, xmax = ax.get_xlim()
#         x = np.linspace(xmin, xmax, 100)
#         p = norm.pdf(x, mu, std)
#         ax.plot(x, p, 'k', linewidth=2)
#         ax.set_title(f"{name}\nMean: {mu:.2f}, Std: {std:.2f}")
#         ax.set_xlabel(name)
#         ax.set_ylabel("Probability Density")
#         ax.grid(True)
#     for j in range(i + 1, len(axes)):
#         fig.delaxes(axes[j])
#     plt.tight_layout()
#     plt.show()

# if __name__ == "__main__":
#     visualization = False
#     num_runs = 100  # Adjust the number of simulation runs as needed
#     collisions_list = []
#     tasks_completed_list = []
#     avg_execution_time_list = []
#     avg_total_time_list = []
#     avg_distance_list = []
#     cpu_runtime_list = []
#     delay_list = []  # New list for average delay KPI
#     error_count = 0
#     tug_count = 8
#     task_interval = 3

#     for i in range(num_runs):
#         print(f"\n--- Simulation run {i+1}/{num_runs} ---")
#         try:
#             kpi_results = run_simulation(visualization_speed, task_interval, tug_count, SIMULATION_TIME)
#             collisions_list.append(kpi_results["collisions"])
#             tasks_completed_list.append(kpi_results["tasks_completed"])
#             avg_execution_time_list.append(kpi_results["avg_execution_time"])
#             avg_total_time_list.append(kpi_results["avg_total_time"])
#             avg_distance_list.append(kpi_results["avg_distance"])
#             if "cpu_runtime" in kpi_results:
#                 cpu_runtime_list.append(kpi_results["cpu_runtime"])
#             # Compute average delay for the simulation run
#             if kpi_results["delays"]:
#                 avg_delay = sum(kpi_results["delays"]) / len(kpi_results["delays"])
#             else:
#                 avg_delay = 0
#             delay_list.append(avg_delay)
#         except Exception as e:
#             error_count += 1
#             print(f"Error in simulation run {i+1}: {e}")

#     error_rate = error_count / num_runs
#     print("\n=== Overall Error Rate ===")
#     print(f"Error Rate: {error_rate * 100:.2f}% ({error_count}/{num_runs} runs encountered errors)")
#     print("-----")

#     if collisions_list:
#         print("\n=== Normality Test Results for KPIs ===")
#         test_normality(collisions_list, "Collisions")
#         test_normality(tasks_completed_list, "Tasks Completed")
#         test_normality(avg_execution_time_list, "Average Execution Time")
#         test_normality(avg_total_time_list, "Average Total Task Time")
#         test_normality(avg_distance_list, "Average Task Distance")
#         test_normality(delay_list, "Average Delay")
#         if cpu_runtime_list:
#             test_normality(cpu_runtime_list, "CPU Runtime")

#         # Combine KPI data into one dictionary for a single comprehensive plot
#         kpi_dict = {
#             "Collisions": collisions_list,
#             "Tasks Completed": tasks_completed_list,
#             "Avg Execution Time": avg_execution_time_list,
#             "Avg Total Task Time": avg_total_time_list,
#             "Avg Task Distance": avg_distance_list,
#             "Avg Delay": delay_list
#         }
#         if cpu_runtime_list:
#             kpi_dict["CPU Runtime"] = cpu_runtime_list

#         plot_all_distributions(kpi_dict)
#     else:
#         print("No successful simulation runs to analyze KPIs.")



# '''SIMULATION DASHBOARD'''
# if __name__ == "__main__":
#     import numpy as np
#     import matplotlib.pyplot as plt
#     import matplotlib.gridspec as gridspec
#     from scipy import stats

#     # Simulation settings
#     SIMULATION_TIME = 300
#     PLANNER = "Prioritized"  # Choose which planner to use (Independent, Prioritized, CBS)
#     DELTA_T = 0.5            # Time step for planning
#     DT = 0.1                 # Time step for movement

#     # Visualization settings (disabled for batch runs)
#     plot_graph = False       # Show graph representation in NetworkX
#     visualization = False    # Pygame visualization
#     visualization_speed = 0.1

#     task_interval = 3        # Generate a task every 3 seconds
#     total_tugs = 5           # Total number of tugs

#     n_runs = 10  # Number of simulation runs

#     # Lists to collect scalar KPIs from each run
#     collisions_list = []
#     tasks_completed_list = []
#     avg_execution_time_list = []
#     avg_total_time_list = []
#     avg_distance_list = []
#     avg_delay_list = []  # For each run, we'll store the average delay (if available)

#     # Dictionary to collect state times (per run) for each state
#     state_times_dict = {"idle": [], "moving_to_task": [], "executing": [], "to_depot": []}

#     # Dictionaries to collect time-series data (each key will have a list of arrays, one per run)
#     idle_time_series = {}    # key: tug_id, value: list of idle time arrays (per time step)
#     battery_time_series = {} # key: tug_id, value: list of battery percentage arrays (per time step)
#     common_time_history = None

#     # Collect all individual delay values (from all runs)
#     all_delays = []

#     print("Running simulation batch...")
#     for run in range(n_runs):
#         print(f"Run {run+1}/{n_runs}")
#         results = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
        
#         collisions_list.append(results["collisions"])
#         tasks_completed_list.append(results["tasks_completed"])
#         avg_execution_time_list.append(results["avg_execution_time"])
#         avg_total_time_list.append(results["avg_total_time"])
#         avg_distance_list.append(results["avg_distance"])
#         if results["delays"]:
#             run_avg_delay = np.mean(results["delays"])
#             avg_delay_list.append(run_avg_delay)
#             all_delays.extend(results["delays"])
#         else:
#             avg_delay_list.append(np.nan)
        
#         for state in state_times_dict.keys():
#             state_times_dict[state].append(results["avg_state_times"][state])
        
#         # For time series, assume each run has the same time_history
#         if common_time_history is None:
#             common_time_history = results["time_history"]
#         for tug_id, idle_list in results["idle_time_history"].items():
#             if tug_id not in idle_time_series:
#                 idle_time_series[tug_id] = []
#             idle_time_series[tug_id].append(np.array(idle_list))
#         for tug_id, batt_list in results["battery_history"].items():
#             if tug_id not in battery_time_series:
#                 battery_time_series[tug_id] = []
#             battery_time_series[tug_id].append(np.array(batt_list))
    
#     # Function to compute mean and 95% confidence interval
#     def compute_mean_ci(data, confidence=0.95):
#         a = np.array(data)
#         mean = np.nanmean(a)
#         std_err = stats.sem(a, nan_policy='omit')
#         h = std_err * stats.t.ppf((1 + confidence) / 2., len(a)-1)
#         return mean, h

#     # Compute aggregated KPIs
#     collisions_mean, collisions_ci = compute_mean_ci(collisions_list)
#     tasks_completed_mean, tasks_completed_ci = compute_mean_ci(tasks_completed_list)
#     avg_exec_mean, avg_exec_ci = compute_mean_ci(avg_execution_time_list)
#     avg_total_mean, avg_total_ci = compute_mean_ci(avg_total_time_list)
#     avg_distance_mean, avg_distance_ci = compute_mean_ci(avg_distance_list)
#     avg_delay_mean, avg_delay_ci = compute_mean_ci(avg_delay_list)

#     # Compute aggregated state times
#     state_means = {}
#     state_cis = {}
#     for state, values in state_times_dict.items():
#         m, ci = compute_mean_ci(values)
#         state_means[state] = m
#         state_cis[state] = ci

#     # Compute average and CI for idle time and battery time series (per tug)
#     idle_time_avg = {}
#     idle_time_ci = {}
#     for tug_id, arrays in idle_time_series.items():
#         data = np.vstack(arrays)  # shape: (n_runs, time_steps)
#         mean_series = np.mean(data, axis=0)
#         std_err_series = stats.sem(data, axis=0)
#         ci_series = std_err_series * stats.t.ppf((1+0.95)/2., n_runs-1)
#         idle_time_avg[tug_id] = mean_series
#         idle_time_ci[tug_id] = ci_series

#     battery_time_avg = {}
#     battery_time_ci = {}
#     for tug_id, arrays in battery_time_series.items():
#         data = np.vstack(arrays)
#         mean_series = np.mean(data, axis=0)
#         std_err_series = stats.sem(data, axis=0)
#         ci_series = std_err_series * stats.t.ppf((1+0.95)/2., n_runs-1)
#         battery_time_avg[tug_id] = mean_series
#         battery_time_ci[tug_id] = ci_series

#     # Build dashboard plots using GridSpec
#     fig = plt.figure(constrained_layout=True, figsize=(12, 10))
#     gs = fig.add_gridspec(3, 2)

#     # Subplot 1: Idle Time History (Mean with 95% CI)
#     ax1 = fig.add_subplot(gs[0, 0])
#     for tug_id, avg_series in idle_time_avg.items():
#         ci_series = idle_time_ci[tug_id]
#         ax1.plot(common_time_history, avg_series, label=f"Tug {tug_id}")
#         ax1.fill_between(common_time_history, avg_series - ci_series, avg_series + ci_series, alpha=0.2)
#     ax1.set_xlabel("Simulation Time (s)")
#     ax1.set_ylabel("Accumulated Idle Time (s)")
#     ax1.set_title("Idle Time of Each Tug (Mean ± 95% CI)")
#     ax1.legend(fontsize='small')

#     # Subplot 2: Battery Percentage History (Mean with 95% CI)
#     ax2 = fig.add_subplot(gs[0, 1])
#     for tug_id, avg_series in battery_time_avg.items():
#         ci_series = battery_time_ci[tug_id]
#         ax2.plot(common_time_history, avg_series, label=f"Tug {tug_id}")
#         ax2.fill_between(common_time_history, avg_series - ci_series, avg_series + ci_series, alpha=0.2)
#     ax2.set_xlabel("Simulation Time (s)")
#     ax2.set_ylabel("Battery Percentage (%)")
#     ax2.set_title("Battery Percentage Over Time (Mean ± 95% CI)")
#     ax2.legend(fontsize='small')

#     # Subplot 3: Tug State Times (Bar Chart with error bars)
#     ax3 = fig.add_subplot(gs[1, 0])
#     states = list(state_means.keys())
#     means = [state_means[s] for s in states]
#     cis = [state_cis[s] for s in states]
#     ax3.bar(states, means, yerr=cis, capsize=5, color='skyblue')
#     ax3.set_xlabel("Tug States")
#     ax3.set_ylabel("Average Time (s)")
#     ax3.set_title("Average Tug State Times (Mean ± 95% CI)")

#     # Subplot 4: Delay Histogram (Aggregated from all runs)
#     ax4 = fig.add_subplot(gs[1, 1])
#     if all_delays:
#         ax4.hist(all_delays, bins=20, edgecolor='black', alpha=0.7)
#         ax4.set_xlabel("Delay (s)")
#         ax4.set_ylabel("Frequency")
#         ax4.set_title("Delay Distribution Over All Runs")
#     else:
#         ax4.text(0.5, 0.5, "No Delay Data", ha='center', va='center', fontsize=12)
#         ax4.set_axis_off()

#     # Subplot 5: KPIs Table (Aggregated Metrics with Confidence Intervals)
#     ax5 = fig.add_subplot(gs[2, :])
#     ax5.axis('off')
#     kpi_labels = ["Total Collisions", "Tasks Completed", "Avg Execution Time (s)",
#                   "Avg Total Task Time (s)", "Avg Task Distance", "Avg Delay (s)"]
#     kpi_means = [collisions_mean, tasks_completed_mean, avg_exec_mean, avg_total_mean, avg_distance_mean, avg_delay_mean]
#     kpi_cis = [collisions_ci, tasks_completed_ci, avg_exec_ci, avg_total_ci, avg_distance_ci, avg_delay_ci]
#     kpi_table_data = []
#     for label, mean, ci in zip(kpi_labels, kpi_means, kpi_cis):
#         kpi_table_data.append([label, f"{mean:.2f} ± {ci:.2f}"])
#     table = ax5.table(cellText=kpi_table_data, colLabels=["KPI", "Mean ± 95% CI"],
#                       cellLoc='center', loc='center')
#     table.auto_set_font_size(False)
#     table.set_fontsize(12)
#     table.scale(1, 2)
#     ax5.set_title(f"Aggregated Simulation KPIs ({n_runs} Runs)", fontweight="bold", fontsize=14)

#     plt.suptitle(f"Simulation Dashboard: {n_runs} Runs with Averages and 95% Confidence Intervals", fontsize=18, fontweight="bold")
#     plt.show()






# '''delay vs num tugs and task interval'''
# if __name__ == "__main__":
#     import numpy as np
#     import matplotlib.pyplot as plt
#     import pandas as pd
#     from scipy import stats

#     # Simulation settings
#     SIMULATION_TIME = 200
#     PLANNER = "Prioritized"   # (Assumed to be set globally in run_simulation)
#     DELTA_T = 0.5             # Time step for planning
#     DT = 0.1                  # Time step for movement

#     # Visualization settings (disabled for batch runs)
#     plot_graph = False        # Show graph representation in NetworkX
#     visualization = False     # Pygame visualization
#     visualization_speed = 0.1

#     # Define the task intervals to test (from 1 to 10 seconds)
#     task_intervals = list(range(1, 11, 2))  # [1, 2, ..., 10]

#     # Define the different numbers of tugs to test
#     tug_counts = [4, 6, 8, 10]

#     # Number of simulation runs per (task_interval, tug_count) combination
#     n_runs_per_combo = 50

#     # Define the delay threshold (in seconds) for our hypothesis.
#     # Hypothesis: With 95% confidence, the average delay is below this threshold.
#     delay_threshold = 30.0

#     # Helper function: compute mean and 95% confidence interval for a given data list.
#     def compute_mean_ci(data, confidence=0.95):
#         a = np.array(data)
#         # Remove NaN values (in case no delays were recorded)
#         a = a[~np.isnan(a)]
#         if len(a) == 0:
#             return np.nan, np.nan
#         mean = np.mean(a)
#         std_err = stats.sem(a)
#         h = std_err * stats.t.ppf((1 + confidence) / 2., len(a) - 1)
#         return mean, h

#     # Containers to store aggregated results
#     results_summary = []  # List of dicts to form a summary DataFrame later.
#     plot_data = {}  # Key: tug_count, Value: dict with task_intervals, avg_delays, CIs

#     print("Running simulation batch for varying task intervals and tug counts...")
#     for tug_count in tug_counts:
#         avg_delays_for_ti = []
#         ci_for_ti = []
#         for ti in task_intervals:
#             run_delays = []
#             print(f"\nTugs: {tug_count} | Task Interval: {ti} s")
#             for run in range(n_runs_per_combo):
#                 print(f"  Run {run+1}/{n_runs_per_combo} ...", end="\r")
#                 results = run_simulation(visualization_speed, ti, tug_count, SIMULATION_TIME)
#                 # Compute the average delay for the current run (if delay data exists)
#                 if results["delays"]:
#                     run_avg_delay = np.mean(results["delays"])
#                 else:
#                     run_avg_delay = np.nan
#                 run_delays.append(run_avg_delay)
#             mean_delay, ci = compute_mean_ci(run_delays)
#             avg_delays_for_ti.append(mean_delay)
#             ci_for_ti.append(ci)
#             meets = (mean_delay + ci) < delay_threshold
#             results_summary.append({
#                 "Tug Count": tug_count,
#                 "Task Interval": ti,
#                 "Mean Delay": mean_delay,
#                 "95% CI": ci,
#                 "Meets Threshold": meets
#             })
#             print(f"  -> Mean Delay = {mean_delay:.2f} ± {ci:.2f} s, Meets Threshold? {meets}")
#         plot_data[tug_count] = {
#             "task_intervals": task_intervals,
#             "avg_delays": avg_delays_for_ti,
#             "cis": ci_for_ti
#         }

#     # Plot: Average Delay vs. Task Interval with 95% Confidence Intervals for each tug count
#     plt.figure(figsize=(12, 8))
#     for tug_count, data in plot_data.items():
#         plt.errorbar(data["task_intervals"], data["avg_delays"], yerr=data["cis"], fmt='o-', capsize=5,
#                      label=f"{tug_count} Tugs")
#     plt.axhline(y=delay_threshold, color='red', linestyle='--', label=f"Delay Threshold ({delay_threshold} s)")
#     plt.xlabel("Task Interval (s)")
#     plt.ylabel("Average Delay (s)")
#     plt.title("Effect of Task Interval on Average Delay for Different Tug Counts")
#     plt.legend()
#     plt.show()

#     # Create a summary DataFrame of the simulation results
#     summary_df = pd.DataFrame(results_summary)
#     print("\nSummary of Simulation Results:")
#     print(summary_df)



'''SIMULATION DASHBOARD'''
# if __name__ == "__main__":
    # import numpy as np
    # import matplotlib.pyplot as plt
    # import matplotlib.gridspec as gridspec
    # from scipy import stats

    # # Simulation settings
    # SIMULATION_TIME = 300
    # PLANNER = "Prioritized"  # Choose which planner to use (Independent, Prioritized, CBS)
    # DELTA_T = 0.5            # Time step for planning
    # DT = 0.1                 # Time step for movement

    # # Visualization settings (disabled for batch runs)
    # plot_graph = False       # Show graph representation in NetworkX
    # visualization = False    # Pygame visualization
    # visualization_speed = 0.1

    # task_interval = 3        # Generate a task every 3 seconds
    # total_tugs = 5           # Total number of tugs

    # n_runs = 100  # Number of simulation runs

    # # Lists to collect scalar KPIs from each run
    # collisions_list = []
    # tasks_completed_list = []
    # avg_execution_time_list = []
    # avg_total_time_list = []
    # avg_distance_list = []
    # avg_delay_list = []  # For each run, we'll store the average delay (if available)

    # # Dictionary to collect state times (per run) for each state
    # state_times_dict = {"idle": [], "moving_to_task": [], "executing": [], "to_depot": []}

    # # Dictionaries to collect time-series data (each key will have a list of arrays, one per run)
    # idle_time_series = {}    # key: tug_id, value: list of idle time arrays (per time step)
    # battery_time_series = {} # key: tug_id, value: list of battery percentage arrays (per time step)
    # common_time_history = None

    # # Collect all individual delay values (from all runs)
    # all_delays = []

    # print("Running simulation batch...")
    # for run in range(n_runs):
    #     print(f"Run {run+1}/{n_runs}")
    #     results = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
        
    #     collisions_list.append(results["collisions"])
    #     tasks_completed_list.append(results["tasks_completed"])
    #     avg_execution_time_list.append(results["avg_execution_time"])
    #     avg_total_time_list.append(results["avg_total_time"])
    #     avg_distance_list.append(results["avg_distance"])
    #     if results["delays"]:
    #         run_avg_delay = np.mean(results["delays"])
    #         avg_delay_list.append(run_avg_delay)
    #         all_delays.extend(results["delays"])
    #     else:
    #         avg_delay_list.append(np.nan)
        
    #     for state in state_times_dict.keys():
    #         state_times_dict[state].append(results["avg_state_times"][state])
        
    #     # For time series, assume each run has the same time_history
    #     if common_time_history is None:
    #         common_time_history = results["time_history"]
    #     for tug_id, idle_list in results["idle_time_history"].items():
    #         if tug_id not in idle_time_series:
    #             idle_time_series[tug_id] = []
    #         idle_time_series[tug_id].append(np.array(idle_list))
    #     for tug_id, batt_list in results["battery_history"].items():
    #         if tug_id not in battery_time_series:
    #             battery_time_series[tug_id] = []
    #         battery_time_series[tug_id].append(np.array(batt_list))
    
    # # Function to compute mean and 95% confidence interval
    # def compute_mean_ci(data, confidence=0.95):
    #     a = np.array(data)
    #     mean = np.nanmean(a)
    #     std_err = stats.sem(a, nan_policy='omit')
    #     h = std_err * stats.t.ppf((1 + confidence) / 2., len(a)-1)
    #     return mean, h

    # # Compute aggregated KPIs
    # collisions_mean, collisions_ci = compute_mean_ci(collisions_list)
    # tasks_completed_mean, tasks_completed_ci = compute_mean_ci(tasks_completed_list)
    # avg_exec_mean, avg_exec_ci = compute_mean_ci(avg_execution_time_list)
    # avg_total_mean, avg_total_ci = compute_mean_ci(avg_total_time_list)
    # avg_distance_mean, avg_distance_ci = compute_mean_ci(avg_distance_list)
    # avg_delay_mean, avg_delay_ci = compute_mean_ci(avg_delay_list)

    # # Compute aggregated state times
    # state_means = {}
    # state_cis = {}
    # for state, values in state_times_dict.items():
    #     m, ci = compute_mean_ci(values)
    #     state_means[state] = m
    #     state_cis[state] = ci

    # # Compute average and CI for idle time and battery time series (per tug)
    # idle_time_avg = {}
    # idle_time_ci = {}
    # for tug_id, arrays in idle_time_series.items():
    #     data = np.vstack(arrays)  # shape: (n_runs, time_steps)
    #     mean_series = np.mean(data, axis=0)
    #     std_err_series = stats.sem(data, axis=0)
    #     ci_series = std_err_series * stats.t.ppf((1+0.95)/2., n_runs-1)
    #     idle_time_avg[tug_id] = mean_series
    #     idle_time_ci[tug_id] = ci_series

    # battery_time_avg = {}
    # battery_time_ci = {}
    # for tug_id, arrays in battery_time_series.items():
    #     data = np.vstack(arrays)
    #     mean_series = np.mean(data, axis=0)
    #     std_err_series = stats.sem(data, axis=0)
    #     ci_series = std_err_series * stats.t.ppf((1+0.95)/2., n_runs-1)
    #     battery_time_avg[tug_id] = mean_series
    #     battery_time_ci[tug_id] = ci_series

    # # Build dashboard plots using GridSpec
    # fig = plt.figure(constrained_layout=True, figsize=(12, 10))
    # gs = fig.add_gridspec(3, 2)

    # # Subplot 1: Idle Time History (Mean with 95% CI)
    # ax1 = fig.add_subplot(gs[0, 0])
    # for tug_id, avg_series in idle_time_avg.items():
    #     ci_series = idle_time_ci[tug_id]
    #     ax1.plot(common_time_history, avg_series, label=f"Tug {tug_id}")
    #     ax1.fill_between(common_time_history, avg_series - ci_series, avg_series + ci_series, alpha=0.2)
    # ax1.set_xlabel("Simulation Time (s)")
    # ax1.set_ylabel("Accumulated Idle Time (s)")
    # ax1.set_title("Idle Time of Each Tug (Mean ± 95% CI)")
    # ax1.legend(fontsize='small')

    # # Subplot 2: Battery Percentage History (Mean with 95% CI)
    # ax2 = fig.add_subplot(gs[0, 1])
    # for tug_id, avg_series in battery_time_avg.items():
    #     ci_series = battery_time_ci[tug_id]
    #     ax2.plot(common_time_history, avg_series, label=f"Tug {tug_id}")
    #     ax2.fill_between(common_time_history, avg_series - ci_series, avg_series + ci_series, alpha=0.2)
    # ax2.set_xlabel("Simulation Time (s)")
    # ax2.set_ylabel("Battery Percentage (%)")
    # ax2.set_title("Battery Percentage Over Time (Mean ± 95% CI)")
    # ax2.legend(fontsize='small')

    # # Subplot 3: Tug State Times (Bar Chart with error bars)
    # ax3 = fig.add_subplot(gs[1, 0])
    # states = list(state_means.keys())
    # means = [state_means[s] for s in states]
    # cis = [state_cis[s] for s in states]
    # ax3.bar(states, means, yerr=cis, capsize=5, color='skyblue')
    # ax3.set_xlabel("Tug States")
    # ax3.set_ylabel("Average Time (s)")
    # ax3.set_title("Average Tug State Times (Mean ± 95% CI)")

    # # Subplot 4: Delay Histogram (Aggregated from all runs)
    # ax4 = fig.add_subplot(gs[1, 1])
    # if all_delays:
    #     ax4.hist(all_delays, bins=20, edgecolor='black', alpha=0.7)
    #     ax4.set_xlabel("Delay (s)")
    #     ax4.set_ylabel("Frequency")
    #     ax4.set_title("Delay Distribution Over All Runs")
    # else:
    #     ax4.text(0.5, 0.5, "No Delay Data", ha='center', va='center', fontsize=12)
    #     ax4.set_axis_off()

    # # Subplot 5: KPIs Table (Aggregated Metrics with Confidence Intervals)
    # ax5 = fig.add_subplot(gs[2, :])
    # ax5.axis('off')
    # kpi_labels = ["Total Collisions", "Tasks Completed", "Avg Execution Time (s)",
    #               "Avg Total Task Time (s)", "Avg Task Distance", "Avg Delay (s)"]
    # kpi_means = [collisions_mean, tasks_completed_mean, avg_exec_mean, avg_total_mean, avg_distance_mean, avg_delay_mean]
    # kpi_cis = [collisions_ci, tasks_completed_ci, avg_exec_ci, avg_total_ci, avg_distance_ci, avg_delay_ci]
    # kpi_table_data = []
    # for label, mean, ci in zip(kpi_labels, kpi_means, kpi_cis):
    #     kpi_table_data.append([label, f"{mean:.2f} ± {ci:.2f}"])
    # table = ax5.table(cellText=kpi_table_data, colLabels=["KPI", "Mean ± 95% CI"],
    #                   cellLoc='center', loc='center')
    # table.auto_set_font_size(False)
    # table.set_fontsize(12)
    # table.scale(1, 2)
    # ax5.set_title(f"Aggregated Simulation KPIs ({n_runs} Runs)", fontweight="bold", fontsize=14)

    # plt.suptitle(f"Simulation Dashboard: {n_runs} Runs with Averages and 95% Confidence Intervals", fontsize=18, fontweight="bold")
    # plt.show()



# if __name__ == "__main__":
#     # Simulation settings
#     SIMULATION_TIME = 300
#     PLANNER = "Prioritized"  # Choose which planner to use (Independent, Prioritized, CBS)
#     DELTA_T = 0.5  # Time step for planning
#     DT = 0.1     # Time step for movement

#     # Visualization settings
#     plot_graph = False          # Show graph representation in NetworkX
#     visualization = False       # Pygame visualization
#     visualization_speed = 0.1

#     task_interval = 3           # Generate a task every x seconds
#     total_tugs = 5              # Total number of tugs

#     # Run the simulation and capture results
#     results = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
    
#     # Extract histories and KPIs
#     idle_time_history = results["idle_time_history"]
#     battery_history = results["battery_history"]
#     time_history = results["time_history"]
#     avg_state_times = results["avg_state_times"]
#     collisions = results["collisions"]
#     tasks_completed = results["tasks_completed"]
#     avg_execution_time = results["avg_execution_time"]
#     avg_total_time = results["avg_total_time"]
#     avg_distance = results["avg_distance"]
#     delays = results["delays"]
#     avg_delay = sum(delays)/len(delays) if delays else None

#     # Create a dashboard with multiple subplots using GridSpec
#     import matplotlib.gridspec as gridspec
#     fig = plt.figure(constrained_layout=True, figsize=(12, 9))
#     gs = fig.add_gridspec(3, 2)

#     # Subplot 1: Idle Time History (Line Plot)
#     ax1 = fig.add_subplot(gs[0, 0])
#     for tug_id, idle_times in idle_time_history.items():
#          ax1.plot(time_history, idle_times, label=f"Tug {tug_id}")
#     ax1.set_xlabel("Simulation Time (s)")
#     ax1.set_ylabel("Accumulated Idle Time (s)")
#     ax1.set_title("Idle Time of Each Tug")
#     ax1.legend(fontsize='small')

#     # Subplot 2: Battery Percentage History (Line Plot)
#     ax2 = fig.add_subplot(gs[0, 1])
#     for tug_id, bat_history in battery_history.items():
#          ax2.plot(time_history, bat_history, label=f"Tug {tug_id}")
#     ax2.set_xlabel("Simulation Time (s)")
#     ax2.set_ylabel("Battery Percentage (%)")
#     ax2.set_title("Battery Percentage Over Time")
#     ax2.legend(fontsize='small')

#     # Subplot 3: Tug State Times (Bar Chart of Average State Times)
#     ax3 = fig.add_subplot(gs[1, 0])
#     states = list(avg_state_times.keys())
#     times = [avg_state_times[state] for state in states]
#     ax3.bar(states, times, color='skyblue')
#     ax3.set_xlabel("Tug States")
#     ax3.set_ylabel("Average Time (s)")
#     ax3.set_title("Average Tug State Times")

#     # Subplot 4: Delay Histogram (if delay data is available)
#     ax4 = fig.add_subplot(gs[1, 1])
#     if delays:
#          ax4.hist(delays, bins=10, edgecolor='black', alpha=0.7)
#          ax4.set_xlabel("Delay (s)")
#          ax4.set_ylabel("Frequency")
#          ax4.set_title("Delay Distribution")
#     else:
#          ax4.text(0.5, 0.5, "No Delay Data", ha='center', va='center', fontsize=12)
#          ax4.set_axis_off()

#     # Subplot 5: KPIs Table (spanning the entire bottom row)
#     ax5 = fig.add_subplot(gs[2, :])
#     ax5.axis('off')
#     kpi_labels = ["Total Collisions", "Tasks Completed", "Avg Execution Time (s)",
#                   "Avg Total Task Time (s)", "Avg Task Distance", "Avg Delay (s)"]
#     kpi_values = [
#          collisions,
#          tasks_completed,
#          f"{avg_execution_time:.2f}" if avg_execution_time else "N/A",
#          f"{avg_total_time:.2f}" if avg_total_time else "N/A",
#          f"{avg_distance:.2f}" if avg_distance else "N/A",
#          f"{avg_delay:.2f}" if avg_delay is not None else "N/A"
#     ]
#     table_data = [[label, value] for label, value in zip(kpi_labels, kpi_values)]
#     table = ax5.table(cellText=table_data, colLabels=["KPI", "Value"],
#                       cellLoc='center', loc='center')
#     table.auto_set_font_size(False)
#     table.set_fontsize(12)
#     table.scale(1, 2)
#     ax5.set_title("Simulation KPIs", fontweight="bold", fontsize=14)

#     plt.suptitle("Simulation Dashboard", fontsize=16, fontweight="bold")
#     plt.show()





# Simulation block for running of hypotheses tests
# if __name__ == "__main__":
#     import pandas as pd
#     from scipy.stats import ttest_ind, mannwhitneyu

#     num_runs = 100
#     visualization = False
#     visualization_speed = 0.0001

#     ttest_kpis = ["collisions", "tasks_completed", "delay", "execution_time"]  
#     mannwhitney_kpis = ["task_time", "idle_time", "cpu_time", "error_rate"]    
#     all_kpis = ttest_kpis + mannwhitney_kpis

#     scenario_A = {k: [] for k in all_kpis}  # 8 tugs, interval 2
#     scenario_B = {k: [] for k in all_kpis}  # 10 tugs, interval 2
#     scenario_C = {k: [] for k in all_kpis}  # 8 tugs, interval 3

#     def run_and_extract_kpis(total_tugs, task_interval):
#         kpi = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
#         return {
#             "collisions": kpi["collisions"],
#             "tasks_completed": kpi["tasks_completed"],
#             "delay": sum(kpi["delays"]) / len(kpi["delays"]) if kpi["delays"] else 0,
#             "execution_time": kpi["avg_execution_time"],
#             "task_time": kpi["avg_total_time"],
#             "idle_time": kpi["avg_state_times"]["idle"],
#             "cpu_time": kpi["avg_state_times"]["executing"], 
#             "error_rate": kpi["tasks_difference"] / max(kpi["tasks_completed"] + kpi["tasks_difference"], 1)
#         }

#     # Scenario A (baseline)
#     print("\n--- Running Scenario A: 8 tugs, task interval = 2 ---")
#     for i in range(num_runs):
#         print(f"A - Run {i + 1}")
#         kpi = run_and_extract_kpis(total_tugs=8, task_interval=2)
#         for key in all_kpis:
#             scenario_A[key].append(kpi[key])

#     # Scenario B (more tugs)
#     print("\n--- Running Scenario B: 10 tugs, task interval = 2 ---")
#     for i in range(num_runs):
#         print(f"B - Run {i + 1}")
#         kpi = run_and_extract_kpis(total_tugs=10, task_interval=2)
#         for key in all_kpis:
#             scenario_B[key].append(kpi[key])

#     # Scenario C (slower task rate)
#     print("\n--- Running Scenario C: 8 tugs, task interval = 3 ---")
#     for i in range(num_runs):
#         print(f"C - Run {i + 1}")
#         kpi = run_and_extract_kpis(total_tugs=8, task_interval=3)
#         for key in all_kpis:
#             scenario_C[key].append(kpi[key])

#     results = []

#     # Compare A vs B 
#     for key in ttest_kpis:
#         stat, p = ttest_ind(scenario_A[key], scenario_B[key])
#         results.append((f"A vs B – {key}", "Independent t-test", p))
#     for key in mannwhitney_kpis:
#         stat, p = mannwhitneyu(scenario_A[key], scenario_B[key], alternative='two-sided')
#         results.append((f"A vs B – {key}", "Mann-Whitney U-test", p))

#     # Compare A vs C
#     for key in ttest_kpis:
#         stat, p = ttest_ind(scenario_A[key], scenario_C[key])
#         results.append((f"A vs C – {key}", "Independent t-test", p))
#     for key in mannwhitney_kpis:
#         stat, p = mannwhitneyu(scenario_A[key], scenario_C[key], alternative='two-sided')
#         results.append((f"A vs C – {key}", "Mann-Whitney U-test", p))

#     # Display results 
#     results_df = pd.DataFrame(results, columns=["Comparison", "Test", "P-Value"])
#     results_df["Statistical Conclusion"] = results_df["P-Value"].apply(lambda p: "Rejected H₀" if p < 0.05 else "Accepted H₀")
#     print("\n--- Statistical Test Results ---")
#     print(results_df)




'''OLD MAIN FUNCTIONS'''

# # New main function: Run simulation and plot idle time histories.
# if __name__ == "__main__":
#     # Simulation settings
#     SIMULATION_TIME = 300
#     PLANNER = "Prioritized"  # Choose which planner to use (Independent, Prioritized, CBS)
#     DELTA_T = 0.5  # Time step for planning
#     DT = 0.1  # Time step for movement

#     #Visualization (can also be changed)
#     plot_graph = False    #show graph representation in NetworkX
#     visualization = False        #pygame visualization
#     visualization_speed = 0.1 #set at 0.1 as default

#     task_interval = 3    # New: generate a task every 5 seconds
#     total_tugs = 10       # New: total number of tugs (will be split evenly between depots)

#     # Run the simulation and capture results (including idle time history)
#     results = run_simulation(visualization_speed, task_interval, total_tugs, SIMULATION_TIME)
    
#     idle_time_history = results["idle_time_history"]
#     time_history = results["time_history"]
    
#     # Plot idle time for each tug versus simulation time.
#     plt.figure(figsize=(10, 6))
#     for tug_id, idle_times in idle_time_history.items():
#          plt.plot(time_history, idle_times, label=f"Tug {tug_id}")
#     plt.xlabel("Simulation Time (s)")
#     plt.ylabel("Accumulated Idle Time (s)")
#     plt.title("Idle Time of Each Tug Over Simulation Time")
#     plt.legend()
#     plt.show()

#     # --- Plot Battery Percentage History ---
#     battery_history = results["battery_history"]
#     plt.figure(figsize=(10, 6))
#     for tug_id, bat_history in battery_history.items():
#          plt.plot(time_history, bat_history, label=f"Tug {tug_id}")
#     plt.xlabel("Simulation Time (s)")
#     plt.ylabel("Battery Percentage (%)")
#     plt.title("Battery Percentage of Tugs Over Simulation Time")
#     plt.legend()
#     plt.show()


# # New main function to run simulation and plot idle time histories
# if __name__ == "__main__":
#     run_simulation()





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
