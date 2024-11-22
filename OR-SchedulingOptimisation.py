# File: task_assignment_solver.py

import math
import logging
from typing import List, Tuple, Dict, Any
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Setup Logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


# Helper Function: Calculate Haversine Distance
def haversine_distance(loc1: Tuple[float, float], loc2: Tuple[float, float]) -> float:
    """Calculate the Haversine distance between two points on the Earth."""
    R = 6371.0  # Earth radius in kilometers
    dlat = math.radians(loc2[0] - loc1[0])
    dlon = math.radians(loc2[1] - loc1[1])
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(loc1[0])) * math.cos(math.radians(loc2[0])) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


# Input Validation
def validate_input(tasks: List[Dict[str, Any]], agents: List[Dict[str, Any]]) -> None:
    """Validate input tasks and agents for required fields."""
    if not tasks or not agents:
        raise ValueError("Tasks and agents cannot be empty.")

    required_task_keys = {"id", "skill", "location", "pincode", "duration"}
    required_agent_keys = {"id", "skills", "location", "availability", "allowed_locations"}

    for task in tasks:
        if not required_task_keys.issubset(task):
            raise ValueError(f"Task {task} is missing required fields: {required_task_keys - set(task.keys())}")
    for agent in agents:
        if not required_agent_keys.issubset(agent):
            raise ValueError(f"Agent {agent} is missing required fields: {required_agent_keys - set(agent.keys())}")


# Build Distance Matrix
def build_distance_matrix(locations: List[Tuple[float, float]]) -> List[List[float]]:
    """Compute the distance matrix between all locations."""
    return [[haversine_distance(loc1, loc2) for loc2 in locations] for loc1 in locations]


# Solve Task Assignment
def solve_task_assignment(tasks: List[Dict[str, Any]], agents: List[Dict[str, Any]]) -> None:
    """Assign tasks to agents using OR-Tools."""
    # Validate Inputs
    validate_input(tasks, agents)

    # Combine Locations
    locations = [agent["location"] for agent in agents] + [task["location"] for task in tasks]

    # Build Distance Matrix
    distance_matrix = build_distance_matrix(locations)

    # Initialize Model
    num_agents = len(agents)
    num_locations = len(locations)
    start_nodes = list(range(num_agents))  # Start nodes for each agent
    end_nodes = list(range(num_agents))  # End nodes for each agent

    manager = pywrapcp.RoutingIndexManager(num_locations, num_agents, start_nodes, end_nodes)
    routing = pywrapcp.RoutingModel(manager)

    # Distance Callback
    def distance_callback(from_index: int, to_index: int) -> int:
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Task Duration Constraints
    def demand_callback(from_index: int) -> int:
        from_node = manager.IndexToNode(from_index)
        return tasks[from_node - num_agents]["duration"] if from_node >= num_agents else 0

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    availabilities = [agent["availability"] for agent in agents]
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # No slack
        availabilities,
        True,  # Start cumulative at zero
        "Availability",
    )

    # Skill and Location Constraints
    for task_index, task in enumerate(tasks, start=num_agents):
        task_skill = task["skill"]
        task_pincode = task["pincode"]
        for agent_index, agent in enumerate(agents):
            if not (task_skill in agent["skills"] and task_pincode in agent["allowed_locations"]):
                routing.VehicleVar(manager.NodeToIndex(task_index)).RemoveValue(agent_index)

    # Allow Unassigned Tasks with Penalty
    penalty = 1000
    for task_index in range(num_agents, num_locations):
        routing.AddDisjunction([manager.NodeToIndex(task_index)], penalty)

    # Solve
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.seconds = 2

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        logging.error("No solution found.")
        return

    # Output Results
    logging.info("Solution found!")
    for agent_index in range(num_agents):
        index = routing.Start(agent_index)
        route = []
        total_distance = 0
        total_duration = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node >= num_agents:
                task_id = tasks[node - num_agents]["id"]
                route.append(task_id)
                total_duration += tasks[node - num_agents]["duration"]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if not routing.IsEnd(index):
                total_distance += distance_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]

        logging.info(
            f"Agent {agent_index}: Assigned Tasks: {route}, "
            f"Total Distance: {total_distance:.2f} km, Total Duration: {total_duration} min"
            f"End Location: {locations[manager.IndexToNode(previous_index)]}"
        )

    # Output Unassigned Tasks
    unassigned_tasks = [
        tasks[task_index - num_agents]["id"]
        for task_index in range(num_agents, num_locations)
        if solution.Value(routing.NextVar(manager.NodeToIndex(task_index))) == manager.NodeToIndex(task_index)
    ]
    logging.info(f"Unassigned Tasks: {unassigned_tasks}")


# Example Usage
if __name__ == "__main__":
    tasks = [
        {"id": 0, "skill": "driver", "location": (12.971598, 77.594566), "pincode": 560001, "duration": 50},
        {"id": 1, "skill": "driver", "location": (12.295810, 76.639381), "pincode": 560002, "duration": 50},
        {"id": 2, "skill": "driver", "location": (13.082680, 80.270721), "pincode": 560002, "duration": 50},
    ]
    agents = [
        {"id": 0, "skills": {"driver"}, "location": (12.914142, 74.856033), "availability": 120, "allowed_locations": [560001]},
    ]
    solve_task_assignment(tasks, agents)
