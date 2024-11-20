from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math


# Helper Function: Calculate Euclidean Distance
def euclidean_distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


# Input Data: Tasks and Agents
tasks = [
    {"id": 0, "type": "single", "skill": "A", "location": (0, 0), "duration": 50},
    {"id": 1, "type": "pickup_delivery", "skill": "B", "pickup_location": (2, 2), "drop_location": (3, 1),
     "duration": 50},
    {"id": 2, "type": "single", "skill": "A", "location": (1, 3), "duration": 50},
    {"id": 3, "type": "single", "skill": "A", "location": (1, 4), "duration": 50},
]

agents = [
    {"id": 0, "skills": {"A"}, "location": (1, 1), "availability": 120},
    {"id": 1, "skills": {"B"}, "location": (4, 4), "availability": 120},
    {"id": 2, "skills": {"A", "B"}, "location": (0, 0), "availability": 120},
]

# Combine Locations (Agents' Start and Tasks)
locations = [agent["location"] for agent in agents]
task_indices = []  # To map task indices back to tasks
pickup_delivery_indices = []

for task in tasks:
    if task["type"] == "pickup_delivery":
        locations.append(task["pickup_location"])
        locations.append(task["drop_location"])
        pickup_index = len(locations) - 2
        drop_index = len(locations) - 1
        pickup_delivery_indices.append((pickup_index, drop_index))
        task_indices.append((pickup_index, task))
        task_indices.append((drop_index, task))
    elif task["type"] == "single":
        locations.append(task["location"])
        task_indices.append((len(locations) - 1, task))

# Compute Distance Matrix
distance_matrix = [[euclidean_distance(loc1, loc2) for loc2 in locations] for loc1 in locations]

# Number of Agents
num_agents = len(agents)

# Define start and end nodes for each agent
start_nodes = list(range(num_agents))  # Each agent starts at its respective index
end_nodes = list(range(num_agents))  # Agents can end at their respective start nodes

# Initialize Routing Model
manager = pywrapcp.RoutingIndexManager(len(locations), num_agents, start_nodes, end_nodes)
routing = pywrapcp.RoutingModel(manager)


# Distance Callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return int(distance_matrix[from_node][to_node])


transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


# Workload (Duration) Constraints
def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    for location_index, task in task_indices:
        if from_node == location_index:
            return task["duration"]
    return 0


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
availabilities = [agent["availability"] for agent in agents]
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # No slack
    availabilities,  # Agent availabilities in minutes
    True,  # Start cumulative at zero
    "Availability",
)

# Add Precedence Constraints for Pickup and Delivery
for pickup, drop in pickup_delivery_indices:
    routing.AddPickupAndDelivery(manager.NodeToIndex(pickup), manager.NodeToIndex(drop))
    routing.solver().Add(
        routing.VehicleVar(manager.NodeToIndex(pickup)) == routing.VehicleVar(manager.NodeToIndex(drop)))

# Skill and Location Matching Constraints
for location_index, task in task_indices:
    task_skill = task["skill"]
    for agent_index, agent in enumerate(agents):
        if task_skill not in agent["skills"]:
            routing.VehicleVar(manager.NodeToIndex(location_index)).RemoveValue(agent_index)

# Allow Unassigned Tasks with Penalty
penalty = 1000  # Penalty for leaving a task unassigned
for location_index, task in task_indices:
    routing.AddDisjunction([manager.NodeToIndex(location_index)], penalty)

# Solve the Problem
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.seconds = 10

solution = routing.SolveWithParameters(search_parameters)

# Output Results
if solution:
    print("Solution found!")
    visited_nodes = set()  # To avoid duplicate reporting of pickup-delivery tasks
    for agent_index in range(num_agents):
        index = routing.Start(agent_index)
        route = []
        total_distance = 0
        total_duration = 0
        last_location = None  # Track the agent's last location
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node not in visited_nodes:  # Skip already visited nodes
                for location_index, task in task_indices:
                    if node == location_index:
                        if task["type"] == "pickup_delivery":
                            # Group pickup-delivery as a single task
                            for pickup, drop in pickup_delivery_indices:
                                if node == pickup:
                                    route.append(f"Task {task['id']} (Pickup → Delivery)")
                                    visited_nodes.update({pickup, drop})
                                    break
                        else:
                            route.append(f"Task {task['id']}")
                            visited_nodes.add(node)
                        total_duration += task["duration"]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            total_distance += distance_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]
            last_location = manager.IndexToNode(previous_index)  # Update the last location

        # Add the agent's starting location if they never moved
        if last_location is None:
            last_location = manager.IndexToNode(routing.Start(agent_index))

        print(f"Agent {agent_index}:")
        print(f"  Assigned Tasks: {route}")
        print(f"  Total Distance: {total_distance:.2f}")
        print(f"  Total Duration: {total_duration} minutes")
        print(f"  End Location: {locations[last_location]}")  # Convert to coordinates

    # Print Unassigned Tasks
    unassigned_tasks = []
    for location_index, task in task_indices:
        if solution.Value(routing.NextVar(manager.NodeToIndex(location_index))) == manager.NodeToIndex(location_index):
            unassigned_tasks.append(task["id"])
    print(f"Unassigned Tasks: {unassigned_tasks}")
else:
    print("No solution found.")

