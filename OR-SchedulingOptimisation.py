from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math, time


# Helper Function: Calculate Euclidean Distance
def euclidean_distance(loc1, loc2):
    return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)


# Input Data: Tasks and Agents
tasks = [
    {"id": 0, "skill": "A", "location": (0, 0), "duration": 50},
    {"id": 1, "skill": "B", "location": (2, 2), "duration": 90},
    {"id": 2, "skill": "A", "location": (3, 1), "duration": 50},
    {"id": 3, "skill": "A", "location": (1, 3), "duration": 90},
    {"id": 4, "skill": "B", "location": (0, 0), "duration": 50},
    {"id": 5, "skill": "B", "location": (0, 0), "duration": 50},
]

agents = [
    {"id": 0, "skills": {"A"}, "location": (1, 1), "availability": 120, "allowed_locations": [(0, 0), (1, 3)]},
    {"id": 1, "skills": {"B"}, "location": (4, 4), "availability": 120, "allowed_locations": [(2, 2), (3, 1)]},
    {"id": 2, "skills": {"A", "B"}, "location": (0, 0), "availability": 120,
     "allowed_locations": [(0, 0), (2, 2), (3, 1)]},
]

start_time = time.time()

# Combine Locations (Agents' Start and Tasks)
locations = [agent["location"] for agent in agents] + [task["location"] for task in tasks]

# Compute Distance Matrix
distance_matrix = [[euclidean_distance(loc1, loc2) for loc2 in locations] for loc1 in locations]

# Number of Agents
num_agents = len(agents)

# Define start and end nodes for each agent
start_nodes = list(range(num_agents))  # Each agent starts at its respective index
end_nodes = list(range(num_agents))  # Agents can end at any task location (default to start nodes)

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
    return tasks[from_node - num_agents]["duration"] if from_node >= num_agents else 0


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
availabilities = [agent["availability"] for agent in agents]
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # No slack
    availabilities,  # Agent availabilities in minutes
    True,  # Start cumulative at zero
    "Availability",
)

# Skill and Location Matching Constraint
for task_index, task in enumerate(tasks, start=num_agents):
    task_skill = task["skill"]
    task_location = task["location"]
    for agent_index, agent in enumerate(agents):
        if not (task_skill in agent["skills"] and task_location in agent["allowed_locations"]):
            routing.VehicleVar(manager.NodeToIndex(task_index)).RemoveValue(agent_index)

# Allow Unassigned Tasks with Penalty
penalty = 1000  # Penalty for leaving a task unassigned
for task_index in range(num_agents, len(locations)):
    routing.AddDisjunction([manager.NodeToIndex(task_index)], penalty)

# Solve the Problem
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search_parameters.time_limit.seconds = 10

solution = routing.SolveWithParameters(search_parameters)

# Output Results
if solution:
    print("Solution found!")
    for agent_index in range(num_agents):
        index = routing.Start(agent_index)
        route = []
        total_distance = 0
        total_duration = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node >= num_agents:  # It's a task
                task_id = tasks[node - num_agents]["id"]
                route.append(task_id)
                total_duration += tasks[node - num_agents]["duration"]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            total_distance += distance_matrix[manager.IndexToNode(previous_index)][manager.IndexToNode(index)]

        # Get the end location
        end_node = manager.IndexToNode(previous_index)
        end_location = locations[end_node]

        print(f"Agent {agent_index}:")
        print(f"  Assigned Tasks: {route}")
        print(f"  Total Distance: {total_distance:.2f}")
        print(f"  Total Duration: {total_duration} minutes")
        print(f"  End Location: {end_location}")

    unassigned_tasks = []
    for task_index in range(num_agents, len(locations)):
        if solution.Value(routing.NextVar(manager.NodeToIndex(task_index))) == manager.NodeToIndex(task_index):
            unassigned_tasks.append(tasks[task_index - num_agents]["id"])
    print(f"Unassigned Tasks: {unassigned_tasks}")
else:
    print("No solution found.")
