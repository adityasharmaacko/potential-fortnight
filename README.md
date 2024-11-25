
# Potential Fortnight - Optimization Algorithms

Welcome to **Potential Fortnight**, a repository containing a variety of optimization algorithms aimed at solving complex logistical and scheduling problems. This project leverages **OR-Tools** and other optimization techniques to improve task assignments, delivery routes, and resource allocation.

## Algorithms Included
1. **OR-SchedulingOptimisation.py**
   Optimizes scheduling problems using Google OR-Tools to allocate resources effectively.

2. **or-toolsOptimisation-pickAndDelivery.py**
   Solves the pick-and-delivery problem, optimizing routes for pickups and deliveries based on specific constraints.

## Features
- **Scheduling Optimization**: Automatically assigns tasks to available resources while minimizing time and cost.
- **Pick-and-Delivery Optimization**: Efficiently handles tasks like delivery routing by minimizing travel distances.
- **Scalability**: Suitable for real-world applications like delivery logistics and scheduling for multiple agents.

## Setup Instructions

### Prerequisites

Before running the project, ensure that you have the following installed:

- **Python 3.x**
- **Required Python Libraries** (listed in `requirements.txt`)

To install the dependencies, run:

```bash
pip install -r requirements.txt

```

### Clone the Repository

First, clone this repository to your local machine:

```
git clone https://github.com/adityasharmaacko/potential-fortnight.git
cd potential-fortnight

```

### Run the Algorithms

You can now run any of the optimization scripts to see how they work.

#### Example 1: Scheduling Optimization

To run the scheduling optimization algorithm, execute the following:

```
python OR-SchedulingOptimisation.py

```

#### Example 2: Pick-and-Delivery Optimization

To run the pick-and-delivery algorithm, execute:

```
python or-toolsOptimisation-pickAndDelivery.py

```

Input Data
----------

### Example Input (for **OR-SchedulingOptimisation.py**)

```
{
  "tasks": [
    {
      "id": 0,
      "skill": "driver",
      "location": [12.971598, 77.594566], // Latitude and Longitude of the task
      "pincode": 560001,
      "duration": 30 // Task duration in minutes
    },
    {
      "id": 1,
      "skill": "driver",
      "location": [12.295810, 76.639381],
      "pincode": 560002,
      "duration": 45
    },
    {
      "id": 2,
      "skill": "inspection",
      "location": [13.082680, 80.270721],
      "pincode": 560002,
      "duration": 60
    }
  ],
  "agents": [
    {
      "id": 0,
      "skills": ["driver"], // Set of skills
      "location": [12.914142, 74.856033], // Starting location of the agent
      "availability": 120, // Total availability in minutes
      "allowedLocations": [560001, 560002] // Pincode restrictions
    },
    {
      "id": 1,
      "skills": ["inspection"],
      "location": [12.914142, 74.856033],
      "availability": 150,
      "allowedLocations": [560002]
    }
  ]
}


```

### Example Output

```
{
    "total_distance_covered": 920.3304418118754,
    "agent_assignments": [
        {
            "total_distance": 333.38431142334764,
            "last_location": [
                12.971598,
                77.594566
            ],
            "tasks": [
                1,
                0
            ],
            "agent_id": 0
        },
        {
            "total_distance": 586.9461303885278,
            "last_location": [
                13.08268,
                80.270721
            ],
            "tasks": [
                2
            ],
            "agent_id": 1
        }
    ],
    "unassigned_tasks": []
}

```

In this example, the agents are assigned tasks based on proximity and skill set, minimizing travel distances.

Output Data
-----------

The output is a JSON object containing:

-   **total_distance_covered**: The total distance covered by all agents.
-   **agent_assignments**: A list of agents and the tasks they were assigned, including the total distance each agent traveled.
-   **unassigned_tasks**: Any tasks that could not be assigned based on the constraints.

Why This Repository?
--------------------

-   **Real-World Use**: This repository tackles real-world problems like optimizing task assignments and delivery routes.
-   **AI-Powered**: Utilizes advanced optimization techniques to provide efficient solutions.
-   **Modular and Scalable**: Easily extendable to meet the needs of larger, more complex scenarios.

License
-------

This project is open-source and available under the MIT License.

Contact
-------

For any questions or contributions, please open an issue or contact [@adityasharmaacko](https://github.com/adityasharmaacko).

### Key Sections:

- **Introduction**: A concise summary of the repository and what it does.
- **Features**: Highlights key functionalities of the algorithms.
- **Setup Instructions**: How to set up and run the code locally.
- **Input and Output**: Sample input and output formats to help users understand how the algorithm works.
- **Why This Repository**: Motivational section to show the real-world applicability and strengths of the repo.

Let me know if you'd like any further adjustments!
