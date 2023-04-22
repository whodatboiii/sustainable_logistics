from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def tsp(data):
    # Create routing index manager.
    # This object holds the locations and helps the solver to manage them.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    # Create routing model.
    # This object represents the TSP problem and contains all the constraints and variables for the solver.
    routing = pywrapcp.RoutingModel(manager)

    # Create the transit callback function.
    # This function returns the distance between two nodes.
    transit_callback_index = routing.RegisterTransitCallback(
        lambda from_index, to_index: data['distance_matrix'][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])
    
    # Set the arc cost evaluator for all vehicles.
    # This sets the cost of each arc (edge) in the graph to the distance between the two nodes.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Define search parameters.
    # This object contains the parameters used to configure the search for a solution.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    # Use PATH_CHEAPEST_ARC strategy for the first solution.
    # This strategy selects the cheapest edge and builds the route incrementally.
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem.
    # This method returns a solution object that can be queried to get the solution.
    solution = routing.SolveWithParameters(search_parameters)

    # If a solution was found, return the route.
    if solution:
        # Get the starting index.
        index = routing.Start(0)
        # Create a list to store the route.
        route = []
        # Loop over the route until the end is reached.
        while not routing.IsEnd(index):
            # Append the node to the route list.
            route.append(manager.IndexToNode(index))
            # Update the index to the next node.
            index = solution.Value(routing.NextVar(index))
        # Append the depot to the end of the route.
        route.append(manager.IndexToNode(index))
        return route


if __name__ == '__main__':
    # Define the data for the problem.
    data = {
        'distance_matrix': [
            [0, 64, 229, 109, 378, 110, 201, 304, 346],
            [64, 0, 280, 161, 370, 266, 253, 360, 398],
            [229, 280, 0, 124, 205, 53, 86, 86, 120],
            [109, 161, 124, 0, 277, 110, 96, 204, 241],
            [378, 370, 205, 277, 0, 167, 262, 248, 146],
            [110, 266, 53, 110, 167, 0, 97, 144, 142],
            [201, 253, 86, 96, 262, 97, 0, 170, 204],
            [304, 360, 86, 204, 248, 144, 170, 0, 104],
            [346, 398, 120, 241, 146, 142, 204, 104, 0],
        ],
        'num_vehicles': 1,
        'depot': 0
    }
    route = tsp(data)
    print(route)
