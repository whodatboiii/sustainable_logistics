from ortools.linear_solver import pywraplp

def solution_value(x):
    if isinstance(x, (int, float)):
        return x
    elif x is None:
        return 0
    elif not x.Integer():
        return x.SolutionValue()
    else:
        return int(x.SolutionValue())

def solve_model_eliminate(distance_matrix, subtours=None):
    solver = pywraplp.Solver.CreateSolver('CBC')
    n = len(distance_matrix)
    x = [[solver.IntVar(0, 0 if distance_matrix[i][j] == 0 else 1, '') for j in range(n)] for i in range(n)] 

    # Basic constraints: 
    # - one only predecessor; 
    # - one only successor; 
    # - no route between same node -> xii=0
    for i in range(n):  
        solver.Add(sum(x[i][j] for j in range(n)) == 1) 
        solver.Add(sum(x[j][i] for j in range(n)) == 1) 
        solver.Add(x[i][i] == 0)

    # Subtour constraint: The key to the elimination is to realize that 
    # for any strict subset of the nodes, 
    # the number of chosen arcs must be less than the number of nodes
    if subtours:
        for sub in subtours:
            k = [x[sub[i]][sub[j]] + x[sub[j]][sub[i]] for i in range(len(sub) - 1) for j in range(i + 1, len(sub))]
            solver.Add(sum(k) <= len(sub) - 1)

    # Objective function
    solver.Minimize(solver.Sum(x[i][j] * (0 if distance_matrix[i][j] is None else distance_matrix[i][j]) for i in range(n) for j in range(n))) 
    status = solver.Solve()
    tours = extract_tours(x, n) 
    print("Tours:", tours)
    return status, solver.Objective().Value(), tours

def extract_tours(solution, n):
    node = 0
    tours = [[0]]
    all_nodes = [0] + [1] * (n - 1)

    while sum(all_nodes) > 0:
        for i in range(n):
            if solution[node][i] == 1:
                next_node = i
                break

        if next_node not in tours[-1]:
            tours[-1].append(next_node)
            node = next_node
        else:
            node = all_nodes.index(1)
            tours.append([node])

        all_nodes[node] = 0

    return tours

def solve_model(distance_matrix):
    subtours = []
    tours = []

    while len(tours) != 1:
        status, value, tours = solve_model_eliminate(distance_matrix, subtours)
        if status == pywraplp.Solver.OPTIMAL:
            subtours.extend(tours)

    return status, value, tours[0]

def main():
  distance_matrix = [[0, 64, 229, 109, 378, 110, 201, 304, 346],
  [64, 0, 280, 161, 370, 266, 253, 360, 398],
  [229, 280, 0, 124, 205, 53, 86, 86, 120],
  [109, 161, 124, 0, 277, 110, 96, 204, 241],
  [378, 370, 205, 277, 0, 167, 262, 248, 146],
  [110, 266, 53, 110, 167, 0, 97, 144, 142],
  [201, 253, 86, 96, 262, 97, 0, 170, 204],
  [304, 360, 86, 204, 248, 144, 170, 0, 104],
  [346, 398, 120, 241, 146, 142, 204, 104, 0],
    ]
  status, value, tour = solve_model(distance_matrix)

  if status == pywraplp.Solver.OPTIMAL:
      print('Objective value =', value)
      print('Solution =', tour)
  else:
      print('The problem does not have an optimal solution.')

main()
