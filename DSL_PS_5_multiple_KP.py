from ortools.linear_solver import pywraplp

def create_data_model():
    data = {}
    weights = [48, 30, 42, 36, 36, 48, 42, 42, 36, 24, 30, 30, 42, 36, 36]
    values = [10, 30, 25, 50, 35, 30, 15, 40, 30, 35, 45, 10, 20, 30, 25]
    data['weights'] = weights
    data['values'] = values
    data['items'] = list(range(len(weights)))
    data['num_items'] = len(weights)
    num_bins = 5
    data['bins'] = list(range(num_bins))
    data['bin_capacities'] = [100, 100, 100, 100, 100]
    return data

def main():
    data = create_data_model()

    # Create the mip solver with the SCIP backend.
    solver = pywraplp.Solver.CreateSolver('SCIP')

    # Variables.
    # x[i, j] = 1 if item i is packed in bin j.
    x = {}
    for i in data['items']:
        for j in data['bins']:
            x[(i, j)] = solver.IntVar(0, 1, 'x_%i_%i' % (i, j))

    # Constraints.
    # Each item is assigned to at most one bin.
    for i in data['items']:
        solver.Add(sum(x[i, j] for j in data['bins']) <= 1)

    # The amount packed in each bin cannot exceed its capacity.
    for j in data['bins']:
        solver.Add(
            sum(x[(i, j)] * data['weights'][i]
                for i in data['items']) <= data['bin_capacities'][j])

    # Objective.
    # Maximize total value of packed items.
    objective = solver.Objective()
    for i in data['items']:
        for j in data['bins']:
            objective.SetCoefficient(x[i, j], data['values'][i])
    objective.SetMaximization()

    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print(f'Total packed value: {objective.Value()}')
        total_weight = 0
        for j in data['bins']:
            print(f'Bin {j}')
            bin_weight = 0
            bin_value = 0
            packed_items = [] #This was missing in the old correction
            for i in data['items']:
                if x[i, j].solution_value() > 0:
                    packed_items.append(i)
                    bin_weight += data['weights'][i]
                    bin_value += data['values'][i]
            print(f'Packed items: {packed_items}')
            print(f'Packed bin weight: {bin_weight}')
            print(f'Packed bin value: {bin_value}\n')
            total_weight += bin_weight
        print(f'Total packed weight: {total_weight}')
    else:
        print('The problem does not have an optimal solution.')

if __name__ == '__main__':
    main()



if __name__ == '__main__':
    main()