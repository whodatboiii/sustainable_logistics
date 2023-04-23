from __future__ import print_function
from ortools.sat.python import cp_model as cp
import math, sys

def main():
  """Model for the Warehouse Location Problem"""
  # Model 
  model = cp.CpModel()

  
  # Data declaration.
  p = 3 #number of warehouses

  num_customers = 6
  customers = list(range(num_customers))

  num_warehouses = 4
  warehouses = list(range(num_warehouses))


  demand = [100, 80, 80, 70, 20, 30]
  distance = [[2, 10, 50, 20],
              [2, 10, 52, 30],
              [50, 60, 3, 10],
              [40, 60, 1, 18],
              [30, 20, 10, 5],
              [15, 30, 20, 4]
              ]

  # Variable declaration.
  open = [model.NewIntVar(0,num_warehouses, 'open[%i]% % i') for w in warehouses]
  x = {}
  for c in customers:
    for w in warehouses:
      x[c, w] = model.NewIntVar(0, 1, 'x[%i,%i]' % (c, w)) 
      #Only the boundaries are set at this step

  z = model.NewIntVar(0, 1000, 'z')

  # Constraints.
  model.Add(z == sum([
      demand[c] * distance[c][w] * x[c, w]
      for c in customers
      for w in warehouses
  ]))

  #"z" variable represents the total distance between 
  # customers and open warehouses, where the distance 
  # between each customer and warehouse is 
  # weighted by the demand of the customer.

  for c in customers: #each customer must be the client of 1 warehouse
    model.Add(sum([x[c, w] for w in warehouses]) == 1)

  model.Add(sum(open) == p) # the number of open warehouses does not exceed the capacity limit "p"

  for c in customers: #each customer is assigned to an open warehouse
    for w in warehouses:
      model.Add(x[c, w] <= open[w])

  # Objective function.
  model.Minimize(z) #we want to minimize that distance

  # Model solution.
  solver = cp.CpSolver()
  status = solver.Solve(model)

  if status == cp.OPTIMAL:
    print('z:', solver.Value(z))
    print('open:', [solver.Value(open[w]) for w in warehouses])
    for c in customers:
      for w in warehouses:
        print(solver.Value(x[c, w]), end=' ')
      print()
    print()

  print('WallTime:', solver.WallTime())

if __name__ == '__main__':
  main()

