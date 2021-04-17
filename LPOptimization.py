import numpy as np
from docplex.mp.model import Model
import OneWay as network

def Solve(T, C, Cg, Cs, CI, Q, N, D, W, AC, X0, state_occupance = True):
    
    if state_occupance:
        X0 = X0 * N
    
    
    #define the cplex model
    model = Model("Traffic metering")
    
    
    #define variables 
    occupancy = [(t, i) for i in C for t in range(T)]
    flow = [(t, k[0], k[1]) for k in AC for t in range(T-1)]
    
    x = model.continuous_var_dict(occupancy, lb=0,  name='x')
    y = model.continuous_var_dict(flow, lb=0, name = 'y')
    
    #define the objective function 
    model.maximize(model.sum(x[t, i] for i in Cs for t in range(T)))
    
    #cons01: flow conservation constraint for all cells except gates
    model.add_constraints( [x [t+1, i] == x[t, i] + model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) - model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i)
                            for i in C if i not in Cg for t in range(T-1)], "Cons01_")
    
    #cons02: flow conservation constraint for gates
    model.add_constraints( [x[t+1, i] == x[t, i] + D[t, ind] - model.sum(y[t, k[0], k[1]]
                         for k in AC if k[0]==i) for ind, i in enumerate(Cg) for t in range(T-1)], "Cons02_")    
     
    #cons03: y<x
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= x[t, i] 
                           for t in range(T-1) for i in C if i not in Cs], "Cons03_")
    
    #cons04: y<N-x
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) <= N[ind]-x[t, i] 
                           for t in range(T-1) for ind, i in enumerate(C) if i not in Cg], "Cons04_")
    
    #cons05: y<Qi
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= Q[ind]
                            for t in range(T-1) for ind, i in enumerate(C) if i not in Cs], "Cons05_")
    
    #cons06: y<Qj
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) <= Q[ind]
                            for t in range(T-1) for ind, i in enumerate(C) if i not in Cg], "Cons06_")
    
    #cons07: y<W Q
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= W[t, ind] * Q[ind]
                            for t in range(T-1) for ind, i in enumerate(CI)], "Cons07_")
    
    #cons08; X0
    model.add_constraints([x[0, i] == X0[ind] for ind, i in enumerate(C)], "Cons08_")
    
    solution = model.solve(log_output=True)
    model.export("./model_export")
    
    print(solution.solve_status) 
    
    return 
    
    
NET = network.info()
X0 = np.zeros(len(NET.C))

solution = Solve(10, NET.C, NET.Cg, NET.Cs, NET.CI, NET.Q, NET.N, NET.D, NET.W, NET.AC, X0)
print(solution)
    
    
    
    

