import numpy as np
from docplex.mp.model import Model
import OneWay as network

def Solve(T, C, Cg, Cs, CI, Q, N, D, W, AC, X0, state_occupance = True, printsol=False):
    
    if state_occupance:
        X0 = X0 * N
        
    #define the model
    model = Model("Traffic metering")
    
    #define variables 
    occupancy = [(t, i) for i in C for t in range(T)]
    flow = [(t, int(k[0]), int(k[1])) for k in AC for t in range(T-1)]
    
    x = model.continuous_var_dict(occupancy, lb=0,  name='x')
    y = model.continuous_var_dict(flow, lb=0, name = 'y')
    
    #define the objective function 
    model.maximize(model.sum(x[t, i] for i in Cs for t in range(T)))
    
    #cons01: flow conservation constraint for all cells except gates and sinks
    model.add_constraints( [x [t+1, i] == x[t, i] + model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) - model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i)
                            for i in C if i not in np.append(Cg, Cs) for t in range(T-1)], "Cons01_")
    
    #cons02: flow conservation constraint for gates
    model.add_constraints( [x[t+1, i] == x[t, i] + D[t, ind] - model.sum(y[t, k[0], k[1]]
                         for k in AC if k[0]==i) for ind, i in enumerate(Cg) for t in range(T-1)], "Cons02_")    
    
    #cons03: flow conservation constraint for sinks
    model.add_constraints( [x [t+1, i] == model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) 
                            for i in Cs for t in range(T-1)], "Cons03_") 
     
    #cons04: y<x
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= x[t, i] 
                           for t in range(T-1) for i in C if i not in Cs], "Cons04_")
    
    #cons05: y<N-x
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) <= N[ind]-x[t, i] 
                           for t in range(T-1) for ind, i in enumerate(C) if i not in Cg], "Cons05_")
    
    #cons06: y<Qi
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= Q[ind]
                            for t in range(T-1) for ind, i in enumerate(C) if i not in Cs], "Cons06_")
    
    #cons07: y<Qj
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[1]==i) <= Q[ind]
                            for t in range(T-1) for ind, i in enumerate(C) if i not in Cg], "Cons07_")
    
    #cons08: y<W Q
    model.add_constraints([model.sum(y[t, k[0], k[1]] for k in AC if k[0]==i) <= W[t, ind] * Q[ind]
                            for t in range(T-1) for ind, i in enumerate(CI)], "Cons08_")
    
    #cons09: y<by
    model.add_constraints([y[t, k[0], k[1]] == k[3] * model.sum(y[t, m[0], m[1]] for m in AC if m[0]==k[0])
                            for t in range(T-1) for k in AC if k[2]==3], "Cons09_")
    
    #cons10; X0
    model.add_constraints([x[0, i] == X0[ind] for ind, i in enumerate(C)], "Cons10_")
    
    solution = model.solve(log_output=True)
    
    #print solutions
    if printsol:
        model.export("./model_export")
        print("\n\n")
        for t in range(T):
            for i in C:
                print("%.2f\t" % solution.get_value(x[t, i]), end="")
            print()
        print("\n\n")
            
        for i in AC:
            print("%d\t%d\t" % (i[0], i[1]), end="")
            for t in range(T-1):
                print("%.2f\t" % solution.get_value(y[t, i[0], i[1]]), end="")
            print()
        print("\n\n")   
    
    return (solution.get_value_dict(x), solution.get_value_dict(y), solution.get_objective_value()) 
    
NET = network.info()
X0 = np.zeros(len(NET.C))

x, y, z = Solve(30, NET.C, NET.Cg, NET.Cs, NET.CI, NET.Q, NET.N, NET.D, NET.W, NET.AC, X0, False, True)

print(x[1,1])

    
    
    
    

