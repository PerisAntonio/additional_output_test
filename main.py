from ortools.linear_solver import pywraplp
import itertools
import json
import paint

def is_cycle(r):
    return len([i for i in range(len(r)-1) if r[i][1]==r[0][0]])>0

def read(input_data):

    values={}
    for a in input_data['edges']:
        for b in input_data['edges'][a]:
            n=input_data['edges'][a][b]
            for c in n:
                if(c not in values):
                    values[c]={}
                values[c][(int(a),int(b))]=n[c]
                values[c][(int(b),int(a))]=n[c]
    nodes=[int(x) for x in list(input_data['edges'].keys())]
    optimization_parameter=input_data['optimization_parameter']
    constraints=[]
    if('constraints' in input_data):
        for c in input_data['constraints']:
            constraints.append(c)
        
    return values,nodes,optimization_parameter,constraints

def calculate(values,nodes,optimization_parameter,constraints):
    cycles=[]
    optimal=False
    solver = pywraplp.Solver.CreateSolver('SAT')

    # Binary variables for each edge
    x = {}
    for i in nodes:
        for j in nodes:
            if i != j:
                x[i, j] = solver.BoolVar(f'x[{i},{j}]')

    # Goal function
    solver.Minimize(solver.Sum(x[i, j] * values[optimization_parameter] [i, j] for i, j in values[optimization_parameter] if i!=j))

    # Restrictions
    for i in nodes:
        solver.Add(solver.Sum(x[i, j] for j in nodes if i != j) == 1)  # going out of i
        solver.Add(solver.Sum(x[j, i] for j in nodes if i != j) == 1)  # going to i


    for constraint in constraints:
        p=constraint[0]
        if(constraint[1]=='<='):
            solver.Add(solver.Sum(x[i, j] * values[p] [i, j] for i, j in values[p] if i!=j)<=float(constraint[2]))
        elif(constraint[1]=='>='):
            solver.Add(solver.Sum(x[i, j] * values[p] [i, j] for i, j in values[p] if i!=j)>=float(constraint[2]))


    for current in []:
        print(current)
        for p in itertools.permutations(nodes,current):
                    ct=solver.Constraint(0,i-1,"ct")
                    for j in range(current):
                        ct.SetCoefficient(x[p[j],p[(j+1)%current]],1)    
    current=2
    while(optimal==False):
        
            
        for p in itertools.permutations(nodes,current):
            ct=solver.Constraint(0,current-1,"ct")
            for j in range(current):
                ct.SetCoefficient(x[p[j],p[(j+1)%current]],1)
        current=current+1
        # Solving the problem
        status = solver.Solve()

        # Results
        if status == pywraplp.Solver.OPTIMAL:        
            route = []
            actual_node = 0
            while len(route) < len(nodes):
                for j in nodes:
                    if actual_node!=j and x[actual_node, j].solution_value() == 1:
                        route.append([actual_node, j])
                        actual_node = j
                        break
            if is_cycle(route)==False:
                optimal=True
        else:
            optimal=True
            route=None
        if(current>(len(nodes)//2+1)):
            optimal=True
            route=None
    ret={}
    ret['route']=route
    for c in values:
        ret[c]=sum([values[c][(x[0],x[1])] for x in route])
    return ret


def run(input_data, solver_params, extra_arguments):
    values,nodes,optimization_parameter,original_constraints=read(input_data)
    constraints=[]
    for c in original_constraints:
        if('%' in c[2]):
            r=calculate(values,nodes,c[0],[])
            p=float(c[2][:-1])/100.
            constraints.append([c[0],c[1],r[c[0]]*p])
        else:
            constraints.append(c)
    r=calculate(values,nodes,optimization_parameter,constraints)
    result={}
    benchmark=[]
    for c in r.keys():
        if(c == 'route'):
            result['route']=r['route']
        else:
            b={}
            b['name']=c
            b['value']=r[c]
            benchmark.append(b)

    result['benchmark']=benchmark
    p=paint.Painter()
    p.paint(values,'consumption',[x[0] for x in r['route']]+[r['route'][0][0]])
    return result
