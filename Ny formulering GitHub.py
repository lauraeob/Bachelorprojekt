from itertools import permutations
import numpy as np
import gurobipy as gp
from gurobipy import Model, GRB
import time

#Læs filen
def read_instance(file):
    with open(file, "r") as f:
        lines = f.readlines()

    vehicles = []
    customers = []
    coords = {}
    params = {"Qk": {}, "Ak": {}, "A": {}, "Q": 0, "Tu": {}, "Tk": {}}

    section = None
    capacities = []

    for line in lines:
        parts = line.strip().split()
        if not parts:
            continue

        if "VEHICLE SECTION" in line:
            section = "vehicles"
            continue
        elif "CUSTOMER SECTION" in line:
            section = "customers"
            continue
        elif "TIME UB:" in line:
            params["Tu"] = int(parts[-1])
            continue

        if section == "vehicles" and parts[0].isdigit():
            k = "k" + parts[0]
            vehicles.append(k)
            coords[k] = (float(parts[1]), float(parts[2]))
            capacity = int(parts[3])
            capacities.append(capacity)
            params["Qk"][k] = int(parts[4])
            params["Ak"][k] = int(parts[5])

        elif section == "customers" and parts[0].isdigit():
            customer = "p" + parts[0]
            customers.append(customer)
            coords[customer] = (float(parts[1]), float(parts[2]))
            params["A"][customer] = int(parts[-1])

    if capacities:
        params["Q"] = max(capacities)

    for k in vehicles:
        params["Tk"][k] = 0

    if "p0" in customers:
        customers.remove("p0")
        coords["d"] = coords.pop("p0")
        params["A"].pop("p0", None)

    return vehicles, customers, coords, params



# Afstand funktion
def Euclidian_distance(punkt1, punkt2):
    return int(round(np.linalg.norm(np.array(punkt2) - np.array(punkt1))))


# Regn afstand mellem punkter og brug som omkostning
def cost_function(coords, vehicles, customers, destination="d"):
    cost = {}
    nodes = list(coords.keys())

    for i in nodes:
        for j in nodes:
            x_i, y_i = map(float, coords[i])
            x_j, y_j = map(float, coords[j])
            if i == j:
                cost[(i, j)] = 0
            else:
                cost[(i, j)] = Euclidian_distance(coords[i], coords[j])

    return cost



#Generér de brugbare ruter
def generate_valid_routes(vehicles, customers, coords, params, C):
    valid_routes = {}

    for k in vehicles:
        start = k
        end = "d"
        max_capacity = params["Q"] - params["Qk"][k]
        max_time = params["Ak"][k]

        possible_customers = [p for p in customers if C[start, p] + C[p, end] <= max_time]

        valid_routes[k] = []

        if C[start, end] <= max_time:
            valid_routes[k].append((start, end))


        for r_length in range(1, min(len(possible_customers), max_capacity) + 1):
            for r in permutations(possible_customers, r_length):
                total_time = sum(C[r[i], r[i + 1]] for i in range(len(r) - 1)) + C[start, r[0]] + C[r[-1], end]

                max_customer_time = min(params["A"].get(i, float('inf')) for i in r)
                if params["Tk"][k] + total_time <= min(max_time, max_customer_time):
                    valid_routes[k].append((start,) + r + (end,))

    return valid_routes

#Definer modellen
def Rute_baseret_model(vehicles, valid_routes, params, C):
    model = Model("Rute_baseret_model")
    v = {}

#Beslutningsvariable
    for k in vehicles:
        for t, route in enumerate(valid_routes[k]):
            v[k, t] = model.addVar(lb=0,ub=1,vtype=GRB.CONTINUOUS)

#Objektfunktion
    model.setObjective(
        gp.quicksum(
            sum(C[route[i], route[i + 1]] for i in range(len(route) - 1)) * v[k, t]
            for k in K
            for t, route in enumerate(valid_routes[k])
        ) -
        gp.quicksum(
            2 * C[i, d] * v[k, t]
            for k in K
            for t, route in enumerate(valid_routes[k])
            for i in route if i in P
        ),
        GRB.MINIMIZE
    )

#C1_2
    for k in vehicles:
        model.addConstr(sum(v[k, t] for t in range(len(valid_routes[k]))) <= 1)

        #C5_2
        if params["Qk"][k] > 0:
            model.addConstr(sum(v[k, t] for t in range(len(valid_routes[k]))) >= 1)

#C4_2
    for k in vehicles:
        for t, route in enumerate(valid_routes[k]):
            total_time = sum(C[route[i], route[i + 1]] for i in range(len(route) - 1))
            model.addConstr(params["Tk"][k] + total_time * v[k, t] <= min(params["Ak"][k], min(
                params["A"].get(i, float('inf')) for i in route)))

#C3_2
    for p in P:
        model.addConstr(
            gp.quicksum(v[k, t] for k in K for t, route in enumerate(valid_routes[k]) if p in route) <= 1)

    return model, v

start_tid=time.time()

#Filen
file = "/Users/lauraobers/Documents/Mat-øk/3. År/Bachelor/Data/Large Instances/P-n14-k4.txt"

#Indlæs filen og regn distancerne
vehicles, customers, coords, params = read_instance(file)

C = cost_function(coords, vehicles, customers, destination="d")
T = C.copy()
K = vehicles
P = customers
d = "d"
I = K + P
J = P + [d]


valid_routes = generate_valid_routes(K, P, coords, params, C)
model, v = Rute_baseret_model(K, valid_routes, params, C)
model.optimize()


print("Antal variable:", model.NumVars)
print("Antal bibetingelser:", model.NumConstrs)

#Optimale beslutningsvariable
v_values = {key: var.x for key, var in v.items()}

# Ruterne
vehicle_routes = {k: [] for k in K}

for (k, t), value in v_values.items():
    if value > 0.5:
        route = valid_routes[k][t]
        vehicle_routes[k].extend(zip(route[:-1], route[1:]))

ordered_routes = {}
for k, arcs in vehicle_routes.items():
    if not arcs:
        continue
    route = [k]
    current = k

    while current != "d":
        next_stop = next((j for i, j in arcs if i == current), None)
        if next_stop is None:
            break
        route.append(next_stop)
        current = next_stop

    ordered_routes[k] = route

#Print ruterne
for k, route in ordered_routes.items():
    print(f"Bil {k}'s rute: {' -> '.join(route)}")


slut_tid = time.time()

print(f"Samlet eksekveringstid: {slut_tid - start_tid} sekunder")
