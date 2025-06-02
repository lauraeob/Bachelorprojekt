import numpy as np
import gurobipy as gp
from gurobipy import Model, GRB

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


# Filen
file = "/Users/lauraobers/Documents/Mat-øk/3. År/Bachelor/Data/Large Instances/E-n17-k5.txt"

# Indlæs filen og regn distancerne
vehicles, customers, coords, params = read_instance(file)
C = cost_function(coords, vehicles, customers, destination="d")
T = C.copy()

# Mængder
K = vehicles
P = customers
d = "d"
I = K + P
J = P + [d]

# Lav modellen
m = Model("Arc_baseret_model")

# Beslutningsvariable
x = m.addVars(I, J, K, lb=0, ub=1, vtype=GRB.BINARY, name="x")
q = m.addVars(I, lb=0, vtype=GRB.INTEGER, name="q")

# Objektfunktionen
objective = gp.quicksum(
    (C[(i, j)] - (2 * C[(i, "d")]) * (i in P)) * x[i, j, k]
    for i in I
    for j in J
    for k in K
)

#Indsæt objektfunktionen
m.setObjective(objective, GRB.MINIMIZE)

#Bibetingelser
# C1_1
for k in K:
    m.addConstr(gp.quicksum(x[k, j, k] for j in J) <= 1)

# C2_1
for k in K:
    m.addConstr(gp.quicksum(x[k, j, k] for j in J) == gp.quicksum(x[i, "d", k] for i in I))

# C3_1
for j in P:
    for k in K:
        m.addConstr(gp.quicksum(x[i, j, k] for i in I) == gp.quicksum(x[j, i, k] for i in J))

# C4_1
for i in I:
    for j in P:
        if i != j:
            m.addConstr(q[j] >= q[i] - (len(P) + 1) * (1 - gp.quicksum(x[i, j, k] for k in K)) + 1)

# C5_1
for i in P:
    m.addConstr(gp.quicksum(x[i, j, k] for j in J for k in K) <= 1)

# C6_1
for k in K:
    m.addConstr(gp.quicksum(x[i, j, k] for i in P for j in J) <= params["Q"] - params["Qk"][k])

# C7_1
for k in K:
    m.addConstr(params["Qk"][k] <= gp.quicksum(params["Qk"][k] * x[k, j, k] for j in J))

# C8_1

for i in P:
    for k in K:
        lhs = (params["Tk"][k] + gp.quicksum(T[i_prime, j] * x[i_prime, j, k] for i_prime in [k] + P for j in J if i_prime != j))
        rhs = min(params["A"][i], params["Ak"][k]) + (params["Ak"][k]-min(params["A"][i], params["Ak"][k])) * (1 - gp.quicksum(x[j, i, k] for j in P + [k]))
        m.addConstr(lhs <= rhs)

# C9_1
for i in I:
    max_value = min(len(P), max(params["Q"] - params["Qk"][k] for k in K))
    m.addConstr(q[i] <= max_value)

# C11_1
for i in I:
    for j in J:
        for k in K:
            if i == j:
                m.addConstr(x[i, j, k] == 0)



# Løs modellen
m.update()
m.optimize()

print("Antal variable:", m.NumVars)
print("Antal bibetingelser:", m.NumConstrs)


# De optimale beslutningsvariable
x_values = {}
for i in I:
    for j in J:
        for k in K:
            if x[i, j, k].X == 1:
                x_values[(i, j, k)] = x[i, j, k].X


# Ruterne
vehicle_routes = {k: [] for k in K}

for (i, j, k), value in x_values.items():
    if value > 0.5:
        vehicle_routes[k].append((i, j))

ordered_routes = {}
for k, arcs in vehicle_routes.items():
    if not arcs:
        continue
    route = []
    current = k
    while True:
        next_arc = next((j for i, j in arcs if i == current), None)
        if next_arc is None:
            break
        route.append(next_arc)
        current = next_arc
    ordered_routes[k] = route

# Print ruterne
for k, route in ordered_routes.items():
    print(f"Bil {k}'s rute: {k} -> {' -> '.join(route)}")


