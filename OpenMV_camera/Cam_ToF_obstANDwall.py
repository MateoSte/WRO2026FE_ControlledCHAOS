import tof
import time

tof.init()

# -----------------------------
# Postavke
# -----------------------------

# Centralni dio 4x4 za određivanje glavne udaljenosti
WALL_START = 2
WALL_END = 6

# Prepreka mora biti barem toliko bliža
# od glavne površine
OBSTACLE_RATIO = 0.80

# Minimalan broj susjednih zona za prepreku
MIN_CLUSTER_SIZE = 3

previous_wall = None


# -----------------------------
# Pomoćne funkcije
# -----------------------------

def get_wall_distance(depth):
    values = []

    for y in range(WALL_START, WALL_END):
        for x in range(WALL_START, WALL_END):

            value = depth[y * 8 + x]

            if value > 0:
                values.append(value)

    if len(values) == 0:
        return None

    # Sortiramo vrijednosti
    values.sort()

    # Odbacimo 15% s obje strane
    # da jedan loš podatak ne pokvari AVG
    count = len(values)

    start = int(count * 0.15)
    end = int(count * 0.85)

    filtered = values[start:end]

    if len(filtered) == 0:
        return None

    return sum(filtered) / len(filtered)


def find_obstacle(depth, wall_distance):

    if wall_distance is None:
        return None

    # Vrijednost mora biti dovoljno manja
    # od glavne udaljenosti
    limit = wall_distance * OBSTACLE_RATIO

    # Matrica koja označava potencijalne prepreke
    obstacle = [[False for x in range(8)] for y in range(8)]

    for y in range(8):
        for x in range(8):

            value = depth[y * 8 + x]

            if value > 0 and value < limit:
                obstacle[y][x] = True

    # -----------------------------------
    # Tražimo povezane zone
    # -----------------------------------

    visited = [[False for x in range(8)] for y in range(8)]

    clusters = []

    for y in range(8):
        for x in range(8):

            if not obstacle[y][x] or visited[y][x]:
                continue

            # BFS za pronalazak povezanog klastera
            queue = [(x, y)]
            visited[y][x] = True

            cluster = []

            while len(queue) > 0:

                cx, cy = queue.pop(0)

                cluster.append((cx, cy))

                # Gledamo 4 susjedne zone
                neighbours = [
                    (cx + 1, cy),
                    (cx - 1, cy),
                    (cx, cy + 1),
                    (cx, cy - 1)
                ]

                for nx, ny in neighbours:

                    if nx < 0 or nx >= 8:
                        continue

                    if ny < 0 or ny >= 8:
                        continue

                    if visited[ny][nx]:
                        continue

                    if not obstacle[ny][nx]:
                        continue

                    visited[ny][nx] = True
                    queue.append((nx, ny))

            clusters.append(cluster)

    # -----------------------------------
    # Tražimo dovoljno velik klaster
    # -----------------------------------

    best_cluster = None

    for cluster in clusters:

        if len(cluster) >= MIN_CLUSTER_SIZE:

            if best_cluster is None:
                best_cluster = cluster

            elif len(cluster) > len(best_cluster):
                best_cluster = cluster

    if best_cluster is None:
        return None

    # -----------------------------------
    # Izračun udaljenosti prepreke
    # -----------------------------------

    obstacle_values = []

    for x, y in best_cluster:

        value = depth[y * 8 + x]

        if value > 0:
            obstacle_values.append(value)

    if len(obstacle_values) == 0:
        return None

    return sum(obstacle_values) / len(obstacle_values)


# -----------------------------
# Glavna petlja
# -----------------------------

while True:

    depth, depth_min, depth_max = tof.read_depth()

    # Glavna udaljenost
    wall_distance = get_wall_distance(depth)

    # Prepreka
    obstacle_distance = find_obstacle(
        depth,
        wall_distance
    )

    # -----------------------------
    # Ispis
    # -----------------------------

    if wall_distance is not None:

        print(
            "Wall:",
            round(wall_distance),
            "mm"
        )

    else:

        print("Wall: nema mjerenja")

    if obstacle_distance is not None:

        print(
            "OBSTACLE:",
            round(obstacle_distance),
            "mm"
        )

    else:

        print("Obstacle: nema")

    print("--------------------")

    time.sleep_ms(100)
