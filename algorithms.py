import heapq

class TSPSolver:
    def __init__(self, cities, distances):
        self.cities = cities
        self.distances = distances

    def depth_first_search(self, start_city):
        def dfs(current_city, visited, path, cost):
            if len(visited) == len(self.cities):
                return path + [start_city], cost + self.distances[current_city][start_city]
            
            min_path = None
            min_cost = float('inf')
            for neighbor in range(len(self.cities)):
                if neighbor not in visited:
                    new_path, new_cost = dfs(neighbor, visited | {neighbor}, path + [neighbor], cost + self.distances[current_city][neighbor])
                    if new_cost < min_cost:
                        min_path, min_cost = new_path, new_cost
            return min_path, min_cost

        path, cost = dfs(start_city, {start_city}, [start_city], 0)
        return path, cost

    def uniform_cost_search(self, start_city):
        queue = [(0, start_city, [start_city])]
        min_cost = float('inf')
        best_path = None

        while queue:
            cost, current_city, path = heapq.heappop(queue)
            if len(path) == len(self.cities):
                cost += self.distances[current_city][start_city]
                if cost < min_cost:
                    min_cost = cost
                    best_path = path + [start_city]
            else:
                for neighbor in range(len(self.cities)):
                    if neighbor not in path:
                        heapq.heappush(queue, (cost + self.distances[current_city][neighbor], neighbor, path + [neighbor]))
        
        return best_path, min_cost

    def a_star_search(self, start_city):
        def heuristic(current_city, remaining_cities):
            # Simple heuristic: use the minimum distance to any remaining city
            if not remaining_cities:
                return 0
            return min(self.distances[current_city][city] for city in remaining_cities)

        queue = [(0 + heuristic(start_city, set(range(len(self.cities))) - {start_city}), 0, start_city, [start_city])]
        min_cost = float('inf')
        best_path = None

        while queue:
            estimated_cost, cost, current_city, path = heapq.heappop(queue)
            if len(path) == len(self.cities):
                cost += self.distances[current_city][start_city]
                if cost < min_cost:
                    min_cost = cost
                    best_path = path + [start_city]
            else:
                remaining_cities = set(range(len(self.cities))) - set(path)
                for neighbor in remaining_cities:
                    new_cost = cost + self.distances[current_city][neighbor]
                    heapq.heappush(queue, (new_cost + heuristic(neighbor, remaining_cities - {neighbor}), new_cost, neighbor, path + [neighbor]))
        
        return best_path, min_cost


if __name__ == "__main__":
    # Example usage
    cities = ["A", "B", "C", "D"]
    distances = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]

    solver = TSPSolver(cities, distances)

    start_city = 0  # Assuming starting city is "A"

    print("Depth-First Search:")
    path, cost = solver.depth_first_search(start_city)
    print("Path:", [cities[i] for i in path])
    print("Cost:", cost)

    print("\nUniform-Cost Search:")
    path, cost = solver.uniform_cost_search(start_city)
    print("Path:", [cities[i] for i in path])
    print("Cost:", cost)

    print("\nA* Search:")
    path, cost = solver.a_star_search(start_city)
    print("Path:", [cities[i] for i in path])
    print("Cost:", cost)
