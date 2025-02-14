#!/usr/bin/env python3
"""
Smart Traffic Management System
--------------------------------
This Flask application simulates a traffic management system for Brampton, Ontario,
using a graph of over 50 intersections (plazas, malls, attractions, etc.) with approximate
pixel-based coordinates. Roads (edges) are automatically created between nodes that are within
a specified distance threshold (350 pixels). The A* algorithm computes routes.
Endpoints support normal routing and emergency vehicle routing.
"""

import math
import heapq
from flask import Flask, render_template, jsonify, request
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, field

app = Flask(__name__)

# ----------------------------
# Graph Implementation
# ----------------------------
class Graph:
    def __init__(self) -> None:
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.edges: Dict[str, List[Tuple[str, float]]] = {}

    def add_node(self, node_id: str, x: float, y: float) -> None:
        self.nodes[node_id] = (x, y)
        if node_id not in self.edges:
            self.edges[node_id] = []

    def add_edge(self, from_node: str, to_node: str, weight: float, bidirectional: bool = True) -> None:
        if from_node not in self.nodes or to_node not in self.nodes:
            raise ValueError("Both nodes must be added before adding an edge.")
        self.edges[from_node].append((to_node, weight))
        if bidirectional:
            self.edges[to_node].append((from_node, weight))

    def get_neighbors(self, node_id: str) -> List[Tuple[str, float]]:
        return self.edges.get(node_id, [])

    def get_coordinates(self, node_id: str) -> Tuple[float, float]:
        return self.nodes[node_id]

    def heuristic(self, node_id: str, goal_id: str) -> float:
        x1, y1 = self.get_coordinates(node_id)
        x2, y2 = self.get_coordinates(goal_id)
        return math.hypot(x2 - x1, y2 - y1)

# ----------------------------
# A* Algorithm Implementation
# ----------------------------
class AStar:
    @staticmethod
    def reconstruct_path(came_from: Dict[str, str], current: str) -> List[str]:
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    @staticmethod
    def search(graph: Graph, start: str, goal: str) -> Optional[List[str]]:
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from: Dict[str, str] = {}
        g_score = {node: math.inf for node in graph.nodes}
        g_score[start] = 0
        f_score = {node: math.inf for node in graph.nodes}
        f_score[start] = graph.heuristic(start, goal)
        open_set_hash = {start}

        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            if current == goal:
                return AStar.reconstruct_path(came_from, current)
            for neighbor, weight in graph.get_neighbors(current):
                tentative = g_score[current] + weight
                if tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + graph.heuristic(neighbor, goal)
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        return None  # No path found

# ----------------------------
# Emergency Vehicle Management
# ----------------------------
@dataclass(order=True)
class EmergencyVehicle:
    priority: int
    vehicle_id: str = field(compare=False)
    source: str = field(compare=False)
    destination: str = field(compare=False)

class EmergencyVehicleManager:
    def __init__(self) -> None:
        self._queue: List[EmergencyVehicle] = []

    def add_vehicle(self, vehicle: EmergencyVehicle) -> None:
        heapq.heappush(self._queue, vehicle)
        print(f"Added emergency vehicle '{vehicle.vehicle_id}' with priority {vehicle.priority}.")

    def process_all(self, graph: Graph) -> List[Tuple[EmergencyVehicle, List[str]]]:
        results = []
        while self._queue:
            vehicle = heapq.heappop(self._queue)
            path = AStar.search(graph, vehicle.source, vehicle.destination)
            results.append((vehicle, path if path is not None else []))
        return results

# ----------------------------
# Traffic Management System Integration
# ----------------------------
class TrafficManagementSystem:
    def __init__(self) -> None:
        self.graph = Graph()
        self.emergency_manager = EmergencyVehicleManager()

    def add_intersection(self, node_id: str, x: float, y: float) -> None:
        self.graph.add_node(node_id, x, y)

    def add_road(self, from_node: str, to_node: str, weight: float, bidirectional: bool = True) -> None:
        self.graph.add_edge(from_node, to_node, weight, bidirectional)

    def compute_route(self, source: str, destination: str) -> Optional[List[str]]:
        return AStar.search(self.graph, source, destination)

    def add_emergency_vehicle(self, vehicle_id: str, source: str, destination: str, priority: int) -> None:
        ev = EmergencyVehicle(priority, vehicle_id, source, destination)
        self.emergency_manager.add_vehicle(ev)

    def process_emergency_requests(self) -> List[Tuple[EmergencyVehicle, List[str]]]:
        return self.emergency_manager.process_all(self.graph)

tms = TrafficManagementSystem()

def build_sample_network(tms: TrafficManagementSystem) -> None:
    # Define over 50 intersections with approximate pixel coordinates.
    intersections = {
        "Downtown Brampton": (100, 100),
        "City Centre": (250, 120),
        "Main & Queen": (400, 150),
        "Civic Centre": (550, 180),
        "Bramalea Mall": (100, 250),
        "Gore Meadows": (250, 260),
        "Central Park": (400, 280),
        "Northgate": (550, 300),
        "Chinguacousy Park": (150, 400),
        "Heart Lake": (300, 420),
        "Mount Pleasant": (450, 440),
        "Rose Theatre": (600, 460),
        "Professor's Park": (150, 550),
        "Landmark Centre": (300, 570),
        "Brampton Conservation": (450, 590),
        "Brampton Gateway": (600, 610),
        "Shoppers Plaza": (700, 130),
        "Westwood Mall": (750, 180),
        "Garden City": (700, 250),
        "Peel Heritage": (750, 300),
        "Maple Grove": (700, 370),
        "Riverside Park": (750, 420),
        "Sunnybrook Plaza": (700, 500),
        "New Horizon Centre": (750, 550),
        "Innovation Hub": (700, 630),
        "Tech Park": (750, 680),
        "Cultural Centre": (900, 100),
        "Civic Plaza": (950, 150),
        "Art District": (900, 220),
        "Historic Market": (950, 270),
        "Sports Complex": (900, 340),
        "Downtown Commons": (950, 390),
        "Uptown Centre": (900, 460),
        "Lakeside Mall": (950, 510),
        "Northside Park": (900, 580),
        "West End": (100, 800),
        "East End": (1100, 800),
        "Central Station": (600, 50),
        "Brampton Village": (350, 30),
        "Crescent Plaza": (800, 30),
        "Heritage Park": (200, 30),
        "Millennium Square": (1050, 30),
        "City Walk": (600, 900),
        "Cornerstone Plaza": (300, 450),
        "Violet Grove": (450, 650),
        "Maple Ridge": (600, 700),
        "Pine Crest": (750, 740),
        "Oak Park": (900, 750),
        "Riverbend": (1050, 750),
        "Summit Plaza": (1150, 800)
    }
    # Add all intersections.
    for name, (x, y) in intersections.items():
        tms.add_intersection(name, x, y)
    # Automatically add roads between nodes within a threshold.
    threshold = 350  # Increase threshold to connect more nodes.
    names = list(intersections.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name1 = names[i]
            name2 = names[j]
            x1, y1 = intersections[name1]
            x2, y2 = intersections[name2]
            dist = math.hypot(x2 - x1, y2 - y1)
            if dist < threshold:
                tms.add_road(name1, name2, dist, bidirectional=True)
    print("Sample network built with the following intersections:")
    for name in intersections.keys():
        print("  -", name)

build_sample_network(tms)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/compute_route')
def compute_route():
    source = request.args.get('source', 'Downtown Brampton')
    destination = request.args.get('destination', 'Summit Plaza')
    path = tms.compute_route(source, destination)
    print(f"Computed route from {source} to {destination}: {path}")
    if path:
        return jsonify({"path": path})
    else:
        return jsonify({"error": "No route found"}), 404

@app.route('/add_emergency_vehicle')
def add_emergency_vehicle():
    vehicle_id = request.args.get('vehicle_id', 'EV')
    source = request.args.get('source', 'Downtown Brampton')
    destination = request.args.get('destination', 'Summit Plaza')
    try:
        priority = int(request.args.get('priority', 1))
    except ValueError:
        priority = 1
    tms.add_emergency_vehicle(vehicle_id, source, destination, priority)
    return jsonify({"message": f"Emergency vehicle '{vehicle_id}' added."})

@app.route('/process_emergency')
def process_emergency():
    results = tms.process_emergency_requests()
    response = []
    for vehicle, route in results:
        response.append({
            "vehicle_id": vehicle.vehicle_id,
            "priority": vehicle.priority,
            "source": vehicle.source,
            "destination": vehicle.destination,
            "route": route
        })
    print("Processed emergency vehicles:", response)
    return jsonify({"results": response})

if __name__ == '__main__':
    app.run(debug=True, port=5001)





