Project Overview:
This project implements a full‑stack smart traffic management system for Brampton, Ontario. The system models a network of over 50 real-world locations (such as malls, plazas, parks, and other attractions) using a graph structure. It utilizes the A* algorithm for route computation and manages emergency vehicles with a priority queue. The front end provides a visually appealing, interactive interface that displays the network on a map, dynamically computes routes for regular vehicles, and displays each emergency vehicle’s route in a unique color.
Backend (app.py):
Graph Construction:
The code defines a Graph class where intersections are represented as nodes with (x, y) coordinates (approximated in pixel values to mimic their relative positions in Brampton). Edges (roads) are automatically created between nodes that fall within a specified threshold distance (350 pixels in our implementation). This ensures that the network is sufficiently connected.
Routing with A* Algorithm:
The AStar class implements the A* search algorithm to compute the shortest route between any two nodes. It uses a Euclidean distance heuristic to guide the search. The algorithm reconstructs the final path by backtracking through a dictionary that records each node’s predecessor.
Emergency Vehicle Management:
Emergency vehicles are represented using a data class (EmergencyVehicle) and managed with a priority queue in the EmergencyVehicleManager class. Vehicles with a lower priority number are considered more urgent. The system processes all emergency vehicle requests by computing a route for each and returns the results.
Flask Endpoints:
The Flask app exposes several endpoints:
/: Renders the main interface.
/compute_route: Accepts source and destination parameters and returns the computed route (if available).
/add_emergency_vehicle: Adds a new emergency vehicle request.
/process_emergency: Processes all emergency vehicle requests and returns their routes.
Sample Network:
The build_sample_network function populates the graph with over 50 intersections and automatically connects nodes that are within the threshold distance. This sample network uses realistic names for locations in Brampton.
Front End (index.html):
Dynamic Interface:
The front end is built with modern HTML, CSS, and JavaScript. It uses CSS variables, gradients, and transitions to create a sleek and professional look.
Interactive Panels:
There are two main panels:
Normal Vehicle Routing: Users can select a source and destination from dynamically populated dropdowns. When the "Compute Normal Route" button is clicked, a red route is drawn on a canvas map.
Emergency Vehicle Management: Users can add emergency vehicle requests with a specified ID, source, destination, and priority. When processing these requests, each emergency route is displayed in a distinct color.
Canvas Map:
A large canvas simulates a city map. The nodes (intersections) are drawn with clear, offset labels to avoid overlap. Roads are rendered as lines connecting nodes that are within range, and computed routes are drawn with arrowheads indicating direction.
Responsiveness & Visual Appeal:
The CSS uses responsive techniques and modern design elements (such as hover effects and smooth transitions) to ensure that the interface is both visually appealing and user-friendly.
