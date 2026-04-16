🚇 Delhi Metro Route Planner (C++)
<p align="center"> <b>Graph-Based Metro Navigation System using Dijkstra & Custom Heap</b><br> <i>Efficient shortest path & route analysis in a real-world metro network</i> </p>
📌 Overview

A high-performance C++ console application that simulates a Delhi Metro navigation system, enabling users to compute:

📏 Shortest distance
⏱️ Minimum travel time
🧭 Optimal route with interchanges

The system models the metro network as a weighted graph and applies advanced data structures and algorithms to deliver efficient route planning.

🚀 Key Highlights
⚡ Dijkstra’s Algorithm (Optimized) using custom heap
🧠 Graph-based modeling of real metro network
🔄 Interchange detection with route formatting
🧮 Dual optimization:
Distance-based routing
Time-based routing
🧾 Multiple input modes:
Station name
Station code
Serial number
🗺️ Full metro map visualization (Adjacency List)
🛠️ Tech Stack
Category	Tools / Concepts
Language	C++ (C++17)
Data Structures	Graph, Heap, HashMap, Vector, Stack
Algorithms	Dijkstra, DFS
STL Used	unordered_map, vector, list, algorithm
🧠 Core Concepts
📊 Graph Representation
Stations → Vertices
Connections → Edges (weighted by distance in km)
Implemented using:
unordered_map<string, Vertex>
⚡ Dijkstra’s Algorithm

Used for:

Shortest distance
Shortest time

Time Model:

Time = 120 + 40 × distance
120 sec → station halt
40 sec/km → travel time
🧱 Custom Min Heap
Built from scratch (instead of priority_queue)
Supports:
✅ Decrease-key operation
✅ Efficient updates in O(log n)
🔍 DFS-Based Path Exploration
Used for:
Path existence check
Full path reconstruction
Implemented using list as stack (LIFO)
🏗️ Project Structure
metro-route-planner/
│── metro_app.cpp   # Complete implementation
│── README.md
💻 Compilation & Execution
🔧 Compile
g++ -std=c++17 -O2 -o metro metro_app.cpp
▶️ Run
./metro
🖥️ Application Interface
****WELCOME TO THE DELHI METRO APP*****

1. LIST ALL THE STATIONS
2. SHOW THE METRO MAP
3. SHORTEST DISTANCE
4. SHORTEST TIME
5. SHORTEST PATH (DISTANCE)
6. SHORTEST PATH (TIME)
7. EXIT
🚉 Station Encoding

Each station includes a line identifier:

Code	Line
B	Blue
Y	Yellow
O	Orange
P	Pink
R	Red

Example:

Rajiv Chowk~BY → Blue + Yellow (Interchange)
New Delhi~YO   → Yellow + Orange
🔀 Interchange Logic
Stations with 2-letter suffix → Interchange nodes
Automatically:
Detects line transitions
Inserts route markers (==>)
Counts total interchanges
📈 Complexity Analysis
Operation	Time Complexity
Dijkstra	O((V + E) log V)
DFS Path Search	O(V + E)
Heap Operations	O(log V)
