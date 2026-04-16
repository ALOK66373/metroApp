/*
 * =====================================================================
 *  Delhi Metro Route Planner — C++ Implementation
 * =====================================================================
 *
 *  Converted and enhanced from the original Java source (Graph_M.java
 *  + Heap.java).  All logic is preserved 1-to-1; only language idioms
 *  have changed (STL containers, operator overloading, etc.).
 *
 *  FEATURES
 *  --------
 *  1. List all metro stations.
 *  2. Display the full metro map (adjacency list with distances).
 *  3. Shortest DISTANCE between two stations (Dijkstra).
 *  4. Shortest TIME between two stations (Dijkstra, time model).
 *  5. Shortest path (distance-wise) with interchange details.
 *  6. Shortest path (time-wise)   with interchange details.
 *
 *  DATA STRUCTURES
 *  ---------------
 *  - Weighted undirected graph stored as an adjacency list using
 *    unordered_map<string, Vertex>.  Each Vertex holds its neighbours
 *    and the edge weight (distance in km).
 *  - Custom min-heap (DijkstraHeap) that supports O(log n) decrease-key
 *    via an index map, required for efficient Dijkstra's algorithm.
 *  - Explicit stack (std::list used as LIFO) for DFS-based path search.
 *
 *  ALGORITHMS
 *  ----------
 *  - Dijkstra's algorithm:
 *      • Distance mode : weight = edge distance (km).
 *      • Time mode     : weight = 120 + 40 * distance  (seconds).
 *  - DFS with visited set for path-existence check (hasPath).
 *  - DFS with explicit stack for enumerating all paths to pick the
 *    minimum (Get_Minimum_Distance / Get_Minimum_Time).
 *
 *  COMPILATION
 *  -----------
 *      g++ -std=c++17 -O2 -o metro metro_app.cpp
 *
 *  STATION NAMING CONVENTION
 *  -------------------------
 *  Every station name carries its metro-line suffix after '~':
 *      "Rajiv Chowk~BY"  ->  station on Blue(B) and Yellow(Y) lines
 *      "New Delhi~YO"    ->  station on Yellow(Y) and Orange(O) lines
 *  A suffix length of 2 means the station is an interchange node
 *  between two different lines.
 * =====================================================================
 */

#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <climits>          // INT_MAX
#include <list>             // used as explicit LIFO stack
#include <cmath>            // ceil()
#include <sstream>          // istringstream for token splitting
#include <algorithm>        // find_if, etc.

using namespace std;

/* ===================================================================
 *  SECTION 1 : CUSTOM MIN-HEAP WITH DECREASE-KEY
 * ===================================================================
 *
 *  Why a custom heap?
 *  ------------------
 *  std::priority_queue does NOT support the "decrease-key" operation
 *  needed by Dijkstra.  We implement it manually:
 *    - data[]     : the heap array (stores raw pointers)
 *    - indexMap   : maps each element pointer -> its current position
 *                   in data[], so we can locate it in O(1) and then
 *                   bubble it up in O(log n).
 *
 *  This is a MIN-HEAP based on DijkstraPair::cost (lower cost = top).
 * =================================================================== */

/* ------------------------------------------------------------------
 * DijkstraPair
 * Represents one entry in the heap during Dijkstra's relaxation.
 * ------------------------------------------------------------------ */
struct DijkstraPair {
    string vname;   // name of the vertex this pair represents
    string psf;     // "path so far" — space-separated station names
    int    cost;    // accumulated cost from the source
};

/* ------------------------------------------------------------------
 * DijkstraHeap
 * A min-heap (lowest cost at root) that also supports O(log n)
 * updatePriority (decrease-key) via an index map.
 * ------------------------------------------------------------------ */
class DijkstraHeap {
private:
    /* ---- internal storage ---- */
    vector<DijkstraPair*>            data;      // heap array
    unordered_map<DijkstraPair*, int> indexMap; // pointer -> index in data

    /* ------------------------------------------------------------------
     * swap(i, j)
     * Exchanges two elements in data[] and keeps indexMap consistent.
     * ------------------------------------------------------------------ */
    void swapAt(int i, int j) {
        // Update the index map before physically swapping
        indexMap[data[i]] = j;
        indexMap[data[j]] = i;
        std::swap(data[i], data[j]);
    }

    /* ------------------------------------------------------------------
     * upheapify(ci)
     * Bubbles the element at index ci UP towards the root until the
     * min-heap property is restored (parent.cost <= child.cost).
     * ------------------------------------------------------------------ */
    void upheapify(int ci) {
        if (ci == 0) return;            // already at root
        int pi = (ci - 1) / 2;         // parent index

        // If child is cheaper than parent, they need to swap
        if (data[ci]->cost < data[pi]->cost) {
            swapAt(pi, ci);
            upheapify(pi);              // recurse on parent position
        }
    }

    /* ------------------------------------------------------------------
     * downheapify(pi)
     * Pushes the element at index pi DOWN until the min-heap property
     * is restored (parent <= both children).
     * ------------------------------------------------------------------ */
    void downheapify(int pi) {
        int lci  = 2 * pi + 1;         // left  child index
        int rci  = 2 * pi + 2;         // right child index
        int mini = pi;                  // index of the minimum so far

        // Check if left child is smaller than current minimum
        if (lci < (int)data.size() && data[lci]->cost < data[mini]->cost)
            mini = lci;

        // Check if right child is smaller than current minimum
        if (rci < (int)data.size() && data[rci]->cost < data[mini]->cost)
            mini = rci;

        // If a child is smaller than the current node, swap and recurse
        if (mini != pi) {
            swapAt(mini, pi);
            downheapify(mini);
        }
    }

public:
    /* ------------------------------------------------------------------
     * add(item)
     * Inserts a new DijkstraPair* into the heap.
     * O(log n) time due to upheapify.
     * ------------------------------------------------------------------ */
    void add(DijkstraPair* item) {
        data.push_back(item);
        indexMap[item] = (int)data.size() - 1;  // record its initial index
        upheapify((int)data.size() - 1);        // restore heap property
    }

    /* ------------------------------------------------------------------
     * remove()
     * Removes and returns the root (minimum-cost element).
     * O(log n) time due to downheapify.
     * ------------------------------------------------------------------ */
    DijkstraPair* remove() {
        // Swap root with last element, then pop the last element
        swapAt(0, (int)data.size() - 1);
        DijkstraPair* rv = data.back();
        data.pop_back();
        indexMap.erase(rv);             // remove from index map

        if (!data.empty())
            downheapify(0);             // restore heap property from root

        return rv;
    }

    /* ------------------------------------------------------------------
     * updatePriority(pair)
     * Called after the cost of an already-inserted element is DECREASED.
     * We simply bubble it up from its current position.
     * O(log n) time.
     * ------------------------------------------------------------------ */
    void updatePriority(DijkstraPair* pair) {
        int index = indexMap.at(pair);  // find current position
        upheapify(index);               // cost decreased -> bubble up
    }

    bool isEmpty() const { return data.empty(); }
};


/* ===================================================================
 *  SECTION 2 : METRO GRAPH
 * =================================================================== */

/* ------------------------------------------------------------------
 * Vertex
 * Represents a metro station.
 * nbrs maps neighbour station names -> edge weight (distance in km).
 * ------------------------------------------------------------------ */
struct Vertex {
    unordered_map<string, int> nbrs;    // adjacent_station_name -> distance_km
};

/* ------------------------------------------------------------------
 * MetroGraph
 * Weighted undirected graph of metro stations.
 * Provides path-finding, shortest distance, and shortest time queries.
 * ------------------------------------------------------------------ */
class MetroGraph {
private:
    /* Main adjacency list: station_name -> Vertex */
    unordered_map<string, Vertex> vtces;

    /* ------------------------------------------------------------------
     * Pair (internal)
     * Used during DFS-based full-path enumeration.
     * Tracks the accumulated distance AND time along the path explored
     * so far, plus the textual path (psf = "path so far").
     * ------------------------------------------------------------------ */
    struct Pair {
        string vname;       // current vertex name
        string psf;         // path so far (space-separated station names)
        int    min_dis;     // accumulated distance from source
        int    min_time;    // accumulated time from source (seconds)
    };

public:
    /* ================================================================
     *  GRAPH PRIMITIVE OPERATIONS
     * ================================================================ */

    /* ------------------------------------------------------------------
     * numVertex()
     * Returns the total number of stations in the graph.
     * ------------------------------------------------------------------ */
    int numVertex() const {
        return (int)vtces.size();
    }

    /* ------------------------------------------------------------------
     * containsVertex(vname)
     * Returns true if a station with the given name exists.
     * ------------------------------------------------------------------ */
    bool containsVertex(const string& vname) const {
        return vtces.count(vname) > 0;
    }

    /* ------------------------------------------------------------------
     * addVertex(vname)
     * Adds a new station (no edges yet) to the graph.
     * ------------------------------------------------------------------ */
    void addVertex(const string& vname) {
        vtces[vname];   // default-constructs an empty Vertex
    }

    /* ------------------------------------------------------------------
     * removeVertex(vname)
     * Removes a station and ALL edges connected to it.
     * ------------------------------------------------------------------ */
    void removeVertex(const string& vname) {
        if (!containsVertex(vname)) return;

        // Remove this station from every neighbour's adjacency list
        for (auto& nbr_name : vtces.at(vname).nbrs) {
            vtces[nbr_name.first].nbrs.erase(vname);
        }

        vtces.erase(vname);
    }

    /* ------------------------------------------------------------------
     * numEdges()
     * Returns the number of undirected edges in the graph.
     * (Each undirected edge is stored twice, so we divide by 2.)
     * ------------------------------------------------------------------ */
    int numEdges() const {
        int count = 0;
        for (const auto& kv : vtces)
            count += (int)kv.second.nbrs.size();
        return count / 2;
    }

    /* ------------------------------------------------------------------
     * containsEdge(vname1, vname2)
     * Returns true if there is a direct connection between the two
     * given stations.
     * ------------------------------------------------------------------ */
    bool containsEdge(const string& vname1, const string& vname2) const {
        if (!containsVertex(vname1) || !containsVertex(vname2))
            return false;
        return vtces.at(vname1).nbrs.count(vname2) > 0;
    }

    /* ------------------------------------------------------------------
     * addEdge(vname1, vname2, value)
     * Adds a bidirectional edge between two stations with the given
     * distance weight.  Does nothing if either station is missing or
     * the edge already exists.
     * ------------------------------------------------------------------ */
    void addEdge(const string& vname1, const string& vname2, int value) {
        if (!containsVertex(vname1) || !containsVertex(vname2))
            return;
        if (containsEdge(vname1, vname2))
            return;

        vtces[vname1].nbrs[vname2] = value;
        vtces[vname2].nbrs[vname1] = value;
    }

    /* ------------------------------------------------------------------
     * removeEdge(vname1, vname2)
     * Removes the bidirectional edge between two stations.
     * ------------------------------------------------------------------ */
    void removeEdge(const string& vname1, const string& vname2) {
        if (!containsEdge(vname1, vname2)) return;
        vtces[vname1].nbrs.erase(vname2);
        vtces[vname2].nbrs.erase(vname1);
    }

    /* ================================================================
     *  DISPLAY UTILITIES
     * ================================================================ */

    /* ------------------------------------------------------------------
     * display_Map()
     * Prints the full metro map as an adjacency list, showing each
     * station and its direct neighbours with distances (km).
     * ------------------------------------------------------------------ */
    void display_Map() const {
        cout << "\n\t Delhi Metro Map\n";
        cout << "\t------------------\n";
        cout << "----------------------------------------------------\n\n";

        for (const auto& kv : vtces) {
            const string& stationName = kv.first;
            const Vertex& vtx         = kv.second;

            cout << stationName << " =>\n";

            for (const auto& nbr : vtx.nbrs) {
                // Pad shorter names with extra tabs for neat alignment
                cout << "\t" << nbr.first;
                if (nbr.first.length() < 16) cout << "\t";
                if (nbr.first.length() < 8)  cout << "\t";
                cout << "\t" << nbr.second << " km\n";
            }
            cout << "\n";
        }

        cout << "\t------------------\n";
        cout << "---------------------------------------------------\n\n";
    }

    /* ------------------------------------------------------------------
     * display_Stations()
     * Lists all station names with serial numbers.
     * ------------------------------------------------------------------ */
    void display_Stations() const {
        cout << "\n***********************************************************************\n\n";
        int i = 1;
        for (const auto& kv : vtces)
            cout << i++ << ". " << kv.first << "\n";
        cout << "\n***********************************************************************\n\n";
    }

    /* ================================================================
     *  PATH EXISTENCE CHECK (DFS)
     * ================================================================ */

    /* ------------------------------------------------------------------
     * hasPath(vname1, vname2, processed)
     * Returns true if there is ANY path between vname1 and vname2.
     * Uses recursive DFS; 'processed' tracks visited stations to
     * prevent infinite loops in cycles.
     * ------------------------------------------------------------------ */
    bool hasPath(const string& vname1,
                 const string& vname2,
                 unordered_map<string, bool>& processed) const
    {
        // Base case: direct edge exists
        if (containsEdge(vname1, vname2))
            return true;

        // Mark current station as visited
        processed[vname1] = true;

        // Recursively explore all unvisited neighbours
        for (const auto& nbr : vtces.at(vname1).nbrs) {
            if (!processed.count(nbr.first)) {
                if (hasPath(nbr.first, vname2, processed))
                    return true;
            }
        }

        return false;   // no path found through this station
    }

    /* ================================================================
     *  DIJKSTRA'S ALGORITHM
     * ================================================================ */

    /* ------------------------------------------------------------------
     * dijkstra(src, des, useTime)
     *
     * Runs Dijkstra's algorithm from src to des.
     *
     * If useTime == false:  weight = edge distance (km).
     * If useTime == true:   weight = 120 + 40 * distance (seconds).
     *   (120 s = ~2 min stop time per station,
     *    40 s/km = average metro running time factor)
     *
     * Returns the minimum accumulated cost (distance or time) from
     * src to des.
     *
     * Complexity: O((V + E) log V) with the custom min-heap.
     * ------------------------------------------------------------------ */
    int dijkstra(const string& src, const string& des, bool useTime) const {
        int result = 0;

        /* ---- Initialise a DijkstraPair for every vertex ---- */
        // map: vertex_name -> its DijkstraPair* (for O(1) cost lookup/update)
        unordered_map<string, DijkstraPair*> map;
        DijkstraHeap heap;

        for (const auto& kv : vtces) {
            DijkstraPair* np = new DijkstraPair();
            np->vname = kv.first;
            np->cost  = INT_MAX;        // initially unreachable

            if (kv.first == src) {
                np->cost = 0;           // source costs 0 to reach itself
                np->psf  = src;
            }

            heap.add(np);
            map[kv.first] = np;
        }

        /* ---- Main Dijkstra relaxation loop ---- */
        while (!heap.isEmpty()) {
            // Extract the station with the currently lowest cost
            DijkstraPair* rp = heap.remove();

            // Always erase from the map first (prevents double-free
            // in the cleanup loop below, regardless of whether this
            // is the destination or a regular intermediate station)
            map.erase(rp->vname);

            // If we've reached the destination, record cost and stop
            if (rp->vname == des) {
                result = rp->cost;
                delete rp;
                break;
            }

            // Examine each neighbour of the current station
            const Vertex& v = vtces.at(rp->vname);
            for (const auto& nbr : v.nbrs) {
                // Only relax edges to stations still in the heap
                if (map.count(nbr.first)) {
                    int oldCost = map.at(nbr.first)->cost;

                    /* Compute new candidate cost:
                     *   Distance mode : rp->cost + edge_km
                     *   Time mode     : rp->cost + 120 + 40 * edge_km
                     */
                    int newCost = useTime
                        ? rp->cost + 120 + 40 * nbr.second
                        : rp->cost + nbr.second;

                    // Relax (update) if new cost is better
                    if (newCost < oldCost) {
                        DijkstraPair* gp = map.at(nbr.first);
                        gp->psf  = rp->psf + "  " + nbr.first;
                        gp->cost = newCost;
                        heap.updatePriority(gp);    // decrease-key
                    }
                }
            }

            delete rp;  // free processed pair
        }

        // Free any remaining pairs still in the map
        for (auto& kv : map) delete kv.second;

        return result;
    }

    /* ================================================================
     *  DFS-BASED MINIMUM PATH SEARCH
     *  (used to recover the actual path string for interchange display)
     * ================================================================ */

    /* ------------------------------------------------------------------
     * Get_Minimum_Distance(src, dst)
     *
     * Finds the path with the minimum total DISTANCE from src to dst
     * using an explicit DFS stack.
     *
     * Returns a string of the form:
     *     "Station1  Station2  Station3  <distance>"
     * (stations separated by double spaces, distance appended at end)
     *
     * Note: Dijkstra is preferred for just the cost; this function is
     * used when we also need the full path string for interchange
     * analysis.
     * ------------------------------------------------------------------ */
    string Get_Minimum_Distance(const string& src, const string& dst) const {
        int    minDist  = INT_MAX;
        string bestPath = "";

        unordered_map<string, bool> processed;  // visited set
        list<Pair> stack;                        // explicit DFS stack (LIFO)

        // Initialise the stack with the source station
        Pair sp;
        sp.vname   = src;
        sp.psf     = src + "  ";
        sp.min_dis = 0;
        sp.min_time = 0;
        stack.push_front(sp);

        while (!stack.empty()) {
            // Pop from front (LIFO = DFS behaviour)
            Pair rp = stack.front();
            stack.pop_front();

            // Skip already-processed vertices (handles back-edges / cycles)
            if (processed.count(rp.vname))
                continue;

            processed[rp.vname] = true;

            // If we reached the destination, check if this path is better
            if (rp.vname == dst) {
                if (rp.min_dis < minDist) {
                    bestPath = rp.psf;
                    minDist  = rp.min_dis;
                }
                continue;   // don't expand further from destination
            }

            // Push all unvisited neighbours onto the stack
            const Vertex& rpvtx = vtces.at(rp.vname);
            for (const auto& nbr : rpvtx.nbrs) {
                if (!processed.count(nbr.first)) {
                    Pair np;
                    np.vname    = nbr.first;
                    np.psf      = rp.psf + nbr.first + "  ";
                    np.min_dis  = rp.min_dis + nbr.second; // accumulate km
                    np.min_time = 0;
                    stack.push_front(np);
                }
            }
        }

        // Append the minimum distance value to the path string
        bestPath += to_string(minDist);
        return bestPath;
    }

    /* ------------------------------------------------------------------
     * Get_Minimum_Time(src, dst)
     *
     * Finds the path with the minimum total TRAVEL TIME from src to dst
     * using an explicit DFS stack.
     *
     * Time formula per hop:
     *     time (seconds) = 120 + 40 * edge_distance_km
     *
     * Returns a string of the form:
     *     "Station1  Station2  Station3  <time_in_minutes>"
     * ------------------------------------------------------------------ */
    string Get_Minimum_Time(const string& src, const string& dst) const {
        int    minTime  = INT_MAX;
        string bestPath = "";

        unordered_map<string, bool> processed;
        list<Pair> stack;

        // Initialise with source
        Pair sp;
        sp.vname    = src;
        sp.psf      = src + "  ";
        sp.min_dis  = 0;
        sp.min_time = 0;
        stack.push_front(sp);

        while (!stack.empty()) {
            Pair rp = stack.front();
            stack.pop_front();

            if (processed.count(rp.vname))
                continue;

            processed[rp.vname] = true;

            // Reached destination — check if this is the fastest path
            if (rp.vname == dst) {
                if (rp.min_time < minTime) {
                    bestPath = rp.psf;
                    minTime  = rp.min_time;
                }
                continue;
            }

            // Expand neighbours
            const Vertex& rpvtx = vtces.at(rp.vname);
            for (const auto& nbr : rpvtx.nbrs) {
                if (!processed.count(nbr.first)) {
                    Pair np;
                    np.vname    = nbr.first;
                    np.psf      = rp.psf + nbr.first + "  ";
                    np.min_dis  = 0;
                    // Accumulate time: 120s stop + 40s per km of edge
                    np.min_time = rp.min_time + 120 + 40 * nbr.second;
                    stack.push_front(np);
                }
            }
        }

        // Convert seconds to minutes (ceiling) and append to path string
        double minutes = ceil((double)minTime / 60.0);
        bestPath += to_string((int)minutes);
        return bestPath;
    }

    /* ================================================================
     *  INTERCHANGE DETECTION
     * ================================================================ */

    /* ------------------------------------------------------------------
     * get_Interchanges(pathStr)
     *
     * Given a path string produced by Get_Minimum_Distance or
     * Get_Minimum_Time (stations separated by "  ", metric at end),
     * this function:
     *   1. Splits the string into individual station tokens.
     *   2. Detects interchange stations (where you switch metro lines).
     *      A station is an interchange if its line-code suffix has
     *      length 2 (e.g., "~BY" means Blue+Yellow — two lines).
     *   3. Inserts "==>" markers between segments of different lines.
     *
     * Returns a vector where:
     *   [0]          = source station
     *   [1..n-3]     = intermediate stations (with ==> for interchanges)
     *   [n-2]        = number of interchanges (as string)
     *   [n-1]        = metric value (distance km or time minutes)
     * ------------------------------------------------------------------ */
    vector<string> get_Interchanges(const string& pathStr) const {
        vector<string> result;

        // Split the path string on double-space delimiter
        vector<string> tokens;
        {
            size_t start = 0, pos = 0;
            while ((pos = pathStr.find("  ", start)) != string::npos) {
                string token = pathStr.substr(start, pos - start);
                if (!token.empty())
                    tokens.push_back(token);
                start = pos + 2;    // skip past the "  " separator
            }
            // Capture the metric value appended at the very end
            string last = pathStr.substr(start);
            if (!last.empty())
                tokens.push_back(last);
        }

        if (tokens.empty()) return result;

        // First token is always the source station
        result.push_back(tokens[0]);

        int interchangeCount = 0;

        /* Iterate over intermediate tokens (skip first and last metric) */
        for (int i = 1; i < (int)tokens.size() - 1; i++) {
            // Extract the line-code part: everything after '~'
            size_t tildePos = tokens[i].rfind('~');
            string lineCode = (tildePos != string::npos)
                                ? tokens[i].substr(tildePos + 1)
                                : "";

            if (lineCode.length() == 2) {
                // This station sits on two lines — potential interchange

                // Get the line code of the previous and next stations
                size_t prevTilde = tokens[i-1].rfind('~');
                string prevCode  = (prevTilde != string::npos)
                                    ? tokens[i-1].substr(prevTilde + 1) : "";

                size_t nextTilde = tokens[i+1].rfind('~');
                string nextCode  = (nextTilde != string::npos)
                                    ? tokens[i+1].substr(nextTilde + 1) : "";

                if (prevCode == nextCode) {
                    // Just passing through the interchange station on
                    // the same line — no actual line change
                    result.push_back(tokens[i]);
                } else {
                    // Line change happens here: mark with arrow to next
                    result.push_back(tokens[i] + " ==> " + tokens[i+1]);
                    i++;                    // skip the next token (already included)
                    interchangeCount++;
                }
            } else {
                // Regular non-interchange station
                result.push_back(tokens[i]);
            }
        }

        // Append interchange count and metric as last two elements
        result.push_back(to_string(interchangeCount));
        result.push_back(tokens.back());    // distance (km) or time (minutes)

        return result;
    }

    /* ================================================================
     *  STATION CODE UTILITY
     * ================================================================ */

    /* ------------------------------------------------------------------
     * printCodeList(codes_out)
     *
     * Prints all stations with a short alphabetic code generated from
     * the first letter of each word in the station name (digits are
     * also included if the word starts with one).
     *
     * Codes are stored in codes_out so callers can match user input
     * to full station names.
     *
     * Returns a vector of station names in the same order as codes.
     * ------------------------------------------------------------------ */
    vector<string> printCodeList(vector<string>& codes_out) const {
        cout << "List of stations along with their codes:\n\n";

        // Collect station names into a list
        vector<string> keys;
        for (const auto& kv : vtces)
            keys.push_back(kv.first);

        codes_out.resize(keys.size());
        int m = 1;  // tracks the number of digits in the serial number

        for (int idx = 0; idx < (int)keys.size(); idx++) {
            const string& key = keys[idx];
            string& code      = codes_out[idx];
            code              = "";

            // Tokenise the station name by spaces
            istringstream iss(key);
            string lastToken;
            while (iss >> lastToken) {
                int j = 0;
                char c = lastToken[j];

                // If word starts with digit(s), include them in the code
                while (c >= '0' && c <= '9') {
                    code += c;
                    j++;
                    c = lastToken[j];
                }

                // Include the first non-digit, non-tilde alphabetic character
                if (c != '~' && (c < '0' || c > '9'))
                    code += c;
            }

            // Ensure code is at least 2 characters
            if (code.length() < 2 && !lastToken.empty() && lastToken.length() >= 2)
                code += (char)toupper(lastToken[1]);

            // Print: serial number, station name (with tab alignment), code
            cout << (idx + 1) << ". " << key;
            if (key.length() < (size_t)(22 - m)) cout << "\t";
            if (key.length() < (size_t)(14 - m)) cout << "\t";
            if (key.length() < (size_t)( 6 - m)) cout << "\t";
            cout << "\t" << code << "\n";

            // Adjust digit-count tracker for serial number alignment
            if (idx + 2 == (int)pow(10.0, (double)m))
                m++;
        }

        return keys;    // return ordered station name list
    }

    /* ================================================================
     *  METRO MAP BUILDER (STATIC DATA)
     * ================================================================ */

    /* ------------------------------------------------------------------
     * Create_Metro_Map(g)
     *
     * Populates the graph with Delhi Metro stations and edges.
     *
     * Station name format: "Station Name~LineCodes"
     *   B  = Blue line
     *   Y  = Yellow line
     *   O  = Orange (Airport Express) line
     *   P  = Pink line
     *   R  = Red line
     *   BY = Blue + Yellow interchange
     *   BO = Blue + Orange interchange
     *   YO = Yellow + Orange interchange
     *   BP = Blue + Pink interchange
     *   PR = Pink + Red interchange
     *
     * Edge weights are distances in kilometres between adjacent stations.
     * ------------------------------------------------------------------ */
    static void Create_Metro_Map(MetroGraph& g) {
        /* ---- Add all station vertices ---- */
        g.addVertex("Noida Sector 62~B");
        g.addVertex("Botanical Garden~B");
        g.addVertex("Yamuna Bank~B");
        g.addVertex("Rajiv Chowk~BY");         // Blue-Yellow interchange
        g.addVertex("Vaishali~B");
        g.addVertex("Moti Nagar~B");
        g.addVertex("Janak Puri West~BO");      // Blue-Orange interchange
        g.addVertex("Dwarka Sector 21~B");
        g.addVertex("Huda City Center~Y");
        g.addVertex("Saket~Y");
        g.addVertex("Vishwavidyalaya~Y");
        g.addVertex("Chandni Chowk~Y");
        g.addVertex("New Delhi~YO");            // Yellow-Orange interchange
        g.addVertex("AIIMS~Y");
        g.addVertex("Shivaji Stadium~O");
        g.addVertex("DDS Campus~O");
        g.addVertex("IGI Airport~O");
        g.addVertex("Rajouri Garden~BP");       // Blue-Pink interchange
        g.addVertex("Netaji Subhash Place~PR"); // Pink-Red interchange
        g.addVertex("Punjabi Bagh West~P");

        /* ---- Add all edges (bidirectional, weights in km) ---- */

        // --- Blue Line segment (Noida side) ---
        g.addEdge("Noida Sector 62~B",    "Botanical Garden~B",    8);
        g.addEdge("Botanical Garden~B",   "Yamuna Bank~B",         10);
        g.addEdge("Yamuna Bank~B",        "Vaishali~B",            8);   // branch
        g.addEdge("Yamuna Bank~B",        "Rajiv Chowk~BY",        6);

        // --- Blue Line segment (West Delhi side) ---
        g.addEdge("Rajiv Chowk~BY",       "Moti Nagar~B",          9);
        g.addEdge("Moti Nagar~B",         "Janak Puri West~BO",    7);
        g.addEdge("Janak Puri West~BO",   "Dwarka Sector 21~B",    6);

        // --- Yellow Line (South Delhi) ---
        g.addEdge("Huda City Center~Y",   "Saket~Y",               15);
        g.addEdge("Saket~Y",              "AIIMS~Y",               6);
        g.addEdge("AIIMS~Y",              "Rajiv Chowk~BY",        7);

        // --- Yellow Line (North Delhi) ---
        g.addEdge("Rajiv Chowk~BY",       "New Delhi~YO",          1);
        g.addEdge("New Delhi~YO",         "Chandni Chowk~Y",       2);
        g.addEdge("Chandni Chowk~Y",      "Vishwavidyalaya~Y",     5);

        // --- Orange Line (Airport Express) ---
        g.addEdge("New Delhi~YO",         "Shivaji Stadium~O",     2);
        g.addEdge("Shivaji Stadium~O",    "DDS Campus~O",          7);
        g.addEdge("DDS Campus~O",         "IGI Airport~O",         8);

        // --- Pink Line segment ---
        g.addEdge("Moti Nagar~B",         "Rajouri Garden~BP",     2);
        g.addEdge("Punjabi Bagh West~P",  "Rajouri Garden~BP",     2);
        g.addEdge("Punjabi Bagh West~P",  "Netaji Subhash Place~PR", 3);
    }
};


/* ===================================================================
 *  SECTION 3 : MAIN — INTERACTIVE MENU
 * =================================================================== */

int main() {
    /* ---- Build the metro network graph ---- */
    MetroGraph g;
    MetroGraph::Create_Metro_Map(g);

    cout << "\n\t\t\t****WELCOME TO THE DELHI METRO APP*****\n";

    /* ---- Interactive menu loop ---- */
    while (true) {
        cout << "\n\t\t\t\t~~LIST OF ACTIONS~~\n\n";
        cout << "1. LIST ALL THE STATIONS IN THE MAP\n";
        cout << "2. SHOW THE METRO MAP\n";
        cout << "3. GET SHORTEST DISTANCE FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "4. GET SHORTEST TIME TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "5. GET SHORTEST PATH (DISTANCE WISE) TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "6. GET SHORTEST PATH (TIME WISE) TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "7. EXIT\n";
        cout << "\nENTER YOUR CHOICE FROM THE ABOVE LIST (1 to 7): ";

        int choice = -1;
        cin >> choice;
        cin.ignore();   // discard trailing newline before getline calls

        cout << "\n***********************************************************\n";

        if (choice == 7) {
            cout << "Thank you for using the Delhi Metro App!\n";
            break;
        }

        switch (choice) {

        /* ---- Case 1: List all stations ---- */
        case 1:
            g.display_Stations();
            break;

        /* ---- Case 2: Show full metro map ---- */
        case 2:
            g.display_Map();
            break;

        /* ---- Case 3: Shortest distance (Dijkstra, km) ---- */
        case 3: {
            // Print station codes and collect ordered keys
            vector<string> codes;
            vector<string> keys = g.printCodeList(codes);

            cout << "\n1. TO ENTER SERIAL NO. OF STATIONS"
                    "\n2. TO ENTER CODE OF STATIONS"
                    "\n3. TO ENTER NAME OF STATIONS\n";
            cout << "ENTER YOUR CHOICE: ";

            int ch;
            cin >> ch;
            cin.ignore();

            string st1, st2;
            cout << "ENTER THE SOURCE AND DESTINATION STATIONS\n";

            if (ch == 1) {
                // User enters 1-based serial numbers
                int n1, n2;
                cin >> n1 >> n2;
                cin.ignore();
                if (n1 < 1 || n1 > (int)keys.size() ||
                    n2 < 1 || n2 > (int)keys.size()) {
                    cout << "Invalid serial numbers.\n";
                    break;
                }
                st1 = keys[n1 - 1];
                st2 = keys[n2 - 1];

            } else if (ch == 2) {
                // User enters station codes (case-insensitive)
                string a, b;
                getline(cin, a);
                getline(cin, b);
                // Convert to uppercase for comparison
                for (char& c : a) c = toupper(c);
                for (char& c : b) c = toupper(c);

                // Find matching station index by code
                int j1 = -1, j2 = -1;
                for (int j = 0; j < (int)codes.size(); j++) {
                    if (a == codes[j]) j1 = j;
                    if (b == codes[j]) j2 = j;
                }
                if (j1 == -1 || j2 == -1) {
                    cout << "Code not found.\n";
                    break;
                }
                st1 = keys[j1];
                st2 = keys[j2];

            } else if (ch == 3) {
                // User types full station names
                getline(cin, st1);
                getline(cin, st2);

            } else {
                cout << "Invalid sub-choice.\n";
                break;
            }

            // Validate and compute shortest distance
            unordered_map<string, bool> processed;
            if (!g.containsVertex(st1) || !g.containsVertex(st2) ||
                !g.hasPath(st1, st2, processed)) {
                cout << "THE INPUTS ARE INVALID (station not found or no path).\n";
            } else {
                int dist = g.dijkstra(st1, st2, false);
                cout << "SHORTEST DISTANCE FROM \"" << st1
                     << "\" TO \"" << st2 << "\" IS " << dist << " KM\n";
            }
            break;
        }

        /* ---- Case 4: Shortest time (Dijkstra, seconds -> minutes) ---- */
        case 4: {
            cout << "ENTER THE SOURCE STATION: ";
            string sat1, sat2;
            getline(cin, sat1);
            cout << "ENTER THE DESTINATION STATION: ";
            getline(cin, sat2);

            // Validate path existence
            unordered_map<string, bool> processed;
            if (!g.containsVertex(sat1) || !g.containsVertex(sat2) ||
                !g.hasPath(sat1, sat2, processed)) {
                cout << "THE INPUTS ARE INVALID.\n";
            } else {
                int timeSeconds = g.dijkstra(sat1, sat2, true);
                cout << "SHORTEST TIME FROM \"" << sat1 << "\" TO \""
                     << sat2 << "\" IS " << timeSeconds / 60
                     << " MINUTES\n";
            }
            break;
        }

        /* ---- Case 5: Shortest path, distance-wise with interchanges ---- */
        case 5: {
            cout << "ENTER THE SOURCE AND DESTINATION STATIONS\n";
            string s1, s2;
            getline(cin, s1);
            getline(cin, s2);

            unordered_map<string, bool> processed;
            if (!g.containsVertex(s1) || !g.containsVertex(s2) ||
                !g.hasPath(s1, s2, processed)) {
                cout << "THE INPUTS ARE INVALID.\n";
            } else {
                // Get path string and parse interchanges
                string pathStr = g.Get_Minimum_Distance(s1, s2);
                vector<string> info = g.get_Interchanges(pathStr);
                int len = (int)info.size();

                cout << "SOURCE STATION      : " << s1                << "\n";
                cout << "DESTINATION STATION : " << s2                << "\n";
                cout << "DISTANCE            : " << info[len - 1]     << " KM\n";
                cout << "NUMBER OF INTERCHANGES : " << info[len - 2]  << "\n";
                cout << "~~~~~~~~~~~~~\n";
                cout << "START  ==>  " << info[0] << "\n";

                // Print all intermediate stations
                for (int i = 1; i < len - 2; i++)
                    cout << info[i] << "\n";

                cout << "END\n~~~~~~~~~~~~~\n";
            }
            break;
        }

        /* ---- Case 6: Shortest path, time-wise with interchanges ---- */
        case 6: {
            cout << "ENTER THE SOURCE STATION: ";
            string ss1, ss2;
            getline(cin, ss1);
            cout << "ENTER THE DESTINATION STATION: ";
            getline(cin, ss2);

            unordered_map<string, bool> processed;
            if (!g.containsVertex(ss1) || !g.containsVertex(ss2) ||
                !g.hasPath(ss1, ss2, processed)) {
                cout << "THE INPUTS ARE INVALID.\n";
            } else {
                string pathStr = g.Get_Minimum_Time(ss1, ss2);
                vector<string> info = g.get_Interchanges(pathStr);
                int len = (int)info.size();

                cout << "SOURCE STATION         : " << ss1               << "\n";
                cout << "DESTINATION STATION    : " << ss2               << "\n";
                cout << "TIME                   : " << info[len - 1]     << " MINUTES\n";
                cout << "NUMBER OF INTERCHANGES : " << info[len - 2]     << "\n";
                cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
                cout << "START  ==>  " << info[0] << "\n";

                for (int i = 1; i < len - 2; i++)
                    cout << info[i] << "\n";

                cout << "END\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n";
            }
            break;
        }

        /* ---- Default: invalid choice ---- */
        default:
            cout << "Please enter a valid option (1 to 7).\n";
            break;
        }
    }

    return 0;
}
