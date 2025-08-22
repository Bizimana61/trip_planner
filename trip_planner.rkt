#lang dssl2

# Final project: Trip Planner
let eight_principles = ["Know your rights.",
                        "Acknowledge your sources.",
                        "Protect your work.",
                        "Avoid suspicion.",
                        "Do your own work.",
                        "Never falsify a record or permit another person to do so.",
                        "Never fabricate data, citations, or experimental results.",
                        "Always tell the truth when discussing your work with your instructor."]

import cons
import sbox_hash
import csv
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = VecKC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = VecKC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = VecKC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs
# Helper struct to represent a position
struct Position:
    let lat: Lat?
    let lon: Lon?
    
# Helper struct to represent a point of interest
struct POI:
    let pos: Position?
    let category: Cat?
    let name: Name?
    
# Compute Euclidean distance between two positions
def distance(p1: Position?, p2: Position?) -> num?:
    let dx = p1.lon - p2.lon
    let dy = p1.lat - p2.lat
    return ((dx * dx) + (dy * dy)).sqrt()

class TripPlanner (TRIP_PLANNER):
    
#   ^ ADD YOUR WORK HERE
    let _position_to_vertex: DICT?      # Maps Position to vertex ID
    let _vertex_to_position: DICT?      # Maps vertex ID to Position
    let _road_graph: WUGRAPH?           # Graph representing the road network
    let _pois_by_category: DICT?        # Maps category to list of POIs
    let _pois_by_name: DICT?            # Maps name to POI
    let _pois_by_position: DICT?        # Maps Position to list of POIs
    let n_pois                          # Number of POIs
    def __init__(self, raw_segs: VecC[RawSeg?], raw_pois: VecC[RawPOI?]):
        
        let unique_vertices = raw_segs.len() * 2 
        # Initialize dictionaries and collections
        self._position_to_vertex = HashTable(unique_vertices, make_sbox_hash())
        self._vertex_to_position = HashTable(unique_vertices, make_sbox_hash())
        self._pois_by_category = HashTable(raw_pois.len(), make_sbox_hash())
        self._pois_by_name = HashTable(raw_pois.len(), make_sbox_hash())
        self._pois_by_position = HashTable(raw_pois.len(), make_sbox_hash())
        self.n_pois = raw_pois.len()
    
        # Process all positions from segments first
        # Vertex counter
        let vertex_count = 0
        #let pos = count
        for i in range(raw_segs.len()):
            let seg = raw_segs[i]
            let pos1 = Position(seg[0], seg[1])
            if not self._position_to_vertex.mem?(pos1): 
                self._position_to_vertex.put(pos1, vertex_count)
                self._vertex_to_position.put(vertex_count, pos1)
            
                vertex_count = vertex_count + 1
            let pos2 =Position(seg[2], seg[3])
            if not self._position_to_vertex.mem?(pos2):
                self._position_to_vertex.put(pos2, vertex_count)
                self._vertex_to_position.put(vertex_count, pos2)
            
                vertex_count = vertex_count + 1
            
        # Create the road graph
        self._road_graph = WUGraph(vertex_count)    
    
        
        # Add all road segments to the graph
        for i in range(raw_segs.len()):
            let seg = raw_segs[i]
            
            let pos1 = Position(seg[0], seg[1])
            let pos2 = Position(seg[2], seg[3])
            #if self._position_to_vertex.mem?(pos1) and self._position_to_vertex.mem?(pos2):
            let v1 = self._position_to_vertex.get(pos1)
            
            
            let v2 = self._position_to_vertex.get(pos2)
            
            let dist = distance(pos1, pos2)
            
            self._road_graph.set_edge(v1, v2, dist)
            
        # Process all POIs
        for i in range(raw_pois.len()):
           
            let poi_data = raw_pois[i]
            let pos = Position(poi_data[0], poi_data[1])
            
            let category = poi_data[2]
            let name = poi_data[3]
            let poi = POI(pos, category, name)
            
        
            # Add to POIs by name dictionary
            self._pois_by_name.put(name, poi)
        
            # Add to POIs by category dictionary
            if not self._pois_by_category.mem?(category):
                self._pois_by_category.put(category, cons(poi, None))
                
            
            else:
                let current_pois = self._pois_by_category.get(category)
                self._pois_by_category.put(category, cons(poi, current_pois))
                
            
        
            # Add to POIs by position dictionary
            if not self._pois_by_position.mem?(pos):
               
                self._pois_by_position.put(pos,cons(poi, None))    
            else:
                let cur_pois = self._pois_by_position.get(pos)
                
                self._pois_by_position.put(pos, cons(poi, cur_pois))
    
    def locate_all(self, dst_cat: Cat?) -> ListC[RawPos?]:
        # Check if the category exists in our records
        if not self._pois_by_category.mem?(dst_cat):
            return None  
        
        # Get all POIs in the category
        let pois = self._pois_by_category.get(dst_cat)
       
        let result = None
        
        
        # Collect unique positions
        
        let ht = HashTable(self.n_pois, make_sbox_hash()) 
        while pois is not None:
            let poi = pois.data    
            let pos = poi.pos
            let newpos = [pos.lat, pos.lon]
            
            if not ht.mem?(newpos):
            
                result = cons(newpos, result)
                ht.put(newpos, pos)
            pois = pois.next
                    
        return result
        
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?) -> ListC[RawPos?]:
        
        # Check if the destination POI exists
        if not self._pois_by_name.mem?(dst_name):
            return None  

        # Get source and destination positions
        let src_pos = Position(src_lat, src_lon)
        let dst_poi = self._pois_by_name.get(dst_name)
        let dst_pos = dst_poi.pos
        
        # Check if source position is valid (must be at a road segment endpoint)
        if not self._position_to_vertex.mem?(src_pos):
            return None  # Return empty list if source position is invalid
            
        if not self._position_to_vertex.mem?(dst_pos):
            return None
        
        # Get vertex IDs for source and destination
        let src_vertex = self._position_to_vertex.get(src_pos)
        let dst_vertex = self._position_to_vertex.get(dst_pos)
        

        # Number of vertices in the graph
        let n_vertices = self._road_graph.n_vertices()

        
        let dist = [inf; n_vertices]  
        let pred = [None; n_vertices]  
        let done = [False; n_vertices]  


        # Initialize priority queue with custom comparison (min-heap)
        let pq = BinHeap[nat?](n_vertices * 2, lambda u, v: dist[u] < dist[v])

        # Distance to start vertex is 0
        dist[src_vertex] = 0

        # Insert the source vertex into the priority queue
        pq.insert(src_vertex)
        

        # Dijkstra's algorithm 
        while pq.len() > 0:
            # Extract the vertex with minimum distance from the priority queue
            let u = pq.find_min()
            pq.remove_min()
            
            if done[u]:
                continue

            # Mark the vertex as processed
            done[u] = True

            

            # Iterate over the adjacent vertices of the current vertex
            let adj_vertices = self._road_graph.get_adjacent(u)
           
            let adj_list = adj_vertices  
            
            while adj_list is not None:
                
                
                let v = adj_list.data
               
                # Get the edge weight from u to v
                let weight = self._road_graph.get_edge(u, v)
                
                # Relaxation an edge 
                if dist[u] + weight < dist[v]:
                    
                    dist[v] = dist[u] + weight
                    pred[v] = u
                    
                    pq.insert(v) 
                adj_list = adj_list.next

        if dist[dst_vertex] == inf:
            return None
        let path = None
        let current = dst_vertex

        while current != None:
            let pos = self._vertex_to_position.get(current)
            path = cons([pos.lat, pos.lon], path)
            current = pred[current]

        return path

        
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        # Check if the category exists
        if not self._pois_by_category.mem?(dst_cat):
            return None  
        
        # Get all POIs in the category
        let category_pois = self._pois_by_category.get(dst_cat)
        if category_pois is None:
            return None
        
        if self._pois_by_category.len() == 0:
            return None  
        
        # Get source position
        let src_pos = Position(src_lat, src_lon)
        
        
        if not self._position_to_vertex.mem?(src_pos):
            return None
            
        
        # Get source vertex
        let src_vertex = self._position_to_vertex.get(src_pos)
        
        # use Dijkstra's algorithm from source to find distances to all positions
        let n_vertices = self._road_graph.n_vertices()
        
        let dist = [inf; n_vertices]  # Distance from source to each vertex
        
        let done = [False; n_vertices]  # Mark vertices as processed (boolean)

        
        
        # Distance to source is 0
        dist[src_vertex] = 0
        
        # Create a binary heap for the priority queue
        let pq = BinHeap(n_vertices, lambda u, v: dist[u] < dist[v])
        pq.insert(src_vertex)
        
        # Dijkstra's algorithm main loop
        while pq.len() > 0:
            # Extract the vertex with minimum distance from the priority queue
            let u = pq.find_min()
            pq.remove_min()
            
            if done[u]:
                continue

            
            done[u] = True
            
            let adj_vertices = self._road_graph.get_adjacent(u)
            
            let adj_list = adj_vertices  
            
            while adj_list is not None:
                
                
                let v = adj_list.data
                
                let weight = self._road_graph.get_edge(v, u)
               
                if dist[u] + weight < dist[v]:
                    
                    dist[v] = dist[u] + weight
                    
                    pq.insert(v) 
                adj_list = adj_list.next
        
        let pq_distance = BinHeap(n_vertices, λ x, y: x[0] < y[0])
        
        # Process each POI in the category
        let curr_pois = category_pois
        while curr_pois is not None:
            let poi = curr_pois.data
            let pos = poi.pos
            
            # Check if position exists in the road network
            if self._position_to_vertex.mem?(pos):
                let vertex = self._position_to_vertex.get(pos)
                let distance = dist[vertex]
                
                # Only consider reachable POIs
                if distance != inf:
                    pq_distance.insert([distance, poi])
                    
            curr_pois = curr_pois.next        
        
        # Get the top n results
        let count = min(pq_distance.len(), n)
        let result = None
        
        for i in range(count):
            if pq_distance.len() == 0:
                break
                
            let min_element = pq_distance.find_min()
            pq_distance.remove_min()
            
            let poi = min_element[1]
            result = cons([poi.pos.lat, poi.pos.lon, poi.category, poi.name], result)
        
        
        return result



def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "Reggie's"],
                        [0,1, "food", "Dumplings"]])
let tp = TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "Reggie's"],
                        [0,1, "food", "Dumplings"]])

                        
                        
test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Dumplings") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Dumplings"], None)
        
test 'Failed test: 4 POIs, 2 in relevant category':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) == [[3, 0], [5, 0]]
    
test 'Failed test: multiple POIs in the same location, relevant one is first':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('bar')
    assert Cons.to_vec(result) == [[5, 0]]
    
test 'Failed test: multiple POIs in the same location, relevant one is last':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('bar')
    assert Cons.to_vec(result) == [[5, 0]]
    
test 'Failed test: multiple POIs in the same location, two relevant ones':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) == [[3, 0], [5, 0]]
    
test 'Failed test: 3 relevant POIs, 2 at same location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.locate_all('barber')
    assert Cons.to_vec(result) == [[3, 0], [5, 0]]
    
test 'Failed test: Destination is not reachable':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.plan_route(0, 0, 'Judy')
    result = Cons.to_vec(result)
    assert result == []
    
test 'Failed test: BFS is not SSSP (route)':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.plan_route(0, 0, 'Cem')
    result = Cons.to_vec(result)
    assert result == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]
    
test 'Failed test: Destination is the 2nd of 3 POIs at that location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'bar', 'Pasta'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'food', 'Jollibee']])
    let result = tp.plan_route(0, 0, 'Judy')
    result = Cons.to_vec(result)
    assert result == [[0, 0], [1.5, 0], [2.5, 0], [3, 0], [4, 0], [5, 0]]
    
test 'Failed test: 2 relevant POIs; 1 reachable':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[3, 0, 'barber', 'Tony']]
    
test 'Failed test: No POIs in requested category':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    
    assert Cons.to_vec(result) == []
    
test 'Failed test: Relevant POI is not reachable':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) == []
    
test 'Failed test: DFS is not SSSP (nearby)':
    let tp = TripPlanner(
      [[1, 0, 0, 0],
       [0, 0, 0, 1],
       [0, 1, 0, 2],
       [1, 0, 2, 0],
       [2, 0, 2, 1],
       [2, 1, 2, 2]],
      [[0, 1, 'food', 'Donuts'],
       [0, 2, 'food', 'Cake'],
       [2, 1, 'food', 'Burgers'],
       [2, 2, 'food', 'Pizza']])
    let result = tp.find_nearby(1, 0, 'food', 2)
    assert Cons.to_vec(result) == [[0, 1, 'food', 'Donuts'], [2, 1, 'food', 'Burgers']]
    
test 'Failed test: BFS is not SSSP (nearby)':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    assert Cons.to_vec(result) == [[8, 8, 'haberdasher', 'Braden'], [7, 7, 'haberdasher', 'Archit']]
    
test 'Failed test: MST is not SSSP (nearby)':
    let tp = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result = tp.find_nearby(-1.1, -1.1, 'barber', 1)
    assert Cons.to_vec(result) == [[3, 4, 'barber', 'Tony']]
    
test 'Failed test: 2 relevant POIs; limit 3':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 3)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
    
test 'Failed test: 2 relevant equidistant POIs; limit 1':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [3.5, 0, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(-1, -1, 'barber', 1)
    assert Cons.to_vec(result) == [[3.5, 0, 'barber', 'Tony']] \
      or Cons.to_vec(result) == [[0, 3.5, 'barber', 'Judy']]
      
test 'Failed test: 3 relevant POIs; farther 2 at same location; limit 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']] \
      or Cons.to_vec(result) == [[5, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
      
test 'Failed test: 3 relevant POIs; farther 2 equidistant; limit 2':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [0, 0, 'barber', 'Lily'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(2.5, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']] \
      or Cons.to_vec(result) == [[0, 0, 'barber', 'Lily'], [3, 0, 'barber', 'Tony']]
      
test 'Failed test: POI is 2nd of 3 in that location':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'bar', 'Pasta']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[5, 0, 'barber', 'Judy'], [3, 0, 'barber', 'Tony']]
    
test 'Failed test: 2 relevant POIs in same location; limit 2':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [0, 3.5, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[0, 3.5, 'barber', 'Tony'], [0, 3.5, 'barber', 'Judy']]
    
test 'Failed test: 2 relevant POIs in same location; limit 2':
    let tp = TripPlanner(
      [[-1, -1, 0, 0],
       [0, 0, 3.5, 0],
       [0, 0, 0, 3.5],
       [3.5, 0, 0, 3.5]],
      [[-1, -1, 'food', 'Jollibee'],
       [0, 0, 'bank', 'Union'],
       [0, 3.5, 'barber', 'Tony'],
       [0, 3.5, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[0, 3.5, 'barber', 'Tony'], [0, 3.5, 'barber', 'Judy']]

