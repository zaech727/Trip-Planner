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
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'
import 'project-lib/dictionaries.rkt'

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

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

struct cons_:
    let pos
    let data

class TripPlanner (TRIP_PLANNER):
    let vertex_count
    let poi_count
    let location_map
    let position_map
    let category_map
    let name_map
    let name_position_map
    let travel_graph
    let neighbors
    
    let path_trace # Path it takes
    let shortest_dist # Vertex distance to end
    let visited_vertex # Vertex is visited
     
    def __init__(self, segm: VecC[RawSeg?], pint: VecC[RawPOI?]):
        self.poi_count = 0
        self.vertex_count = 0
        self.category_map = HashTable(60,make_sbox_hash())
        self.location_map = HashTable(segm.len() * 2 + pint.len(),make_sbox_hash())
        self.position_map = HashTable(segm.len() * 2 + pint.len(),make_sbox_hash())
        self.name_map = HashTable(pint.len(),make_sbox_hash())
        self.name_position_map = HashTable(pint.len(),make_sbox_hash())
        
        self.initialize_maps(pint)
        self.initialize_vertices(segm)
        
        # For Djikstra's
        self.path_trace = None
        self.shortest_dist =  None
        self.visited_vertex = None
        
        self.travel_graph = WuGraph(self.vertex_count)
        self.initialize_graph(segm)
        self.neighbors = [None; self.vertex_count]
        for pos in range(self.vertex_count):
            self.neighbors[pos] = self.travel_graph.get_adjacent(pos)
        
    def initialize_maps(self, pint: VecC[RawPOI?]): #  initialize several maps related to the POIs
        for i in range (pint.len()):
            let position = [pint[i][0], pint[i][1]]
            let category = pint[i][2]
            let name = pint[i][3]
        
            if not self.location_map.mem?(position): # Adds the position to the location_map dictionary if not already there             
                self.location_map.put(position, self.poi_count)
                self.position_map.put(self.poi_count, position)
                self.poi_count = 1 + self.poi_count
                
            let poisMap = self.category_map.get(category) if self.category_map.mem?(category) else HashTable(60,make_sbox_hash())
            poisMap.put(self.poi_count - 1, name) # Adds a key-value pair to the poisMap dictionary
        
            if not self.category_map.mem?(category):
                self.category_map.put(category, poisMap)
                
            self.name_map.put(name, self.poi_count - 1)
            self.name_position_map.put([self.poi_count - 1, category], name)

    def initialize_vertices(self, segm: VecC[RawSeg?]): # Initialize the vertices of a graph by iterating over the segments 
        self.vertex_count = self.poi_count
        for seg in segm:
            for j in range(0, 4, 2): # Iterates over the pairs of coordinates (x and y) in each segment
                let position = [seg[j], seg[j+1]]
                if not self.location_map.mem?(position):
                    self.location_map.put(position, self.vertex_count)
                    self.position_map.put(self.vertex_count, position)
                    self.vertex_count = 1 + self.vertex_count
    
    def initialize_graph(self, segm: VecC[RawSeg?]):
        for i in range(segm.len()):
            let weight = ((segm[i][2] - segm[i][0])**2 + (segm[i][3] - segm[i][1])**2)**0.5
            self.travel_graph.set_edge(self.location_map.get([segm[i][0],segm[i][1]]), 
            self.location_map.get([segm[i][2],segm[i][3]]), weight) 

    def locate_all(self, dst_cat:Cat?) ->ListC[RawPos?]:
        if not self.category_map.mem?(dst_cat):  # Check if the category exists in the map
            return None  # If it doesn't, return an empty list
        let category_list = self.category_map.get(dst_cat)
        let space = None
        for i in range(self.poi_count):
            if category_list.mem?(i):
                let pos = self.position_map.get(i)
                space = cons(pos, space)
        return space
    
    def dijkstra(self, start: nat?):
        self.shortest_dist = [inf; self.vertex_count]
        self.path_trace = [None; self.vertex_count]
        self.visited_vertex = [False; self.vertex_count]
        self.shortest_dist[start] = 0
        
        let nodes_d = BinHeap[cons_?](self.vertex_count, λ x, y: x.data < y.data)
        nodes_d.insert(cons_(start, 0))
        
        while nodes_d.len() != 0: 
            let curr = nodes_d.find_min()
            nodes_d.remove_min()
            if not self.visited_vertex[curr.pos]:
                self.visited_vertex[curr.pos] = True
                let neighbor = self.neighbors[curr.pos]
                while neighbor != None:
                    if curr.data + self.travel_graph.get_edge(curr.pos, neighbor.data) < self.shortest_dist[neighbor.data]:
                        self.shortest_dist[neighbor.data] = curr.data + self.travel_graph.get_edge(curr.pos, neighbor.data)
                        self.path_trace[neighbor.data] = curr.pos
                        nodes_d.insert(cons_(neighbor.data, self.shortest_dist[neighbor.data]))
                    neighbor = neighbor.next
        return
        
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?) -> ListC[RawPos?]:
        if not self.name_map.mem?(dst_name): # Check if destination exists in name_map
            return None
            
        let startVert = self.location_map.get([src_lat, src_lon]) # Get the starting vertex and destination vertex
        let destVert = self.name_map.get(dst_name)

        self.dijkstra(startVert) # Find the shortest path

        let route = cons(self.position_map.get(destVert), None)
        if startVert == destVert:
            return route

        for _ in range(self.vertex_count): # Traverse the path_trace to construct the route
            let r = self.path_trace[destVert]      
                   
            if r == None: # No path return None
                return None                 
            
            if r == startVert: # If the next vertex is the starting vertex, return the final route
                return cons(self.position_map.get(r), route)
                        
            if (not r == -1) and (not r == startVert): # If the next vertex is valid and not the starting vertex, add it to the route
                route = cons(self.position_map.get(r), route)
                destVert = r
        return None

        return route
         
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        
        if not self.category_map.mem?(dst_cat):
            return None # Check if the category exists in category_map
        
        let categories = self.category_map.get(dst_cat)
        let start_vertex = self.location_map.get([src_lat, src_lon])

        self.dijkstra(start_vertex) # Find the shortest path
        
        # Initialize a priority queue to store vertices and their shortest distances
        let priority_queue = BinHeap[TupC[num?, num?]](self.vertex_count, lambda x, y: x[1] < y[1])
        for i in range(self.vertex_count):
            priority_queue.insert([i, self.shortest_dist[i]])

        let nearby_pois = None
        let poi_count = 0
        for _ in range(self.vertex_count): # Traverse the priority queue to find nearby POIs
            let node = priority_queue.find_min()
            priority_queue.remove_min()
            let vertex = node[0]
            if (poi_count < n) and (not node[1] == inf) and categories.mem?(vertex):
                poi_count = 1 + poi_count
                let position = self.position_map.get(vertex)
                let name = self.name_position_map.get([vertex, dst_cat])
                nearby_pois = cons([position[0], position[1], dst_cat, name], nearby_pois)

        return nearby_pois
        
#   ^ YOUR CODE GOES HERE

def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)    

test 'No point of interest':
    let tp = TripPlanner([[0, 0, 1, 0]], [])
    assert tp.locate_all('bank') == None

test 'Single point of interest, wrong category':
    let tp = TripPlanner([[0, 0, 1, 0]], [[1, 0, 'bank', 'Union']])
    assert tp.locate_all('food') == None
    
test 'Destination doesnt exist':
    let tp = TripPlanner([[0, 0, 1.5, 0], [1.5, 0, 2.5, 0], [2.5, 0, 3, 0]], [[1.5, 0, 'bank', 'Union'], [3, 0, 'barber', 'Tony']])
    assert tp.plan_route(0, 0, 'Judy') == None
    
test 'No POIs in requested category':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    assert tp.find_nearby(0, 0, 'food', 1) == None