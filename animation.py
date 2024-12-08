import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from matplotlib import rcParams
import matplotlib.patches as patches
import networkx as nx
import osmnx as ox
import random
import math
import numpy as np
import json
from collections import deque
from tqdm import tqdm
import contextily as ctx
from shapely.geometry import LineString, Point
import geopandas as gpd

rcParams['axes.unicode_minus'] = False

# G = ox.graph_from_place("Honolulu, Hawaii, USA", network_type="drive")
G = ox.load_graphml(filepath='honolulu.graphml')
num_nodes = len(G.nodes)
print(f"There are {num_nodes} nodes")
nodes, edges = ox.graph_to_gdfs(G, nodes=True, edges=True)
# edges = edges.reset_index()

# Define speed limits based on road type

speed_limits = {
    "motorway": 60,
    "motorway_link": 50,
    "trunk": 45,
    "trunk_link": 40,
    "primary": 35,
    "primary_link": 30,
    "secondary": 30,
    "secondary_link": 25,
    "tertiary": 25,
    "tertiary_link": 20,
    "residential": 25,
    "living_street": 15,
    "unclassified": 25
}


G_sub = G

nodes_sub, edges_sub = ox.graph_to_gdfs(G_sub, nodes=True, edges=True)

node_positions = {node_id: (data['x'], data['y']) for node_id, data in G_sub.nodes(data=True)}
if 12377487811 not in node_positions:
    print("Key 12377487811 is missing in node_positions!")
    print(len(node_positions))
from shapely.geometry import LineString, Point

print_once = True

def get_max_speed(direction, speed_limits):
    scale_para = 0.00005
    """
    # Return the maximum speed limit of a road based on its endpoints' direction
    # :param direction: A two-element array [u, v] containing the IDs of the start and end points
    # :param G_sub: A subgraph representing the road network
    # :param speed_limits: A dictionary mapping road types to their maximum speeds
    # :return: The maximum speed limit of the road (integer)
    """
    u, v = direction


    if u in G_sub and v in G_sub[u]:
        edge_data = G_sub[u][v][0] # Retrieve the attributes of the edge, assuming the first entry is valid
        highway_type = edge_data.get('highway', None)

        # Check the road type and return the maximum speed
        if highway_type:
            if isinstance(highway_type, list):
                highway_type = highway_type[0]
            return speed_limits.get(highway_type, 25)*scale_para
        else:
            print(f"{u}-{v} has no 'highway'")
            return 25*scale_para
    else:
        print(f"{u}-{v} does not exist")
        return 25*scale_para


class IntersectionManager:
    def __init__(self):
        self.queue_pool = {}

    def get_or_create_queue(self, intersection_id):
        if intersection_id not in self.queue_pool:
            self.queue_pool[intersection_id] = []
        return self.queue_pool[intersection_id]

    def remove_queue_if_empty(self, intersection_id):
        if intersection_id in self.queue_pool and not self.queue_pool[intersection_id]:
            del self.queue_pool[intersection_id]

    def add_vehicle_to_queue(self, intersection_id, vehicle):
        queue = self.get_or_create_queue(intersection_id)
        queue.append(vehicle)
        # print(f"{vehicle} added to queue at intersection {intersection_id}")

    def process_vehicle(self, intersection_id):
        if intersection_id in self.queue_pool and self.queue_pool[intersection_id]:
            vehicle = self.queue_pool[intersection_id].pop(0)
            self.remove_queue_if_empty(intersection_id)
            return vehicle
        return None

    def is_vehicle_in_queue(self, intersection_id, vehicle):
        if intersection_id in self.queue_pool:
            return vehicle in self.queue_pool[intersection_id]
        return False

intersection_manager = IntersectionManager()


class Vehicle:
    def __init__(self, id, path, speed, start_time=0, detection_range=0.002):
        self.id = id
        self.remaining_nodes = deque(path)
        self.passed_nodes = deque()
        # self.maxspeed = speed
        self.start_time = start_time
        self.current_time = 0
        self.position = None
        self.positions = []
        self.distances = self.compute_distances()
        self.total_distance = sum(self.distances)
        self.progress = 0
        self.detection_range = detection_range
        self.last_node = self.remaining_nodes[0]
        self.next_node = self.remaining_nodes[1]
        self.direction = [self.last_node, self.next_node]
        self.maxspeed = get_max_speed(self.direction, speed_limits)
        if len(self.remaining_nodes) > 2:
              self.nextdirection = [self.next_node,self.remaining_nodes[2]]
        else:
              self.nextdirection = []
        self.part_progress = 0
        self.current_segment_index = 0
        self.working = False
        self.adjusted_speed = 0
        self.turn_limit = 0.003  #distance for turning speed slowdown
        self.start_check = 0.0007   #distance that starts turning check
        self.check_range = 0.0015   #distance for cars that are entering

    def join_queue(self, intersection_id):

        global intersection_manager
        if not intersection_manager.is_vehicle_in_queue(intersection_id, self):
          intersection_manager.add_vehicle_to_queue(intersection_id, self)
          # print(f"{self} joined queue at intersection {intersection_id}")
        # else:
          # print(f"{self} already in queue at intersection {intersection_id}")

    def leave_queue(self, intersection_id):

        global intersection_manager
        vehicle = intersection_manager.process_vehicle(intersection_id)
        if vehicle == self:
            # print(f"{self} left queue at intersection {intersection_id}")
            pass
        else:
            # print(f"{self} was not at the front of queue {intersection_id}")
            pass

    def check_head(self, intersection_id):

        queue = intersection_manager.get_or_create_queue(intersection_id)
        # print(len(queue))
        if len(queue) != 0:
          if queue[0] == self:
            return True
        return False

    def compute_distances(self):
        distances = []
        self.edge_geometries = []
        path = list(self.remaining_nodes)
        # print(path)
        for i in range(len(path) - 1):
            u = path[i]
            v = path[i + 1]
            edge_data = G_sub.get_edge_data(u, v)
            geometry = edge_data[0].get("geometry")
            if geometry is None:
                coord_u = node_positions[u]
                coord_v = node_positions[v]
                geometry = LineString([coord_u, coord_v])
            self.edge_geometries.append(geometry)
            distances.append(geometry.length)
        # print(distances)
        return distances

    def remain_dist(self):
        return self.distances[self.current_segment_index]-self.part_progress

    def others_enter(self):
        for other_vehicle in vehicles:
            if other_vehicle == self:
                # print(self.direction)
                continue
            if other_vehicle.working and len(other_vehicle.direction)>1:
              if other_vehicle.direction != self.direction and other_vehicle.direction[1] == self.direction[1] and other_vehicle.remain_dist() < self.check_range:
                # print("detected other cars!")
                return True
        return False


    @staticmethod
    def compute_angle(direction, next_direction):
    # Extract the geometric information of the road
      geometry1 = G_sub[direction[0]][direction[1]][0].get('geometry', None)
      geometry2 = G_sub[next_direction[0]][next_direction[1]][0].get('geometry', None)
      intersection_node = set(direction).intersection(next_direction)
      if intersection_node:
          intersection_node = intersection_node.pop()
      else:
          raise ValueError("There's no intersection")

      if geometry1 is None:
          # If geometry1 is None, directly generate a vector using the coordinates of the nodes
          seg1 = [(G.nodes[direction[0]]['x'], G.nodes[direction[0]]['y']),
                  (G.nodes[direction[1]]['x'], G.nodes[direction[1]]['y'])]
      else:
          # If geometry1 exists, extract the line segment near the intersection according to the original logic
          if geometry1.coords[0] == (G.nodes[intersection_node]['x'], G.nodes[intersection_node]['y']):
              seg1 = geometry1.coords[:2]
          else:
              seg1 = geometry1.coords[-2:]

      if geometry2 is None:
          # If geometry2 is None, directly generate a vector using the coordinates of the nodes
          seg2 = [(G.nodes[next_direction[0]]['x'], G.nodes[next_direction[0]]['y']),
                  (G.nodes[next_direction[1]]['x'], G.nodes[next_direction[1]]['y'])]
      else:
          # If geometry2 exists, extract the line segment near the intersection according to the original logic
          if geometry2.coords[0] == (G.nodes[intersection_node]['x'], G.nodes[intersection_node]['y']):
              seg2 = geometry2.coords[:2]
          else:
              seg2 = geometry2.coords[-2:]

      # Calculate the direction vectors of the two line segments
      def direction_vector(segment):
          #If the input is a list of coordinates (not a LineString object), directly calculate the vector
          coords = list(segment)
          return (coords[1][0] - coords[0][0], coords[1][1] - coords[0][1])

      vec1 = direction_vector(seg1)
      vec2 = direction_vector(seg2)


      def calculate_angle(v1, v2):
          dot_product = v1[0] * v2[0] + v1[1] * v2[1]
          norm1 = math.sqrt(v1[0]**2 + v1[1]**2)
          norm2 = math.sqrt(v2[0]**2 + v2[1]**2)
          angle = math.acos(dot_product / (norm1 * norm2))
          return math.degrees(angle)

      angle = calculate_angle(vec1, vec2)
      angle = 180-angle
      # print(f"The angle is：{angle:.2f}°")
      return max(10,angle)


    def detect_vehicle_ahead(self, vehicles):
        """
        # Detect the vehicle ahead and return the nearest vehicle and its distance
        """
        global print_once
        if print_once:
            print("using lane capacity mode!")
            print_once = False
        u, v = self.direction
        edge_data = G_sub.get_edge_data(u, v)
        if edge_data:
            edge_attr = edge_data[0]
            lanes = edge_attr.get('lanes', 1)
            if isinstance(lanes, list):
                lanes = lanes[0]
            try:
                lanes = int(lanes)
            except (ValueError, TypeError):
                lanes = 1
        else:
            lanes = 1


        vehicles_ahead = 0

        min_distance = None
        nearest_vehicle = None

        for other_vehicle in vehicles:
            if other_vehicle == self:
                # print(self.direction)
                continue
            # else:
            #   print(other_vehicle.direction)
            # Determine if the vehicles are on the same road (assuming they follow the same path)
            if other_vehicle.working and other_vehicle.direction != [] and other_vehicle.remaining_nodes and other_vehicle.direction == self.direction:
                distance = other_vehicle.part_progress - self.part_progress
                # print("same road!")
                # print(distance)
                if self.detection_range>=distance > 0:
                  if min_distance is None or distance < min_distance:
                     vehicles_ahead += 1
                     min_distance = distance
                     nearest_vehicle = other_vehicle
                    #  print(min_distance)
            elif other_vehicle.working and other_vehicle.direction != [] and other_vehicle.remaining_nodes and other_vehicle.direction == self.nextdirection:
                distance = other_vehicle.part_progress + (self.distances[self.current_segment_index]-self.part_progress)
                # print(distance)
                if self.detection_range>=distance > 0:
                  if min_distance is None or distance < min_distance:
                     vehicles_ahead += 1
                     min_distance = distance
                     nearest_vehicle = other_vehicle

        if vehicles_ahead >= lanes:

            return nearest_vehicle, min_distance
        else:
            return False, self.detection_range

#one lane version
    # def detect_vehicle_ahead(self, vehicles):

    #     min_distance = None
    #     nearest_vehicle = None
    #
    #     for other_vehicle in vehicles:
    #         if other_vehicle == self:
    #             # print(self.direction)
    #             continue
    #         # else:
    #         #   print(other_vehicle.direction)

    #         if other_vehicle.working and other_vehicle.direction != [] and other_vehicle.remaining_nodes and other_vehicle.direction == self.direction:
    #             distance = other_vehicle.part_progress - self.part_progress
    #             # print("same road!")
    #             # print(distance)
    #             if self.detection_range>=distance > 0:
    #               if min_distance is None or distance < min_distance:
    #                  min_distance = distance
    #                  nearest_vehicle = other_vehicle
    #                 #  print(min_distance)
    #         elif other_vehicle.working and other_vehicle.direction != [] and other_vehicle.remaining_nodes and other_vehicle.direction == self.nextdirection:
    #             distance = other_vehicle.part_progress + (self.distances[self.current_segment_index]-self.part_progress)
    #             # print(distance)
    #             if self.detection_range>=distance > 0:
    #               if min_distance is None or distance < min_distance:
    #                  min_distance = distance
    #                  nearest_vehicle = other_vehicle
    #
    #     return nearest_vehicle, min_distance

    def move(self, delta_t, vehicles):
        if self.current_time < self.start_time:
            self.last_node = self.remaining_nodes[0]
            self.next_node = self.remaining_nodes[1]

            self.position = node_positions[self.remaining_nodes[0]]
            self.positions.append(self.position)
            self.current_time += delta_t
            self.direction = [self.last_node,self.next_node]
            if len(self.remaining_nodes) > 2:
              self.nextdirection = [self.next_node,self.remaining_nodes[2]]

            return True
        else:
            self.working = True


        if not self.remaining_nodes:
            self.last_node = self.remaining_nodes[-2]
            self.next_node = self.remaining_nodes[-1]
            self.direction = []
            self.nextdirection = []
            self.passed_nodes.append(self.remaining_nodes.popleft())
            self.position = node_positions[self.passed_nodes[-1]]
            self.positions.append(self.position)
            self.current_time += delta_t
            return False



        nearest_vehicle, distance = self.detect_vehicle_ahead(vehicles)
        dist_tonode = self.distances[self.current_segment_index]-self.part_progress
        if self.id == 0:
          # print(self.nextdirection)
          pass
        if nearest_vehicle and distance <= self.detection_range:
            if dist_tonode <= self.start_check and self.others_enter():
              self.join_queue(self.direction[1])
              if not self.check_head(self.direction[1]):
                # print("waiting in queue!")
                self.adjusted_speed = min(self.maxspeed * dist_tonode / (self.start_check), max(0, self.maxspeed * (distance / self.detection_range)))
              else:
                if dist_tonode <= self.turn_limit and self.nextdirection != []:
                  deceleration = (self.adjusted_speed - self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection) / 180)) / max(dist_tonode*200, 1)
                  self.adjusted_speed = min(self.maxspeed * dist_tonode / (self.start_check), max(0,max(self.adjusted_speed - deceleration, self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection)/180))))
                else:
                  self.adjusted_speed = min(self.maxspeed * dist_tonode / (self.start_check), max(0,self.maxspeed))
            else:
              # deceleration
              self.adjusted_speed = self.maxspeed * (distance / self.detection_range)
              self.adjusted_speed = max(0, self.adjusted_speed)
        elif dist_tonode <= self.start_check and self.others_enter():
            # print("crossing mode")
            self.join_queue(self.direction[1])
            if not self.check_head(self.direction[1]):
              # print("waiting in queue!")
              self.adjusted_speed = max((self.maxspeed * dist_tonode / (self.start_check)),0.00005)
            else:
              if dist_tonode <= self.turn_limit and self.nextdirection != []:
                deceleration = (self.adjusted_speed - self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection) / 180)) / max(dist_tonode*200, 1)
                self.adjusted_speed = max(self.adjusted_speed - deceleration, self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection)/180))
              else:
                self.adjusted_speed = self.adjusted_speed + (self.maxspeed-self.adjusted_speed)/2
        elif dist_tonode <= self.turn_limit and self.nextdirection != []:
            # print("turning mode")
            # print(f"Car {self.id} is turning! ")
            # print(self.distances)
            deceleration = (self.adjusted_speed - self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection) / 180)) / max(dist_tonode*200, 1)
            # print(f"deceleration is {deceleration}")
            self.adjusted_speed = max(max(self.adjusted_speed - deceleration, self.maxspeed * (vehicle.compute_angle(self.direction, self.nextdirection)/180)),0.00005)
            # print(f"current speed {self.adjusted_speed}")
        else:
            self.adjusted_speed = self.adjusted_speed + (self.maxspeed-self.adjusted_speed)/2

        self.progress += self.adjusted_speed * delta_t
        self.part_progress += self.adjusted_speed * delta_t
        for i, (segment_distance, geometry) in enumerate(zip(self.distances, self.edge_geometries)):
          if i < self.current_segment_index:
            continue
          else:
            if self.part_progress > segment_distance:
                if self.check_head(self.direction[1]):
                    # print(f"leave queue! Node is {self.direction[1]}")
                    self.leave_queue(self.direction[1])
                self.part_progress -= segment_distance
                self.current_segment_index += 1
                # print(self.current_segment_index)
                if len(self.remaining_nodes) > 2:
                    # print("changing direction!")
                    passed_node = self.remaining_nodes.popleft()
                    self.passed_nodes.append(passed_node)
                    self.last_node = self.remaining_nodes[0]
                    self.next_node = self.remaining_nodes[1]
                    self.direction = [self.last_node,self.next_node]
                    self.maxspeed = get_max_speed(self.direction, speed_limits)
                    if len(self.remaining_nodes) > 2:
                      self.nextdirection = [self.next_node,self.remaining_nodes[2]]
                    else:
                      self.nextdirection = []

                else:
                    self.direction = []
            else:
                # interpolate along geometry
                interpolated_point = geometry.interpolate(self.part_progress)
                self.position = (interpolated_point.x, interpolated_point.y)
                self.positions.append(self.position)
                self.current_time += delta_t
                return True

        self.position = node_positions[self.remaining_nodes[-1]]
        self.passed_nodes.append(self.remaining_nodes.popleft())
        self.positions.append(self.position)
        self.current_time += delta_t
        return False



# file_path = 'routes_simple_nodes.json'  # your path
# with open(file_path, 'r') as file:
#     data = json.load(file)
#
#
# paths = []
#
#
# for index, entry in enumerate(data, start=1):
#     path_name = f"path{index}"
#     path = entry['nodes']
#     globals()[path_name] = path
#     paths.append(globals()[path_name])



file_path = 'optimal_routes.json'
with open(file_path, 'r') as file:
    data = json.load(file)


paths = []


for index, route in enumerate(data['evacuation_routes'], start=1):
    path_name = f"path{index}"
    path = route['route']
    globals()[path_name] = path
    paths.append(globals()[path_name])


# for index, path in enumerate(paths, start=1):
#     print(f"path{index}: {path}")


print("\nAll paths added to 'paths' list:")

# Vehicle para
vehicle_params = []
vehicle_id = 0

for path_index, path in enumerate(paths):
    for i in range(2):  # 2 cars each route
        vehicle_params.append({
            'id': vehicle_id,
            'path': path,
            'speed': 0.001,
            'start_time': i*0.1
        })
        vehicle_id += 1


# for vehicle in vehicle_params:
#     print(vehicle)
vehicles = [Vehicle(**params) for params in vehicle_params]


moving = [True] * len(vehicles)

delta_t = 0.1
max_frames = 9000

for _ in range(max_frames):
    all_stopped = True
    for i, vehicle in enumerate(vehicles):
        if moving[i]:
            moving[i] = vehicle.move(delta_t, vehicles)
            if moving[i]:
                all_stopped = False
    if all_stopped:
        print("All vehicles have stopped.")
        break

path_coords = [vehicle.positions for vehicle in vehicles]


# ANIMATION

fig, ax = plt.subplots(figsize=(40,20))
edges_sub.plot(ax=ax, linewidth=0.9, color="gray", alpha=0.9)
# ctx.add_basemap(ax, crs=edges_sub.crs.to_string(), source=ctx.providers.OpenStreetMap.Mapnik,  alpha=0.3)

longitude = -157.8494
latitude = 21.2969
radius = 0.05

center_point = Point(longitude, latitude)
disaster_zone_circle = center_point.buffer(radius)

disaster_artist = Circle((longitude, latitude), radius, edgecolor='red', facecolor='none', linewidth=2, zorder=2, alpha = 0.5)
ax.add_patch(disaster_artist)

time_text = ax.text(1, 1, '', transform=ax.transAxes, ha='right', va='top',  fontsize=20)

ax.set_title("Vehicle Movement on Road Network")
ax.set_xlim(edges_sub["geometry"].bounds.minx.min(), edges_sub["geometry"].bounds.maxx.max())
ax.set_ylim(edges_sub["geometry"].bounds.miny.min(), edges_sub["geometry"].bounds.maxy.max())
ax.set_axis_off()
plt.tight_layout()
# plot_params = [
#     {"color": "red", "label": "Vehicle 1"},
#     {"color": "blue", "label": "Vehicle 2"},
#     {"color": "cyan", "label": "Vehicle 3"},
#     {"color": "green", "label": "Vehicle 4"},
#     {"color": "orange", "label": "Vehicle 5"},
#     {"color": "purple", "label": "Vehicle 6"},
#     {"color": "grey", "label": "Vehicle 7"},
#     {"color": "brown", "label": "Vehicle 8"},
#
# ]
colors = [
    "red", "blue", "cyan", "green", "orange",
    "purple", "grey", "brown", "yellow", "pink"
]

plot_params = [
    {
        "color": colors[i % len(colors)],
        "label": f"Vehicle {i + 1}"
    }
    for i in range(len(vehicles))
]

points = [ax.plot([], [], 'o', markersize=8, **params)[0] for params in plot_params]

# ax.legend(loc='upper right')

timer_running = True
evacuation_time = None

def update(frame):
    global timer_running, evacuation_time


    for i, point in enumerate(points):
        if frame < len(path_coords[i]):
            x, y = path_coords[i][frame]
            point.set_data([x], [y])
        else:
            point.set_data([], [])

    all_outside = True
    for i in range(len(points)):
        if frame < len(path_coords[i]):
            x, y = path_coords[i][frame]
            vehicle_point = Point(x, y)
            if disaster_zone_circle.contains(vehicle_point):
                all_outside = False
                break

    if all_outside:
        disaster_artist.set_color('blue')
        if timer_running:
            timer_running = False
            evacuation_time = frame * delta_t
    else:
        disaster_artist.set_color('red')

    if timer_running:
        current_time = frame * delta_t
        time_text.set_text(f'Time elapsed: {current_time:.1f}s')
    else:
        time_text.set_text(f'Total time: {evacuation_time:.1f}s')
        time_text.set_fontsize(40)

    return points + [disaster_artist, time_text]


num_frames = max(len(coords) for coords in path_coords)
print(f"{num_frames} frames")

ani = animation.FuncAnimation(
    fig, update, frames=num_frames, interval=40, blit=True
)


# from IPython.display import HTML
# HTML(ani.to_html5_video())
pbar = tqdm(total=num_frames)
def progress_callback(current_frame, total_frames):
    pbar.update(current_frame - pbar.n)
ani.save("long_vehicle_path_animation.mp4", writer="ffmpeg", fps=30, extra_args=['-threads', '16'], progress_callback=progress_callback)
pbar.close()
