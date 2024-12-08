We recommend you run this project in your own IDE

1.Install the requirements
2.Get a project code from Google Earth Engine
3.Run 500_500_of_Density_based[D]_large_scale_Dij[D]_Ant[D]_LLM[D]_ROUTE(1)
4.Run animation.py

There are a few parameters that you might want to adjust. In  " 500_500_of_Density_based[D]_large_scale_Dij[D]_Ant[D]_LLM[D]_ROUTE(1)", you can change the number of inside points at:
# Generate evacuation points
m, points_inside = generate_random_points_inside_circle(
    m,
    center_longitude=longitude,
    center_latitude=latitude,
    population_density_image=pop_density_2020,
    num_points=500,
    radius=0.05
)
and outside points at:
# Generate evacuation points just outside the disaster zone
m, points_outside = generate_evacuation_points_outside_circle(
    m, longitude, latitude, num_points=100, radius=disaster_radius, ring_width=ring_width
)
Also, don't forget to fill in your own ee project code at:
ee.Initialize(project='')

For animation.py, you also have some choices
If you want to change the number of cars generated at each starting point, go to:
for path_index, path in enumerate(paths):
    for i in range(2):  # 2 cars each route
        vehicle_params.append({
            'id': vehicle_id,
            'path': path,
            'speed': 0.001,
            'start_time': i*0.1
        })
        vehicle_id += 1

Since we have three kinds of routes every time, you can also use the other two path files to animate.
Simply  comment :
"file_path = 'optimal_routes.json'
with open(file_path, 'r') as file:
    data = json.load(file)


paths = []


for index, route in enumerate(data['evacuation_routes'], start=1):
    path_name = f"path{index}"
    path = route['route']
    globals()[path_name] = path
    paths.append(globals()[path_name])"

And uncomment :
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
If you want aco path, change "simple" to "aco" 

Then you can run the code.