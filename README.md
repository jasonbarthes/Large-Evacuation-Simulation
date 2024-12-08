# Project Instructions

We recommend you run this project in your own IDE. Otherwise you need to put animation.py in your jupyter notebook and modify it a bit.

## Steps to Run the Project

1. **Install the requirements**  
   Install all required dependencies using `pip install -r requirements.txt`.

2. **Get a project code from Google Earth Engine**  
   Fill in your Earth Engine project code in the relevant section.

3. **Run the following scripts:**
   - `500_500_of_Density_based[D]_large_scale_Dij[D]_Ant[D]_LLM[D]_ROUTE(1)`
   - `animation.py`


## Adjusting Parameters

### In `500_500_of_Density_based[D]_large_scale_Dij[D]_Ant[D]_LLM[D]_ROUTE(1)`:
- To change the **number of inside points**, modify:
  ```python
  # Generate evacuation points
  m, points_inside = generate_random_points_inside_circle(
      m,
      center_longitude=longitude,
      center_latitude=latitude,
      population_density_image=pop_density_2020,
      num_points=500,
      radius=0.05
  )
- To change the number of outside points, modify:
  ```python
  # Generate evacuation points just outside the disaster zone
  m, points_outside = generate_evacuation_points_outside_circle(
      m, longitude, latitude, num_points=100, radius=disaster_radius, ring_width=ring_width
  )
- Add your Earth Engine project code:
  ```python
  ee.Initialize(project='')
### In animation.py:
- To change the number of cars generated at each starting point, adjust:
  ```python
  for path_index, path in enumerate(paths):
    for i in range(2):  # 2 cars each route
        vehicle_params.append({
            'id': vehicle_id,
            'path': path,
            'speed': 0.001,
            'start_time': i*0.1
        })
        vehicle_id += 1
- Using alternative route files: To use the other two path files for animation:
- - Comment out:
    ```python
    file_path = 'optimal_routes.json'
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    paths = []
    
    for index, route in enumerate(data['evacuation_routes'], start=1):
        path_name = f"path{index}"
        path = route['route']
        globals()[path_name] = path
        paths.append(globals()[path_name])
- - Uncomment:
    ```python
    file_path = 'routes_simple_nodes.json'  # your path
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    paths = []
    
    for index, entry in enumerate(data, start=1):
        path_name = f"path{index}"
        path = entry['nodes']
        globals()[path_name] = path
        paths.append(globals()[path_name])
- - Switching to ACO path: Replace "simple" with "aco" in the file path.

## Running the Code
After making the necessary adjustments, run the scripts in your IDE to execute the project.
