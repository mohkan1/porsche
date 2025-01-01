import carla
import random

def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Set up the simulator in synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enable synchronous mode
    settings.fixed_delta_seconds = 0.05  # Fixed time step
    world.apply_settings(settings)

    # Set up the Traffic Manager (TM) in synchronous mode
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_random_device_seed(0)
    random.seed(0)

    # Get spawn points and draw them for debugging
    spawn_points = world.get_map().get_spawn_points()
    for i, spawn_point in enumerate(spawn_points):
        world.debug.draw_string(spawn_point.location, str(i), life_time=10)

    # Spawn vehicles at random spawn points
    max_vehicles = 50
    max_vehicles = min([max_vehicles, len(spawn_points)])
    blueprints = []
    models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
    for vehicle in world.get_blueprint_library().filter('*vehicle*'):
        if any(model in vehicle.id for model in models):
            blueprints.append(vehicle)

    vehicles = []
    for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
        blueprint = random.choice(blueprints)
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle:
            vehicles.append(vehicle)

    # Let TM control the vehicles
    for vehicle in vehicles:
        vehicle.set_autopilot(True)
        traffic_manager.ignore_lights_percentage(vehicle, random.randint(0, 50))

    # Define two routes for traffic
    route_1_indices = [129, 28, 124, 33, 97, 119, 58, 154, 147]
    route_2_indices = [21, 76, 38, 34, 90, 3]
    route_1 = [spawn_points[ind].location for ind in route_1_indices]
    route_2 = [spawn_points[ind].location for ind in route_2_indices]

    spawn_point_1 = spawn_points[32]
    spawn_point_2 = spawn_points[149]

    world.debug.draw_string(spawn_point_1.location, 'Spawn point 1', life_time=30, color=carla.Color(255, 0, 0))
    world.debug.draw_string(spawn_point_2.location, 'Spawn point 2', life_time=30, color=carla.Color(0, 0, 255))

    for ind in route_1_indices:
        world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(255, 0, 0))

    for ind in route_2_indices:
        world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(0, 0, 255))

    # Spawn additional vehicles following routes
    spawn_delay = 20
    counter = spawn_delay
    alt = False
    max_route_vehicles = 200

    while True:
        world.tick()

        n_vehicles = len(world.get_actors().filter('*vehicle*'))
        if counter == spawn_delay and n_vehicles < max_route_vehicles:
            vehicle_bp = random.choice(blueprints)
            spawn_point = spawn_point_1 if alt else spawn_point_2
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

            if vehicle:
                vehicle.set_autopilot(True)
                traffic_manager.update_vehicle_lights(vehicle, True)
                traffic_manager.random_left_lanechange_percentage(vehicle, 0)
                traffic_manager.random_right_lanechange_percentage(vehicle, 0)
                traffic_manager.auto_lane_change(vehicle, False)

                # Assign route
                if alt:
                    traffic_manager.set_path(vehicle, route_1)
                else:
                    traffic_manager.set_path(vehicle, route_2)
                alt = not alt

            counter = 0
        counter += 1

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Simulation interrupted.")
