import carla

def main() -> None:
    client = carla.Client('host.docker.internal', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    print(f'CARLA server: {client.get_server_version()}')
    print(f'Map: {world.get_map().name}')

    vehicles = world.get_actors().filter('vehicle.*')
    ego = [
        actor for actor in vehicles
        if actor.attributes.get('role_name', '') == 'ego_vehicle'
    ]

    print(f'Vehicles: {len(vehicles)}')
    print(f'Ego vehicles: {len(ego)}')

    for actor in ego:
        print(
            f'Ego: id={actor.id} type={actor.type_id} '
            f'location={actor.get_location()}'
        )


if __name__ == '__main__':
    main()
