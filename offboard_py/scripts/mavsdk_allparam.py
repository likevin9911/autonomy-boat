#DO NOT USE MAVSDK-PYTHON NOT COMPATIBLE WITH ARM SYSTEMS
# TODO pip3 uninstall mavsdk-python
#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Get the list of parameters
    all_params = await drone.param.get_all_params()
    print("Connected to the drone")


    # Iterate through all int parameters
    for param in all_params.int_params:
        print(f"{param.name}: {param.value}")

    for param in all_params.float_params:
        print(f"{param.name}: {param.value}")

# Run the asyncio loop
asyncio.run(run())
