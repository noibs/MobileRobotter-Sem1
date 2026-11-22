from robot_config import collector

NUM_PACKAGES = 5

# Same order as a real run: fill the magazine, then empty it
try:
    for i in range(NUM_PACKAGES):
        print(f"Package {i + 1}/{NUM_PACKAGES}: picking up...")
        collector.pickup_package()

    for i in range(NUM_PACKAGES):
        print(f"Package {i + 1}/{NUM_PACKAGES}: delivering...")
        collector.deliver_package()

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    collector.deinit()
    print("Collector shut down cleanly.")
