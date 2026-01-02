import carla
import random
import math
import time
import numpy as np
import cv2


# ================= CONFIG =================
DT = 0.05            

# PID (CRUISE)
KP = 0.08
KI = 0.03
KD = 0.02

MAX_THROTTLE = 0.75
MAX_BRAKE = 0.5
CREEP_THROTTLE = 0.30

ACCEL_MARGIN = 6.0
DECEL_MARGIN = 3.0

integral = 0.0
prev_error = 0.0

latest_frame = None


# NEW: Look ahead distance for steering (meters)
LOOKAHEAD_DIST = 3.0 


class SpeedLimitManager:
    def __init__(self):
        self.current_limit = 10.0  # initial speed limit
        self.start_time = time.time()

    def update(self):
        """Simulate road speed-limit changes over time"""
        elapsed = time.time() - self.start_time

        if elapsed < 5:
            self.current_limit = 20.0
        elif elapsed < 10:
            self.current_limit = 5.0
        elif elapsed < 15:
            self.current_limit = 20.0
        else:
            self.current_limit = 30.0

        return self.current_limit

def get_speed_kmh(vehicle):
    v = vehicle.get_velocity()
    return math.sqrt(v.x**2 + v.y**2 + v.z**2) * 3.6


def process_image(image):
    global latest_frame

    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    frame = array[:, :, :3]
    latest_frame = frame




# --- NEW STEERING FUNCTION ---
def compute_steering(vehicle, waypoint):
    transform = vehicle.get_transform()
    loc = transform.location
    rot = transform.rotation
    target_loc = waypoint.transform.location
    
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    target_yaw = math.degrees(math.atan2(dy, dx))
    
    diff = target_yaw - rot.yaw
    while diff > 180.0: diff -= 360.0
    while diff < -180.0: diff += 360.0
    
    return max(-1.0, min(1.0, diff * 0.03)) # 0.03 is the steering sensitivity

def main():
    global integral, prev_error

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)

    # Clean previous cars? (Optional safety)
    
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    spawn_points = world.get_map().get_spawn_points()
    spawn = spawn_points[0] if spawn_points else random.choice(spawn_points)
    
    vehicle = world.try_spawn_actor(bp, spawn)

    if not vehicle:
        vehicle = world.try_spawn_actor(bp, random.choice(spawn_points))
        if not vehicle: return

    vehicle.set_autopilot(False)
    vehicle.set_simulate_physics(True)

        # ----- attach camera -----
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "800")
    camera_bp.set_attribute("image_size_y", "600")
    camera_bp.set_attribute("fov", "90")

    camera_transform = carla.Transform(
        carla.Location(x=1.5, z=2.4)
    )

    camera = world.spawn_actor(
        camera_bp,
        camera_transform,
        attach_to=vehicle
    )

    camera.listen(lambda image: process_image(image))

    speed_manager = SpeedLimitManager()

    # Set spectator camera to follow car
    spectator = world.get_spectator()

    print("✅ CARLA CRUISE + STEERING RUNNING")

    try:
        cv2.namedWindow("Front Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Front Camera", 800, 600)
        while True:
            world.tick()
            TARGET_SPEED = speed_manager.update()


            # 1. Update Camera
            t = vehicle.get_transform()
            spectator.set_transform(carla.Transform(t.location + carla.Location(z=20), carla.Rotation(pitch=-90)))

            # 2. Get Speed & Error
            speed = get_speed_kmh(vehicle)
            error = TARGET_SPEED - speed

            # 3. CALCULATE STEERING (The Fix)
            # Find the closest waypoint on the road
            current_waypoint = world.get_map().get_waypoint(vehicle.get_location())
            # Look ahead on the road (returns a list, we take the first one)
            next_waypoint = current_waypoint.next(LOOKAHEAD_DIST)[0]
            
            steer = compute_steering(vehicle, next_waypoint)


            # 4. PID Throttle Logic
            throttle = 0.0
            brake = 0.0

            if speed < TARGET_SPEED - ACCEL_MARGIN:
                state = "ACCEL"
                throttle = MAX_THROTTLE
                integral = 0.0
            elif speed > TARGET_SPEED + DECEL_MARGIN:
                state = "DECEL"
                brake = min((speed - TARGET_SPEED) * 0.1, MAX_BRAKE)
                integral = 0.0
            else:
                state = "CRUISE"
                integral += error * DT
                derivative = (error - prev_error) / DT
                pid = KP * error + KI * integral + KD * derivative
                throttle = min(pid, MAX_THROTTLE) if pid > 0 else 0.0

            if speed < 5.0 and brake == 0.0:
                throttle = max(throttle, CREEP_THROTTLE)

            # 5. Apply Control
            vehicle.apply_control(
                carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=steer,         # <--- USING CALCULATED STEER
                    manual_gear_shift=False
                )
            )

            if latest_frame is not None:
                cv2.imshow("Front Camera", latest_frame)
            cv2.waitKey(1)
            time.sleep(0.001)


            prev_error = error
            
            # Simple debug print
            print(f"Speed: {speed:4.1f} | Limit: {TARGET_SPEED:3.0f} | State: {state}")

    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()

        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("Done.")

if __name__ == "__main__":
    main()