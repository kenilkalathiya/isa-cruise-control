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

# Global Overlay Variables
overlay_speed = 0.0
overlay_limit = 0.0
overlay_state = "INIT"


# NEW: Look ahead distance for steering (meters)
LOOKAHEAD_DIST = 3.0 


# class SpeedLimitManager:
#     def __init__(self):
#         self.current_limit = 10.0  # initial speed limit
#         self.start_time = time.time()

#     def update(self):
#         """Simulate road speed-limit changes over time"""
#         elapsed = time.time() - self.start_time

#         if elapsed < 5:
#             self.current_limit = 20.0
#         elif elapsed < 10:
#             self.current_limit = 5.0
#         elif elapsed < 15:
#             self.current_limit = 20.0
#         else:
#             self.current_limit = 30.0

#         return self.current_limit

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

def spawn_visual_speed_sign(world, transform):
    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.find("static.prop.constructioncone")

    sign = world.try_spawn_actor(bp, transform)
    if sign:
        print("✅ Spawned visual speed sign (cone)")
    return sign

def get_forced_speed_limit(vehicle, start_location):
    """
    Simulate road speed limits using distance traveled
    """
    loc = vehicle.get_location()
    dist = loc.distance(start_location)

    if dist < 50:
        return 30.0
    elif dist < 100:
        return 50.0
    elif dist < 150:
        return 20.0
    else:
        return 40.0




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

    start_location = vehicle.get_location()


    vehicle.set_autopilot(False)
    vehicle.set_simulate_physics(True)

    # -------- SPAWN VISUAL SPEED SIGNS (VISION TARGETS) --------
    spawned_signs = []
    road_map = world.get_map()
    wp = road_map.get_waypoint(vehicle.get_location())

    for i in range(3):
        sign_transform = wp.transform

        # Right side of road
        sign_transform.location += sign_transform.get_right_vector() * 3.0

        # Much closer
        sign_transform.location += sign_transform.get_forward_vector() * (8 + i * 8)

        # Raise it clearly into camera view
        sign_transform.location.z += 1.0

        sign = spawn_visual_speed_sign(world, sign_transform)
        if sign:
            spawned_signs.append(sign)




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

    # speed_manager = SpeedLimitManager()

    # Set spectator camera to follow car
    spectator = world.get_spectator()

    print("✅ CARLA CRUISE + STEERING RUNNING")

    try:
        cv2.namedWindow("Front Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Front Camera", 800, 600)
        while True:
            world.tick()
            # TARGET_SPEED = speed_manager.update()
            # current_wp = world.get_map().get_waypoint(vehicle.get_location())
            # TARGET_SPEED = vehicle.get_speed_limit()
            TARGET_SPEED = get_forced_speed_limit(vehicle, start_location)







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


            overlay_speed = speed
            overlay_limit = TARGET_SPEED
            overlay_state = state


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
                frame = latest_frame.copy()

                    # -------- ROI FOR SPEED LIMIT SIGNS --------
                h, w, _ = frame.shape

                roi_x1 = int(0.45 * w)
                roi_y1 = int(0.05 * h)
                roi_x2 = int(0.95 * w)
                roi_y2 = int(0.60 * h)

                roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]

                # Convert ROI to grayscale
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

                # Blur to reduce noise
                gray = cv2.GaussianBlur(gray, (9, 9), 1.5)

                circles = cv2.HoughCircles(
                    gray,
                    cv2.HOUGH_GRADIENT,
                    dp=1.4,          # ↑ less sensitive
                    minDist=80,      # ↑ prevents clustered detections
                    param1=120,      # ↑ stronger edge requirement
                    param2=40,       # ↑ stricter circle threshold
                    minRadius=20,    # ignore tiny circles
                    maxRadius=60     # ignore large objects
                )

                edges = cv2.Canny(gray, 100, 200)

                
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    for c in circles[0, :]:
                        cx, cy, r = c

                        # Edge validation: check how many edges exist in the circle
                        mask = np.zeros_like(edges)
                        cv2.circle(mask, (cx, cy), r, 255, -1)

                        edge_pixels = cv2.countNonZero(cv2.bitwise_and(edges, edges, mask=mask))

                        # Reject weak edge circles
                        if edge_pixels < 150:
                            continue

                        # Draw circle on original frame (offset by ROI)
                        cv2.circle(frame,(roi_x1 + cx, roi_y1 + cy),r,(0, 255, 0),2)
                        cv2.circle(frame,(roi_x1 + cx, roi_y1 + cy),2,(0, 0, 255),3)


                # Draw ROI box
                cv2.rectangle(frame,
                    (roi_x1, roi_y1),
                    (roi_x2, roi_y2),
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    frame,
                    "ROI: Speed Limit Signs",
                    (roi_x1, roi_y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    frame,
                    "LOOK FOR CONES HERE",
                    (roi_x1, roi_y2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )


                cv2.putText(
                    frame,
                    f"Speed: {overlay_speed:.1f} km/h",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Limit: {overlay_limit:.0f} km/h",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"State: {overlay_state}",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2
                )


                cv2.imshow("Front Camera", frame)
            cv2.waitKey(1)
            time.sleep(0.001)


            prev_error = error
            
            # Simple debug print
            # print(f"Speed: {speed:4.1f} | Limit: {TARGET_SPEED:3.0f} | State: {state}")

    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()

        for sign in spawned_signs:
            sign.destroy()


        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        print("Done.")

if __name__ == "__main__":
    main()