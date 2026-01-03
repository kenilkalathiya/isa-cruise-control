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

# ===== ISA STATES =====
ISA_ACTIVE = "ACTIVE"
ISA_OVERRIDDEN = "OVERRIDDEN"
ISA_DISABLED = "DISABLED"

# ===== TIMING CONFIG =====
STARTUP_OVERRIDE_DELAY = 5.0   # seconds
OVERRIDE_DURATION = 6.0        # seconds
OVERRIDE_LOCKOUT = 7.0         # seconds

isa_state = ISA_ACTIVE
startup_delay_timer = STARTUP_OVERRIDE_DELAY
override_timer = 0.0
override_lockout_timer = 0.0


# ===== ACC CONFIG =====
ACC_ENABLED = True
TIME_GAP = 1.5        # seconds
MIN_DISTANCE = 8.0    # meters


# NEW: Look ahead distance for steering (meters)
LOOKAHEAD_DIST = 3.0 


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


def compute_lead_steering(vehicle, lookahead=6.0):
    world = vehicle.get_world()
    current_wp = world.get_map().get_waypoint(
        vehicle.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    next_wps = current_wp.next(lookahead)
    if not next_wps:
        return 0.0

    target_wp = next_wps[0]

    veh_tf = vehicle.get_transform()
    veh_loc = veh_tf.location
    veh_yaw = math.radians(veh_tf.rotation.yaw)

    wp_loc = target_wp.transform.location

    dx = wp_loc.x - veh_loc.x
    dy = wp_loc.y - veh_loc.y

    target_angle = math.atan2(dy, dx)
    angle_diff = target_angle - veh_yaw

    # Normalize angle to [-pi, pi]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    steer = max(-1.0, min(1.0, angle_diff * 0.7))
    return steer


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

def get_lead_vehicle(vehicle, world):
    ego_tf = vehicle.get_transform()
    ego_loc = ego_tf.location
    ego_forward = ego_tf.get_forward_vector()

    min_dist = None
    lead_vehicle = None

    for actor in world.get_actors().filter("vehicle.*"):
        if actor.id == vehicle.id:
            continue

        loc = actor.get_location()
        vec = loc - ego_loc

        # Check if vehicle is in front
        dot = ego_forward.x * vec.x + ego_forward.y * vec.y
        if dot <= 0:
            continue

        dist = ego_loc.distance(loc)
        if min_dist is None or dist < min_dist:
            min_dist = dist
            lead_vehicle = actor

    return lead_vehicle, min_dist



def spawn_simple_lead_vehicle(world, ego_vehicle, target_distance=200.0):
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")

    ego_wp = world.get_map().get_waypoint(
        ego_vehicle.get_location(),
        project_to_road=True,
        lane_type=carla.LaneType.Driving
    )

    travelled = 0.0
    wp = ego_wp

    while travelled < target_distance:
        next_wps = wp.next(2.0)
        if not next_wps:
            break
        wp = next_wps[0]
        travelled += 2.0

    lead_tf = wp.transform
    lead_tf.location.z += 1.5

    lead = world.try_spawn_actor(bp, lead_tf)

    if lead:
        lead.set_autopilot(False)
        lead.set_simulate_physics(True)
        # lead.apply_control(carla.VehicleControl(brake=1.0))
        print(f"✅ Lead vehicle spawned ~{travelled:.1f} m ahead")

    else:
        print("❌ Lead vehicle spawn failed")

    return lead



def main():
    global integral, prev_error
    global isa_state
    global startup_delay_timer, override_timer, override_lockout_timer

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

    for _ in range(10):
        world.tick()
        time.sleep(0.05)


    lead_vehicle_test = spawn_simple_lead_vehicle(world, vehicle)



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

    start_time = time.time()

    try:
        cv2.namedWindow("Front Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Front Camera", 800, 600)
        while True:
            world.tick()

           # ===== LEAD VEHICLE CONTROL =====
            sim_time = time.time() - start_time

            if lead_vehicle_test:
                steer = compute_lead_steering(lead_vehicle_test)

                throttle = 0.0
                brake = 0.0

                if sim_time < 6.0:
                    throttle = 0.15
                    brake = 0.0
                elif sim_time < 10.0:
                    throttle = 0.0
                    brake = 0.5
                else:
                    throttle = 0.15
                    brake = 0.0

                lead_vehicle_test.apply_control(
                    carla.VehicleControl(
                        throttle=throttle,
                        brake=brake,
                        steer=steer,
                        hand_brake=False
                    )
                )


            # ===== TIMER UPDATE =====
            if startup_delay_timer > 0:
                startup_delay_timer -= DT

            if override_timer > 0:
                override_timer -= DT

            if override_lockout_timer > 0:
                override_lockout_timer -= DT


            speed = get_speed_kmh(vehicle)

            # TARGET_SPEED = speed_manager.update()
            # current_wp = world.get_map().get_waypoint(vehicle.get_location())
            # TARGET_SPEED = vehicle.get_speed_limit()
            if isa_state == ISA_ACTIVE:
                TARGET_SPEED = get_forced_speed_limit(vehicle, start_location)
            else:
                TARGET_SPEED = 80.0  # effectively disables ISA

            
            # ===== ACC LOGIC (v1) =====
            lead_vehicle = None
            lead_dist = None

            # if ACC_ENABLED and isa_state == ISA_ACTIVE:
            if ACC_ENABLED:
                lead_vehicle, lead_dist = get_lead_vehicle(vehicle, world)

                if lead_vehicle is not None and lead_dist is not None:
                    safe_dist = max(MIN_DISTANCE, (speed / 3.6) * TIME_GAP)

                    if lead_dist < safe_dist:
                        TARGET_SPEED = min(
                            TARGET_SPEED,
                            speed * (lead_dist / safe_dist)
                        )

            # ===== ACC SAFETY LAYER (ALWAYS ACTIVE) =====
            lead_vehicle, lead_dist = get_lead_vehicle(vehicle, world)

            if lead_vehicle and lead_dist is not None:
                safe_dist = max(MIN_DISTANCE, (speed / 3.6) * TIME_GAP)

                if lead_dist < safe_dist:
                    acc_speed = speed * (lead_dist / safe_dist)
                    TARGET_SPEED = min(TARGET_SPEED, acc_speed)



            # ===== DRIVER OVERRIDE DETECTION (STABLE) =====
            control = vehicle.get_control()

            # Driver override allowed only if ISA is active
            if (
                control.throttle > 0.7
                and isa_state == ISA_ACTIVE
                and startup_delay_timer <= 0
                and override_lockout_timer <= 0
            ):
                isa_state = ISA_OVERRIDDEN
                override_timer = OVERRIDE_DURATION

            if isa_state == ISA_OVERRIDDEN and override_timer <= 0:
                isa_state = ISA_ACTIVE
                override_lockout_timer = OVERRIDE_LOCKOUT


            # 1. Update Camera
            t = vehicle.get_transform()
            spectator.set_transform(carla.Transform(t.location + carla.Location(z=20), carla.Rotation(pitch=-90)))

            # 2. Get Speed & Error
            
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
            # overlay_state = state
            overlay_state = f"{state} | ISA: {isa_state}"


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

                cv2.putText(
                    frame,
                    f"Startup: {startup_delay_timer:.1f}s | Override: {override_timer:.1f}s | Lockout: {override_lockout_timer:.1f}s",
                    (20, 160),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

                if lead_vehicle and lead_dist:
                    cv2.putText(
                        frame,
                        f"ACC: Lead {lead_dist:.1f} m",
                        (20, 200),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 180, 255),
                        2
                    )

                cv2.imshow("Front Camera", frame)
            cv2.waitKey(1)
            time.sleep(0.001)

            prev_error = error
            

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