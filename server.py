import asyncio
import json
import numpy as np
import trimesh
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

# Import the Aircraft class from the other file
from aircraft import Aircraft

# --- Configuration ---
DT = 0.02  # Simulation time step
CANYON_MODEL_PATH = "box_canyon.glb"

# Frontend's implied scaling/centering constants (must match main.js)
BOX_W_THREE_UNITS = 10.0
BOX_H_THREE_UNITS = 7.5
BOX_D_THREE_UNITS = 17.5
METERS_PER_THREE_UNIT = 20.0 # This must match main.js

# --- Setup ---
app = FastAPI()

# Load the canyon mesh using trimesh for proximity checks
print(f"Loading collision mesh from {CANYON_MODEL_PATH}...")
try:
    mesh = trimesh.load(CANYON_MODEL_PATH, force='mesh')
    print("Mesh loaded successfully.")
except Exception as e:
    print(f"FATAL: Could not load mesh. Error: {e}")
    exit()

# --- Apply the same scaling and centering to the backend mesh ---
print("Applying frontend-matching transformations to backend mesh...")

# 1. Calculate the target dimensions in meters (what the frontend *visually* represents)
target_width_meters = BOX_W_THREE_UNITS * METERS_PER_THREE_UNIT
target_height_meters = BOX_H_THREE_UNITS * METERS_PER_THREE_UNIT
target_depth_meters = BOX_D_THREE_UNITS * METERS_PER_THREE_UNIT
target_extents_meters = np.array([target_width_meters, target_depth_meters, target_height_meters]) # Trimesh uses (x,y,z) == (width, depth, height)

# 2. Get the original dimensions (extents) of the loaded trimesh in its native units (assumed meters)
# trimesh.extents returns [width, depth, height]
original_extents_meters = mesh.extents

# 3. Calculate the uniform scale factor
# This ensures the mesh is scaled to fit within the target dimensions while preserving aspect ratio
scale_factors = target_extents_meters / original_extents_meters
uniform_scale = np.min(scale_factors)
mesh.apply_scale(uniform_scale)
print(f"Mesh scaled by factor: {uniform_scale:.4f}")

# 4. Center the mesh at the origin (0,0,0) in meters
# Get the center of the mesh AFTER scaling
scaled_center = mesh.bounds.mean(axis=0)
# Calculate the translation needed to move its center to the origin
translation_vector = -scaled_center
mesh.apply_translation(translation_vector)
print(f"Mesh translated by: {np.round(translation_vector, 2)} to center at origin.")

# Verify new bounds (optional)
# print(f"New mesh bounds (meters): {np.round(mesh.bounds, 2)}")
# print(f"New mesh extents (meters): {np.round(mesh.extents, 2)}")
# print(f"New mesh center (meters): {np.round(mesh.bounds.mean(axis=0), 2)}")
print("Backend mesh transformations complete.")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Frontend connected. Waiting for initialization...")

    # --- Default Starting Conditions ---
    # These will be used if the frontend doesn't send valid data.
    default_pos = [-150.0, 0.0, 5.0]
    default_vel = [20.0, 0.0, 0.0]

    start_pos = default_pos
    start_vel = default_vel

    try:
        # Wait for the initialization message from the client
        init_data = await websocket.receive_text()
        message = json.loads(init_data)

        # Check if it's the expected 'init' message
        if message.get('type') == 'init':
            # Safely get position and velocity, with fallback to defaults
            pos_from_client = message.get('position')
            vel_from_client = message.get('velocity')

            if isinstance(pos_from_client, list) and len(pos_from_client) == 3 and isinstance(vel_from_client, list) and len(vel_from_client) == 3:
                start_pos = [float(p) for p in pos_from_client]
                start_vel = [float(v) for v in vel_from_client]
                print(f"Received starting conditions: Position={start_pos}, Velocity={start_vel}")
            else:
                print("WARNING: Invalid 'init' message format. Using default starting conditions.")
        else:
            print("WARNING: First message was not of type 'init'. Using default starting conditions.")

    except (WebSocketDisconnect, json.JSONDecodeError, TypeError) as e:
        print(f"Error receiving init message: {e}. Using default starting conditions.")
        # If the client disconnects or sends malformed data before init, we can just end the connection.
        await websocket.close()
        return # Exit the function early

    # Initialize the aircraft with the determined starting conditions
    aircraft = Aircraft(initial_pos=start_pos, initial_velocity=start_vel)
    print(f"Aircraft initialized at Position={aircraft.pos.tolist()}, Velocity={aircraft.velocity.tolist()}")

    try:
        while True:
            # --- Threat Detection using Trimesh ---
            # Find the closest point on the canyon mesh to the aircraft's current position
            closest_point, distance, face_id = mesh.nearest.on_surface([aircraft.pos])

            # The 'closest_point' is a numpy array within a list, so we extract it
            surface_point = closest_point[0]

            # The threat normal is the normal of the closest face
            surface_normal = mesh.face_normals[face_id[0]]

            # The ROA needs a list of threats
            current_threats = [(surface_point, surface_normal)]

            # --- Simulation Update ---
            aircraft.update(DT, current_threats)

            # --- Send State to Frontend ---
            # Prepare data packet (convert numpy arrays to lists for JSON)
            data = {
                "position": aircraft.pos.tolist(),
                "velocity": aircraft.velocity.tolist()
            }
            await websocket.send_text(json.dumps(data))

            # Wait for the next time step
            await asyncio.sleep(DT)

    except WebSocketDisconnect:
        print("Frontend disconnected.")