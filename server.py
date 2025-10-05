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
    print("Frontend connected.")

    # Initialize the aircraft at a starting position
    # The mesh is now centered at [0,0,0] and scaled to match frontend's meters.
    # So, we need to base the start_pos on the *transformed* mesh's bounds.
    transformed_center = mesh.bounds.mean(axis=0) # This should be very close to [0,0,0]
    transformed_max_z = mesh.bounds[1, 2]

    # Start 20m above the highest point of the transformed mesh
    start_pos = [transformed_center[0] - 150, transformed_center[1], transformed_center[2] + 5]
    start_vel = [20, 0, 0] # Give it some initial velocity
    
    aircraft = Aircraft(initial_pos=start_pos, initial_velocity=start_vel)

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