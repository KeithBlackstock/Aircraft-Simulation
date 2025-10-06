// main.js
import * as THREE from 'https://unpkg.com/three@0.154.0/build/three.module.js';
import { OrbitControls } from 'https://unpkg.com/three@0.154.0/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'https://unpkg.com/three@0.154.0/examples/jsm/loaders/GLTFLoader.js';

//
// Config (Target dimensions in Three.js units, where 1 unit = 20m)
//
const BOX_W = 10;   // 200m / 20
const BOX_H = 7.5;  // 150m / 20
const BOX_D = 17.5; // 350m / 20
const METERS_PER_THREE_UNIT = 20; // This is the scale factor you've implied: 1 Three.js unit = 20 meters

// Renderer + scene + camera
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf2f3f5);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 2000);
camera.position.set(15, 10, 22);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Controls
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;

// Lights
const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.9);
hemi.position.set(0, 50, 0);
scene.add(hemi);

const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(6, 10, 6);
dir.castShadow = true;
scene.add(dir);

// Grid / Ground (for reference)
const grid = new THREE.GridHelper(40, 40, 0xcccccc, 0xeeeeee);
grid.position.y = -BOX_H / 2 - 0.01;
scene.add(grid);

// This will hold our loaded, pre-carved model
let carvedModel;

// This will hold our aircraft model (now the jet1.glb)
let aircraftModel;
let latestAircraftPosition = new THREE.Vector3(); // Store the latest received position
let latestAircraftVelocity = new THREE.Vector3(); // NEW: Store the latest received velocity
let aircraftModelLoaded = false; // Flag to ensure we don't try to move it before it's loaded

//
// Load GLTF model (the pre-carved canyon)
//
const gltfLoader = new GLTFLoader();

gltfLoader.load(
  'box_canyon.glb',
  (gltf) => {
    carvedModel = gltf.scene;

    // --- Bounding Box-Based Uniform Scaling ---
    const targetSize = new THREE.Vector3(BOX_W, BOX_H, BOX_D);
    const modelBox = new THREE.Box3().setFromObject(carvedModel);
    const modelSize = new THREE.Vector3();
    modelBox.getSize(modelSize);
    const scaleX = targetSize.x / modelSize.x;
    const scaleY = targetSize.y / modelSize.y;
    const scaleZ = targetSize.z / modelSize.z;
    const scale = Math.min(scaleX, scaleY, scaleZ);
    carvedModel.scale.set(scale, scale, scale);

    const scaledModelBox = new THREE.Box3().setFromObject(carvedModel);
    const center = new THREE.Vector3();
    scaledModelBox.getCenter(center);
    carvedModel.position.sub(center);
    // --- End of Scaling Logic ---

    carvedModel.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
        // Optional: add wireframe edges for clarity
        const edgeGeom = new THREE.EdgesGeometry(child.geometry);
        const edgeMat = new THREE.LineBasicMaterial({ color: 0x222222, linewidth: 1 });
        const edges = new THREE.LineSegments(edgeGeom, edgeMat);
        child.add(edges);
      }
    });

    scene.add(carvedModel);
    console.log("Canyon model loaded and scaled.");
  },
  undefined,
  (error) => {
    console.error('An error occurred loading the GLTF model:', error);
  }
);


//
// Load the jet1.glb model for the aircraft
//
gltfLoader.load(
    'jet1.glb',
    (gltf) => {
        aircraftModel = gltf.scene;

        // Scale the jet1 model. Adjust this value until the jet1 looks appropriately sized.
        const jet1Scale = 0.1; // Example scale factor, you might need to adjust this
        aircraftModel.scale.set(jet1Scale, jet1Scale, jet1Scale);

        // --- IMPORTANT: Initial Rotation for Model Orientation ---
        // The `lookAt` function orients an object so its local +Z axis points towards the target.

        aircraftModel.traverse((child) => {
            if (child.isMesh) {
                child.castShadow = true;
                child.receiveShadow = true;
            }
        });

        scene.add(aircraftModel);
        aircraftModelLoaded = true; // Set flag once loaded
        console.log("Jet1 model loaded.");
    },
    undefined,
    (error) => {
        console.error('An error occurred loading the jet1 GLTF model:', error);
    }
);


//
// WebSocket setup for aircraft data
//
let ws;

document.getElementById('start-button').addEventListener('click', () => {
    // Hide the control panel after starting
    document.getElementById('control-panel').style.display = 'none';

    // Get values from input fields
    const posX = parseFloat(document.getElementById('posX').value);
    const posY = parseFloat(document.getElementById('posY').value);
    const posZ = parseFloat(document.getElementById('posZ').value);
    const velX = parseFloat(document.getElementById('velX').value);
    const velY = parseFloat(document.getElementById('velY').value);
    const velZ = parseFloat(document.getElementById('velZ').value);

    // Create the WebSocket connection
    ws = new WebSocket("ws://localhost:8000/ws");

    ws.onopen = (event) => {
        console.log("WebSocket connected!");
        const initMessage = {
            type: 'init',
            position: [posX, posY, posZ],
            velocity: [velX, velY, velZ]
        };
        ws.send(JSON.stringify(initMessage));
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);

        // Position (convert from meters to Three.js units)
        latestAircraftPosition.set(
            data.position[0] / METERS_PER_THREE_UNIT,
            data.position[1] / METERS_PER_THREE_UNIT,
            data.position[2] / METERS_PER_THREE_UNIT
        );

        // Velocity (store as is for direction, normalization will handle magnitude)
        latestAircraftVelocity.set(
            data.velocity[0],
            data.velocity[1],
            data.velocity[2]
        );
    };

    ws.onerror = (error) => {
        console.error("WebSocket Error:", error);
    };

    ws.onclose = (event) => {
        console.log("WebSocket disconnected:", event);
    };
});


//
// Resize handler
//
window.addEventListener('resize', onWindowResize, false);
function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

//
// Animation loop
//
function animate() {
  requestAnimationFrame(animate);

  // Update aircraft position and orientation if the model has been loaded and data received
  if (aircraftModelLoaded) {
      aircraftModel.position.copy(latestAircraftPosition);

      // Only orient if there's significant velocity to avoid issues with zero-length vectors
      if (latestAircraftVelocity.length() > 0.1) { // 0.1 is an arbitrary small threshold
          // Calculate a target point slightly ahead of the aircraft in its direction of travel
          const forwardDirection = latestAircraftVelocity.clone().normalize();
          const targetPoint = new THREE.Vector3().copy(aircraftModel.position).add(forwardDirection);

          // Make the aircraft model look at that target point
          aircraftModel.lookAt(targetPoint);

          // Note: The `lookAt` function implicitly tries to keep the object's local +Y axis
          // pointing "up" towards the world +Y. If your aircraft needs to roll significantly
          // (which it might in a flight simulation), `lookAt` alone might fight that roll.
          // For full control, you'd calculate Quaternions based on forward, up, and right vectors
          // from the backend. For now, `lookAt` provides good basic directional orientation.
      }
  }

  controls.update();
  renderer.render(scene, camera);
}
animate();