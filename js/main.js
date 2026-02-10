/**
 * main.js — Entry Point & Game Loop
 *
 * Initializes the Three.js scene, creates all subsystems, and runs the
 * simulation with fixed-timestep physics (1000 Hz) decoupled from rendering.
 */

import * as THREE from 'three';
import { QuadrotorPhysics } from './physics.js';
import { FlightController }  from './flight-controller.js';
import { InputHandler }       from './input.js';
import { DroneModel }         from './drone-model.js';
import { World }              from './world.js';
import { HUD }                from './hud.js';
import { CameraController }   from './camera-controller.js';

// ── Constants ──
const PHYSICS_DT   = 1 / 1000;  // 1 ms physics timestep
const MAX_SUB_STEPS = 40;        // max physics steps per frame (prevents spiral of death)

// ── Global state ──
let renderer, scene, camera;
let physics, fc, input, droneModel, world, hud, cameraCtrl;
let lastTime = 0;
let physicsAccumulator = 0;

// ── Initialization ──

function init() {
  // Renderer
  const canvas = document.getElementById('sim-canvas');
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;

  // Scene
  scene = new THREE.Scene();

  // Camera
  camera = new THREE.PerspectiveCamera(65, window.innerWidth / window.innerHeight, 0.05, 600);
  camera.position.set(0, 2, 5);

  // Physics
  physics = new QuadrotorPhysics();

  // Flight controller
  fc = new FlightController(physics);

  // Input
  input = new InputHandler();

  // 3D Drone model
  droneModel = new DroneModel(physics);
  scene.add(droneModel.getObject());

  // World
  world = new World(scene);

  // HUD
  hud = new HUD();

  // Camera controller
  cameraCtrl = new CameraController(camera, canvas, droneModel);

  // ── Bind input actions ──
  input.onToggleMode = () => {
    const mode = fc.toggleMode();
    console.log('Flight mode:', mode);
  };

  input.onReset = () => {
    physics.reset();
    fc.resetPIDs();
    input.throttle = 0;
    input.smoothThrottle = 0;
  };

  input.onToggleHelp = () => {
    const overlay = document.getElementById('help-overlay');
    overlay.classList.toggle('hidden');
  };

  input.onCameraChange = (mode) => {
    cameraCtrl.setMode(mode);
  };

  input.onToggleWind = () => {
    physics.wind.gustEnabled = !physics.wind.gustEnabled;
    console.log('Wind gusts:', physics.wind.gustEnabled ? 'ON' : 'OFF');
  };

  // ── Resize handler ──
  window.addEventListener('resize', onResize);

  // ── Start help visible ──
  // Help overlay starts visible; press H to dismiss

  // ── Start the loop ──
  lastTime = performance.now() / 1000;
  requestAnimationFrame(gameLoop);
}

function onResize() {
  const w = window.innerWidth;
  const h = window.innerHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}

// ── Game Loop ──

function gameLoop(timestamp) {
  requestAnimationFrame(gameLoop);

  const now = timestamp / 1000;
  let frameDt = now - lastTime;
  lastTime = now;

  // Clamp large frame deltas (e.g., after tab switch)
  frameDt = Math.min(frameDt, 0.05);

  // Update input
  input.update(frameDt);

  // ── Fixed-timestep physics ──
  physicsAccumulator += frameDt;
  let steps = 0;

  while (physicsAccumulator >= PHYSICS_DT && steps < MAX_SUB_STEPS) {
    const cmd = input.getCommand();
    const motorCmds = fc.update(cmd, PHYSICS_DT);
    physics.step(PHYSICS_DT, motorCmds);
    physicsAccumulator -= PHYSICS_DT;
    steps++;
  }

  // If we hit max steps, drain the accumulator to prevent buildup
  if (steps >= MAX_SUB_STEPS) {
    physicsAccumulator = 0;
  }

  // Update 3D model
  droneModel.update(frameDt);

  // Update camera
  cameraCtrl.update(frameDt);

  // Update HUD
  hud.update(physics, input.getRaw(), fc.mode, frameDt);

  // Render
  renderer.render(scene, camera);
}

// ── Launch ──
init();
