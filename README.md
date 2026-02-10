# Drone Simulator

A realistic browser-based 6-DoF quadrotor flight simulator built with Three.js.

## Physics Model

Implements a full Newton-Euler dynamics model inspired by [RotorPy](https://github.com/spencerfolk/rotorpy):

- **6-DoF rigid body dynamics** — quaternion-based orientation, proper inertia tensor
- **First-order motor dynamics** — realistic spool-up/spool-down lag (25ms time constant)
- **Parasitic (frame) drag** — quadratic drag on the airframe
- **Rotor drag** — apparent drag from advancing blade dissymmetry
- **Blade flapping moments** — pitch/roll coupling from forward flight
- **Induced drag** — drag from tilted rotor disc
- **Translational lift** — increased efficiency in forward flight
- **Ground effect** — thrust augmentation near the ground
- **Gyroscopic precession** — torque from spinning rotor angular momentum
- **Dryden-like wind model** — turbulence and gusts

## Flight Controller

Cascaded PID controller (similar to Betaflight):

- **Angle mode** (default): player commands desired attitude angles, FC stabilizes
- **Acro mode**: player commands desired angular rates directly
- Rate PID with D-on-measurement for smooth response
- Motor mixer for X-configuration quadrotor

## Controls

### Keyboard (Mode 2)

| Key | Action |
|-----|--------|
| W / S | Throttle up / down |
| A / D | Yaw left / right |
| ↑ / ↓ | Pitch forward / back |
| ← / → | Roll left / right |
| 1 / 2 / 3 | Camera: Chase / FPV / Free |
| M | Toggle Angle / Acro mode |
| R | Reset drone |
| H | Toggle help overlay |
| G | Toggle wind gusts |

### Gamepad

Standard Mode 2 layout — left stick: throttle/yaw, right stick: pitch/roll.

## Running Locally

Simply serve the directory with any HTTP server:

```bash
npx http-server -p 8080
```

Then open http://localhost:8080 in a modern browser.

## Tech Stack

- **Three.js** (v0.170.0 from CDN) — 3D rendering
- **Vanilla JavaScript** ES modules — no build step required
- **Static files only** — runs directly on GitHub Pages
