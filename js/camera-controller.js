/**
 * camera-controller.js — Camera Modes
 *
 * Three camera modes:
 *   0 - Chase: smooth third-person behind the drone
 *   1 - FPV:   first-person from the drone's camera mount
 *   2 - Free:  orbit camera (mouse drag to rotate, scroll to zoom)
 */

import * as THREE from 'three';

export class CameraController {
  constructor(camera, canvas, droneModel) {
    this.camera = camera;
    this.canvas = canvas;
    this.drone  = droneModel;

    this.mode = 0; // 0=chase, 1=fpv, 2=free

    // Chase camera params
    this.chaseOffset   = new THREE.Vector3(0, 1.2, 3.5);  // behind and above
    this.chaseLookAhead = new THREE.Vector3(0, 0.3, -2);  // look ahead of drone
    this.chaseSmoothing = 5.0;

    // Free camera params
    this.freeDistance = 8;
    this.freeTheta   = Math.PI / 4;  // elevation
    this.freePhi     = 0;            // azimuth
    this.freeTarget  = new THREE.Vector3(0, 1, 0);

    // Internal smoothed position
    this._smoothPos = new THREE.Vector3(0, 1.5, 4);
    this._smoothTarget = new THREE.Vector3(0, 0.2, 0);

    this._initMouseControls();
  }

  _initMouseControls() {
    let dragging = false;
    let lastX = 0, lastY = 0;

    this.canvas.addEventListener('mousedown', (e) => {
      if (this.mode === 2) {
        dragging = true;
        lastX = e.clientX;
        lastY = e.clientY;
      }
    });

    window.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;

      this.freePhi   -= dx * 0.005;
      this.freeTheta -= dy * 0.005;
      this.freeTheta  = Math.max(0.05, Math.min(Math.PI/2 - 0.05, this.freeTheta));
    });

    window.addEventListener('mouseup', () => { dragging = false; });

    this.canvas.addEventListener('wheel', (e) => {
      if (this.mode === 2) {
        this.freeDistance *= (1 + e.deltaY * 0.001);
        this.freeDistance = Math.max(2, Math.min(60, this.freeDistance));
        e.preventDefault();
      }
    }, { passive: false });
  }

  setMode(mode) {
    this.mode = mode;
    // Reset smooth position when switching to avoid jarring transitions
    if (mode === 2) {
      // Set free camera to current drone vicinity
      const dp = this.drone.group.position;
      this.freeTarget.copy(dp);
    }
  }

  update(dt) {
    const dronePos = this.drone.group.position;
    const droneQuat = this.drone.group.quaternion;

    switch (this.mode) {
      case 0: this._updateChase(dt, dronePos, droneQuat); break;
      case 1: this._updateFPV(dt); break;
      case 2: this._updateFree(dt, dronePos); break;
    }
  }

  _updateChase(dt, dronePos, droneQuat) {
    // Desired position: offset behind and above, rotated by drone yaw only
    // Extract yaw from quaternion
    const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(droneQuat);
    const yaw = Math.atan2(forward.x, -forward.z);

    const yawQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);

    const offset = this.chaseOffset.clone().applyQuaternion(yawQuat);
    const desiredPos = dronePos.clone().add(offset);

    // Smooth interpolation
    const t = 1 - Math.exp(-this.chaseSmoothing * dt);
    this._smoothPos.lerp(desiredPos, t);

    // Look target: slightly ahead of drone
    const lookAhead = this.chaseLookAhead.clone().applyQuaternion(yawQuat);
    const desiredTarget = dronePos.clone().add(lookAhead);
    this._smoothTarget.lerp(desiredTarget, t);

    // Prevent camera going below ground
    this._smoothPos.y = Math.max(0.3, this._smoothPos.y);

    this.camera.position.copy(this._smoothPos);
    this.camera.lookAt(this._smoothTarget);
  }

  _updateFPV(dt) {
    const fpv = this.drone.getFPVTransform();
    // Smooth just a tiny bit for stability
    const t = 1 - Math.exp(-25 * dt);
    this._smoothPos.lerp(fpv.position, t);
    this._smoothTarget.lerp(fpv.target, t);

    this.camera.position.copy(this._smoothPos);
    this.camera.lookAt(this._smoothTarget);
  }

  _updateFree(dt, dronePos) {
    // Slowly follow drone
    this.freeTarget.lerp(dronePos, 1 - Math.exp(-1.0 * dt));

    // Orbit around target
    const x = this.freeDistance * Math.cos(this.freeTheta) * Math.sin(this.freePhi);
    const y = this.freeDistance * Math.sin(this.freeTheta);
    const z = this.freeDistance * Math.cos(this.freeTheta) * Math.cos(this.freePhi);

    this.camera.position.set(
      this.freeTarget.x + x,
      this.freeTarget.y + y,
      this.freeTarget.z + z,
    );
    this.camera.lookAt(this.freeTarget);
  }
}
