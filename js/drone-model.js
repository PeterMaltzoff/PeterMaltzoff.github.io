/**
 * drone-model.js — 3D Drone Model
 *
 * Creates a detailed quadrotor 3D model with:
 *   - Central body
 *   - 4 arms with motors
 *   - 4 spinning propellers (CW/CCW)
 *   - Front/rear LEDs
 *   - Camera mount (for FPV reference)
 *   - Motor speed visualization
 */

import * as THREE from 'three';

export class DroneModel {
  constructor(physics) {
    this.physics = physics;
    this.group = new THREE.Group();
    this.propellers = [];
    this.motorLights = [];

    this._buildDrone();
  }

  _buildDrone() {
    const armLen = this.physics.armLength;
    const d = this.physics.d;

    // ── Materials ──
    const bodyMat   = new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.6, metalness: 0.4 });
    const armMat    = new THREE.MeshStandardMaterial({ color: 0x333333, roughness: 0.5, metalness: 0.5 });
    const motorMat  = new THREE.MeshStandardMaterial({ color: 0x444444, roughness: 0.4, metalness: 0.6 });
    const propMatCW  = new THREE.MeshStandardMaterial({
      color: 0x1a1a2e, roughness: 0.3, metalness: 0.2, transparent: true, opacity: 0.85,
      side: THREE.DoubleSide
    });
    const propMatCCW = new THREE.MeshStandardMaterial({
      color: 0x2e1a1a, roughness: 0.3, metalness: 0.2, transparent: true, opacity: 0.85,
      side: THREE.DoubleSide
    });

    // ── Central body ──
    const bodyGeo = new THREE.BoxGeometry(0.06, 0.025, 0.08);
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.castShadow = true;
    this.group.add(body);

    // Top plate (flight controller cover)
    const topPlateGeo = new THREE.BoxGeometry(0.04, 0.005, 0.05);
    const topPlate = new THREE.Mesh(topPlateGeo, new THREE.MeshStandardMaterial({ color: 0x004400, roughness: 0.7 }));
    topPlate.position.y = 0.015;
    this.group.add(topPlate);

    // Battery underneath
    const battGeo = new THREE.BoxGeometry(0.035, 0.018, 0.065);
    const battMat = new THREE.MeshStandardMaterial({ color: 0x111155, roughness: 0.8 });
    const battery = new THREE.Mesh(battGeo, battMat);
    battery.position.y = -0.018;
    battery.castShadow = true;
    this.group.add(battery);

    // ── Camera mount (front, facing -z) ──
    const camMountGeo = new THREE.BoxGeometry(0.02, 0.015, 0.015);
    const camMat = new THREE.MeshStandardMaterial({ color: 0x660000, roughness: 0.5 });
    const camMount = new THREE.Mesh(camMountGeo, camMat);
    camMount.position.set(0, -0.005, -0.045);
    this.group.add(camMount);

    // Camera lens
    const lensGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.005, 8);
    const lensMat = new THREE.MeshStandardMaterial({ color: 0x111111, roughness: 0.2, metalness: 0.8 });
    const lens = new THREE.Mesh(lensGeo, lensMat);
    lens.rotation.x = Math.PI / 2;
    lens.position.set(0, -0.005, -0.052);
    this.group.add(lens);

    // ── Arms & Motors ──
    const motorPositions = this.physics.motorPos;
    const motorDirs = this.physics.motorDir;

    for (let i = 0; i < 4; i++) {
      const [mx, , mz] = motorPositions[i];

      // Arm
      const armGeo = new THREE.BoxGeometry(0.012, 0.008, armLen);
      const arm = new THREE.Mesh(armGeo, armMat);
      arm.position.set(mx / 2, 0, mz / 2);
      arm.lookAt(new THREE.Vector3(mx, 0, mz));
      arm.castShadow = true;
      this.group.add(arm);

      // Motor housing
      const motorGeo = new THREE.CylinderGeometry(0.012, 0.014, 0.015, 12);
      const motor = new THREE.Mesh(motorGeo, motorMat);
      motor.position.set(mx, 0.01, mz);
      motor.castShadow = true;
      this.group.add(motor);

      // Propeller group (two blades that rotate together)
      const propRadius = this.physics.rotorRadius;
      const propGeo = new THREE.CircleGeometry(propRadius, 3); // triangle blade shape
      const propMat = motorDirs[i] > 0 ? propMatCW : propMatCCW;

      const propGroup = new THREE.Group();
      propGroup.position.set(mx, 0.02, mz);

      const blade1 = new THREE.Mesh(propGeo, propMat);
      blade1.rotation.x = -Math.PI / 2; // flat, facing up
      blade1.castShadow = true;
      propGroup.add(blade1);

      const blade2 = new THREE.Mesh(propGeo, propMat);
      blade2.rotation.x = -Math.PI / 2;
      blade2.rotation.z = Math.PI; // 180° opposite
      propGroup.add(blade2);

      this.group.add(propGroup);
      this.propellers.push(propGroup);

      // Motor speed indicator light
      const lightGeo = new THREE.SphereGeometry(0.003, 6, 6);
      const lightMat = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
      const light = new THREE.Mesh(lightGeo, lightMat);
      light.position.set(mx, 0.025, mz);
      this.group.add(light);
      this.motorLights.push(light);
    }

    // ── LED indicators ──
    // Front LEDs (white)
    const ledGeo = new THREE.SphereGeometry(0.004, 6, 6);
    const frontLedMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const ledFL = new THREE.Mesh(ledGeo, frontLedMat);
    ledFL.position.set(-0.02, 0, -0.04);
    this.group.add(ledFL);
    const ledFR = new THREE.Mesh(ledGeo, frontLedMat);
    ledFR.position.set(0.02, 0, -0.04);
    this.group.add(ledFR);

    // Rear LEDs (red)
    const rearLedMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const ledRL = new THREE.Mesh(ledGeo, rearLedMat);
    ledRL.position.set(-0.02, 0, 0.04);
    this.group.add(ledRL);
    const ledRR = new THREE.Mesh(ledGeo, rearLedMat);
    ledRR.position.set(0.02, 0, 0.04);
    this.group.add(ledRR);

    // Scale up for visibility (the real drone is tiny at 0.125m arm length)
    // Actually keep 1:1 scale — the camera will be close enough
  }

  /**
   * Update drone model transform and propeller animation.
   * @param {number} dt – frame delta time
   */
  update(dt) {
    const phys = this.physics;

    // Position
    this.group.position.set(phys.position[0], phys.position[1], phys.position[2]);

    // Orientation (quaternion: physics=[w,x,y,z], Three.js Quaternion=[x,y,z,w])
    const [qw, qx, qy, qz] = phys.quaternion;
    this.group.quaternion.set(qx, qy, qz, qw);

    // Propeller animation
    for (let i = 0; i < 4; i++) {
      const speed = phys.motorSpeeds[i];
      const dir = phys.motorDir[i]; // +1 CW from above, -1 CCW from above
      // CW from above (looking down -y) = negative rotation about y in Three.js right-hand rule
      this.propellers[i].rotation.y += -dir * speed * dt;

      // At high speed, make prop semi-transparent (motion blur effect)
      const opacity = speed > 100 ? Math.max(0.15, 1.0 - speed / 2000) : 0.85;
      this.propellers[i].children.forEach(child => {
        if (child.material) child.material.opacity = opacity;
      });

      // Motor light color: green (idle) → yellow → red (max)
      const ratio = speed / phys.maxMotorRad;
      const r = Math.min(1, ratio * 2);
      const g = Math.max(0, 1 - ratio * 1.5);
      this.motorLights[i].material.color.setRGB(r, g, 0);
    }
  }

  /** Get the Three.js Group to add to the scene. */
  getObject() {
    return this.group;
  }

  /** Get FPV camera position/target in world coordinates. */
  getFPVTransform() {
    const camOffset = new THREE.Vector3(0, 0.01, -0.05); // slightly below, at front
    const camTarget = new THREE.Vector3(0, -0.02, -1);   // looking forward and slightly down
    camOffset.applyQuaternion(this.group.quaternion);
    camTarget.applyQuaternion(this.group.quaternion);
    return {
      position: this.group.position.clone().add(camOffset),
      target:   this.group.position.clone().add(camTarget),
    };
  }
}
