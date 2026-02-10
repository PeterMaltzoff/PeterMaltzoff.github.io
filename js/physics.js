/**
 * physics.js — Full 6-DoF Quadrotor Physics Engine
 *
 * Implements Newton-Euler dynamics with aerodynamic effects inspired by RotorPy:
 *   - Parasitic (frame) drag
 *   - Rotor drag
 *   - Blade flapping moments
 *   - Induced drag
 *   - Translational lift
 *   - Ground effect
 *   - Gyroscopic precession from spinning rotors
 *   - First-order motor dynamics (spool-up / spool-down)
 *
 * Coordinate system (Three.js native):
 *   x-right, y-up, z-toward-viewer.
 *   Drone forward = -z,  gravity = -y.
 *
 * Motor layout (X-config, viewed from above looking down -y):
 *
 *          -z (forward)
 *      M2(FL,CCW)   M1(FR,CW)
 *            \       /
 *             [body]
 *            /       \
 *      M3(RL,CW)    M4(RR,CCW)
 *          +z (backward)
 *
 * Thrust along +y body axis.
 * CW  motors: M1 (front-right), M3 (rear-left)  → yaw reaction +y
 * CCW motors: M2 (front-left),  M4 (rear-right)  → yaw reaction -y
 */

const GRAVITY = 9.81;

// ─── Quaternion helpers (w,x,y,z) ────────────────────────────────────────────

function qMultiply(a, b) {
  return [
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
  ];
}

function qNormalize(q) {
  const len = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (len < 1e-12) return [1, 0, 0, 0];
  return [q[0]/len, q[1]/len, q[2]/len, q[3]/len];
}

function qConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

/** Rotate a vector by quaternion q. */
function qRotateVec(q, v) {
  const qv = [0, v[0], v[1], v[2]];
  const r  = qMultiply(qMultiply(q, qv), qConjugate(q));
  return [r[1], r[2], r[3]];
}

/** Rotate vector from world to body frame. */
function worldToBody(q, v) {
  return qRotateVec(qConjugate(q), v);
}

/** Rotate vector from body to world frame. */
function bodyToWorld(q, v) {
  return qRotateVec(q, v);
}

/** Build rotation matrix (3×3 column-major) from quaternion. */
function qToMatrix3(q) {
  const [w, x, y, z] = q;
  return [
    1-2*(y*y+z*z),  2*(x*y+w*z),    2*(x*z-w*y),
    2*(x*y-w*z),    1-2*(x*x+z*z),  2*(y*z+w*x),
    2*(x*z+w*y),    2*(y*z-w*x),    1-2*(x*x+y*y),
  ];
}

/** Extract yaw angle (rotation about world y-axis) from quaternion. */
function qToYaw(q) {
  // Project body -z axis onto world xz plane
  const fwd = bodyToWorld(q, [0, 0, -1]);
  return Math.atan2(fwd[0], -fwd[2]); // angle from -z toward +x
}

/** Extract Euler angles (roll, pitch, yaw) — used only for display. */
function qToEuler(q) {
  const [w, x, y, z] = q;
  // Roll  (about body z / world -z when forward=-z … we compute from rotation matrix)
  const sinr =  2*(w*z + x*y);
  const cosr =  1 - 2*(y*y + z*z);
  const roll  = Math.atan2(sinr, cosr);

  // Pitch (about body x)
  const sinp = 2*(w*x - y*z);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp)*Math.PI/2 : Math.asin(sinp);

  // Yaw (about world y)
  const siny =  2*(w*y + x*z);
  const cosy =  1 - 2*(x*x + y*y);
  const yaw  = Math.atan2(siny, cosy);

  return { roll, pitch, yaw };
}

// ─── Wind model ──────────────────────────────────────────────────────────────

class WindModel {
  constructor() {
    this.baseWind     = [0, 0, 0]; // constant wind (world frame, m/s)
    this.gustEnabled  = false;
    this.gustStrength = 2.5;       // m/s peak gust
    this.turbulence   = 0.8;       // m/s RMS turbulence
    this._phase       = [Math.random()*100, Math.random()*100, Math.random()*100];
  }

  /** Return wind velocity at given time (world frame). */
  getWind(t) {
    const wx = this.baseWind[0];
    const wy = this.baseWind[1];
    const wz = this.baseWind[2];
    if (!this.gustEnabled) return [wx, wy, wz];

    // Dryden-like turbulence with multiple harmonics
    const turb = (axis) => {
      const p = this._phase[axis];
      return this.turbulence * (
        0.5 *Math.sin(0.7*t + p) +
        0.3 *Math.sin(1.9*t + p*2.1) +
        0.2 *Math.sin(4.3*t + p*0.7)
      );
    };
    // Occasional gusts (slow sine envelope)
    const gustEnv = Math.max(0, Math.sin(0.15*t)) ** 3;
    const gust = this.gustStrength * gustEnv;

    return [
      wx + turb(0) + gust * Math.cos(0.05*t),
      wy + turb(1) * 0.3,
      wz + turb(2) + gust * Math.sin(0.05*t),
    ];
  }
}

// ─── Quadrotor Physics ───────────────────────────────────────────────────────

export class QuadrotorPhysics {
  constructor(params = {}) {
    // ── Drone parameters ──
    this.mass         = params.mass         ?? 0.752;   // kg
    this.armLength    = params.armLength    ?? 0.125;   // m (center to motor)
    this.Ixx          = params.Ixx          ?? 0.0025;  // kg⋅m²
    this.Iyy          = params.Iyy          ?? 0.0045;  // yaw inertia
    this.Izz          = params.Izz          ?? 0.0025;  // same as Ixx for symmetric frame
    this.kF           = params.kF           ?? 6.5e-6;  // thrust coeff  N/(rad/s)²
    this.kM           = params.kM           ?? 1.2e-7;  // torque coeff  N⋅m/(rad/s)²
    this.motorTau     = params.motorTau     ?? 0.025;   // motor time constant (s)
    this.maxMotorRad  = params.maxMotorRad  ?? 1200;    // max motor speed (rad/s) ≈ 11500 RPM → T/W ≈ 5
    this.minMotorRad  = params.minMotorRad  ?? 150;     // idle speed (rad/s)
    this.rotorInertia = params.rotorInertia ?? 3.5e-5;  // single rotor Ir (kg⋅m²)
    this.rotorRadius  = params.rotorRadius  ?? 0.066;   // propeller radius (m) ≈ 5″

    // Aerodynamic coefficients
    this.dragCoeffBody   = params.dragCoeffBody   ?? [0.12, 0.15, 0.12]; // Cd*A per axis (body)
    this.rotorDragCoeff  = params.rotorDragCoeff  ?? 0.016;
    this.bladeFlapCoeff  = params.bladeFlapCoeff  ?? 0.008;
    this.inducedDragCoeff = params.inducedDragCoeff ?? 0.01;

    // Derived
    this.d = this.armLength * Math.SQRT1_2; // motor offset along each axis
    this.hoverMotorSpeed = Math.sqrt(this.mass * GRAVITY / (4 * this.kF)); // ≈ 535 rad/s
    this.maxThrust = 4 * this.kF * this.maxMotorRad * this.maxMotorRad;

    // ── Motor positions in body frame (x-right, y-up, z-back when identity) ──
    // With drone facing -z:
    //   M1 FR: (+d, 0, -d)   M2 FL: (-d, 0, -d)
    //   M3 RL: (-d, 0, +d)   M4 RR: (+d, 0, +d)
    this.motorPos = [
      [ this.d, 0, -this.d], // M1 front-right
      [-this.d, 0, -this.d], // M2 front-left
      [-this.d, 0,  this.d], // M3 rear-left
      [ this.d, 0,  this.d], // M4 rear-right
    ];
    // Spin direction multiplier (+1 for CW from above → +y reaction, -1 for CCW)
    this.motorDir = [1, -1, 1, -1]; // M1 CW, M2 CCW, M3 CW, M4 CCW

    // ── State ──
    this.position        = [0, 0, 0];          // world (starts on ground)
    this.velocity        = [0, 0, 0];          // world
    this.quaternion      = [1, 0, 0, 0];       // body→world
    this.angularVelocity = [0, 0, 0];          // body frame (rad/s)
    this.motorSpeeds     = [0, 0, 0, 0];       // rad/s per motor

    this.wind = new WindModel();
    this.simTime = 0;
    this.crashed = false;
  }

  /** Reset to ground position. */
  reset() {
    this.position        = [0, 0, 0];
    this.velocity        = [0, 0, 0];
    this.quaternion      = [1, 0, 0, 0];
    this.angularVelocity = [0, 0, 0];
    this.motorSpeeds     = [0, 0, 0, 0];
    this.crashed         = false;
  }

  /**
   * Advance the physics simulation by dt seconds.
   * @param {number}   dt            – timestep (seconds)
   * @param {number[]} motorCommands – desired motor speeds [ω1..ω4] (rad/s)
   */
  step(dt, motorCommands) {
    if (this.crashed) return;
    this.simTime += dt;

    // ── 1. Motor dynamics (first-order lag) ──
    for (let i = 0; i < 4; i++) {
      const cmd = Math.max(0, Math.min(this.maxMotorRad, motorCommands[i]));
      const alpha = 1 - Math.exp(-dt / this.motorTau);
      this.motorSpeeds[i] += (cmd - this.motorSpeeds[i]) * alpha;
    }

    // ── 2. Motor thrusts & torques ──
    const T = new Array(4); // per-motor thrust
    let totalThrust = 0;
    for (let i = 0; i < 4; i++) {
      T[i] = this.kF * this.motorSpeeds[i] * this.motorSpeeds[i];
      totalThrust += T[i];
    }

    // Body torques from thrust (cross product of motor position with thrust vector)
    //   F_motor = [0, Ti, 0]  →  r × F  = (pz*Ti, 0, -px*Ti)  ... wait let me recompute:
    //   (px,0,pz) × (0,Ti,0) = (0*0 - pz*Ti,  pz*0 - px*0,  px*Ti - 0*0) = (-pz*Ti, 0, px*Ti)
    //   Hmm: cross(a,b) = (a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
    //   a=(px,0,pz), b=(0,Ti,0):
    //   x: 0*0 - pz*Ti  = -pz*Ti
    //   y: pz*0 - px*0  = 0
    //   z: px*Ti - 0*0   = px*Ti

    let tauX = 0, tauY = 0, tauZ = 0;
    for (let i = 0; i < 4; i++) {
      const [px, , pz] = this.motorPos[i];
      tauX += -pz * T[i];           // pitch torque (body x-axis)
      tauZ +=  px * T[i];           // roll torque  (body z-axis)
      // Yaw reaction torque from motor drag (body y-axis)
      tauY += this.motorDir[i] * this.kM * this.motorSpeeds[i] * this.motorSpeeds[i];
    }

    // ── 3. Aerodynamic forces & torques ──
    const windWorld   = this.wind.getWind(this.simTime);
    const airVelWorld = [
      this.velocity[0] - windWorld[0],
      this.velocity[1] - windWorld[1],
      this.velocity[2] - windWorld[2],
    ];
    const airVelBody  = worldToBody(this.quaternion, airVelWorld);
    const airSpeed    = Math.sqrt(airVelBody[0]**2 + airVelBody[1]**2 + airVelBody[2]**2);

    // 3a. Parasitic (frame) drag — quadratic, in body frame
    const fDragBody = [0, 0, 0];
    const rho = 1.225; // air density kg/m³
    for (let i = 0; i < 3; i++) {
      fDragBody[i] = -0.5 * rho * this.dragCoeffBody[i] * Math.abs(airVelBody[i]) * airVelBody[i];
    }

    // 3b. Rotor drag — linear in horizontal airspeed, opposing motion
    //     Each spinning rotor experiences an apparent drag proportional to advance ratio
    const horizAirBody = [airVelBody[0], 0, airVelBody[2]]; // horizontal body-frame airspeed
    const horizSpeed   = Math.sqrt(horizAirBody[0]**2 + horizAirBody[2]**2);
    const avgMotorSpeed = (this.motorSpeeds[0]+this.motorSpeeds[1]+this.motorSpeeds[2]+this.motorSpeeds[3]) / 4;
    const fRotorDrag = [0, 0, 0];
    if (avgMotorSpeed > 10 && horizSpeed > 0.01) {
      const factor = -this.rotorDragCoeff * avgMotorSpeed;
      fRotorDrag[0] = factor * horizAirBody[0];
      fRotorDrag[2] = factor * horizAirBody[2];
    }

    // 3c. Blade flapping moment — causes pitch/roll moment proportional to airspeed
    //     When flying forward, advancing blade generates more lift → disc tilts → moment
    let tauFlap = [0, 0, 0];
    if (avgMotorSpeed > 10 && horizSpeed > 0.01) {
      const flapFactor = this.bladeFlapCoeff * totalThrust;
      // Forward airspeed (body -z) → pitch-up moment (body +x)
      // Rightward airspeed (body +x) → roll moment (body +z)
      tauFlap[0] = -flapFactor * airVelBody[2]; // pitch from forward speed
      tauFlap[2] =  flapFactor * airVelBody[0]; // roll from lateral speed
    }

    // 3d. Induced drag — additional drag from tilted rotor disc in forward flight
    const fInducedDrag = [0, 0, 0];
    if (totalThrust > 0.01 && horizSpeed > 0.01) {
      const factor = -this.inducedDragCoeff * totalThrust / Math.max(horizSpeed, 0.1);
      fInducedDrag[0] = factor * horizAirBody[0];
      fInducedDrag[2] = factor * horizAirBody[2];
    }

    // 3e. Translational lift — in forward flight, induced velocity decreases,
    //     increasing effective thrust.  Model as a small thrust boost proportional to airspeed.
    const transLiftForce = [0, 0, 0];
    if (totalThrust > 0.01 && horizSpeed > 0.5) {
      const liftBoost = 0.015 * totalThrust * Math.min(horizSpeed / 5.0, 1.0);
      transLiftForce[1] = liftBoost; // extra lift in body +y
    }

    // Total aerodynamic force (body frame)
    const fAeroBody = [
      fDragBody[0] + fRotorDrag[0] + fInducedDrag[0] + transLiftForce[0],
      fDragBody[1] + transLiftForce[1],
      fDragBody[2] + fRotorDrag[2] + fInducedDrag[2] + transLiftForce[2],
    ];

    // ── 4. Ground effect ──
    //   When h < 2*rotorRadius, thrust is augmented.
    //   T_eff = T / (1 - (R/(4h))²)   for h > R/4
    const h = Math.max(this.position[1], 0.01); // height above ground
    let groundEffectFactor = 1.0;
    const geThreshold = 2.0 * this.rotorRadius;
    if (h < geThreshold) {
      const ratio = this.rotorRadius / (4 * h);
      groundEffectFactor = 1.0 / (1.0 - Math.min(ratio * ratio, 0.8));
    }

    // ── 5. Gyroscopic torque from spinning rotors ──
    //   τ_gyro = -ω_body × (Σ Ir * ωi * dir_i * e_y)
    //   where dir_i is spin direction (+1 CW = angular momentum -y, so we sum -dir_i)
    let rotorAngMomY = 0;
    for (let i = 0; i < 4; i++) {
      rotorAngMomY += -this.motorDir[i] * this.rotorInertia * this.motorSpeeds[i];
    }
    // For CW rotor (dir=+1), angular momentum in body frame is along -y (spin about -y from above)
    // So total rotor angular momentum vector = [0, rotorAngMomY, 0]
    // Gyroscopic torque = -ω × L_rotor
    const [wx, wy, wz] = this.angularVelocity;
    const tauGyro = [
      -(wy * 0 - wz * rotorAngMomY),       // = wz * rotorAngMomY
       -(wz * 0 - wx * 0),                  // = 0 (symmetric)
      -(wx * rotorAngMomY - wy * 0),        // = -wx * rotorAngMomY
    ];
    // Simplified: tauGyro = [wz*rotorAngMomY, 0, -wx*rotorAngMomY]

    // ── 6. Total torques (body frame) ──
    const tauTotal = [
      tauX + tauFlap[0] + tauGyro[0],
      tauY + tauFlap[1] + tauGyro[1],
      tauZ + tauFlap[2] + tauGyro[2],
    ];

    // ── 7. Total force (body frame) → convert to world ──
    const thrustBody = [0, totalThrust * groundEffectFactor, 0];
    const totalForceBody = [
      thrustBody[0] + fAeroBody[0],
      thrustBody[1] + fAeroBody[1],
      thrustBody[2] + fAeroBody[2],
    ];
    const totalForceWorld = bodyToWorld(this.quaternion, totalForceBody);

    // ── 8. Integrate translational dynamics (semi-implicit Euler) ──
    const ax = totalForceWorld[0] / this.mass;
    const ay = totalForceWorld[1] / this.mass - GRAVITY;
    const az = totalForceWorld[2] / this.mass;

    this.velocity[0] += ax * dt;
    this.velocity[1] += ay * dt;
    this.velocity[2] += az * dt;

    this.position[0] += this.velocity[0] * dt;
    this.position[1] += this.velocity[1] * dt;
    this.position[2] += this.velocity[2] * dt;

    // ── 9. Integrate rotational dynamics ──
    //   α = J⁻¹ * (τ - ω × Jω)
    const Jw = [this.Ixx * wx, this.Iyy * wy, this.Izz * wz];
    const crossJw = [
      wy * Jw[2] - wz * Jw[1],
      wz * Jw[0] - wx * Jw[2],
      wx * Jw[1] - wy * Jw[0],
    ];
    const alphaX = (tauTotal[0] - crossJw[0]) / this.Ixx;
    const alphaY = (tauTotal[1] - crossJw[1]) / this.Iyy;
    const alphaZ = (tauTotal[2] - crossJw[2]) / this.Izz;

    this.angularVelocity[0] += alphaX * dt;
    this.angularVelocity[1] += alphaY * dt;
    this.angularVelocity[2] += alphaZ * dt;

    // ── 10. Integrate quaternion ──
    //   dq/dt = 0.5 * q ⊗ [0, ω]
    const omegaQ = [0, this.angularVelocity[0], this.angularVelocity[1], this.angularVelocity[2]];
    const dq = qMultiply(this.quaternion, omegaQ);
    this.quaternion[0] += 0.5 * dq[0] * dt;
    this.quaternion[1] += 0.5 * dq[1] * dt;
    this.quaternion[2] += 0.5 * dq[2] * dt;
    this.quaternion[3] += 0.5 * dq[3] * dt;
    this.quaternion = qNormalize(this.quaternion);

    // ── 11. Ground collision ──
    if (this.position[1] <= 0) {
      this.position[1] = 0;
      if (this.velocity[1] < -5.0) {
        // Hard crash (impact > 5 m/s vertical)
        this.crashed = true;
      }
      // Ground contact — stop downward velocity, add friction
      if (this.velocity[1] < 0) this.velocity[1] = 0;
      // Friction
      this.velocity[0] *= 0.95;
      this.velocity[2] *= 0.95;
      // Damp angular velocity on ground
      this.angularVelocity[0] *= 0.9;
      this.angularVelocity[1] *= 0.9;
      this.angularVelocity[2] *= 0.9;
    }

    // ── 12. Safety clamp (prevent NaN explosion) ──
    const maxVel = 50;
    const maxAngVel = 30;
    for (let i = 0; i < 3; i++) {
      this.velocity[i] = Math.max(-maxVel, Math.min(maxVel, this.velocity[i]));
      this.angularVelocity[i] = Math.max(-maxAngVel, Math.min(maxAngVel, this.angularVelocity[i]));
    }
  }

  // ── Convenience getters ──

  getAltitude()     { return this.position[1]; }
  getSpeed()        { return Math.sqrt(this.velocity[0]**2 + this.velocity[2]**2); }
  getVerticalSpeed(){ return this.velocity[1]; }
  getHeading()      { return qToYaw(this.quaternion) * 180 / Math.PI; }
  getEuler()        { return qToEuler(this.quaternion); }
  getMotorRPM()     { return this.motorSpeeds.map(w => w * 60 / (2*Math.PI)); }
  getWindSpeed()    { const w = this.wind.getWind(this.simTime); return Math.sqrt(w[0]**2+w[1]**2+w[2]**2); }
}

export { qToEuler, bodyToWorld, worldToBody, WindModel, GRAVITY };
