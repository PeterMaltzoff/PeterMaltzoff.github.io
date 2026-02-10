/**
 * flight-controller.js — PID Flight Controller & Motor Mixer
 *
 * Implements a cascaded PID controller similar to Betaflight/PX4:
 *   ANGLE mode:  desired angles → attitude PID → rate setpoints → rate PID → motor outputs
 *   ACRO mode:   desired rates  → rate PID → motor outputs
 *
 * The motor mixer converts desired [thrust, τx, τy, τz] into 4 motor speed commands.
 */

// ── PID Controller ───────────────────────────────────────────────────────────

class PID {
  constructor(kp, ki, kd, iMax = 1.0, dFilterAlpha = 0.8) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.iMax = iMax;
    this.dFilterAlpha = dFilterAlpha; // low-pass on derivative

    this.integral = 0;
    this.prevError = 0;
    this.dFiltered = 0;
  }

  reset() {
    this.integral = 0;
    this.prevError = 0;
    this.dFiltered = 0;
  }

  /**
   * @param {number} error    – current error (setpoint - measured)
   * @param {number} dt       – timestep
   * @param {number} [dMeas]  – optional measured rate (for D-on-measurement)
   * @returns {number}        – control output
   */
  update(error, dt, dMeas) {
    if (dt <= 0) return 0;

    // P term
    const p = this.kp * error;

    // I term with anti-windup clamping
    this.integral += error * dt;
    this.integral = Math.max(-this.iMax, Math.min(this.iMax, this.integral));
    const i = this.ki * this.integral;

    // D term (low-pass filtered)
    let rawD;
    if (dMeas !== undefined) {
      rawD = -dMeas; // D-on-measurement: avoids derivative kick on setpoint change
    } else {
      rawD = (error - this.prevError) / dt;
    }
    this.dFiltered = this.dFilterAlpha * this.dFiltered + (1 - this.dFilterAlpha) * rawD;
    const d = this.kd * this.dFiltered;

    this.prevError = error;
    return p + i + d;
  }
}

// ── Flight Controller ────────────────────────────────────────────────────────

export class FlightController {
  constructor(physics) {
    this.physics = physics;

    // Mode: 'angle' or 'acro'
    this.mode = 'angle';

    // ── PID gains (tuned for our drone parameters) ──

    // Rate PIDs (inner loop) — rad/s error → torque
    this.ratePID = [
      new PID(0.012, 0.005, 0.0004, 0.5, 0.85),  // roll  (body z-axis)
      new PID(0.015, 0.005, 0.0003, 0.5, 0.85),  // yaw   (body y-axis)
      new PID(0.012, 0.005, 0.0004, 0.5, 0.85),  // pitch (body x-axis)
    ];

    // Attitude PIDs (outer loop, angle mode) — rad error → desired rate (rad/s)
    this.attPID = [
      new PID(8.0, 0.8, 0.15, 2.0, 0.85),  // roll
      new PID(8.0, 0.8, 0.15, 2.0, 0.85),  // pitch
    ];

    // Max angles for angle mode (radians)
    this.maxAngle = 35 * Math.PI / 180;  // 35 degrees
    // Max rates
    this.maxRate  = 8.0;    // rad/s  (≈460 deg/s)
    this.maxYawRate = 4.0;  // rad/s  (≈230 deg/s)
  }

  toggleMode() {
    this.mode = this.mode === 'angle' ? 'acro' : 'angle';
    this.resetPIDs();
    return this.mode;
  }

  resetPIDs() {
    this.ratePID.forEach(p => p.reset());
    this.attPID.forEach(p => p.reset());
  }

  /**
   * Compute motor commands from pilot input.
   *
   * @param {object} input – { throttle: 0..1, roll: -1..1, pitch: -1..1, yaw: -1..1 }
   * @param {number} dt    – timestep
   * @returns {number[]}   – motor speed commands [ω1, ω2, ω3, ω4] in rad/s
   */
  update(input, dt) {
    const phys = this.physics;
    const euler = phys.getEuler();
    const [wx, wy, wz] = phys.angularVelocity; // body rates

    let desiredRollRate, desiredPitchRate, desiredYawRate;

    if (this.mode === 'angle') {
      // ── ANGLE MODE ──
      // Input maps to desired angles (roll, pitch) and yaw rate.
      // Sign convention: in Three.js (y-up, forward=-z), positive rotations
      // about body axes are opposite to intuitive pilot directions:
      //   +wx = pitch backward, +wy = yaw left, +wz = roll left
      // So we negate inputs to match pilot expectations.
      const desiredRoll  = -input.roll  * this.maxAngle;
      const desiredPitch = -input.pitch * this.maxAngle;
      desiredYawRate     = -input.yaw   * this.maxYawRate;

      // Current angles (from Euler decomposition)
      // roll ≈ rotation about body z mapped to a "tilt" angle
      // For small angles: roll ≈ euler.roll, pitch ≈ euler.pitch
      const currentRoll  = euler.roll;
      const currentPitch = euler.pitch;

      // Attitude PID → desired rates
      const rollError  = desiredRoll  - currentRoll;
      const pitchError = desiredPitch - currentPitch;

      desiredRollRate  = this.attPID[0].update(rollError,  dt);
      desiredPitchRate = this.attPID[1].update(pitchError, dt);

      // Clamp desired rates
      desiredRollRate  = Math.max(-this.maxRate, Math.min(this.maxRate, desiredRollRate));
      desiredPitchRate = Math.max(-this.maxRate, Math.min(this.maxRate, desiredPitchRate));
    } else {
      // ── ACRO MODE ──
      // Input directly maps to desired rates (negated for coordinate convention)
      desiredRollRate  = -input.roll  * this.maxRate;
      desiredPitchRate = -input.pitch * this.maxRate;
      desiredYawRate   = -input.yaw   * this.maxYawRate;
    }

    // ── Rate PID (inner loop) ──
    // Error = desired rate - current rate
    const rollRateError  = desiredRollRate  - wz;  // roll is about body z
    const yawRateError   = desiredYawRate   - wy;  // yaw is about body y
    const pitchRateError = desiredPitchRate - wx;  // pitch is about body x

    const rollTorque  = this.ratePID[0].update(rollRateError,  dt, wz);
    const yawTorque   = this.ratePID[1].update(yawRateError,   dt, wy);
    const pitchTorque = this.ratePID[2].update(pitchRateError, dt, wx);

    // ── Throttle → total thrust ──
    // Map throttle [0,1] to motor speed range
    // At hover, each motor ≈ hoverMotorSpeed
    const throttle = Math.max(0, Math.min(1, input.throttle));

    // Map to average motor speed squared (thrust is proportional to ω²)
    const maxOmegaSq = phys.maxMotorRad * phys.maxMotorRad;
    const avgOmegaSq = throttle * maxOmegaSq;
    // This gives us the "base" thrust for each motor in terms of ω²

    // ── Motor mixer ──
    // Convert desired thrust + torques to individual motor ω² values
    //
    // From physics.js derivation:
    //   totalThrust = kF * (ω1² + ω2² + ω3² + ω4²)
    //   τx (pitch)  = d * kF * (ω1² + ω2² - ω3² - ω4²)       ... wait, let me re-derive from motor positions
    //
    // Using motorPos from physics.js:
    //   M1(+d,0,-d), M2(-d,0,-d), M3(-d,0,+d), M4(+d,0,+d)
    //   τx = Σ -pz_i * T_i = d*(T1+T2-T3-T4)     [pitch]
    //   τz = Σ  px_i * T_i = d*(T1-T2-T3+T4)      [roll]
    //   τy = kM/kF * (T1-T2+T3-T4) * dir           [yaw]
    //     with dir: M1=+1, M2=-1, M3=+1, M4=-1
    //     τy = (kM/kF)*(T1-T2+T3-T4)
    //
    // Setting Ti = kF * ωi², we can write:
    //   F  = kF*(s1+s2+s3+s4)        where si = ωi²
    //   τx = kF*d*(s1+s2-s3-s4)
    //   τz = kF*d*(s1-s2-s3+s4)
    //   τy = kM*(s1-s2+s3-s4)
    //
    // Inverse mixer (solving for si):
    //   s1 = F/(4kF) + τx/(4kF*d) + τz/(4kF*d) + τy/(4kM)
    //   s2 = F/(4kF) + τx/(4kF*d) - τz/(4kF*d) - τy/(4kM)
    //   s3 = F/(4kF) - τx/(4kF*d) - τz/(4kF*d) + τy/(4kM)
    //   s4 = F/(4kF) - τx/(4kF*d) + τz/(4kF*d) - τy/(4kM)
    //
    // Simplify by working directly with ωi² = si:

    const d = phys.d;
    const kF = phys.kF;
    const kM = phys.kM;

    // Scale torque commands to motor ω² contributions
    const pitchMix = pitchTorque / (4 * kF * d);
    const rollMix  = rollTorque  / (4 * kF * d);
    const yawMix   = yawTorque   / (4 * kM);

    const s1 = avgOmegaSq + pitchMix + rollMix + yawMix;
    const s2 = avgOmegaSq + pitchMix - rollMix - yawMix;
    const s3 = avgOmegaSq - pitchMix - rollMix + yawMix;
    const s4 = avgOmegaSq - pitchMix + rollMix - yawMix;

    // Convert ω² to ω (clamp to valid range)
    const minSq = phys.minMotorRad * phys.minMotorRad;
    const toSpeed = (sq) => {
      if (throttle < 0.02) return 0; // motors off at zero throttle
      const clamped = Math.max(minSq, Math.min(maxOmegaSq, sq));
      return Math.sqrt(clamped);
    };

    return [toSpeed(s1), toSpeed(s2), toSpeed(s3), toSpeed(s4)];
  }
}
