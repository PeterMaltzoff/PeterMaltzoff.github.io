/**
 * input.js — Keyboard & Gamepad Input Handler
 *
 * Mode 2 layout (standard RC):
 *   Left stick:  Throttle (up/down) + Yaw (left/right)
 *   Right stick: Pitch (up/down) + Roll (left/right)
 *
 * Keyboard mapping:
 *   W/S = Throttle up/down
 *   A/D = Yaw left/right
 *   Arrow Up/Down = Pitch forward/back
 *   Arrow Left/Right = Roll left/right
 *
 * Features:
 *   - Exponential curve on sticks for fine control near center
 *   - Dead zone on gamepad sticks
 *   - Throttle hold (doesn't snap back, like a real throttle stick with no spring)
 */

export class InputHandler {
  constructor() {
    // Raw input state (all in -1..1 except throttle 0..1)
    this.throttle = 0;   // 0 .. 1
    this.yaw      = 0;   // -1 .. 1
    this.pitch    = 0;   // -1 .. 1
    this.roll     = 0;   // -1 .. 1

    // Smoothed output
    this.smoothThrottle = 0;
    this.smoothYaw      = 0;
    this.smoothPitch    = 0;
    this.smoothRoll     = 0;

    // Config
    this.expo        = 0.3;        // exponential curve (0 = linear, 1 = max expo)
    this.deadZone    = 0.08;       // gamepad dead zone
    this.smoothing   = 0.15;       // input smoothing factor (lower = smoother)
    this.throttleRate = 0.6;       // throttle change rate per second (keyboard)
    this.stickReturnRate = 12.0;   // how fast sticks return to center (keyboard, per second)

    // Key state
    this._keys = {};

    // Action callbacks
    this.onToggleMode   = null;
    this.onReset        = null;
    this.onToggleHelp   = null;
    this.onCameraChange = null;
    this.onToggleWind   = null;

    this._initKeyboard();
  }

  _initKeyboard() {
    window.addEventListener('keydown', (e) => {
      this._keys[e.code] = true;
      // Action keys
      if (e.code === 'KeyM')  this.onToggleMode?.();
      if (e.code === 'KeyR')  this.onReset?.();
      if (e.code === 'KeyH')  this.onToggleHelp?.();
      if (e.code === 'KeyG')  this.onToggleWind?.();
      if (e.code === 'Digit1') this.onCameraChange?.(0);
      if (e.code === 'Digit2') this.onCameraChange?.(1);
      if (e.code === 'Digit3') this.onCameraChange?.(2);
    });
    window.addEventListener('keyup', (e) => {
      this._keys[e.code] = false;
    });
  }

  /** Apply exponential curve: more precision near center. */
  _applyExpo(value) {
    const e = this.expo;
    return (1 - e) * value + e * value * value * value;
  }

  /** Apply dead zone to gamepad axis. */
  _applyDeadZone(value) {
    if (Math.abs(value) < this.deadZone) return 0;
    const sign = Math.sign(value);
    return sign * (Math.abs(value) - this.deadZone) / (1 - this.deadZone);
  }

  /**
   * Update input state. Call once per frame.
   * @param {number} dt – frame delta time
   */
  update(dt) {
    // ── Gamepad input (takes priority if connected) ──
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    let gpConnected = false;

    for (const gp of gamepads) {
      if (!gp) continue;
      gpConnected = true;

      // Standard gamepad mapping (Mode 2):
      //   Left stick:  axes[0]=yaw, axes[1]=throttle (inverted)
      //   Right stick: axes[2]=roll, axes[3]=pitch (inverted)
      const rawYaw      = this._applyDeadZone(gp.axes[0] || 0);
      const rawThrottle = this._applyDeadZone(-(gp.axes[1] || 0));
      const rawRoll     = this._applyDeadZone(gp.axes[2] || 0);
      const rawPitch    = this._applyDeadZone(-(gp.axes[3] || 0));

      this.yaw   = this._applyExpo(rawYaw);
      this.pitch = this._applyExpo(rawPitch);
      this.roll  = this._applyExpo(rawRoll);
      // Throttle: convert from -1..1 to 0..1
      this.throttle = Math.max(0, Math.min(1, (rawThrottle + 1) / 2));

      break; // use first connected gamepad
    }

    // ── Keyboard input (if no gamepad) ──
    if (!gpConnected) {
      const k = this._keys;

      // Throttle (held, incremental)
      if (k['KeyW'] || k['Space']) {
        this.throttle = Math.min(1, this.throttle + this.throttleRate * dt);
      } else if (k['KeyS']) {
        this.throttle = Math.max(0, this.throttle - this.throttleRate * dt);
      }
      // No return-to-center for throttle (like a real spring-less throttle stick)

      // Yaw
      const yawTarget = (k['KeyA'] ? -1 : 0) + (k['KeyD'] ? 1 : 0);
      // Pitch
      const pitchTarget = (k['ArrowUp'] ? 1 : 0) + (k['ArrowDown'] ? -1 : 0);
      // Roll
      const rollTarget = (k['ArrowLeft'] ? -1 : 0) + (k['ArrowRight'] ? 1 : 0);

      // Snap toward target with rate limiting
      const approach = (current, target, rate) => {
        if (Math.abs(target) > 0.01) {
          // Moving toward target
          return current + (target - current) * Math.min(1, rate * dt);
        } else {
          // Return to center
          return current * Math.max(0, 1 - this.stickReturnRate * dt);
        }
      };

      this.yaw   = approach(this.yaw,   yawTarget,   this.stickReturnRate);
      this.pitch = approach(this.pitch,  pitchTarget, this.stickReturnRate);
      this.roll  = approach(this.roll,   rollTarget,  this.stickReturnRate);
    }

    // ── Smooth output ──
    const s = this.smoothing;
    this.smoothThrottle = this.smoothThrottle + (this.throttle - this.smoothThrottle) * s;
    this.smoothYaw      = this.smoothYaw      + (this.yaw      - this.smoothYaw)      * s;
    this.smoothPitch    = this.smoothPitch     + (this.pitch    - this.smoothPitch)     * s;
    this.smoothRoll     = this.smoothRoll      + (this.roll     - this.smoothRoll)      * s;
  }

  /** Get the current input as a command object for the flight controller. */
  getCommand() {
    return {
      throttle: this.smoothThrottle,
      yaw:      this.smoothYaw,
      pitch:    this.smoothPitch,
      roll:     this.smoothRoll,
    };
  }

  /** Get raw (unsmoothed) values for stick display. */
  getRaw() {
    return {
      throttle: this.throttle,
      yaw:      this.yaw,
      pitch:    this.pitch,
      roll:     this.roll,
    };
  }
}
