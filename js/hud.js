/**
 * hud.js — Head-Up Display
 *
 * Updates the HTML/CSS HUD overlay and renders:
 *   - Telemetry data (altitude, speed, heading, etc.)
 *   - Motor RPM bars
 *   - Attitude indicator (artificial horizon) on canvas
 *   - Stick position indicators on canvas
 */

export class HUD {
  constructor() {
    // Cache DOM elements
    this.elMode      = document.getElementById('hud-mode');
    this.elFps       = document.getElementById('hud-fps');
    this.elAlt       = document.getElementById('hud-alt');
    this.elSpeed     = document.getElementById('hud-speed');
    this.elVSpeed    = document.getElementById('hud-vspeed');
    this.elHeading   = document.getElementById('hud-heading');
    this.elWind      = document.getElementById('hud-wind-value');
    this.elPos       = document.getElementById('hud-pos-value');

    this.motorBars   = [1,2,3,4].map(i => document.getElementById(`motor-bar-${i}`));
    this.motorLabels = [1,2,3,4].map(i => document.getElementById(`motor-rpm-${i}`));

    // Attitude canvas
    this.attCanvas = document.getElementById('attitude-canvas');
    this.attCtx    = this.attCanvas.getContext('2d');

    // Stick canvases
    this.stickLeftCanvas  = document.getElementById('stick-left');
    this.stickRightCanvas = document.getElementById('stick-right');
    this.stickLeftCtx     = this.stickLeftCanvas.getContext('2d');
    this.stickRightCtx    = this.stickRightCanvas.getContext('2d');

    this._fpsFrames = 0;
    this._fpsTime   = 0;
    this._fpsValue  = 60;
  }

  /**
   * Update HUD display.
   * @param {QuadrotorPhysics} physics
   * @param {object}           input   – { throttle, yaw, pitch, roll } raw values
   * @param {string}           mode    – 'angle' or 'acro'
   * @param {number}           dt      – frame delta
   */
  update(physics, input, mode, dt) {
    // FPS counter
    this._fpsFrames++;
    this._fpsTime += dt;
    if (this._fpsTime >= 0.5) {
      this._fpsValue = Math.round(this._fpsFrames / this._fpsTime);
      this._fpsFrames = 0;
      this._fpsTime = 0;
    }

    // Mode
    this.elMode.textContent = mode.toUpperCase() + ' MODE';
    if (physics.crashed) {
      this.elMode.textContent = 'CRASHED — Press R to Reset';
      this.elMode.style.color = '#f44';
      this.elMode.style.borderColor = 'rgba(255,60,60,0.5)';
    } else {
      this.elMode.style.color = '';
      this.elMode.style.borderColor = '';
    }

    this.elFps.textContent = `${this._fpsValue} FPS`;

    // Telemetry
    this.elAlt.textContent     = physics.getAltitude().toFixed(1) + ' m';
    this.elSpeed.textContent   = physics.getSpeed().toFixed(1) + ' m/s';

    const vs = physics.getVerticalSpeed();
    this.elVSpeed.textContent  = (vs >= 0 ? '+' : '') + vs.toFixed(1) + ' m/s';
    this.elVSpeed.style.color  = vs < -2 ? '#f44' : vs > 1 ? '#4f4' : '#0f0';

    const hdg = ((physics.getHeading() % 360) + 360) % 360;
    this.elHeading.textContent = Math.round(hdg) + '°';

    this.elWind.textContent    = physics.getWindSpeed().toFixed(1) + ' m/s';

    const [px, py, pz] = physics.position;
    this.elPos.textContent = `X:${px.toFixed(1)} Y:${py.toFixed(1)} Z:${pz.toFixed(1)}`;

    // Motor bars
    const rpms = physics.getMotorRPM();
    const maxRPM = physics.maxMotorRad * 60 / (2 * Math.PI);
    for (let i = 0; i < 4; i++) {
      const pct = Math.min(100, (rpms[i] / maxRPM) * 100);
      this.motorBars[i].style.height = (pct * 0.4) + 'px';
      this.motorLabels[i].textContent = Math.round(rpms[i]);
      // Color
      const ratio = rpms[i] / maxRPM;
      if (ratio > 0.85) {
        this.motorBars[i].style.background = 'linear-gradient(to top, #a00, #f44)';
      } else if (ratio > 0.6) {
        this.motorBars[i].style.background = 'linear-gradient(to top, #aa0, #ff4)';
      } else {
        this.motorBars[i].style.background = 'linear-gradient(to top, #0a0, #0f0)';
      }
    }

    // Attitude indicator
    this._drawAttitude(physics);

    // Stick indicators
    this._drawStick(this.stickLeftCtx, input.yaw, input.throttle * 2 - 1); // throttle mapped to -1..1
    this._drawStick(this.stickRightCtx, input.roll, input.pitch);
  }

  _drawAttitude(physics) {
    const ctx = this.attCtx;
    const w = this.attCanvas.width;
    const h = this.attCanvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r  = Math.min(cx, cy) - 4;

    const euler = physics.getEuler();
    const roll  = euler.roll;
    const pitch = euler.pitch;

    ctx.clearRect(0, 0, w, h);

    // Clip to circle
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.clip();

    // Sky / ground split based on pitch
    const pitchOffset = (pitch / (Math.PI/2)) * r;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);

    // Sky
    ctx.fillStyle = '#2255aa';
    ctx.fillRect(-r*2, -r*2, r*4, r*2 + pitchOffset);

    // Ground
    ctx.fillStyle = '#665533';
    ctx.fillRect(-r*2, pitchOffset, r*4, r*4);

    // Horizon line
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(-r*2, pitchOffset);
    ctx.lineTo(r*2, pitchOffset);
    ctx.stroke();

    // Pitch ladder lines
    ctx.strokeStyle = 'rgba(255,255,255,0.4)';
    ctx.lineWidth = 0.5;
    ctx.font = '7px monospace';
    ctx.fillStyle = 'rgba(255,255,255,0.6)';
    for (let deg = -60; deg <= 60; deg += 10) {
      if (deg === 0) continue;
      const yOff = pitchOffset - (deg / 90) * r;
      const lineW = Math.abs(deg) % 30 === 0 ? 20 : 10;
      ctx.beginPath();
      ctx.moveTo(-lineW, yOff);
      ctx.lineTo(lineW, yOff);
      ctx.stroke();
      if (Math.abs(deg) % 20 === 0) {
        ctx.fillText(`${deg}`, lineW + 2, yOff + 3);
      }
    }

    ctx.restore();

    // Aircraft reference (fixed)
    ctx.strokeStyle = '#ff0';
    ctx.lineWidth = 2;
    // Left wing
    ctx.beginPath();
    ctx.moveTo(cx - 30, cy);
    ctx.lineTo(cx - 10, cy);
    ctx.lineTo(cx - 10, cy + 5);
    ctx.stroke();
    // Right wing
    ctx.beginPath();
    ctx.moveTo(cx + 30, cy);
    ctx.lineTo(cx + 10, cy);
    ctx.lineTo(cx + 10, cy + 5);
    ctx.stroke();
    // Center dot
    ctx.fillStyle = '#ff0';
    ctx.beginPath();
    ctx.arc(cx, cy, 2, 0, Math.PI*2);
    ctx.fill();

    ctx.restore();

    // Border circle
    ctx.strokeStyle = 'rgba(0,255,0,0.4)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // Roll indicator triangle at top
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    ctx.fillStyle = '#ff0';
    ctx.beginPath();
    ctx.moveTo(0, -r + 2);
    ctx.lineTo(-4, -r + 8);
    ctx.lineTo(4, -r + 8);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  _drawStick(ctx, x, y) {
    const w = ctx.canvas.width;
    const h = ctx.canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const maxR = cx - 8;

    ctx.clearRect(0, 0, w, h);

    // Background cross
    ctx.strokeStyle = 'rgba(0,255,0,0.15)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(cx, 4); ctx.lineTo(cx, h-4);
    ctx.moveTo(4, cy); ctx.lineTo(w-4, cy);
    ctx.stroke();

    // Outline circle
    ctx.strokeStyle = 'rgba(0,255,0,0.2)';
    ctx.beginPath();
    ctx.arc(cx, cy, maxR, 0, Math.PI*2);
    ctx.stroke();

    // Stick position
    const sx = cx + x * maxR;
    const sy = cy - y * maxR; // invert y (up is positive)

    ctx.fillStyle = 'rgba(0,255,0,0.7)';
    ctx.beginPath();
    ctx.arc(sx, sy, 5, 0, Math.PI*2);
    ctx.fill();

    ctx.strokeStyle = '#0f0';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(sx, sy, 5, 0, Math.PI*2);
    ctx.stroke();
  }
}
