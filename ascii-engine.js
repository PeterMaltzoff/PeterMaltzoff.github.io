/* ============================================================
   ASCII Art Engine — Procedural renderer & animation system
   Inspired by asciigen.art. Pure text, no images.
   ============================================================ */

/* ---------- core renderer ---------- */
class ASCIIRenderer {
  constructor(width, height) {
    this.width = width;
    this.height = height;
    this.buffer = [];
    this.clear();
  }

  clear() {
    this.buffer = Array.from({ length: this.height }, () =>
      Array(this.width).fill(" ")
    );
  }

  set(x, y, ch) {
    const ix = Math.round(x);
    const iy = Math.round(y);
    if (ix >= 0 && ix < this.width && iy >= 0 && iy < this.height) {
      this.buffer[iy][ix] = ch;
    }
  }

  get(x, y) {
    const ix = Math.round(x);
    const iy = Math.round(y);
    if (ix >= 0 && ix < this.width && iy >= 0 && iy < this.height) {
      return this.buffer[iy][ix];
    }
    return " ";
  }

  /* Bresenham line */
  line(x0, y0, x1, y1, ch) {
    x0 = Math.round(x0); y0 = Math.round(y0);
    x1 = Math.round(x1); y1 = Math.round(y1);
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;
    while (true) {
      this.set(x0, y0, ch);
      if (x0 === x1 && y0 === y1) break;
      const e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx)  { err += dx; y0 += sy; }
    }
  }

  /* Midpoint circle */
  circle(cx, cy, r, ch) {
    cx = Math.round(cx); cy = Math.round(cy); r = Math.round(r);
    let x = r, y = 0, d = 1 - r;
    while (x >= y) {
      this.set(cx + x, cy + y, ch);
      this.set(cx - x, cy + y, ch);
      this.set(cx + x, cy - y, ch);
      this.set(cx - x, cy - y, ch);
      this.set(cx + y, cy + x, ch);
      this.set(cx - y, cy + x, ch);
      this.set(cx + y, cy - x, ch);
      this.set(cx - y, cy - x, ch);
      y++;
      if (d <= 0) { d += 2 * y + 1; }
      else        { x--; d += 2 * (y - x) + 1; }
    }
  }

  filledCircle(cx, cy, r, ch) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (dx * dx + dy * dy <= r * r) {
          this.set(cx + dx, cy + dy, ch);
        }
      }
    }
  }

  text(x, y, str) {
    for (let i = 0; i < str.length; i++) {
      this.set(x + i, y, str[i]);
    }
  }

  render() {
    return this.buffer.map(row => row.join("")).join("\n");
  }
}

/* ---------- easing helpers ---------- */
function lerp(a, b, t) { return a + (b - a) * t; }
function smoothstep(t) { return t * t * (3 - 2 * t); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

/* ==========================================================
   ANIMATION: Neural Network
   A multi-layer network with animated data pulses
   ========================================================== */
class NeuralNetworkAnimation {
  constructor(w, h) {
    this.r = new ASCIIRenderer(w, h);
    this.w = w;
    this.h = h;
    this.time = 0;

    /* define layers — each layer is an array of {x,y} */
    const layers = 5;
    const maxNodes = 7;
    const marginX = 6;
    const spacingX = (w - marginX * 2) / (layers - 1);

    this.layers = [];
    const nodeCounts = [3, 5, maxNodes, 5, 2];
    for (let l = 0; l < layers; l++) {
      const count = nodeCounts[l];
      const lx = marginX + l * spacingX;
      const spacingY = (h - 4) / (count + 1);
      const nodes = [];
      for (let n = 0; n < count; n++) {
        nodes.push({ x: Math.round(lx), y: Math.round(spacingY * (n + 1) + 2) });
      }
      this.layers.push(nodes);
    }

    /* pre-compute connections */
    this.connections = [];
    for (let l = 0; l < this.layers.length - 1; l++) {
      for (const a of this.layers[l]) {
        for (const b of this.layers[l + 1]) {
          this.connections.push({ ax: a.x, ay: a.y, bx: b.x, by: b.y });
        }
      }
    }

    /* pulses */
    this.pulses = [];
    this.spawnTimer = 0;
  }

  update() {
    this.time++;
    this.spawnTimer++;

    /* spawn new pulses periodically */
    if (this.spawnTimer > 4) {
      this.spawnTimer = 0;
      const conn = this.connections[Math.floor(Math.random() * this.connections.length)];
      this.pulses.push({
        ax: conn.ax, ay: conn.ay,
        bx: conn.bx, by: conn.by,
        t: 0, speed: 0.04 + Math.random() * 0.03,
      });
    }

    /* advance pulses */
    for (const p of this.pulses) p.t += p.speed;
    this.pulses = this.pulses.filter(p => p.t <= 1);
  }

  draw() {
    this.r.clear();

    const connectionChars = ["-", "~", "·"];

    /* draw connections (subtle) */
    for (const c of this.connections) {
      const dx = c.bx - c.ax;
      const dy = c.by - c.ay;
      const steps = Math.max(Math.abs(dx), Math.abs(dy));
      for (let i = 0; i <= steps; i += 2) {
        const t = i / steps;
        const x = Math.round(lerp(c.ax, c.bx, t));
        const y = Math.round(lerp(c.ay, c.by, t));
        if (this.r.get(x, y) === " ") {
          this.r.set(x, y, "·");
        }
      }
    }

    /* draw pulses */
    const pulseChars = ["█", "▓", "▒", "░"];
    for (const p of this.pulses) {
      const x = Math.round(lerp(p.ax, p.bx, p.t));
      const y = Math.round(lerp(p.ay, p.by, p.t));
      this.r.set(x, y, pulseChars[0]);
      /* trail */
      for (let trail = 1; trail < pulseChars.length; trail++) {
        const tt = p.t - trail * p.speed * 1.5;
        if (tt >= 0) {
          const tx = Math.round(lerp(p.ax, p.bx, tt));
          const ty = Math.round(lerp(p.ay, p.by, tt));
          if (this.r.get(tx, ty) !== pulseChars[0]) {
            this.r.set(tx, ty, pulseChars[trail]);
          }
        }
      }
    }

    /* draw nodes */
    const nodeGlyphs = ["◉", "◎", "●", "○"];
    for (let l = 0; l < this.layers.length; l++) {
      for (const node of this.layers[l]) {
        /* glow ring */
        const phase = (this.time * 0.05 + node.x * 0.1 + node.y * 0.1) % 1;
        const glow = phase < 0.5 ? "○" : "◌";
        this.r.set(node.x - 1, node.y, "[");
        this.r.set(node.x, node.y, "●");
        this.r.set(node.x + 1, node.y, "]");
      }
    }

    /* layer labels */
    const labels = ["INPUT", "HIDDEN", "DEEP", "HIDDEN", "OUTPUT"];
    for (let l = 0; l < this.layers.length; l++) {
      const lx = this.layers[l][0].x;
      const label = labels[l];
      this.r.text(lx - Math.floor(label.length / 2), 0, label);
    }

    return this.r.render();
  }
}

/* ==========================================================
   ANIMATION: Data Stream / Matrix Rain (subtle, blue-themed)
   ========================================================== */
class DataStreamAnimation {
  constructor(w, h) {
    this.r = new ASCIIRenderer(w, h);
    this.w = w;
    this.h = h;
    this.time = 0;
    this.chars = "01αβγδεζηθλμνξπρσφψω∑∏∫∂∇≈≠≤≥∞".split("");
    this.columns = [];
    for (let x = 0; x < w; x++) {
      this.columns.push({
        y: Math.random() * h * 2 - h,
        speed: 0.2 + Math.random() * 0.5,
        length: 4 + Math.floor(Math.random() * 12),
        active: Math.random() > 0.5,
      });
    }
  }

  update() {
    this.time++;
    for (const col of this.columns) {
      if (!col.active) {
        if (Math.random() < 0.005) col.active = true;
        continue;
      }
      col.y += col.speed;
      if (col.y - col.length > this.h) {
        col.y = -col.length;
        col.speed = 0.2 + Math.random() * 0.5;
        col.length = 4 + Math.floor(Math.random() * 12);
        if (Math.random() < 0.3) col.active = false;
      }
    }
  }

  draw() {
    this.r.clear();
    for (let x = 0; x < this.w; x++) {
      const col = this.columns[x];
      if (!col.active) continue;
      const head = Math.floor(col.y);
      for (let i = 0; i < col.length; i++) {
        const y = head - i;
        if (y >= 0 && y < this.h) {
          if (i === 0) {
            this.r.set(x, y, this.chars[Math.floor(Math.random() * this.chars.length)]);
          } else {
            const fade = 1 - i / col.length;
            if (fade > 0.6) {
              this.r.set(x, y, this.chars[(this.time + x + y) % this.chars.length]);
            } else if (fade > 0.3) {
              this.r.set(x, y, "·");
            } else {
              this.r.set(x, y, ".");
            }
          }
        }
      }
    }
    return this.r.render();
  }
}

/* ==========================================================
   ANIMATION: Waveform / Signal Processing
   ========================================================== */
class WaveformAnimation {
  constructor(w, h) {
    this.r = new ASCIIRenderer(w, h);
    this.w = w;
    this.h = h;
    this.time = 0;
  }

  update() {
    this.time += 0.06;
  }

  draw() {
    this.r.clear();
    const midY = this.h / 2;

    /* draw multiple overlapping waves */
    const waves = [
      { amp: this.h * 0.3, freq: 0.08, phase: 0, ch: "█" },
      { amp: this.h * 0.22, freq: 0.12, phase: 2, ch: "▓" },
      { amp: this.h * 0.15, freq: 0.18, phase: 4, ch: "░" },
    ];

    for (const wave of waves) {
      for (let x = 0; x < this.w; x++) {
        const y = midY + Math.sin(x * wave.freq + this.time + wave.phase) * wave.amp;
        this.r.set(x, Math.round(y), wave.ch);
        /* fill towards midline for thickness */
        const dir = y > midY ? -1 : 1;
        for (let f = 1; f < 2; f++) {
          this.r.set(x, Math.round(y + f * dir), "·");
        }
      }
    }

    /* center line */
    for (let x = 0; x < this.w; x++) {
      if (this.r.get(x, Math.round(midY)) === " ") {
        this.r.set(x, Math.round(midY), "─");
      }
    }

    return this.r.render();
  }
}

/* ==========================================================
   ANIMATION: Brain / AI Visualization
   A stylized brain outline with pulsing activity
   ========================================================== */
class BrainAnimation {
  constructor(w, h) {
    this.r = new ASCIIRenderer(w, h);
    this.w = w;
    this.h = h;
    this.time = 0;

    /* pre-compute brain outline using parametric curves */
    this.outlinePoints = [];
    const cx = w / 2;
    const cy = h / 2;
    const scaleX = w * 0.35;
    const scaleY = h * 0.4;

    /* left hemisphere */
    for (let t = 0; t <= Math.PI; t += 0.05) {
      const x = cx - Math.sin(t) * scaleX * (0.5 + 0.2 * Math.sin(t * 3));
      const y = cy - Math.cos(t) * scaleY;
      this.outlinePoints.push({ x: Math.round(x), y: Math.round(y), side: "L" });
    }
    /* right hemisphere */
    for (let t = 0; t <= Math.PI; t += 0.05) {
      const x = cx + Math.sin(t) * scaleX * (0.5 + 0.2 * Math.sin(t * 3));
      const y = cy - Math.cos(t) * scaleY;
      this.outlinePoints.push({ x: Math.round(x), y: Math.round(y), side: "R" });
    }

    /* internal activity points */
    this.sparks = [];
    for (let i = 0; i < 25; i++) {
      const angle = Math.random() * Math.PI * 2;
      const radius = Math.random() * 0.7;
      this.sparks.push({
        x: cx + Math.cos(angle) * scaleX * 0.4 * radius,
        y: cy + Math.sin(angle) * scaleY * 0.35 * radius,
        phase: Math.random() * Math.PI * 2,
        freq: 0.03 + Math.random() * 0.05,
      });
    }
  }

  update() {
    this.time++;
  }

  draw() {
    this.r.clear();

    /* draw outline */
    for (const p of this.outlinePoints) {
      this.r.set(p.x, p.y, "█");
    }

    /* center line (corpus callosum) */
    const cx = Math.round(this.w / 2);
    for (let y = Math.round(this.h * 0.2); y < Math.round(this.h * 0.8); y++) {
      this.r.set(cx, y, "│");
    }

    /* folds / gyri */
    for (let i = 0; i < 4; i++) {
      const fy = Math.round(this.h * (0.25 + i * 0.15));
      const fw = Math.round(this.w * (0.15 + i * 0.03));
      /* left side */
      for (let dx = 2; dx < fw; dx++) {
        this.r.set(cx - dx, fy, "~");
      }
      /* right side */
      for (let dx = 2; dx < fw; dx++) {
        this.r.set(cx + dx, fy, "~");
      }
    }

    /* neural sparks */
    const sparkChars = ["✦", "◆", "●", "○", "·"];
    for (const s of this.sparks) {
      const intensity = (Math.sin(this.time * s.freq + s.phase) + 1) / 2;
      const idx = Math.floor((1 - intensity) * (sparkChars.length - 1));
      this.r.set(Math.round(s.x), Math.round(s.y), sparkChars[idx]);
    }

    return this.r.render();
  }
}

/* ==========================================================
   ANIMATION: Floating Nodes / Graph Network (for services)
   ========================================================== */
class GraphAnimation {
  constructor(w, h) {
    this.r = new ASCIIRenderer(w, h);
    this.w = w;
    this.h = h;
    this.time = 0;

    this.nodes = [];
    for (let i = 0; i < 12; i++) {
      this.nodes.push({
        x: 4 + Math.random() * (w - 8),
        y: 2 + Math.random() * (h - 4),
        vx: (Math.random() - 0.5) * 0.3,
        vy: (Math.random() - 0.5) * 0.15,
        label: ["ML", "AI", "DL", "NLP", "CV", "LLM", "GAN", "RL", "CNN", "RNN", "GPU", "TPU"][i],
      });
    }
  }

  update() {
    this.time++;
    for (const n of this.nodes) {
      n.x += n.vx;
      n.y += n.vy;
      if (n.x < 3 || n.x > this.w - 4) n.vx *= -1;
      if (n.y < 1 || n.y > this.h - 2) n.vy *= -1;
      n.x = clamp(n.x, 3, this.w - 4);
      n.y = clamp(n.y, 1, this.h - 2);
    }
  }

  draw() {
    this.r.clear();

    /* draw connections between close nodes */
    for (let i = 0; i < this.nodes.length; i++) {
      for (let j = i + 1; j < this.nodes.length; j++) {
        const a = this.nodes[i];
        const b = this.nodes[j];
        const dist = Math.sqrt((a.x - b.x) ** 2 + ((a.y - b.y) * 2) ** 2);
        if (dist < 30) {
          const ch = dist < 15 ? "─" : "·";
          const steps = Math.max(Math.abs(b.x - a.x), Math.abs(b.y - a.y));
          for (let s = 0; s <= steps; s += 2) {
            const t = s / steps;
            const x = Math.round(lerp(a.x, b.x, t));
            const y = Math.round(lerp(a.y, b.y, t));
            if (this.r.get(x, y) === " ") {
              this.r.set(x, y, ch);
            }
          }
        }
      }
    }

    /* draw nodes */
    for (const n of this.nodes) {
      const nx = Math.round(n.x);
      const ny = Math.round(n.y);
      this.r.set(nx - 1, ny, "[");
      this.r.text(nx, ny, n.label);
      this.r.set(nx + n.label.length, ny, "]");
    }

    return this.r.render();
  }
}

/* ==========================================================
   ANIMATION CONTROLLER — mounts animations to DOM elements
   ========================================================== */
class ASCIIAnimationController {
  constructor(element, AnimationClass, opts = {}) {
    this.el = element;
    this.fps = opts.fps || 20;
    this.autoScale = opts.autoScale !== false;
    this.running = false;
    this.frameId = null;
    this.lastFrameTime = 0;
    this.frameInterval = 1000 / this.fps;
    this.animation = null;
    this.AnimationClass = AnimationClass;
    this.intersectionObserver = null;
    this.visible = true;

    this._init();
  }

  _init() {
    /* make unselectable */
    this.el.style.userSelect = "none";
    this.el.style.webkitUserSelect = "none";
    this.el.style.pointerEvents = "none";
    this.el.setAttribute("aria-hidden", "true");

    /* ensure container is a positioning context */
    const pos = window.getComputedStyle(this.el).position;
    if (pos === "static") this.el.style.position = "relative";

    /* create <pre> — absolutely positioned so content can never affect layout */
    this.pre = document.createElement("pre");
    this.pre.style.position = "absolute";
    this.pre.style.top = "0";
    this.pre.style.left = "0";
    this.pre.style.right = "0";
    this.pre.style.bottom = "0";
    this.pre.style.margin = "0";
    this.pre.style.lineHeight = "1.2";
    this.pre.style.fontFamily = "'JetBrains Mono', 'Fira Code', 'Cascadia Code', 'Consolas', monospace";
    this.pre.style.overflow = "hidden";
    this.pre.style.whiteSpace = "pre";
    this.pre.style.contain = "strict";
    /* persistent text node — avoids DOM node churn on every frame */
    this._textNode = document.createTextNode("");
    this.pre.appendChild(this._textNode);
    this.el.appendChild(this.pre);

    /* compute grid size from container — defer to allow CSS to apply */
    requestAnimationFrame(() => this._computeSize());

    /* recompute on window resize only (debounced to avoid feedback loops) */
    this._resizeHandler = () => {
      clearTimeout(this._resizeTimer);
      this._resizeTimer = setTimeout(() => this._computeSize(), 200);
    };
    window.addEventListener("resize", this._resizeHandler);

    /* intersection observer — pause when off-screen */
    this.intersectionObserver = new IntersectionObserver(
      (entries) => {
        this.visible = entries[0].isIntersecting;
        if (this.visible && this.running) this._loop();
      },
      { threshold: 0.05 }
    );
    this.intersectionObserver.observe(this.el);

    /* respect prefers-reduced-motion */
    this.prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  }

  _computeSize() {
    const rect = this.el.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;

    /* measure a single character */
    const computed = window.getComputedStyle(this.pre);
    const probe = document.createElement("span");
    probe.style.fontFamily = computed.fontFamily;
    probe.style.fontSize = computed.fontSize || "14px";
    probe.style.lineHeight = computed.lineHeight;
    probe.style.position = "absolute";
    probe.style.visibility = "hidden";
    probe.style.whiteSpace = "pre";
    probe.textContent = "M";
    document.body.appendChild(probe);
    const charW = probe.getBoundingClientRect().width;
    const charH = probe.getBoundingClientRect().height;
    document.body.removeChild(probe);

    if (charW === 0 || charH === 0) return;

    /* account for padding on the <pre> */
    const padTop = parseFloat(computed.paddingTop) || 0;
    const padBot = parseFloat(computed.paddingBottom) || 0;
    const padLeft = parseFloat(computed.paddingLeft) || 0;
    const padRight = parseFloat(computed.paddingRight) || 0;

    const availW = rect.width - padLeft - padRight;
    const availH = rect.height - padTop - padBot;

    const cols = Math.floor(availW / charW);
    const rows = Math.floor(availH / charH);

    if (cols > 0 && rows > 0 && (!this.animation || cols !== this.cols || rows !== this.rows)) {
      this.cols = cols;
      this.rows = rows;
      this.animation = new this.AnimationClass(cols, rows);
    }
  }

  start() {
    this.running = true;
    this._loop();
  }

  stop() {
    this.running = false;
    if (this.frameId) cancelAnimationFrame(this.frameId);
  }

  _loop(timestamp) {
    if (!this.running || !this.visible) return;

    this.frameId = requestAnimationFrame((ts) => this._loop(ts));

    if (timestamp === undefined) return;
    const delta = timestamp - this.lastFrameTime;
    if (delta < this.frameInterval) return;
    this.lastFrameTime = timestamp - (delta % this.frameInterval);

    if (!this.animation) return;

    if (!this.prefersReducedMotion) {
      this.animation.update();
    }
    this._textNode.nodeValue = this.animation.draw();
  }

  destroy() {
    this.stop();
    if (this._resizeHandler) window.removeEventListener("resize", this._resizeHandler);
    clearTimeout(this._resizeTimer);
    if (this.intersectionObserver) this.intersectionObserver.disconnect();
  }
}

/* export for use in main.js */
window.ASCIIEngine = {
  Renderer: ASCIIRenderer,
  NeuralNetwork: NeuralNetworkAnimation,
  DataStream: DataStreamAnimation,
  Waveform: WaveformAnimation,
  Brain: BrainAnimation,
  Graph: GraphAnimation,
  Controller: ASCIIAnimationController,
};
