/**
 * world.js — 3D Playground Environment
 *
 * Creates an explorable drone playground with:
 *   - Textured ground with grid
 *   - Buildings of various sizes
 *   - Trees
 *   - Landing pads
 *   - Race gates
 *   - Perimeter walls
 *   - Sky gradient
 *   - Lighting (sun + ambient + hemisphere)
 *   - Fog
 */

import * as THREE from 'three';

export class World {
  constructor(scene) {
    this.scene = scene;
    this.objects = []; // collidable objects for future use

    this._buildSky();
    this._buildLighting();
    this._buildGround();
    this._buildLandingPad();
    this._buildBuildings();
    this._buildTrees();
    this._buildGates();
    this._buildPerimeter();
    this._buildDecorations();
  }

  _buildSky() {
    // Gradient sky using a large sphere
    const skyGeo = new THREE.SphereGeometry(500, 32, 32);
    const skyMat = new THREE.ShaderMaterial({
      uniforms: {
        topColor:    { value: new THREE.Color(0x0044aa) },
        bottomColor: { value: new THREE.Color(0x88ccff) },
        offset:      { value: 20 },
        exponent:    { value: 0.4 },
      },
      vertexShader: `
        varying vec3 vWorldPosition;
        void main() {
          vec4 worldPos = modelMatrix * vec4(position, 1.0);
          vWorldPosition = worldPos.xyz;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
      `,
      fragmentShader: `
        uniform vec3 topColor;
        uniform vec3 bottomColor;
        uniform float offset;
        uniform float exponent;
        varying vec3 vWorldPosition;
        void main() {
          float h = normalize(vWorldPosition + offset).y;
          gl_FragColor = vec4(mix(bottomColor, topColor, max(pow(max(h, 0.0), exponent), 0.0)), 1.0);
        }
      `,
      side: THREE.BackSide,
    });
    this.scene.add(new THREE.Mesh(skyGeo, skyMat));

    // Fog
    this.scene.fog = new THREE.FogExp2(0x88ccff, 0.008);
  }

  _buildLighting() {
    // Hemisphere light (sky/ground color)
    const hemiLight = new THREE.HemisphereLight(0x88bbff, 0x445522, 0.6);
    this.scene.add(hemiLight);

    // Directional light (sun)
    const sunLight = new THREE.DirectionalLight(0xffffee, 1.2);
    sunLight.position.set(50, 80, -30);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width  = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.left   = -60;
    sunLight.shadow.camera.right  =  60;
    sunLight.shadow.camera.top    =  60;
    sunLight.shadow.camera.bottom = -60;
    sunLight.shadow.camera.near   = 1;
    sunLight.shadow.camera.far    = 200;
    sunLight.shadow.bias = -0.001;
    this.scene.add(sunLight);

    // Subtle ambient
    const ambient = new THREE.AmbientLight(0x404040, 0.3);
    this.scene.add(ambient);
  }

  _buildGround() {
    // Main ground plane
    const groundGeo = new THREE.PlaneGeometry(200, 200, 1, 1);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x3a6b35,
      roughness: 0.9,
      metalness: 0.0,
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // Grid overlay
    const gridHelper = new THREE.GridHelper(200, 100, 0x2a5525, 0x2a5525);
    gridHelper.position.y = 0.01;
    gridHelper.material.opacity = 0.2;
    gridHelper.material.transparent = true;
    this.scene.add(gridHelper);

    // Concrete pad area around the landing zone
    const padAreaGeo = new THREE.PlaneGeometry(20, 20);
    const padAreaMat = new THREE.MeshStandardMaterial({
      color: 0x888888,
      roughness: 0.85,
      metalness: 0.1,
    });
    const padArea = new THREE.Mesh(padAreaGeo, padAreaMat);
    padArea.rotation.x = -Math.PI / 2;
    padArea.position.y = 0.005;
    padArea.receiveShadow = true;
    this.scene.add(padArea);
  }

  _buildLandingPad() {
    // Landing pad with H marker
    const padGeo = new THREE.CylinderGeometry(1.5, 1.5, 0.02, 32);
    const padMat = new THREE.MeshStandardMaterial({ color: 0xcccccc, roughness: 0.7 });
    const pad = new THREE.Mesh(padGeo, padMat);
    pad.position.y = 0.01;
    pad.receiveShadow = true;
    pad.castShadow = true;
    this.scene.add(pad);

    // H letter
    const hMat = new THREE.MeshStandardMaterial({ color: 0xdd4400, roughness: 0.6 });

    // H - left vertical
    const hLeft = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.005, 0.8), hMat);
    hLeft.position.set(-0.25, 0.025, 0);
    this.scene.add(hLeft);

    // H - right vertical
    const hRight = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.005, 0.8), hMat);
    hRight.position.set(0.25, 0.025, 0);
    this.scene.add(hRight);

    // H - cross bar
    const hBar = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.005, 0.08), hMat);
    hBar.position.set(0, 0.025, 0);
    this.scene.add(hBar);

    // Circle markers
    const ringGeo = new THREE.RingGeometry(1.2, 1.4, 32);
    const ringMat = new THREE.MeshStandardMaterial({
      color: 0xdd4400, side: THREE.DoubleSide, roughness: 0.6
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    this.scene.add(ring);
  }

  _buildBuildings() {
    const buildingMats = [
      new THREE.MeshStandardMaterial({ color: 0x667788, roughness: 0.7, metalness: 0.2 }),
      new THREE.MeshStandardMaterial({ color: 0x887766, roughness: 0.8, metalness: 0.1 }),
      new THREE.MeshStandardMaterial({ color: 0x556677, roughness: 0.7, metalness: 0.3 }),
      new THREE.MeshStandardMaterial({ color: 0x778866, roughness: 0.75, metalness: 0.15 }),
    ];

    const windowMat = new THREE.MeshStandardMaterial({
      color: 0x88bbff, roughness: 0.1, metalness: 0.8, emissive: 0x112244, emissiveIntensity: 0.2
    });

    // Building specifications: [x, z, width, depth, height]
    const buildings = [
      [15, -20, 6, 6, 12],
      [25, -15, 4, 8, 18],
      [20, -28, 8, 5, 8],
      [-18, -25, 5, 5, 15],
      [-25, -18, 7, 4, 10],
      [-30, -30, 6, 6, 22],
      [30, 15, 5, 5, 14],
      [22, 25, 8, 4, 9],
      [-20, 20, 4, 7, 16],
      [-28, 28, 6, 6, 11],
      [35, -35, 5, 5, 20],
      [-35, -10, 4, 8, 7],
      [10, 35, 6, 4, 13],
      [-15, 35, 5, 5, 17],
    ];

    buildings.forEach(([bx, bz, bw, bd, bh], idx) => {
      const mat = buildingMats[idx % buildingMats.length];
      const geo = new THREE.BoxGeometry(bw, bh, bd);
      const mesh = new THREE.Mesh(geo, mat);
      mesh.position.set(bx, bh / 2, bz);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      this.objects.push(mesh);

      // Add some window rows
      const windowGeo = new THREE.PlaneGeometry(bw * 0.85, 0.6);
      for (let wy = 2; wy < bh - 1; wy += 2.5) {
        // Front face
        const win1 = new THREE.Mesh(windowGeo, windowMat);
        win1.position.set(bx, wy, bz - bd/2 - 0.01);
        this.scene.add(win1);
        // Back face
        const win2 = new THREE.Mesh(windowGeo, windowMat);
        win2.position.set(bx, wy, bz + bd/2 + 0.01);
        win2.rotation.y = Math.PI;
        this.scene.add(win2);
      }
    });
  }

  _buildTrees() {
    const trunkMat = new THREE.MeshStandardMaterial({ color: 0x553311, roughness: 0.9 });
    const leafMats = [
      new THREE.MeshStandardMaterial({ color: 0x227722, roughness: 0.8 }),
      new THREE.MeshStandardMaterial({ color: 0x2a8830, roughness: 0.8 }),
      new THREE.MeshStandardMaterial({ color: 0x1e6620, roughness: 0.8 }),
    ];

    // Scatter trees
    const treePositions = [];
    const rng = mulberry32(42); // seeded random for deterministic layout
    for (let i = 0; i < 60; i++) {
      const x = (rng() - 0.5) * 160;
      const z = (rng() - 0.5) * 160;
      // Don't place trees too close to center (landing area)
      if (Math.abs(x) < 12 && Math.abs(z) < 12) continue;
      // Don't place on buildings (rough check)
      let onBuilding = false;
      for (const obj of this.objects) {
        const dx = Math.abs(x - obj.position.x);
        const dz = Math.abs(z - obj.position.z);
        if (dx < 6 && dz < 6) { onBuilding = true; break; }
      }
      if (onBuilding) continue;

      const height = 2 + rng() * 4;
      const leafSize = 1 + rng() * 1.5;

      // Trunk
      const trunkGeo = new THREE.CylinderGeometry(0.1, 0.15, height, 6);
      const trunk = new THREE.Mesh(trunkGeo, trunkMat);
      trunk.position.set(x, height / 2, z);
      trunk.castShadow = true;
      this.scene.add(trunk);

      // Foliage (stacked cones)
      const leafMat = leafMats[Math.floor(rng() * leafMats.length)];
      for (let j = 0; j < 3; j++) {
        const coneGeo = new THREE.ConeGeometry(leafSize * (1 - j * 0.2), leafSize * 0.8, 8);
        const cone = new THREE.Mesh(coneGeo, leafMat);
        cone.position.set(x, height + j * leafSize * 0.4, z);
        cone.castShadow = true;
        this.scene.add(cone);
      }
    }
  }

  _buildGates() {
    // Race gates — torii-like structures to fly through
    const gateMat = new THREE.MeshStandardMaterial({
      color: 0xff4400,
      roughness: 0.4,
      metalness: 0.6,
      emissive: 0xff2200,
      emissiveIntensity: 0.15,
    });

    const gatePositions = [
      { x: 8,  z: -10, rot: 0,           w: 3, h: 3 },
      { x: 20, z: -5,  rot: Math.PI/4,   w: 3, h: 3 },
      { x: 15, z: 10,  rot: Math.PI/2,   w: 4, h: 4 },
      { x: -5, z: 15,  rot: Math.PI*0.8, w: 3, h: 3 },
      { x:-15, z: 5,   rot: Math.PI,     w: 3.5, h: 3.5 },
      { x:-10, z:-12,  rot: -Math.PI/3,  w: 3, h: 2.5 },
    ];

    gatePositions.forEach(({ x, z, rot, w, h }) => {
      const group = new THREE.Group();
      group.position.set(x, h/2 + 1, z);
      group.rotation.y = rot;

      // Gate ring (torus)
      const torusGeo = new THREE.TorusGeometry(Math.min(w, h) / 2, 0.08, 8, 24);
      const torus = new THREE.Mesh(torusGeo, gateMat);
      torus.rotation.y = Math.PI / 2;
      group.add(torus);

      // Support poles
      const poleGeo = new THREE.CylinderGeometry(0.05, 0.05, h/2 + 1, 6);
      const poleMat = new THREE.MeshStandardMaterial({ color: 0x666666, roughness: 0.5 });
      const poleL = new THREE.Mesh(poleGeo, poleMat);
      poleL.position.set(0, -(h/4 + 0.5), -w/2 * 0.7);
      group.add(poleL);
      const poleR = new THREE.Mesh(poleGeo, poleMat);
      poleR.position.set(0, -(h/4 + 0.5), w/2 * 0.7);
      group.add(poleR);

      group.castShadow = true;
      this.scene.add(group);
    });
  }

  _buildPerimeter() {
    // Low walls around the area
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x666666, roughness: 0.8 });
    const wallHeight = 1;
    const extent = 80;

    const walls = [
      { x: 0,       z: -extent, w: extent*2, d: 0.5 },
      { x: 0,       z:  extent, w: extent*2, d: 0.5 },
      { x: -extent, z: 0,       w: 0.5,      d: extent*2 },
      { x:  extent, z: 0,       w: 0.5,      d: extent*2 },
    ];

    walls.forEach(({ x, z, w, d }) => {
      const geo = new THREE.BoxGeometry(w, wallHeight, d);
      const mesh = new THREE.Mesh(geo, wallMat);
      mesh.position.set(x, wallHeight/2, z);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
    });
  }

  _buildDecorations() {
    // Road/path markers from landing pad outward
    const markerMat = new THREE.MeshStandardMaterial({ color: 0xdddd44, roughness: 0.5 });
    for (let i = 3; i < 50; i += 3) {
      const marker = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.01, 1), markerMat);
      marker.position.set(0, 0.01, -i);
      this.scene.add(marker);
    }

    // Some crates/containers for obstacles
    const crateMat = new THREE.MeshStandardMaterial({ color: 0x996633, roughness: 0.8 });
    const cratePositions = [
      [5, 0.5, -5], [6, 0.5, -4], [5, 1.5, -5],
      [-7, 0.5, 8], [-6, 0.5, 8],
      [12, 0.5, 3], [12, 0.5, 4], [12, 1.5, 3.5],
    ];
    cratePositions.forEach(([cx, cy, cz]) => {
      const crate = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), crateMat);
      crate.position.set(cx, cy, cz);
      crate.castShadow = true;
      crate.receiveShadow = true;
      this.scene.add(crate);
    });
  }
}

// ── Seeded PRNG ──
function mulberry32(a) {
  return function() {
    a |= 0; a = a + 0x6D2B79F5 | 0;
    let t = Math.imul(a ^ a >>> 15, 1 | a);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}
