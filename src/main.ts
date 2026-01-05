import "./style.css";
import * as THREE from "three";
import RAPIER from "@dimforge/rapier3d-compat";

// --- DOM / HUD ---
const app = document.querySelector<HTMLDivElement>("#app")!;
app.innerHTML = `<div id="hud">
  <div><b>Pinball Physics Sandbox</b></div>
  <div>Space: launch</div>
  <div>←/→: flippers</div>
  <div>R: reset</div>
</div>`;

// --- Three.js setup ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 200);
camera.position.set(0, 10, 16);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
app.appendChild(renderer.domElement);

scene.add(new THREE.AmbientLight(0xffffff, 0.35));
const dir = new THREE.DirectionalLight(0xffffff, 0.9);
dir.position.set(8, 14, 6);
scene.add(dir);

// --- Helpers ---
type SyncPair = { body: RAPIER.RigidBody; mesh: THREE.Object3D };
const syncPairs: SyncPair[] = [];

function rapierQuatFromThree(q: THREE.Quaternion): { x: number; y: number; z: number; w: number } {
  return { x: q.x, y: q.y, z: q.z, w: q.w };
}

function syncMeshFromBody(pair: SyncPair) {
  const t = pair.body.translation();
  const r = pair.body.rotation();
  pair.mesh.position.set(t.x, t.y, t.z);
  pair.mesh.quaternion.set(r.x, r.y, r.z, r.w);
}

function addMesh(mesh: THREE.Object3D) {
  scene.add(mesh);
  return mesh;
}

// --- Main ---
async function main() {
  // Rapier compat package embeds WASM and needs init(). :contentReference[oaicite:5]{index=5}
  await RAPIER.init();

  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  const eventQueue = new RAPIER.EventQueue(true);

  // ---- Level dimensions (meters-ish) ----
  const fieldW = 8;
  const fieldL = 16;
  const wallH = 1.0;
  const slopeH = wallH - 0.05;
  const slingshotH = slopeH - 0.05;
  const wallT = 0.25;
  const plungerW = 0.75;

  // Tilt the playfield slightly so the ball rolls “down” +Z
  const tilt = new THREE.Quaternion().setFromEuler(new THREE.Euler(THREE.MathUtils.degToRad(6.5), 0, 0));
  const tiltQ = tilt;
  const tiltR = rapierQuatFromThree(tiltQ);

  function tiltedPos(x: number, y: number, z: number) {
    return new THREE.Vector3(x, y, z).applyQuaternion(tiltQ);
  }

  // --- Playfield (fixed) ---
  {
    const rbDesc = RAPIER.RigidBodyDesc.fixed()
      .setTranslation(0, 0, 0)
      .setRotation(rapierQuatFromThree(tilt));
    const body = world.createRigidBody(rbDesc);

    // Use a thin cuboid as the playfield surface
    // Total width = main field + inner wall + plunger channel
    const totalW = fieldW + wallT + plungerW;
    // Shift center to accommodate the extra width on the right (+X)
    const offsetX = (wallT + plungerW) / 2;

    const collider = RAPIER.ColliderDesc.cuboid(totalW * 0.5, 0.1, fieldL * 0.5)
      .setTranslation(offsetX, 0, 0)
      .setFriction(0.1)
      .setRestitution(0.25);
    world.createCollider(collider, body);

    const geo = new THREE.BoxGeometry(totalW, 0.2, fieldL);
    const mat = new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.9 });
    const mesh = addMesh(new THREE.Mesh(geo, mat));
    mesh.position.set(offsetX, 0, 0);
    mesh.quaternion.copy(tilt);
  }

  // --- Walls (fixed) ---
  function addWall(x: number, y: number, z: number, sx: number, sy: number, sz: number) {
    const p = tiltedPos(x, y, z);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(tiltR)
    );

    world.createCollider(
      RAPIER.ColliderDesc.cuboid(sx * 0.5, sy * 0.5, sz * 0.5)
        .setFriction(0.1)
        .setRestitution(0.25),
      body
    );

    const mesh = addMesh(
      new THREE.Mesh(
        new THREE.BoxGeometry(sx, sy, sz),
        new THREE.MeshStandardMaterial({ metalness: 0.0, roughness: 0.95 })
      )
    );
    mesh.position.copy(p);
    mesh.quaternion.copy(tiltQ);
  }

  // left wall (unchanged)
  addWall(-(fieldW * 0.5 + wallT * 0.5), wallH * 0.5, 0, wallT, wallH, fieldL + wallT * 2);

  // Inner right wall (with gap at top)
  const gap = 1.5; // Gap at the top (-Z end)
  const fullWallL = fieldL + wallT * 2;
  const innerRightL = fullWallL - gap;
  const innerRightZ = gap / 2; // Shifted "down" (+Z) by half the gap
  addWall(+(fieldW * 0.5 + wallT * 0.5), wallH * 0.5, innerRightZ, wallT, wallH, innerRightL);

  // Outer right wall (new)
  const outerRightX = (fieldW * 0.5) + wallT + plungerW + (wallT * 0.5);
  addWall(outerRightX, wallH * 0.5, 0, wallT, wallH, fieldL + wallT * 2);

  // top/bottom walls (extended)
  const tbWidth = fieldW + wallT + plungerW;
  const tbOffset = (wallT + plungerW) / 2;
  addWall(tbOffset, wallH * 0.5, -(fieldL * 0.5 + wallT * 0.5), tbWidth, wallH, wallT);
  addWall(tbOffset, wallH * 0.5, +(fieldL * 0.5 + wallT * 0.5), tbWidth, wallH, wallT);

  // Plunger Channel Arc (redirects launched ball into playfield)
  // Replaces the top-right corner with a curved wall
  function addFixedCurve(cx: number, cz: number, radius: number, startAngle: number, endAngle: number) {
    const shape = new THREE.Shape();
    // absarc(x, y, radius, startAngle, endAngle, clockwise)
    // Note: shape X = world X, shape Y = world -Z.
    // Angles: 0 is +X, PI/2 is +Y (World -Z)
    shape.absarc(0, 0, radius, startAngle, endAngle, false);

    // Create inner thickness for the wall
    const wT = 0.25;
    const innerRadius = radius - wT;

    const shape2 = new THREE.Shape();
    shape2.absarc(0, 0, radius, startAngle, endAngle, false);
    const endX = Math.cos(endAngle);
    const endY = Math.sin(endAngle);
    // Line to inner end
    shape2.lineTo(innerRadius * endX, innerRadius * endY);
    // Inner arc backwards
    shape2.absarc(0, 0, innerRadius, endAngle, startAngle, true);
    shape2.closePath();

    const geometry = new THREE.ExtrudeGeometry(shape2, { depth: slopeH, bevelEnabled: false });

    // Rotate and position
    // Center geometry vertically (depth is Z -> mapped to World Y)
    geometry.translate(0, 0, -slopeH / 2);
    geometry.rotateX(-Math.PI / 2);

    const p = tiltedPos(cx, slopeH * 0.5, cz);

    const mesh = addMesh(
      new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.8, color: 0x888888 })
      )
    );
    mesh.position.copy(p);
    mesh.quaternion.copy(tiltQ);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(tiltR)
    );

    // Trimesh for generic static shape
    const vertices = new Float32Array(geometry.attributes.position.array);
    const indices = new Uint32Array(geometry.index ? geometry.index.array : []);

    let finalIndices = indices;
    if (!geometry.index) {
      const count = vertices.length / 3;
      finalIndices = new Uint32Array(count);
      for (let i = 0; i < count; i++) finalIndices[i] = i;
    }

    world.createCollider(
      RAPIER.ColliderDesc.trimesh(vertices, finalIndices)
        .setFriction(0.1)
        .setRestitution(0.25),
      body
    );
  }

  // Right Corner Arc
  addFixedCurve(3.75, -6.75, 1.5, 0, Math.PI / 2);
  // Left Corner Arc
  addFixedCurve(-2.75, -6.75, 1.5, Math.PI / 2, Math.PI);
  // Ball Lock Cap Arc
  addFixedCurve(4.55, -2.75, 8, Math.PI, -7 * Math.PI / 8);
  // Ball Lock Arc
  addFixedCurve(4.75, -2.12, 9, Math.PI, -27 * Math.PI / 32);
  // Left Channel Cap Arc
  addFixedCurve(-2.25, 3.75, 2, 5 * Math.PI / 8, Math.PI);
  // Right Channel Cap Arc
  addFixedCurve(2.25, 3.75, 2, 0, 3 * Math.PI / 8);
  // Right Mid Arc
  addFixedCurve(-9.75, -4.75, 14, -4 * Math.PI / 32, 0);
  // Left Channel Arc
  //addFixedCurve(-0.25, 3.4, 4, Math.PI, -3 * Math.PI / 4);
  // Right Channel Arc
  //addFixedCurve(0.25, 3.4, 4, -Math.PI / 4, 0);
  // Left Flipper Slope Arc
  //addFixedCurve(-1.67, 4, 1.7, Math.PI, -3 * Math.PI / 4);
  // Right Flipper Slope Arc
  //addFixedCurve(1.67, 4, 1.7, -Math.PI / 4, 0);
  // Left Slingshot Arc
  //addFixedCurve(-2.02, 4, 0.5, Math.PI, -3 * Math.PI / 4);
  // Right Slingshot Arc
  //addFixedCurve(2.02, 4, 0.5, -Math.PI / 4, 0);

  // --- Slopes (funnel) ---
  function addSlope(x1: number, z1: number, x2: number, z2: number) {
    const midX = (x1 + x2) / 2;
    const midZ = (z1 + z2) / 2;
    const dx = x2 - x1;
    const dz = z2 - z1;
    const length = Math.sqrt(dx * dx + dz * dz);
    const angle = Math.atan2(dx, dz);

    const p = tiltedPos(midX, slopeH * 0.5, midZ);
    const wallRot = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, angle, 0));
    // Combine tilt with wall rotation
    const totalQ = tiltQ.clone().multiply(wallRot);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(rapierQuatFromThree(totalQ))
    );

    const wT = 0.25;
    world.createCollider(
      RAPIER.ColliderDesc.cuboid(wT * 0.5, slopeH * 0.5, length * 0.5)
        .setFriction(0.1)
        .setRestitution(0.25),
      body
    );

    const mesh = addMesh(
      new THREE.Mesh(
        new THREE.BoxGeometry(wT, slopeH, length),
        new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.8, color: 0x888888 })
      )
    );
    mesh.position.copy(p);
    mesh.quaternion.copy(totalQ);
  }

  // Flipper slopes
  addSlope(3.4, 5.1, 1.5, 6.25); // Right
  addSlope(-3.4, 5.1, -1.5, 6.25); // Left

  // Flipper upper vert
  addSlope(3.35, 3.5, 3.35, 5.2);       // Right
  addSlope(-3.35, 3.5, -3.35, 5.2);      // Left

  // Slingshot slopes
  addSlope(2.65, 4.5, 1.65, 5.1); // Right
  addSlope(-2.65, 4.5, -1.65, 5.1); // Left

  // Slingshot upper vert
  addSlope(2.6, 3.0, 2.6, 4.6);       // Right
  addSlope(-2.6, 3.0, -2.6, 4.6);      // Left

  // Right mid vert
  addSlope(3.05, 0.53, 3.05, 2.1);       // Right

  // Lower slopes (below flippers, guiding to drain)
  addSlope(4.15, 5.65, 1.5, 7.25);       // Right
  addSlope(-4.15, 5.65, -1.5, 7.25);      // Left

  // --- Bumpers (fixed) ---
  // function addBumper(x: number, z: number, radius: number) {
  //   const p = tiltedPos(x, 0.35, z);

  //   const body = world.createRigidBody(
  //     RAPIER.RigidBodyDesc.fixed()
  //       .setTranslation(p.x, p.y, p.z)
  //       .setRotation(tiltR)
  //   );

  //   world.createCollider(
  //     RAPIER.ColliderDesc.ball(radius).setRestitution(0.95).setFriction(0.2),
  //     body
  //   );

  //   const mesh = addMesh(
  //     new THREE.Mesh(
  //       new THREE.SphereGeometry(radius, 24, 16),
  //       new THREE.MeshStandardMaterial({ metalness: 0.2, roughness: 0.4 })
  //     )
  //   );
  //   mesh.position.copy(p);
  //   mesh.quaternion.copy(tiltQ);
  // }

  //addBumper(-2.0, 1.0, 0.45);
  //addBumper(+2.0, 2.5, 0.45);

  // --- Slingshots (Kickers) ---
  const kickers = new Set<number>(); // Store collider handles
  const kickerMeshes = new Map<number, THREE.Mesh>(); // Store mesh for each collider handle

  function addSlingshot(x1: number, z1: number, x2: number, z2: number) {
    const midX = (x1 + x2) / 2;
    const midZ = (z1 + z2) / 2;
    const dx = x2 - x1;
    const dz = z2 - z1;
    const length = Math.sqrt(dx * dx + dz * dz);
    const angle = Math.atan2(dx, dz);

    const p = tiltedPos(midX, slingshotH * 0.5, midZ);
    const wallRot = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, angle, 0));
    const totalQ = tiltQ.clone().multiply(wallRot);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(rapierQuatFromThree(totalQ))
    );

    const wT = 0.25;
    const collider = world.createCollider(
      RAPIER.ColliderDesc.cuboid(wT * 0.5, slingshotH * 0.5, length * 0.5)
        .setFriction(0.1)
        .setRestitution(0.5)
        .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
      body
    );

    kickers.add(collider.handle);

    const mesh = addMesh(
      new THREE.Mesh(
        new THREE.BoxGeometry(wT, slingshotH, length),
        new THREE.MeshStandardMaterial({
          color: 0xccff00,
          emissive: 0x444400,
          metalness: 0.2,
          roughness: 0.1
        })
      )
    );
    mesh.position.copy(p);
    mesh.quaternion.copy(totalQ);

    kickerMeshes.set(collider.handle, mesh as THREE.Mesh);
  }

  // Right Slingshot
  addSlingshot(2.6, 3.0, 1.75, 5);
  // Left Slingshot
  addSlingshot(-2.6, 3.0, -1.75, 5);


  // --- Flippers (dynamic, jointed) ---
  type MotorizedJoint = RAPIER.ImpulseJoint & {
    configureMotorPosition(targetPos: number, stiffness: number, damping: number): void;
    setLimits(min: number, max: number): void;
  };
  type Flipper = {
    body: RAPIER.RigidBody;
    joint: MotorizedJoint;
    restAngle: number;
    fireAngle: number;
  };
  const flippers: Flipper[] = [];

  function createFlipperProfile(r1: number, r2: number, dist: number) {
    const shape = new THREE.Shape();
    const d = Math.abs(dist);
    const cosTheta = (r1 - r2) / d;
    const theta = Math.acos(Math.min(Math.max(cosTheta, -1), 1)); // Clamp for safety

    // Start at top of base circle (C1)
    // CCW order
    shape.absarc(0, 0, r1, theta, -theta, false); // Back of base
    shape.lineTo(d + r2 * Math.cos(-theta), r2 * Math.sin(-theta)); // To bottom of tip
    shape.absarc(d, 0, r2, -theta, theta, false); // Tip cap
    shape.lineTo(r1 * Math.cos(theta), r1 * Math.sin(theta)); // Back to top of base

    return shape;
  }

  function addFlipper(isLeft: boolean) {
    const rBase = 0.15;
    const rTip = 0.05;
    const length = 1.2;
    const thickness = 0.4;

    // Distance between circle centers to achieve total visual length
    // Total len = rBase + dist + rTip
    const dist = length - rBase - rTip;

    const direction = isLeft ? 1 : -1;
    const pivot = new THREE.Vector3(isLeft ? -1.35 : 1.35, 0.3, 6.37);
    const pivotPos = tiltedPos(pivot.x, pivot.y, pivot.z);

    const yaw = THREE.MathUtils.degToRad(isLeft ? 20 : -20);
    const flipperRot = tiltQ.clone().multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yaw, 0)));
    const flipperR = rapierQuatFromThree(flipperRot);

    // Create Geometry
    const shape = createFlipperProfile(rBase, rTip, dist);
    const geometry = new THREE.ExtrudeGeometry(shape, { depth: thickness, bevelEnabled: false });

    // Center in thickness (Z -> Y after rotation) 
    // and orient flat
    geometry.translate(0, 0, -thickness * 0.5);
    geometry.rotateX(-Math.PI * 0.5);

    // For right flipper, rotate 180 degrees so it points along -X
    if (!isLeft) {
      geometry.rotateY(Math.PI);
    }

    // Mesh
    const mesh = addMesh(
      new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({ color: isLeft ? 0xff5555 : 0x55aaff, roughness: 0.35 })
      )
    );
    // Mesh is child of body, so init locally to 0,0,0
    mesh.position.copy(pivotPos);
    mesh.quaternion.copy(flipperRot);

    // Physics
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(pivotPos.x, pivotPos.y, pivotPos.z)
        .setRotation(flipperR)
        //.setLinearDamping(0.6)
        //.setAngularDamping(2.0)
        .setCcdEnabled(true)
        .setCanSleep(false)
    );

    // Collider from geometry vertices
    const vertices = new Float32Array(geometry.attributes.position.array);
    world.createCollider(
      RAPIER.ColliderDesc.convexHull(vertices)!
        .setFriction(0.1)
        .setDensity(50.0)
        .setRestitution(0.25),
      body
    );

    const pivotBody = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(pivotPos.x, pivotPos.y, pivotPos.z)
        .setRotation(flipperR)
    );

    // Joint at origin of both bodies (the pivot point)
    const jointData = RAPIER.JointData.revolute(
      { x: 0, y: 0, z: 0 },
      { x: 0, y: 0, z: 0 },
      { x: 0, y: 1, z: 0 }
    );
    const joint = world.createImpulseJoint(jointData, pivotBody, body, true);
    const motorJoint = joint as MotorizedJoint;

    const restAngle = direction * -0.75;
    const fireAngle = direction * 0.2;
    const min = Math.min(restAngle, fireAngle) - 0.02;
    const max = Math.max(restAngle, fireAngle) + 0.02;
    motorJoint.setLimits(min, max);
    motorJoint.configureMotorPosition(restAngle, 90, 6);

    flippers.push({ body, joint: motorJoint, restAngle, fireAngle });
    syncPairs.push({ body, mesh });
  }

  addFlipper(true);
  addFlipper(false);

  // --- Ball (dynamic) ---
  const ballRadius = 0.25;
  const ballMesh = addMesh(new THREE.Mesh(
    new THREE.SphereGeometry(ballRadius, 28, 18),
    new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.35 })
  ));

  const ballBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation((fieldW * 0.5) + wallT + (plungerW * 0.5), 1.0, (fieldL * 0.5) - 0.5)
      .setCcdEnabled(true)
    //.setLinearDamping(0.1)
    //.setAngularDamping(0.1)
  );

  const ballCollider = world.createCollider(
    RAPIER.ColliderDesc.ball(ballRadius)
      .setFriction(0.1)
      .setRestitution(0.25)
      .setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS),
    ballBody
  );

  syncPairs.push({ body: ballBody, mesh: ballMesh });

  const plungerX = (fieldW * 0.5) + wallT + (plungerW * 0.5);
  const plungerStartZ = (fieldL * 0.5) - 0.5;

  function resetBall() {
    ballBody.setTranslation({ x: plungerX, y: 1.0, z: plungerStartZ }, true);
    ballBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    ballBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  function launchBall() {
    // impulse mostly -Z to launch up the plunger lane
    ballBody.applyImpulse({ x: 0, y: 0.0, z: -1.0 }, true);
  }

  window.addEventListener("keydown", (e) => {
    if (e.code === "Space") launchBall();
    if (e.code === "KeyR") resetBall();
    if (e.code === "ArrowLeft" || e.code === "KeyA") {
      for (const f of flippers) {
        if (f.restAngle < 0) f.joint.configureMotorPosition(f.fireAngle, 1250, 50);
      }
    }
    if (e.code === "ArrowRight" || e.code === "KeyD") {
      for (const f of flippers) {
        if (f.restAngle > 0) f.joint.configureMotorPosition(f.fireAngle, 1250, 50);
      }
    }
  });

  window.addEventListener("keyup", (e) => {
    if (e.code === "ArrowLeft" || e.code === "KeyA") {
      for (const f of flippers) {
        if (f.restAngle < 0) f.joint.configureMotorPosition(f.restAngle, 1250, 50);
      }
    }
    if (e.code === "ArrowRight" || e.code === "KeyD") {
      for (const f of flippers) {
        if (f.restAngle > 0) f.joint.configureMotorPosition(f.restAngle, 1250, 50);
      }
    }
  });

  // --- Mouse Drag ---
  const raycaster = new THREE.Raycaster();
  const mouse = new THREE.Vector2();
  const dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -1.0); // Plane at y=1.0
  let isDragging = false;

  window.addEventListener("pointerdown", (e) => {
    mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    const intersects = raycaster.intersectObject(ballMesh);
    if (intersects.length > 0) {
      isDragging = true;
      document.body.style.cursor = "grabbing";
      // steady the ball
      ballBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
      ballBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
  });

  window.addEventListener("pointermove", (e) => {
    if (!isDragging) return;

    mouse.x = (e.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    const target = new THREE.Vector3();
    if (raycaster.ray.intersectPlane(dragPlane, target)) {
      ballBody.setTranslation({ x: target.x, y: 1.0, z: target.z }, true);
      ballBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
      ballBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
  });

  window.addEventListener("pointerup", () => {
    if (isDragging) {
      isDragging = false;
      document.body.style.cursor = "auto";
    }
  });

  // --- Fixed timestep loop ---
  const fixedDt = 1 / 60;
  let last = performance.now() / 1000;
  let acc = 0;

  function frame() {
    const now = performance.now() / 1000;
    let dt = now - last;
    last = now;

    // avoid “spiral of death” if tab was hidden
    dt = Math.min(dt, 0.25);
    acc += dt;

    while (acc >= fixedDt) {
      world.step(eventQueue);

      eventQueue.drainCollisionEvents((handle1, handle2, started) => {
        if (!started) return;

        let otherHandle: number | undefined;

        if (handle1 === ballCollider.handle) otherHandle = handle2;
        else if (handle2 === ballCollider.handle) otherHandle = handle1;

        if (otherHandle !== undefined && kickers.has(otherHandle)) {
          const kickerBody = world.getCollider(otherHandle).parent();
          if (kickerBody) {
            const ballPos = ballBody.translation();
            const kickerPos = kickerBody.translation();

            const rot = kickerBody.rotation();
            const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);

            // Reference normal (local +X)
            const normal = new THREE.Vector3(1, 0, 0).applyQuaternion(q);

            const dirToBall = new THREE.Vector3().subVectors(
              new THREE.Vector3(ballPos.x, ballPos.y, ballPos.z),
              new THREE.Vector3(kickerPos.x, kickerPos.y, kickerPos.z)
            );

            // Flip normal if needed
            if (normal.dot(dirToBall) < 0) {
              normal.negate();
            }

            // Only kick if the ball hits with sufficient force
            const vel = ballBody.linvel();
            const impactSpeed = normal.x * vel.x + normal.y * vel.y + normal.z * vel.z;
            if (impactSpeed < 0.25) return;

            const strength = 0.35;
            ballBody.applyImpulse({
              x: normal.x * strength,
              y: normal.y * strength,
              z: normal.z * strength
            }, true);

            // Light up effect
            const mesh = kickerMeshes.get(otherHandle);
            if (mesh) {
              const mat = mesh.material as THREE.MeshStandardMaterial;
              const originalEmissive = mat.emissive.getHex();
              mat.emissive.setHex(0xffff00); // Flash yellow
              setTimeout(() => {
                mat.emissive.setHex(originalEmissive);
              }, 100);
            }
          }
        }
      });
      acc -= fixedDt;
    }

    for (const p of syncPairs) syncMeshFromBody(p);

    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  frame();

  // --- Resize ---
  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });
}

main().catch((err) => {
  console.error(err);
  alert(String(err));
});
