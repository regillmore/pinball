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

  // ---- Level dimensions (meters-ish) ----
  const fieldW = 8;
  const fieldL = 16;
  const wallH = 1.0;
  const wallT = 0.25;

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
    const collider = RAPIER.ColliderDesc.cuboid(fieldW * 0.5, 0.1, fieldL * 0.5)
      .setFriction(0.1)
      .setRestitution(0.75);
    world.createCollider(collider, body);

    const geo = new THREE.BoxGeometry(fieldW, 0.2, fieldL);
    const mat = new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.9 });
    const mesh = addMesh(new THREE.Mesh(geo, mat));
    mesh.position.set(0, 0, 0);
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
        .setRestitution(0.75),
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

  // left/right walls
  addWall(-(fieldW * 0.5 + wallT * 0.5), wallH * 0.5, 0, wallT, wallH, fieldL + wallT * 2);
  addWall(+(fieldW * 0.5 + wallT * 0.5), wallH * 0.5, 0, wallT, wallH, fieldL + wallT * 2);
  // top/bottom walls
  addWall(0, wallH * 0.5, -(fieldL * 0.5 + wallT * 0.5), fieldW, wallH, wallT);
  addWall(0, wallH * 0.5, +(fieldL * 0.5 + wallT * 0.5), fieldW, wallH, wallT);

  // --- Slopes (funnel) ---
  function addSlope(x1: number, z1: number, x2: number, z2: number) {
    const midX = (x1 + x2) / 2;
    const midZ = (z1 + z2) / 2;
    const dx = x2 - x1;
    const dz = z2 - z1;
    const length = Math.sqrt(dx * dx + dz * dz);
    const angle = Math.atan2(dx, dz);

    const p = tiltedPos(midX, wallH * 0.5, midZ);
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
      RAPIER.ColliderDesc.cuboid(wT * 0.5, wallH * 0.5, length * 0.5)
        .setFriction(0.1)
        .setRestitution(0.75),
      body
    );

    const mesh = addMesh(
      new THREE.Mesh(
        new THREE.BoxGeometry(wT, wallH, length),
        new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.8, color: 0x888888 })
      )
    );
    mesh.position.copy(p);
    mesh.quaternion.copy(totalQ);
  }

  // Flipper slopes
  addSlope(3.25, 4.5, 1.5, 6.0); // Right
  addSlope(-3.25, 4.5, -1.5, 6.0); // Left

  // Upper straight segments (inlanes)
  addSlope(3.25, 2.0, 3.25, 4.5);       // Right
  addSlope(-3.25, 2.0, -3.25, 4.5);      // Left

  // Lower slopes (below flippers, guiding to drain)
  addSlope(4.0, 5.0, 0.5, 8.0);       // Right
  addSlope(-4.0, 5.0, -0.5, 8.0);      // Left

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
  function addSlingshot(x1: number, z1: number, x2: number, z2: number, x3: number, z3: number) {
    // Calculate centroid
    const cx = (x1 + x2 + x3) / 3;
    const cz = (z1 + z2 + z3) / 3;

    const shape = new THREE.Shape();
    // Map World Z (down) to Shape Y (up) -> Y = -Z_local
    // Shape X = World X_local
    shape.moveTo(x1 - cx, -(z1 - cz));
    shape.lineTo(x2 - cx, -(z2 - cz));
    shape.lineTo(x3 - cx, -(z3 - cz));
    shape.lineTo(x1 - cx, -(z1 - cz));

    const height = 0.5;
    const geometry = new THREE.ExtrudeGeometry(shape, { depth: height, bevelEnabled: false });

    // Center geometry vertically (depth) and orient flat on floor
    // Extrude creates depth in Z (mapped to World Y by rotateX)
    geometry.translate(0, 0, -height / 2);
    geometry.rotateX(-Math.PI / 2);

    const p = tiltedPos(cx, wallH * 0.5, cz);

    const mat = new THREE.MeshStandardMaterial({
      color: 0xccff00,
      emissive: 0x222200,
      metalness: 0.2,
      roughness: 0.1
    });

    const mesh = addMesh(new THREE.Mesh(geometry, mat));
    mesh.position.copy(p);
    mesh.quaternion.copy(tiltQ);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed()
        .setTranslation(p.x, p.y, p.z)
        .setRotation(tiltR)
    );

    // Convex Hull for physics
    const vertices = new Float32Array(geometry.attributes.position.array);
    world.createCollider(
      RAPIER.ColliderDesc.convexHull(vertices)!
        .setFriction(0.1)
        .setRestitution(0.75),
      body
    );
  }

  // Right Slingshot
  addSlingshot(2.5, 4, 1.5, 5, 2.5, 2.5);
  // Left Slingshot
  addSlingshot(-2.5, 4, -1.5, 5, -2.5, 2.5);

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
    const length = 1.25;
    const thickness = 0.4;

    // Distance between circle centers to achieve total visual length
    // Total len = rBase + dist + rTip
    const dist = length - rBase - rTip;

    const direction = isLeft ? 1 : -1;
    const pivot = new THREE.Vector3(isLeft ? -1.25 : 1.25, 0.3, 6.25);
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
        .setLinearDamping(0.6)
        .setAngularDamping(2.0)
        .setCcdEnabled(true)
        .setCanSleep(false)
    );

    // Collider from geometry vertices
    const vertices = new Float32Array(geometry.attributes.position.array);
    world.createCollider(
      RAPIER.ColliderDesc.convexHull(vertices)!
        .setFriction(0.1)
        .setRestitution(0.75),
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

    const restAngle = direction * -0.85;
    const fireAngle = direction * 0.2;
    const min = Math.min(restAngle, fireAngle) - 0.05;
    const max = Math.max(restAngle, fireAngle) + 0.05;
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
      .setTranslation(1.7, 2.2, -(fieldL * 0.5) + 1.2)
      .setCcdEnabled(true)
      .setLinearDamping(0.1)
      .setAngularDamping(0.1)
  );

  world.createCollider(
    RAPIER.ColliderDesc.ball(ballRadius)
      .setRestitution(0.75)
      .setFriction(0.01),
    ballBody
  );

  syncPairs.push({ body: ballBody, mesh: ballMesh });

  function resetBall() {
    ballBody.setTranslation({ x: 1.7, y: 2.2, z: -(fieldL * 0.5) + 1.2 }, true);
    ballBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    ballBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  function launchBall() {
    // impulse mostly +Z, slightly +Y to keep it from scraping immediately
    ballBody.applyImpulse({ x: 0, y: 0.1, z: -1.0 }, true);
  }

  window.addEventListener("keydown", (e) => {
    if (e.code === "Space") launchBall();
    if (e.code === "KeyR") resetBall();
    if (e.code === "ArrowLeft" || e.code === "KeyA") {
      for (const f of flippers) {
        if (f.restAngle < 0) f.joint.configureMotorPosition(f.fireAngle, 1250, 10);
      }
    }
    if (e.code === "ArrowRight" || e.code === "KeyD") {
      for (const f of flippers) {
        if (f.restAngle > 0) f.joint.configureMotorPosition(f.fireAngle, 1250, 10);
      }
    }
  });

  window.addEventListener("keyup", (e) => {
    if (e.code === "ArrowLeft" || e.code === "KeyA") {
      for (const f of flippers) {
        if (f.restAngle < 0) f.joint.configureMotorPosition(f.restAngle, 1250, 8);
      }
    }
    if (e.code === "ArrowRight" || e.code === "KeyD") {
      for (const f of flippers) {
        if (f.restAngle > 0) f.joint.configureMotorPosition(f.restAngle, 1250, 8);
      }
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
      world.step();
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
