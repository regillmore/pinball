import "./style.css";
import * as THREE from "three";
import RAPIER from "@dimforge/rapier3d-compat";

// --- DOM / HUD ---
const app = document.querySelector<HTMLDivElement>("#app")!;
app.innerHTML = `<div id="hud">
  <div><b>Pinball Physics Sandbox</b></div>
  <div>Space: launch</div>
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
  const tilt = new THREE.Quaternion().setFromEuler(new THREE.Euler(THREE.MathUtils.degToRad(10), 0, 0));
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
      .setFriction(0.7)
      .setRestitution(0.2);
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
      .setFriction(0.6)
      .setRestitution(0.2),
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

  // --- Bumpers (fixed) ---
function addBumper(x: number, z: number, radius: number) {
  const p = tiltedPos(x, 0.35, z);

  const body = world.createRigidBody(
    RAPIER.RigidBodyDesc.fixed()
      .setTranslation(p.x, p.y, p.z)
      .setRotation(tiltR)
  );

  world.createCollider(
    RAPIER.ColliderDesc.ball(radius).setRestitution(0.95).setFriction(0.2),
    body
  );

  const mesh = addMesh(
    new THREE.Mesh(
      new THREE.SphereGeometry(radius, 24, 16),
      new THREE.MeshStandardMaterial({ metalness: 0.2, roughness: 0.4 })
    )
  );
  mesh.position.copy(p);
  mesh.quaternion.copy(tiltQ);
}

  addBumper(-2.0, 1.0, 0.45);
  addBumper(+2.0, 2.5, 0.45);

  // --- Ball (dynamic) ---
  const ballRadius = 0.25;
  const ballMesh = addMesh(new THREE.Mesh(
    new THREE.SphereGeometry(ballRadius, 28, 18),
    new THREE.MeshStandardMaterial({ metalness: 0.1, roughness: 0.35 })
  ));

  const ballBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(0, 2.2, -(fieldL * 0.5) + 1.2)
      .setCcdEnabled(true)
      .setLinearDamping(0.15)
      .setAngularDamping(0.15)
  );

  world.createCollider(
    RAPIER.ColliderDesc.ball(ballRadius)
      .setRestitution(0.45)
      .setFriction(0.6),
    ballBody
  );

  syncPairs.push({ body: ballBody, mesh: ballMesh });

  function resetBall() {
    ballBody.setTranslation({ x: 0, y: 2.2, z: -(fieldL * 0.5) + 1.2 }, true);
    ballBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    ballBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  function launchBall() {
    // impulse mostly +Z, slightly +Y to keep it from scraping immediately
    ballBody.applyImpulse({ x: 0, y: 0.3, z: 6.0 }, true);
  }

  window.addEventListener("keydown", (e) => {
    if (e.code === "Space") launchBall();
    if (e.code === "KeyR") resetBall();
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
