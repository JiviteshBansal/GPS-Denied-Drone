const engine = new SimEngine();

const elStatusTag = document.getElementById('system-status-tag');
const elBtnExplore = document.getElementById('btn-start-explore');
const elBtnExploreText = document.getElementById('btn-explore-text');
const elFsmBadge = document.getElementById('fsm-state-badge');
const elTimer = document.getElementById('sim-timer');
const elExploredPct = document.getElementById('val-explored-pct');
const elExploredSqm = document.getElementById('val-explored-sqm');
const elExploredFill = document.getElementById('bar-explored-fill');
const elFrontierCount = document.getElementById('val-frontier-count');
const elClusterCount = document.getElementById('val-cluster-count');
const elTargetCoords = document.getElementById('val-target-coords');
const elTargetScore = document.getElementById('val-target-score');
const elFlightDist = document.getElementById('val-flight-dist');
const elWaypointsVisited = document.getElementById('val-waypoints-visited');
const elScoringList = document.getElementById('scoring-list');
const elImuPos = document.getElementById('imu-pos');
const elImuAtt = document.getElementById('imu-att');
const elBattery = document.getElementById('drone-battery');
const elHudAlt = document.getElementById('hud-alt');
const elHudSpd = document.getElementById('hud-spd');
const elHudHdg = document.getElementById('hud-hdg');

const occCanvas = document.getElementById('occupancy-canvas');
const occCtx = occCanvas.getContext('2d');
const overlayCanvas = document.getElementById('overlay-canvas');
const overlayCtx = overlayCanvas.getContext('2d');
const depthCanvas = document.getElementById('depth-stream-canvas');
const depthCtx = depthCanvas.getContext('2d');

let cameraFollow = true;
let currentSlideIndex = 0;
const totalSlides = 7;
let lastTimestamp = performance.now();
let simTimeElapsed = 0;

const container3D = document.getElementById('three-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf1f5f9);
scene.fog = new THREE.FogExp2(0xf1f5f9, 0.015);

const camera = new THREE.PerspectiveCamera(45, container3D.clientWidth / container3D.clientHeight, 0.1, 100);
camera.position.set(0, -18, 22);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container3D.clientWidth, container3D.clientHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container3D.appendChild(renderer.domElement);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.9);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
dirLight.position.set(15, -20, 25);
dirLight.castShadow = true;
dirLight.shadow.mapSize.width = 1024;
dirLight.shadow.mapSize.height = 1024;
scene.add(dirLight);

const gridHelper = new THREE.GridHelper(40, 40, 0x60a5fa, 0xcbd5e1);
gridHelper.rotation.x = Math.PI / 2;
scene.add(gridHelper);

const droneGroup = new THREE.Group();

const bodyGeo = new THREE.BoxGeometry(0.6, 0.6, 0.15);
const bodyMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3, metalness: 0.5 });
const bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
droneGroup.add(bodyMesh);

const armMat = new THREE.MeshStandardMaterial({ color: 0x475569 });
const arm1 = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 1.2), armMat);
arm1.rotation.z = Math.PI / 4;
arm1.rotation.x = Math.PI / 2;
const arm2 = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 1.2), armMat);
arm2.rotation.z = -Math.PI / 4;
arm2.rotation.x = Math.PI / 2;
droneGroup.add(arm1, arm2);

const rotors = [];
const rotorGeo = new THREE.BoxGeometry(0.45, 0.04, 0.01);
const rotorMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.2 });
const rotorOffsets = [
  { x: 0.42, y: 0.42 },
  { x: -0.42, y: 0.42 },
  { x: 0.42, y: -0.42 },
  { x: -0.42, y: -0.42 }
];

rotorOffsets.forEach(pos => {
  const r = new THREE.Mesh(rotorGeo, rotorMat);
  r.position.set(pos.x, pos.y, 0.1);
  droneGroup.add(r);
  rotors.push(r);
});

const frustumGeo = new THREE.ConeGeometry(3.5, 7.0, 4, 1, true);
frustumGeo.rotateX(-Math.PI / 2);
frustumGeo.translate(0, 3.5, 0);
const frustumMat = new THREE.MeshBasicMaterial({
  color: 0x0284c7,
  wireframe: true,
  transparent: true,
  opacity: 0.3
});
const frustumMesh = new THREE.Mesh(frustumGeo, frustumMat);
droneGroup.add(frustumMesh);

scene.add(droneGroup);

const obstaclesGroup = new THREE.Group();
scene.add(obstaclesGroup);

function build3DWorld() {
  while (obstaclesGroup.children.length > 0) {
    obstaclesGroup.remove(obstaclesGroup.children[0]);
  }

  const wallMat = new THREE.MeshStandardMaterial({
    color: 0xdc2626,
    roughness: 0.45,
    metalness: 0.15
  });

  engine.obstacles.forEach(obs => {
    const width = obs.maxX - obs.minX;
    const depth = obs.maxY - obs.minY;
    const height = 3.5;
    const geo = new THREE.BoxGeometry(width, depth, height);
    const mesh = new THREE.Mesh(geo, wallMat);
    mesh.position.set((obs.minX + obs.maxX) / 2, (obs.minY + obs.maxY) / 2, height / 2);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    obstaclesGroup.add(mesh);
  });
}
build3DWorld();

const maxTrailPoints = 500;
const trailGeo = new THREE.BufferGeometry();
const trailPositions = new Float32Array(maxTrailPoints * 3);
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
const trailMat = new THREE.LineBasicMaterial({ color: 0x2563eb, linewidth: 3, transparent: true, opacity: 0.85 });
const trailLine = new THREE.Line(trailGeo, trailMat);
scene.add(trailLine);

function switchViewMode(mode) {
  document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
  document.getElementById(`tab-${mode}`).classList.add('active');

  document.getElementById('view-simulation').classList.add('hidden');
  document.getElementById('view-presentation').classList.add('hidden');
  document.getElementById('view-architecture').classList.add('hidden');

  if (mode === 'simulation') {
    document.getElementById('view-simulation').classList.remove('hidden');
    onWindowResize();
  } else if (mode === 'presentation') {
    document.getElementById('view-presentation').classList.remove('hidden');
  } else if (mode === 'architecture') {
    document.getElementById('view-architecture').classList.remove('hidden');
    showCodeSnippet('mapping');
  }
}

function setSimSpeed(spd) {
  engine.simSpeed = spd;
  document.querySelectorAll('.speed-btn').forEach(btn => btn.classList.remove('active'));
  event.target.classList.add('active');
}

function toggleExploration() {
  engine.isExploring = !engine.isExploring;
  if (engine.isExploring) {
    elBtnExplore.classList.add('active-running');
    elBtnExploreText.innerText = 'Pause Exploration';
    if (elStatusTag) {
      elStatusTag.innerText = 'AUTONOMOUS ACTIVE';
      elStatusTag.style.color = '#16a34a';
    }
  } else {
    elBtnExplore.classList.remove('active-running');
    elBtnExploreText.innerText = 'Start Autonomous Exploration';
    if (elStatusTag) {
      elStatusTag.innerText = 'AUTONOMOUS PAUSED';
      elStatusTag.style.color = '#d97706';
    }
  }
}

function resetSimulation() {
  engine.resetMap();
  simTimeElapsed = 0;
  elBtnExplore.classList.remove('active-running');
  elBtnExploreText.innerText = 'Start Autonomous Exploration';
  if (elStatusTag) {
    elStatusTag.innerText = 'SYSTEM READY';
  }
  build3DWorld();
}

function changeWorld(type) {
  engine.loadWorld(type);
  build3DWorld();
  resetSimulation();
}

function toggleCameraFollow() {
  cameraFollow = !cameraFollow;
  document.getElementById('btn-cam-follow').innerText = cameraFollow ? 'Follow Drone: ON' : 'Follow Drone: OFF';
}

function triggerTakeoff() {
  engine.fsmState = 'TAKEOFF';
  engine.isExploring = true;
  elBtnExplore.classList.add('active-running');
  elBtnExploreText.innerText = 'Pause Exploration';
}

function triggerHover() {
  engine.fsmState = 'HOVER';
  engine.isExploring = false;
  engine.drone.vx = 0;
  engine.drone.vy = 0;
  elBtnExplore.classList.remove('active-running');
}

function triggerLand() {
  engine.fsmState = 'LAND';
  engine.isExploring = false;
  elBtnExplore.classList.remove('active-running');
}

function triggerEmergencyStop() {
  engine.isExploring = false;
  engine.drone.vx = 0;
  engine.drone.vy = 0;
  engine.drone.z = 0;
  engine.fsmState = 'IDLE';
  elBtnExplore.classList.remove('active-running');
}

function updateSlideView() {
  document.querySelectorAll('.presentation-slide').forEach(s => s.classList.remove('active'));
  document.querySelectorAll('.thumb-btn').forEach(b => b.classList.remove('active'));

  const activeSlide = document.getElementById(`slide-${currentSlideIndex}`);
  if (activeSlide) activeSlide.classList.add('active');

  const thumbs = document.querySelectorAll('.thumb-btn');
  if (thumbs[currentSlideIndex]) thumbs[currentSlideIndex].classList.add('active');

  document.getElementById('slide-number-indicator').innerText = `Slide ${currentSlideIndex + 1} of ${totalSlides}`;
}

function nextSlide() {
  if (currentSlideIndex < totalSlides - 1) {
    currentSlideIndex++;
    updateSlideView();
  }
}

function prevSlide() {
  if (currentSlideIndex > 0) {
    currentSlideIndex--;
    updateSlideView();
  }
}

function goToSlide(idx) {
  currentSlideIndex = idx;
  updateSlideView();
}

window.addEventListener('keydown', (e) => {
  if (!document.getElementById('view-presentation').classList.contains('hidden')) {
    if (e.key === 'ArrowRight' || e.key === 'Space') nextSlide();
    if (e.key === 'ArrowLeft') prevSlide();
  }
});

const codeSnippets = {
  mapping: {
    name: 'src/mapping/src/occupancy_map.cpp',
    code: `void OccupancyMap::updateFromDepthRay(float origin_x, float origin_y, float hit_x, float hit_y, bool hit_obstacle) {
    int x0 = worldToGridX(origin_x);
    int y0 = worldToGridY(origin_y);
    int x1 = worldToGridX(hit_x);
    int y1 = worldToGridY(hit_y);

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    int cx = x0, cy = y0;
    while (true) {
        bool is_endpoint = (cx == x1 && cy == y1);
        if (isInBounds(cx, cy)) {
            if (is_endpoint && hit_obstacle) {
                setCell(cx, cy, OCCUPIED_100);
            } else {
                if (getCell(cx, cy) != OCCUPIED_100) {
                    setCell(cx, cy, FREE_0);
                }
            }
        }
        if (is_endpoint) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; cx += sx; }
        if (e2 < dx)  { err += dx; cy += sy; }
    }
}`
  },
  frontier_detect: {
    name: 'src/exploration/src/frontier_detector.cpp',
    code: `bool FrontierDetector::isFrontier(const std::vector<int8_t>& grid, uint32_t w, uint32_t h, uint32_t gx, uint32_t gy) {
    if (grid[gy * w + gx] != 0) return false;

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int ngx = static_cast<int>(gx) + dx;
            int ngy = static_cast<int>(gy) + dy;
            if (ngx < 0 || ngx >= (int)w || ngy < 0 || ngy >= (int)h) return true;
            if (grid[ngy * w + ngx] == -1) return true;
        }
    }
    return false;
}`
  },
  localization: {
    name: 'src/localization/src/ground_truth_pose_publisher.cpp',
    code: `void GroundTruthPosePublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "map";
    pose.pose = msg->pose.pose;
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
}`
  },
  control: {
    name: 'src/control/src/px4_control_node.cpp',
    code: `void PX4ControlNode::publishOffboardSetpoint(float target_x, float target_y, float target_z, float target_yaw) {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = {target_x, target_y, target_z};
    msg.yaw = target_yaw;
    trajectory_setpoint_pub_->publish(msg);
}`
  }
};

function showCodeSnippet(key) {
  const item = codeSnippets[key];
  if (!item) return;
  document.getElementById('code-filename').innerText = item.name;
  document.getElementById('code-display').innerHTML = `<code>${item.code}</code>`;
}

const imgData = occCtx.createImageData(engine.mapWidth, engine.mapHeight);

function render2DOccupancyGrid() {
  const d = imgData.data;
  const grid = engine.grid;

  for (let i = 0; i < grid.length; i++) {
    const val = grid[i];
    const p = i * 4;

    if (val === -1) {
      d[p] = 148; d[p+1] = 163; d[p+2] = 184; d[p+3] = 255;
    } else if (val === 0) {
      d[p] = 34; d[p+1] = 197; d[p+2] = 94; d[p+3] = 255;
    } else if (val === 100) {
      d[p] = 220; d[p+1] = 38; d[p+2] = 38; d[p+3] = 255;
    }
  }

  occCtx.putImageData(imgData, 0, 0);

  overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);

  const scaleX = overlayCanvas.width / engine.mapWidth;
  const scaleY = overlayCanvas.height / engine.mapHeight;

  overlayCtx.fillStyle = '#fbbf24';
  for (const f of engine.frontiers) {
    overlayCtx.fillRect(f.gx * scaleX, f.gy * scaleY, scaleX, scaleY);
  }

  const colors = ['#0284c7', '#16a34a', '#7c3aed', '#e11d48', '#ea580c', '#2563eb'];
  engine.clusters.forEach((c, idx) => {
    const col = colors[idx % colors.length];
    const cg = engine.worldToGrid(c.centroidX, c.centroidY);
    const cx = cg.gx * scaleX;
    const cy = cg.gy * scaleY;

    overlayCtx.strokeStyle = col;
    overlayCtx.lineWidth = 2.0;
    overlayCtx.beginPath();
    overlayCtx.arc(cx, cy, 6, 0, Math.PI * 2);
    overlayCtx.stroke();

    overlayCtx.fillStyle = col;
    overlayCtx.font = 'bold 10px Outfit';
    overlayCtx.fillText(`C${c.id}`, cx + 8, cy - 2);
  });

  if (engine.plannedPath && engine.plannedPath.length > 0) {
    const dg = engine.worldToGrid(engine.drone.x, engine.drone.y);
    overlayCtx.strokeStyle = '#0284c7';
    overlayCtx.lineWidth = 2.5;
    overlayCtx.setLineDash([4, 4]);
    overlayCtx.beginPath();
    overlayCtx.moveTo(dg.gx * scaleX, dg.gy * scaleY);

    for (const pt of engine.plannedPath) {
      const pg = engine.worldToGrid(pt.x, pt.y);
      overlayCtx.lineTo(pg.gx * scaleX, pg.gy * scaleY);
    }
    overlayCtx.stroke();
    overlayCtx.setLineDash([]);

    overlayCtx.fillStyle = '#0284c7';
    for (let i = 0; i < engine.plannedPath.length - 1; i++) {
      const pt = engine.plannedPath[i];
      const pg = engine.worldToGrid(pt.x, pt.y);
      overlayCtx.beginPath();
      overlayCtx.arc(pg.gx * scaleX, pg.gy * scaleY, 3, 0, Math.PI * 2);
      overlayCtx.fill();
    }
  }

  if (engine.currentWaypoint) {
    const wg = engine.worldToGrid(
      engine.currentWaypoint.finalX || engine.currentWaypoint.x,
      engine.currentWaypoint.finalY || engine.currentWaypoint.y
    );
    const wx = wg.gx * scaleX;
    const wy = wg.gy * scaleY;

    overlayCtx.fillStyle = '#ec4899';
    overlayCtx.beginPath();
    overlayCtx.arc(wx, wy, 7, 0, Math.PI * 2);
    overlayCtx.fill();
    overlayCtx.strokeStyle = '#ffffff';
    overlayCtx.lineWidth = 2;
    overlayCtx.stroke();
  }

  const dg = engine.worldToGrid(engine.drone.x, engine.drone.y);
  const dx = dg.gx * scaleX;
  const dy = dg.gy * scaleY;

  overlayCtx.fillStyle = '#2563eb';
  overlayCtx.beginPath();
  overlayCtx.arc(dx, dy, 5, 0, Math.PI * 2);
  overlayCtx.fill();
  overlayCtx.strokeStyle = '#ffffff';
  overlayCtx.lineWidth = 2;
  overlayCtx.stroke();

  overlayCtx.strokeStyle = '#2563eb';
  overlayCtx.lineWidth = 2.5;
  overlayCtx.beginPath();
  overlayCtx.moveTo(dx, dy);
  overlayCtx.lineTo(dx + Math.cos(engine.drone.yaw) * 14, dy + Math.sin(engine.drone.yaw) * 14);
  overlayCtx.stroke();
}

function renderDepthSensorMiniView() {
  depthCtx.fillStyle = '#f8fafc';
  depthCtx.fillRect(0, 0, depthCanvas.width, depthCanvas.height);

  const barW = depthCanvas.width / engine.camera.numRays;
  for (let i = 0; i < engine.camera.numRays; i++) {
    const d = engine.depthBuffer[i];
    const norm = Math.min(1.0, d / engine.camera.maxRange);
    const h = (1.0 - norm) * depthCanvas.height;

    const r = Math.floor(37 * (1.0 - norm) + 2 * norm);
    const g = Math.floor(99 * (1.0 - norm) + 132 * norm);
    const b = Math.floor(235 * (1.0 - norm) + 199 * norm);

    depthCtx.fillStyle = `rgb(${r},${g},${b})`;
    depthCtx.fillRect(i * barW, depthCanvas.height - h, barW - 1, h);
  }
}

function updateTelemetryUI() {
  const mins = Math.floor(simTimeElapsed / 60).toString().padStart(2, '0');
  const secs = (simTimeElapsed % 60).toFixed(1).padStart(4, '0');
  elTimer.innerText = `${mins}:${secs}`;

  const totalCells = engine.mapWidth * engine.mapHeight;
  const pct = ((engine.totalExploredCells / totalCells) * 100).toFixed(1);
  const sqm = (engine.totalExploredCells * (engine.resolution * engine.resolution)).toFixed(1);
  elExploredPct.innerText = `${pct}%`;
  elExploredSqm.innerText = `${sqm} m²`;
  elExploredFill.style.width = `${pct}%`;

  elFrontierCount.innerText = engine.frontiers.length;
  elClusterCount.innerText = `${engine.clusters.length} Clusters`;

  if (engine.selectedCluster) {
    elTargetCoords.innerText = `X: ${engine.selectedCluster.centroidX.toFixed(1)}, Y: ${engine.selectedCluster.centroidY.toFixed(1)}`;
    elTargetScore.innerText = `Score: ${engine.selectedCluster.score.toFixed(2)}`;
  } else {
    elTargetCoords.innerText = `Searching...`;
    elTargetScore.innerText = `Score: 0.00`;
  }

  elFlightDist.innerText = `${engine.drone.flightDistance.toFixed(1)} m`;
  elWaypointsVisited.innerText = `${engine.drone.waypointsVisited} Waypoints`;

  elFsmBadge.innerText = engine.fsmState;
  document.querySelectorAll('.fsm-step').forEach(s => s.classList.remove('active'));
  if (engine.fsmState === 'TAKEOFF') document.getElementById('step-takeoff').classList.add('active');
  else if (engine.fsmState === 'MAPPING') document.getElementById('step-mapping').classList.add('active');
  else if (engine.fsmState === 'DETECT_FRONTIERS') document.getElementById('step-frontiers').classList.add('active');
  else if (engine.fsmState === 'SELECT_TARGET') document.getElementById('step-cluster').classList.add('active');
  else if (engine.fsmState === 'NAVIGATING') document.getElementById('step-nav').classList.add('active');

  elHudAlt.innerText = `${engine.drone.z.toFixed(2)} m`;
  const spd = Math.hypot(engine.drone.vx, engine.drone.vy).toFixed(2);
  elHudSpd.innerText = `${spd} m/s`;
  elHudHdg.innerText = `${((engine.drone.yaw * 180 / Math.PI + 360) % 360).toFixed(0)}°`;

  elImuPos.innerText = `X: ${engine.drone.x.toFixed(2)}, Y: ${engine.drone.y.toFixed(2)}, Z: ${engine.drone.z.toFixed(2)}`;
  elImuAtt.innerText = `Yaw: ${(engine.drone.yaw * 180 / Math.PI).toFixed(1)}°`;
  elBattery.innerText = `${engine.drone.battery.toFixed(1)}% (LiPo 4S)`;

  if (engine.clusters.length > 0) {
    let html = '';
    const sorted = [...engine.clusters].sort((a, b) => b.score - a.score).slice(0, 4);
    sorted.forEach(c => {
      const isSel = engine.selectedCluster && engine.selectedCluster.id === c.id;
      html += `
        <div class="cluster-score-row ${isSel ? 'selected' : ''}">
          <span><b>Cluster #${c.id}</b> (N=${c.size} cells)</span>
          <span>Dist: ${c.distance.toFixed(1)}m</span>
          <span style="color: ${isSel ? '#2563eb' : '#475569'}"><b>Score: ${c.score.toFixed(1)}</b></span>
        </div>
      `;
    });
    elScoringList.innerHTML = html;
  } else {
    elScoringList.innerHTML = `<div class="scoring-empty-state">No candidate clusters detected</div>`;
  }
}

function animate(timestamp) {
  requestAnimationFrame(animate);

  const dt = (timestamp - lastTimestamp) / 1000.0;
  lastTimestamp = timestamp;

  if (engine.isExploring || engine.drone.z > 0) {
    simTimeElapsed += dt * engine.simSpeed;
  }

  engine.step(dt);

  droneGroup.position.set(engine.drone.x, engine.drone.y, engine.drone.z);
  droneGroup.rotation.z = engine.drone.yaw - Math.PI / 2;

  rotors.forEach(r => {
    r.rotation.z += (engine.drone.z > 0 ? 0.6 : 0.0);
  });

  const pts = engine.flightPath;
  const count = Math.min(pts.length, maxTrailPoints);
  const posArr = trailGeo.attributes.position.array;
  for (let i = 0; i < count; i++) {
    posArr[i * 3] = pts[i].x;
    posArr[i * 3 + 1] = pts[i].y;
    posArr[i * 3 + 2] = pts[i].z;
  }
  trailGeo.setDrawRange(0, count);
  trailGeo.attributes.position.needsUpdate = true;

  if (cameraFollow) {
    camera.position.x = THREE.MathUtils.lerp(camera.position.x, engine.drone.x, 0.05);
    camera.position.y = THREE.MathUtils.lerp(camera.position.y, engine.drone.y - 14, 0.05);
    camera.position.z = THREE.MathUtils.lerp(camera.position.z, engine.drone.z + 18, 0.05);
    camera.lookAt(engine.drone.x, engine.drone.y, engine.drone.z);
  }

  renderer.render(scene, camera);

  render2DOccupancyGrid();
  renderDepthSensorMiniView();
  updateTelemetryUI();
}

function onWindowResize() {
  if (!container3D) return;
  camera.aspect = container3D.clientWidth / container3D.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container3D.clientWidth, container3D.clientHeight);
}

window.addEventListener('resize', onWindowResize);

requestAnimationFrame(animate);
