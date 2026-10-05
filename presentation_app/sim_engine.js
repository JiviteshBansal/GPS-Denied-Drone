class SimEngine {
  constructor() {
    this.mapWidth = 200;
    this.mapHeight = 200;
    this.resolution = 0.2;
    this.originX = -20.0;
    this.originY = -20.0;

    this.grid = new Int8Array(this.mapWidth * this.mapHeight).fill(-1);

    this.drone = {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      yaw: 0.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
      radius: 0.55,
      maxSpeed: 2.2,
      battery: 100.0,
      flightDistance: 0.0,
      waypointsVisited: 0
    };

    this.camera = {
      fov: (70.0 * Math.PI) / 180.0,
      maxRange: 12.0,
      numRays: 48,
      minRange: 0.3
    };

    this.worldType = 'warehouse';
    this.obstacles = [];
    this.loadWorld(this.worldType);

    this.frontiers = [];
    this.clusters = [];
    this.selectedCluster = null;
    this.currentWaypoint = null;
    this.plannedPath = [];

    this.fsmState = 'IDLE';
    this.stateTimer = 0.0;
    this.totalExploredCells = 0;
    this.simSpeed = 1.0;
    this.isExploring = false;

    this.flightPath = [{ x: 0, y: 0, z: 0 }];

    this.depthBuffer = new Float32Array(this.camera.numRays).fill(this.camera.maxRange);
  }

  loadWorld(type) {
    this.worldType = type;
    this.obstacles = [];

    const bounds = 19.5;
    this.obstacles.push({ minX: -bounds, maxX: bounds, minY: bounds - 0.6, maxY: bounds + 0.6 });
    this.obstacles.push({ minX: -bounds, maxX: bounds, minY: -bounds - 0.6, maxY: -bounds + 0.6 });
    this.obstacles.push({ minX: -bounds - 0.6, maxX: -bounds + 0.6, minY: -bounds, maxY: bounds });
    this.obstacles.push({ minX: bounds - 0.6, maxX: bounds + 0.6, minY: -bounds, maxY: bounds });

    if (type === 'warehouse') {
      this.obstacles.push({ minX: -14, maxX: -4, minY: 7, maxY: 9 });
      this.obstacles.push({ minX: 4, maxX: 14, minY: 7, maxY: 9 });
      this.obstacles.push({ minX: -14, maxX: -4, minY: -9, maxY: -7 });
      this.obstacles.push({ minX: 4, maxX: 14, minY: -9, maxY: -7 });

      this.obstacles.push({ minX: -4, maxX: -2, minY: -2, maxY: 2 });
      this.obstacles.push({ minX: 2, maxX: 4, minY: -2, maxY: 2 });

      this.obstacles.push({ minX: -15, maxX: -11, minY: -2, maxY: 2 });
      this.obstacles.push({ minX: 11, maxX: 15, minY: -2, maxY: 2 });
      this.obstacles.push({ minX: -7, maxX: -4, minY: 13, maxY: 15 });
      this.obstacles.push({ minX: 4, maxX: 7, minY: 13, maxY: 15 });
      this.obstacles.push({ minX: -7, maxX: -4, minY: -15, maxY: -13 });
      this.obstacles.push({ minX: 4, maxX: 7, minY: -15, maxY: -13 });
      this.obstacles.push({ minX: -1, maxX: 1, minY: 10, maxY: 13 });
      this.obstacles.push({ minX: -1, maxX: 1, minY: -13, maxY: -10 });
    } else if (type === 'maze') {
      this.obstacles.push({ minX: -16, maxX: -2, minY: 11, maxY: 12.5 });
      this.obstacles.push({ minX: 2, maxX: 16, minY: 11, maxY: 12.5 });
      this.obstacles.push({ minX: -16, maxX: 6, minY: 4, maxY: 5.5 });
      this.obstacles.push({ minX: -6, maxX: 16, minY: -5.5, maxY: -4 });
      this.obstacles.push({ minX: -16, maxX: 2, minY: -12.5, maxY: -11 });
      this.obstacles.push({ minX: 6, maxX: 16, minY: -12.5, maxY: -11 });
      this.obstacles.push({ minX: 5.5, maxX: 7, minY: -4, maxY: 4 });
      this.obstacles.push({ minX: -7, maxX: -5.5, minY: -11, maxY: -4 });
      this.obstacles.push({ minX: -2, maxX: -0.5, minY: -4, maxY: 2 });
      this.obstacles.push({ minX: 0.5, maxX: 2, minY: 6, maxY: 10 });
    } else if (type === 'corridor') {
      for (let x = -15; x <= 15; x += 6) {
        this.obstacles.push({ minX: x - 1.2, maxX: x + 1.2, minY: 4.5, maxY: 15 });
        this.obstacles.push({ minX: x - 1.2, maxX: x + 1.2, minY: -15, maxY: -4.5 });
      }
      this.obstacles.push({ minX: -3, maxX: 3, minY: -1, maxY: 1 });
    } else if (type === 'pillars') {
      for (let x = -15; x <= 15; x += 5.0) {
        for (let y = -15; y <= 15; y += 5.0) {
          if (Math.abs(x) > 2 || Math.abs(y) > 2) {
            this.obstacles.push({ minX: x - 0.9, maxX: x + 0.9, minY: y - 0.9, maxY: y + 0.9 });
          }
        }
      }
      this.obstacles.push({ minX: -8, maxX: -5, minY: -1, maxY: 1 });
      this.obstacles.push({ minX: 5, maxX: 8, minY: -1, maxY: 1 });
    }
    this.resetMap();
  }

  resetMap() {
    this.grid.fill(-1);
    this.frontiers = [];
    this.clusters = [];
    this.selectedCluster = null;
    this.currentWaypoint = null;
    this.plannedPath = [];
    this.drone.x = 0.0;
    this.drone.y = 0.0;
    this.drone.z = 0.0;
    this.drone.yaw = 0.0;
    this.drone.vx = 0.0;
    this.drone.vy = 0.0;
    this.drone.battery = 100.0;
    this.drone.flightDistance = 0.0;
    this.drone.waypointsVisited = 0;
    this.flightPath = [{ x: 0, y: 0, z: 0 }];
    this.fsmState = 'IDLE';
    this.isExploring = false;
    this.totalExploredCells = 0;
  }

  worldToGrid(x, y) {
    const gx = Math.floor((x - this.originX) / this.resolution);
    const gy = Math.floor((y - this.originY) / this.resolution);
    return { gx, gy };
  }

  gridToWorld(gx, gy) {
    const x = this.originX + (gx + 0.5) * this.resolution;
    const y = this.originY + (gy + 0.5) * this.resolution;
    return { x, y };
  }

  inBounds(gx, gy) {
    return gx >= 0 && gx < this.mapWidth && gy >= 0 && gy < this.mapHeight;
  }

  isCollidingWithObstacle(x, y, radius = 0.55) {
    for (const obs of this.obstacles) {
      if (
        x + radius >= obs.minX &&
        x - radius <= obs.maxX &&
        y + radius >= obs.minY &&
        y - radius <= obs.maxY
      ) {
        return true;
      }
    }
    return false;
  }

  castPhysicalRay(originX, originY, angle, maxDist) {
    const step = 0.1;
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);

    for (let d = this.camera.minRange; d < maxDist; d += step) {
      const px = originX + d * cosA;
      const py = originY + d * sinA;

      for (const obs of this.obstacles) {
        if (px >= obs.minX && px <= obs.maxX && py >= obs.minY && py <= obs.maxY) {
          return { hit: true, distance: d, hitX: px, hitY: py };
        }
      }
    }
    return { hit: false, distance: maxDist, hitX: originX + maxDist * cosA, hitY: originY + maxDist * sinA };
  }

  updateOccupancyGrid() {
    if (this.drone.z < 0.2) return;

    const droneG = this.worldToGrid(this.drone.x, this.drone.y);
    const angleStart = this.drone.yaw - this.camera.fov / 2;
    const angleStep = this.camera.fov / (this.camera.numRays - 1);

    for (let i = 0; i < this.camera.numRays; i++) {
      const rayAngle = angleStart + i * angleStep;
      const rayResult = this.castPhysicalRay(this.drone.x, this.drone.y, rayAngle, this.camera.maxRange);

      this.depthBuffer[i] = rayResult.distance;
      const hitG = this.worldToGrid(rayResult.hitX, rayResult.hitY);

      this.raytraceLine(droneG.gx, droneG.gy, hitG.gx, hitG.gy, rayResult.hit);
    }

    let explored = 0;
    for (let i = 0; i < this.grid.length; i++) {
      if (this.grid[i] !== -1) explored++;
    }
    this.totalExploredCells = explored;
  }

  raytraceLine(x0, y0, x1, y1, hitObstacle) {
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = x0 < x1 ? 1 : -1;
    const sy = y0 < y1 ? 1 : -1;
    let err = dx - dy;

    let currX = x0;
    let currY = y0;

    while (true) {
      const isEnd = (currX === x1 && currY === y1);

      if (this.inBounds(currX, currY)) {
        const idx = currY * this.mapWidth + currX;
        if (isEnd && hitObstacle) {
          this.grid[idx] = 100;
        } else {
          if (this.grid[idx] !== 100) {
            this.grid[idx] = 0;
          }
        }
      }

      if (isEnd) break;

      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        currX += sx;
      }
      if (e2 < dx) {
        err += dx;
        currY += sy;
      }
    }
  }

  detectFrontiers() {
    this.frontiers = [];

    for (let gy = 2; gy < this.mapHeight - 2; gy++) {
      for (let gx = 2; gx < this.mapWidth - 2; gx++) {
        const idx = gy * this.mapWidth + gx;

        if (this.grid[idx] === 0) {
          let hasUnknownNeighbor = false;

          for (let dy = -1; dy <= 1 && !hasUnknownNeighbor; dy++) {
            for (let dx = -1; dx <= 1; dx++) {
              if (dx === 0 && dy === 0) continue;
              const nIdx = (gy + dy) * this.mapWidth + (gx + dx);
              if (this.grid[nIdx] === -1) {
                hasUnknownNeighbor = true;
                break;
              }
            }
          }

          if (hasUnknownNeighbor) {
            const worldPos = this.gridToWorld(gx, gy);
            if (!this.isCollidingWithObstacle(worldPos.x, worldPos.y, 0.4)) {
              this.frontiers.push({ gx, gy, x: worldPos.x, y: worldPos.y });
            }
          }
        }
      }
    }
  }

  clusterFrontiers() {
    if (this.frontiers.length === 0) {
      this.clusters = [];
      return;
    }

    const visited = new Uint8Array(this.mapWidth * this.mapHeight).fill(0);
    const isFrontierMap = new Uint8Array(this.mapWidth * this.mapHeight).fill(0);

    for (const f of this.frontiers) {
      isFrontierMap[f.gy * this.mapWidth + f.gx] = 1;
    }

    this.clusters = [];
    let clusterId = 0;
    const minClusterSize = 4;

    for (const f of this.frontiers) {
      const idx = f.gy * this.mapWidth + f.gx;
      if (visited[idx] === 0) {
        const cluster = {
          id: clusterId++,
          cells: [],
          centroidX: 0,
          centroidY: 0,
          score: 0,
          size: 0,
          distance: 0
        };

        const queue = [{ gx: f.gx, gy: f.gy }];
        visited[idx] = 1;

        let sumX = 0;
        let sumY = 0;

        while (queue.length > 0) {
          const curr = queue.shift();
          const wPos = this.gridToWorld(curr.gx, curr.gy);
          cluster.cells.push({ gx: curr.gx, gy: curr.gy, x: wPos.x, y: wPos.y });
          sumX += wPos.x;
          sumY += wPos.y;

          for (let dy = -1; dy <= 1; dy++) {
            for (let dx = -1; dx <= 1; dx++) {
              if (dx === 0 && dy === 0) continue;
              const ngx = curr.gx + dx;
              const ngy = curr.gy + dy;

              if (this.inBounds(ngx, ngy)) {
                const nIdx = ngy * this.mapWidth + ngx;
                if (visited[nIdx] === 0 && isFrontierMap[nIdx] === 1) {
                  visited[nIdx] = 1;
                  queue.push({ gx: ngx, gy: ngy });
                }
              }
            }
          }
        }

        cluster.size = cluster.cells.length;
        if (cluster.size >= minClusterSize) {
          cluster.centroidX = sumX / cluster.size;
          cluster.centroidY = sumY / cluster.size;
          this.clusters.push(cluster);
        }
      }
    }
  }

  scoreAndSelectFrontier() {
    if (this.clusters.length === 0) {
      this.selectedCluster = null;
      return null;
    }

    const lambda = 0.18;
    let bestCluster = null;
    let maxScore = -Infinity;

    for (const cluster of this.clusters) {
      const dist = Math.hypot(cluster.centroidX - this.drone.x, cluster.centroidY - this.drone.y);
      cluster.distance = dist;
      cluster.score = cluster.size - (dist * lambda);

      if (cluster.score > maxScore) {
        maxScore = cluster.score;
        bestCluster = cluster;
      }
    }

    this.selectedCluster = bestCluster;
    return bestCluster;
  }

  planAStarPath(startX, startY, goalX, goalY) {
    const startG = this.worldToGrid(startX, startY);
    let goalG = this.worldToGrid(goalX, goalY);

    if (!this.inBounds(startG.gx, startG.gy) || !this.inBounds(goalG.gx, goalG.gy)) {
      return [{ x: goalX, y: goalY }];
    }

    if (this.isCollidingWithObstacle(goalX, goalY, 0.9)) {
      const angle = Math.atan2(startY - goalY, startX - goalX);
      for (let offset = 0.8; offset <= 3.5; offset += 0.4) {
        const testX = goalX + offset * Math.cos(angle);
        const testY = goalY + offset * Math.sin(angle);
        if (!this.isCollidingWithObstacle(testX, testY, 0.9)) {
          goalX = testX;
          goalY = testY;
          goalG = this.worldToGrid(goalX, goalY);
          break;
        }
      }
    }

    const startNodeKey = `${startG.gx},${startG.gy}`;
    const openSet = [{ gx: startG.gx, gy: startG.gy, f: 0, g: 0 }];
    const cameFrom = new Map();
    const gScore = new Map();
    gScore.set(startNodeKey, 0);

    const heuristic = (gx, gy) => Math.hypot(gx - goalG.gx, gy - goalG.gy);

    let iterations = 0;
    const maxIterations = 2500;

    while (openSet.length > 0 && iterations < maxIterations) {
      iterations++;
      openSet.sort((a, b) => a.f - b.f);
      const current = openSet.shift();
      const currentKey = `${current.gx},${current.gy}`;

      if (Math.hypot(current.gx - goalG.gx, current.gy - goalG.gy) <= 2) {
        const path = [{ x: goalX, y: goalY }];
        let currKey = currentKey;
        while (cameFrom.has(currKey)) {
          const prev = cameFrom.get(currKey);
          const wPos = this.gridToWorld(prev.gx, prev.gy);
          path.unshift({ x: wPos.x, y: wPos.y });
          currKey = `${prev.gx},${prev.gy}`;
        }
        return this.smoothPath(path);
      }

      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          const ngx = current.gx + dx;
          const ngy = current.gy + dy;

          if (!this.inBounds(ngx, ngy)) continue;

          const wPos = this.gridToWorld(ngx, ngy);
          if (this.isCollidingWithObstacle(wPos.x, wPos.y, 0.85)) continue;

          const gridVal = this.grid[ngy * this.mapWidth + ngx];
          if (gridVal === 100) continue;

          const moveCost = (dx !== 0 && dy !== 0) ? 1.414 : 1.0;
          const tentativeG = current.g + moveCost;
          const neighborKey = `${ngx},${ngy}`;

          if (!gScore.has(neighborKey) || tentativeG < gScore.get(neighborKey)) {
            cameFrom.set(neighborKey, { gx: current.gx, gy: current.gy });
            gScore.set(neighborKey, tentativeG);
            const f = tentativeG + heuristic(ngx, ngy);

            const existing = openSet.find(n => n.gx === ngx && n.gy === ngy);
            if (existing) {
              existing.g = tentativeG;
              existing.f = f;
            } else {
              openSet.push({ gx: ngx, gy: ngy, g: tentativeG, f: f });
            }
          }
        }
      }
    }

    return [{ x: goalX, y: goalY }];
  }

  smoothPath(rawPath) {
    if (rawPath.length <= 2) return rawPath;
    const smoothed = [rawPath[0]];

    for (let i = 1; i < rawPath.length; i++) {
      const prev = smoothed[smoothed.length - 1];
      const curr = rawPath[i];
      if (Math.hypot(curr.x - prev.x, curr.y - prev.y) > 1.8 || i === rawPath.length - 1) {
        smoothed.push(curr);
      }
    }
    return smoothed;
  }

  generateWaypoint(cluster) {
    if (!cluster) return null;

    let targetX = cluster.centroidX;
    let targetY = cluster.centroidY;

    this.plannedPath = this.planAStarPath(this.drone.x, this.drone.y, targetX, targetY);

    if (this.plannedPath.length > 0) {
      this.currentWaypoint = {
        x: this.plannedPath[0].x,
        y: this.plannedPath[0].y,
        z: 1.5,
        finalX: targetX,
        finalY: targetY,
        reachedThreshold: 0.75
      };
    }
    return this.currentWaypoint;
  }

  updateDronePhysics(dt) {
    if (this.drone.z > 0) {
      this.drone.battery = Math.max(0, this.drone.battery - dt * 0.02);
    }

    if (this.fsmState === 'TAKEOFF') {
      this.drone.z = Math.min(1.5, this.drone.z + dt * 1.0);
      if (this.drone.z >= 1.5) {
        this.fsmState = 'MAPPING';
      }
    } else if (this.fsmState === 'NAVIGATING' && this.currentWaypoint) {
      const dx = this.currentWaypoint.x - this.drone.x;
      const dy = this.currentWaypoint.y - this.drone.y;
      const dist = Math.hypot(dx, dy);

      let targetYaw = Math.atan2(dy, dx);

      let repX = 0;
      let repY = 0;
      for (let i = 0; i < this.camera.numRays; i++) {
        const d = this.depthBuffer[i];
        if (d < 1.2 && d > 0.1) {
          const rayAngle = this.drone.yaw - this.camera.fov / 2 + i * (this.camera.fov / (this.camera.numRays - 1));
          const force = (1.2 - d) / 1.2;
          repX -= Math.cos(rayAngle) * force * 1.5;
          repY -= Math.sin(rayAngle) * force * 1.5;
        }
      }

      let moveDirX = Math.cos(targetYaw) + repX;
      let moveDirY = Math.sin(targetYaw) + repY;
      const moveLen = Math.hypot(moveDirX, moveDirY);
      if (moveLen > 0.001) {
        moveDirX /= moveLen;
        moveDirY /= moveLen;
        targetYaw = Math.atan2(moveDirY, moveDirX);
      }

      let yawDiff = targetYaw - this.drone.yaw;
      while (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
      while (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;
      this.drone.yaw += yawDiff * Math.min(1.0, dt * 3.5);

      const speed = Math.min(this.drone.maxSpeed, Math.max(0.6, dist * 0.95));
      this.drone.vx = Math.cos(this.drone.yaw) * speed;
      this.drone.vy = Math.sin(this.drone.yaw) * speed;

      let nextX = this.drone.x + this.drone.vx * dt;
      let nextY = this.drone.y + this.drone.vy * dt;

      if (this.isCollidingWithObstacle(nextX, nextY, this.drone.radius)) {
        if (!this.isCollidingWithObstacle(nextX, this.drone.y, this.drone.radius)) {
          nextY = this.drone.y;
        } else if (!this.isCollidingWithObstacle(this.drone.x, nextY, this.drone.radius)) {
          nextX = this.drone.x;
        } else {
          nextX = this.drone.x;
          nextY = this.drone.y;
          this.drone.vx = 0;
          this.drone.vy = 0;
        }
      }

      const stepDist = Math.hypot(nextX - this.drone.x, nextY - this.drone.y);
      this.drone.x = nextX;
      this.drone.y = nextY;
      this.drone.flightDistance += stepDist;

      if (dist < this.currentWaypoint.reachedThreshold) {
        if (this.plannedPath.length > 1) {
          this.plannedPath.shift();
          this.currentWaypoint = {
            x: this.plannedPath[0].x,
            y: this.plannedPath[0].y,
            z: 1.5,
            reachedThreshold: 0.75
          };
        } else {
          this.drone.waypointsVisited++;
          this.currentWaypoint = null;
          this.plannedPath = [];
          this.fsmState = 'MAPPING';
        }
      }
    } else if (this.fsmState === 'LAND') {
      this.drone.z = Math.max(0.0, this.drone.z - dt * 0.8);
      if (this.drone.z <= 0.0) {
        this.fsmState = 'IDLE';
        this.isExploring = false;
      }
    }

    const lastP = this.flightPath[this.flightPath.length - 1];
    if (!lastP || Math.hypot(this.drone.x - lastP.x, this.drone.y - lastP.y) > 0.25) {
      this.flightPath.push({ x: this.drone.x, y: this.drone.y, z: this.drone.z });
    }
  }

  step(rawDt) {
    const dt = Math.min(0.1, rawDt * this.simSpeed);

    this.updateDronePhysics(dt);

    if (this.isExploring) {
      this.stateTimer += dt;

      if (this.fsmState === 'IDLE' && this.drone.z < 0.1) {
        this.fsmState = 'TAKEOFF';
      } else if (this.fsmState === 'MAPPING') {
        this.updateOccupancyGrid();
        this.fsmState = 'DETECT_FRONTIERS';
      } else if (this.fsmState === 'DETECT_FRONTIERS') {
        this.detectFrontiers();
        this.clusterFrontiers();
        this.fsmState = 'SELECT_TARGET';
      } else if (this.fsmState === 'SELECT_TARGET') {
        const bestCluster = this.scoreAndSelectFrontier();
        if (bestCluster) {
          this.generateWaypoint(bestCluster);
          this.fsmState = 'NAVIGATING';
        } else {
          const coveragePct = (this.totalExploredCells / (this.mapWidth * this.mapHeight)) * 100;
          if (coveragePct > 80 || this.frontiers.length === 0) {
            this.fsmState = 'COMPLETED';
            this.isExploring = false;
          } else {
            this.drone.yaw += 0.5;
            this.updateOccupancyGrid();
            this.fsmState = 'MAPPING';
          }
        }
      }
    } else {
      if (this.drone.z > 0.1) {
        this.updateOccupancyGrid();
      }
    }
  }
}

window.SimEngine = SimEngine;
