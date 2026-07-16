const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

class FakeContext {
  constructor() {
    this.ops = [];
    this.fillStyle = '';
    this.strokeStyle = '';
    this.lineWidth = 1;
    this.font = '';
  }
  setTransform() {}
  fillRect(x, y, w, h) { this.ops.push({ type: 'fillRect', x, y, w, h, fillStyle: this.fillStyle }); }
  strokeRect(x, y, w, h) { this.ops.push({ type: 'strokeRect', x, y, w, h, strokeStyle: this.strokeStyle }); }
  beginPath() {}
  moveTo(x, y) { this.ops.push({ type: 'moveTo', x, y }); }
  lineTo(x, y) { this.ops.push({ type: 'lineTo', x, y }); }
  stroke() { this.ops.push({ type: 'stroke', strokeStyle: this.strokeStyle, lineWidth: this.lineWidth }); }
  fill() { this.ops.push({ type: 'fill', fillStyle: this.fillStyle, strokeStyle: this.strokeStyle }); }
  arc(x, y, r) { this.ops.push({ type: 'arc', x, y, r }); }
  translate() {}
  rotate() {}
  save() {}
  restore() {}
  closePath() {}
  fillText(text, x, y) { this.ops.push({ type: 'fillText', text, x, y }); }
  setLineDash() {}
}

class FakeCanvas {
  constructor(ctx) {
    this.clientWidth = 800;
    this.clientHeight = 600;
    this.width = 0;
    this.height = 0;
    this._ctx = ctx;
    this._listeners = new Map();
  }
  getContext() { return this._ctx; }
  addEventListener(type, callback) {
    if (!this._listeners.has(type)) this._listeners.set(type, []);
    this._listeners.get(type).push(callback);
  }
  emit(type, x, y, extra = {}) {
    const event = {
      clientX: x,
      clientY: y,
      button: 0,
      preventDefault() {},
      ...extra,
    };
    for (const callback of this._listeners.get(type) || []) callback(event);
  }
  getBoundingClientRect() { return { left: 0, top: 0 }; }
}

function loadMapClass() {
  const source = fs.readFileSync(path.join(__dirname, 'static', 'map.js'), 'utf8');
  const context = {
    window: { devicePixelRatio: 1 },
    console,
  };
  vm.createContext(context);
  vm.runInContext(source, context, { filename: 'map.js' });
  return context.window.Go2WMap;
}

function createMap(opts = {}) {
  const ctx = new FakeContext();
  const canvas = new FakeCanvas(ctx);
  const Go2WMap = loadMapClass();
  const map = new Go2WMap(canvas, opts);
  map._resize();
  return { map, ctx, canvas };
}

function near(a, b) {
  return Math.abs(a - b) < 1e-6;
}

function testWorldScanPointsAreDrawnWithoutSecondTransform() {
  const { map, ctx } = createMap();
  map.update({
    x: 5,
    y: -5,
    yaw: Math.PI / 2,
    trail: [[5, -5]],
    map: [],
    scan: [[7, -5]],
    detections: [],
    waypoints: [],
    currentWP: -1,
    slam_source: 'ros2_nx',
  });
  map._draw();

  const expectedX = map._tf.toX(7);
  const expectedY = map._tf.toY(-5);
  const scanArc = ctx.ops
    .filter(op => op.type === 'arc' && op.r === 2)
    .find(op => near(op.x, expectedX) && near(op.y, expectedY));

  assert(
    scanArc,
    `expected scan point to be drawn at world coordinate (7,-5), ` +
      `screen=(${expectedX},${expectedY}); arcs=${JSON.stringify(ctx.ops.filter(op => op.type === 'arc' && op.r === 2))}`,
  );
}

function testTransformRecomputesAfterSlamUpdate() {
  const { map } = createMap();
  map.update({ x: 0, y: 0, trail: [[0, 0]], scan: [], map: [] });
  map._draw();
  assert.strictEqual(map._tf.maxX, 0);

  map.update({ x: 20, y: 0, trail: [[20, 0]], scan: [[21, 0]], map: [] });
  map._draw();

  assert(
    map._tf.maxX >= 21,
    `expected transform bounds to include updated robot/scan position, got maxX=${map._tf.maxX}`,
  );
}

function testTransformIgnoresStaleRemoteMapOutliers() {
  const { map } = createMap();
  map.update({
    x: 4,
    y: -1,
    trail: [[4, -1]],
    scan: [[5, -1]],
    map: [[-61000, -9700], [100000, 100000]],
  });
  map._draw();

  assert(map._tf.scale >= 20, `stale remote cells collapsed click scale to ${map._tf.scale}`);
  const center = map.screenToWorld(400, 300);
  assert(Math.hypot(center.x - 4, center.y + 1) < 1e-6,
    `map center should remain near robot, got ${JSON.stringify(center)}`);
}

function testLocalLidarPointsAccumulateIntoObstacleMap() {
  const { map } = createMap();
  map.update({ x: 5, y: -5, yaw: Math.PI / 2, trail: [[5, -5]], scan: [], map: [] });
  assert.strictEqual(typeof map.addLocalObstaclePoints, 'function');

  map.addLocalObstaclePoints([[2, 0]]);

  const hasPoint = map.slam.lidarMapPoints.some(([x, y]) => near(x, 5) && near(y, -3));
  assert(hasPoint, `expected local MID360 point [2,0] to map to world point [5,-3], got ${JSON.stringify(map.slam.lidarMapPoints)}`);
}

function testBrowserObstacleMapIsBoundedForRealtimePanelUpdates() {
  const { map } = createMap();
  const points = Array.from({ length: 9000 }, (_, i) => [i / 10, 0]);
  map.update({ x: 0, y: 0, map: points });
  assert.strictEqual(map.slam.mapPoints.length, 2000);
  assert.deepStrictEqual(Array.from(map.slam.mapPoints[0]), [700, 0]);
}

function testPersonMarkersRenderWithLabelsAndQualityStyles() {
  const { map, ctx } = createMap();
  map.update({
    x: 0,
    y: 0,
    trail: [[0, 0]],
    scan: [],
    map: [],
    detections: [],
    person_markers: [
      { x: 24, y: 0, world_z: 0.5, position_dimension: 3,
        position_quality: 'range_lidar' },
      { world_x: -18, world_y: 0, position_quality: 'bearing_only' },
    ],
  });

  map._draw();

  const labels = ctx.ops
    .filter(op => op.type === 'fillText')
    .map(op => op.text);
  assert(labels.includes('人1'), `expected person marker label 人1, got ${JSON.stringify(labels)}`);
  assert(labels.includes('人2'), `expected person marker label 人2, got ${JSON.stringify(labels)}`);
  assert(labels.includes('z 0.5m'), `expected 3D height evidence, got ${JSON.stringify(labels)}`);
  assert(map._tf.maxX >= 24, `expected transform maxX to include x/y person marker, got ${map._tf.maxX}`);
  assert(map._tf.minX <= -18, `expected transform minX to include world_x/world_y person marker, got ${map._tf.minX}`);

  const confirmedFill = ctx.ops.find(op => op.type === 'fill' && op.fillStyle === '#ff1744');
  const confirmedStroke = ctx.ops.find(op => op.type === 'stroke' && op.strokeStyle === '#ffffff');
  const bearingFill = ctx.ops.find(op => op.type === 'fill' && op.fillStyle === '#ffc107');
  const bearingStroke = ctx.ops.find(op => op.type === 'stroke' && op.strokeStyle === '#263238');
  assert(confirmedFill, `expected confirmed person marker red fill, got ${JSON.stringify(ctx.ops.filter(op => op.type === 'fill'))}`);
  assert(confirmedStroke, `expected confirmed person marker white stroke, got ${JSON.stringify(ctx.ops.filter(op => op.type === 'stroke'))}`);
  assert(bearingFill, `expected bearing-only person marker amber fill, got ${JSON.stringify(ctx.ops.filter(op => op.type === 'fill'))}`);
  assert(bearingStroke, `expected bearing-only person marker dark stroke, got ${JSON.stringify(ctx.ops.filter(op => op.type === 'stroke'))}`);
}

function testGenericTargetMarkersRenderLocalizedTableLabel() {
  const { map, ctx } = createMap();
  map.update({
    x: 0,
    y: 0,
    trail: [[0, 0]],
    scan: [],
    map: [],
    detections: [],
    target_markers: [
      { id: 'dining_table_001', class: 'dining table', x: 2, y: 1,
        position_quality: 'range_lidar' },
    ],
  });

  map._draw();

  const labels = ctx.ops
    .filter(op => op.type === 'fillText')
    .map(op => op.text);
  assert(labels.includes('桌子1'), `expected table marker label 桌子1, got ${JSON.stringify(labels)}`);
  assert.strictEqual(map.slam.targetMarkers[0].class, 'dining table');
}

function worldFromFrozenTransform(tf, sx, sy) {
  return {
    x: (sx - tf.toX(0)) / tf.scale,
    y: (tf.toY(0) - sy) / tf.scale,
  };
}

function testPrimaryClickSelectsGoalUsingMouseDownTransform() {
  const goals = [];
  const regions = [];
  const { map, canvas } = createMap({
    onSelectGoal: goal => goals.push(goal),
    onSelectRegion: region => regions.push(region),
  });
  map.update({ x: 0, y: 0, trail: [[0, 0]], scan: [], map: [] });
  map._draw();
  const frozen = map._tf;
  const expected = worldFromFrozenTransform(frozen, 404, 303);

  canvas.emit('mousedown', 400, 300);
  // A live SLAM update between down/up must not move the selected world point.
  map.update({ x: 100, y: -80, trail: [[100, -80]], scan: [], map: [] });
  canvas.emit('mouseup', 404, 303);

  assert.strictEqual(goals.length, 1);
  assert.strictEqual(regions.length, 0);
  assert(near(goals[0].x, expected.x), `${goals[0].x} != frozen ${expected.x}`);
  assert(near(goals[0].y, expected.y), `${goals[0].y} != frozen ${expected.y}`);
}

function testEuclideanThresholdSeparatesClickFromRegionDrag() {
  const goals = [];
  const regions = [];
  const { map, canvas } = createMap({
    onSelectGoal: goal => goals.push(goal),
    onSelectRegion: region => regions.push(region),
  });
  map._draw();

  // hypot(6, 6) > 8, even though each individual axis is below 8.
  canvas.emit('mousedown', 300, 250);
  canvas.emit('mousemove', 306, 256);
  canvas.emit('mouseup', 306, 256);
  assert.strictEqual(regions.length, 1);
  assert(regions[0].w > 0 && regions[0].h > 0, `expected positive region dimensions, got ${JSON.stringify(regions[0])}`);
  assert.strictEqual(goals.length, 0);

  // Exactly eight pixels remains an intentional point selection.
  canvas.emit('mousedown', 420, 280);
  canvas.emit('mousemove', 428, 280);
  canvas.emit('mouseup', 428, 280);
  assert.strictEqual(goals.length, 1);
  assert.strictEqual(regions.length, 1);
}

function testMouseLeaveAndNonPrimaryButtonNeverCreateGoals() {
  const goals = [];
  const regions = [];
  const { map, canvas } = createMap({
    onSelectGoal: goal => goals.push(goal),
    onSelectRegion: region => regions.push(region),
  });
  map._draw();

  canvas.emit('mousedown', 200, 200);
  canvas.emit('mousemove', 260, 260);
  canvas.emit('mouseleave', 260, 260);
  canvas.emit('mouseup', 260, 260);
  canvas.emit('mousedown', 500, 300, { button: 2 });
  canvas.emit('mouseup', 500, 300, { button: 2 });

  assert.strictEqual(goals.length, 0);
  assert.strictEqual(regions.length, 0);
}

function testPersonMarkerHitWinsOverGoalSelection() {
  const goals = [];
  const markers = [];
  const regions = [];
  const { map, canvas } = createMap({
    onSelectGoal: goal => goals.push(goal),
    onSelectRegion: region => regions.push(region),
  });
  map.onSelectMarker = marker => markers.push(marker);
  const marker = { x: 1, y: -2, position_quality: 'range_lidar' };
  map.update({ person_markers: [marker], trail: [[0, 0]], scan: [], map: [] });
  map._draw();
  const sx = map._tf.toX(marker.x);
  const sy = map._tf.toY(marker.y);

  canvas.emit('mousedown', sx, sy);
  canvas.emit('mouseup', sx, sy);

  assert.deepStrictEqual(markers, [marker]);
  assert.strictEqual(goals.length, 0);
  assert.strictEqual(regions.length, 0);
}

function testNavigationGoalIsPartOfBoundsAndRendersReticle() {
  const { map, ctx } = createMap();
  assert.strictEqual(typeof map.setNavGoal, 'function');
  map.setNavGoal({
    generation: 7,
    status: 'active',
    x: 30,
    y: -4,
    yaw: Math.PI / 3,
    frame_id: 'map',
  });
  map._draw();

  assert(map._tf.maxX >= 30, `expected target in bounds, maxX=${map._tf.maxX}`);
  const gx = map._tf.toX(30);
  const gy = map._tf.toY(-4);
  assert(
    ctx.ops.some(op => op.type === 'arc' && op.r === 9 && near(op.x, gx) && near(op.y, gy)),
    `expected nav reticle at (${gx},${gy})`,
  );
  assert.strictEqual(map.navGoal.generation, 7);
  assert.strictEqual(map.navGoal.status, 'active');
}

function testNullGoalIsRejectedAndServerUnavailableUsesFailureColor() {
  const { map, ctx } = createMap();
  assert.strictEqual(map.setNavGoal({ status: 'idle', x: null, y: null, yaw: null }), false);
  assert.strictEqual(map.navGoal, null);
  assert.strictEqual(map.setNavGoal({
    generation: 2,
    status: 'server_unavailable',
    x: 1,
    y: 2,
    yaw: 0,
    frame_id: 'map',
  }), true);
  map._draw();
  assert(
    ctx.ops.some(op => op.type === 'stroke' && op.strokeStyle === '#ef5350'),
    'expected server_unavailable reticle to use failure red',
  );
}

function testCostmapIsRenderedBelowGoalAndRobotOverlays() {
  const source = fs.readFileSync(path.join(__dirname, 'static', 'map.js'), 'utf8');
  const costmapDraw = source.indexOf('ctx.drawImage(this._cmCanvas');
  const goalDraw = source.indexOf('if (this.navGoal)', costmapDraw);
  const robotDraw = source.indexOf("ctx.fillStyle = '#00e5ff'", goalDraw);

  assert(costmapDraw >= 0, 'costmap draw call missing');
  assert(goalDraw > costmapDraw, 'navigation goal must be above costmap');
  assert(robotDraw > goalDraw, 'robot marker must be above costmap and goal');
}

function testRoomSearchCoverageAndViewpointsRenderOnMap() {
  const { map, ctx } = createMap();
  map.update({
    room_search: {
      phase: 'ACTIVE_SEARCH',
      room: '客厅',
      room_area: { origin_x: 1, origin_y: 2, width: 4, height: 3, spacing: 1 },
      candidate_viewpoints: [{ x: 2, y: 3 }, { x: 4, y: 4 }],
      visited_viewpoints: [{ x: 2, y: 3 }],
      observed_cells: [{ x: 1, y: 2 }, { x: 2, y: 2 }],
      coverage_ratio: 0.5,
      coverage_threshold: 0.9,
      visual_range_m: 2.5,
    },
  });
  map._draw();

  assert.strictEqual(map.slam.roomSearch.coverageRatio, 0.5);
  assert.strictEqual(map.slam.roomSearch.observedCells.length, 2);
  assert(
    ctx.ops.some(op => op.type === 'strokeRect' && op.strokeStyle === '#7e57c2'),
    'expected calibrated room boundary',
  );
  assert(
    ctx.ops.some(op => op.type === 'arc' && op.r === 3),
    'expected candidate viewpoint markers',
  );
  assert(
    ctx.ops.some(op => op.type === 'arc' && op.r === 6),
    'expected visited viewpoint markers',
  );
  assert(
    ctx.ops.some(op => op.type === 'fillText' && op.text.includes('覆盖 50%')),
    'expected coverage label',
  );
}

function testPanelForwardsSearchRoomProgressIntoMap() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const branch = panel.indexOf("data.type === 'search_room'");
  assert(branch >= 0, 'search_room WebSocket branch missing');
  assert(
    panel.indexOf('room_search: data.data', branch) > branch,
    'search_room progress must be forwarded to the map renderer',
  );
}

function testPanelForwardsGenericTargetMarkerEventsIntoMap() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const branch = panel.indexOf("data.type === 'target_markers'");
  assert(branch >= 0, 'target_markers WebSocket branch missing');
  assert(
    panel.indexOf('target_markers: markers', branch) > branch,
    'generic target markers must be forwarded to the map renderer',
  );
  const reportBranch = panel.indexOf("data.type === 'mission_report'");
  assert(
    panel.indexOf('target_markers: detections', reportBranch) > reportBranch,
    'mission report detections must be forwarded as generic target markers',
  );
}

testWorldScanPointsAreDrawnWithoutSecondTransform();
testTransformRecomputesAfterSlamUpdate();
testTransformIgnoresStaleRemoteMapOutliers();
testLocalLidarPointsAccumulateIntoObstacleMap();
testBrowserObstacleMapIsBoundedForRealtimePanelUpdates();
testPersonMarkersRenderWithLabelsAndQualityStyles();
testGenericTargetMarkersRenderLocalizedTableLabel();
testPrimaryClickSelectsGoalUsingMouseDownTransform();
testEuclideanThresholdSeparatesClickFromRegionDrag();
testMouseLeaveAndNonPrimaryButtonNeverCreateGoals();
testPersonMarkerHitWinsOverGoalSelection();
testNavigationGoalIsPartOfBoundsAndRendersReticle();
testNullGoalIsRejectedAndServerUnavailableUsesFailureColor();
testCostmapIsRenderedBelowGoalAndRobotOverlays();
testRoomSearchCoverageAndViewpointsRenderOnMap();
testPanelForwardsSearchRoomProgressIntoMap();
testPanelForwardsGenericTargetMarkerEventsIntoMap();
console.log('map contract tests passed');
