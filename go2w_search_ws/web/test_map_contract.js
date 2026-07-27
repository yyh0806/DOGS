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
  clearRect(x, y, w, h) { this.ops.push({ type: 'clearRect', x, y, w, h }); }
  createImageData(w, h) { return { width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }; }
  putImageData(image, x, y) { this.ops.push({ type: 'putImageData', image, x, y }); }
  drawImage(image, ...args) { this.ops.push({ type: 'drawImage', image, args }); }
  strokeRect(x, y, w, h) { this.ops.push({ type: 'strokeRect', x, y, w, h, strokeStyle: this.strokeStyle }); }
  beginPath() {}
  moveTo(x, y) { this.ops.push({ type: 'moveTo', x, y }); }
  lineTo(x, y) { this.ops.push({ type: 'lineTo', x, y }); }
  stroke() { this.ops.push({ type: 'stroke', strokeStyle: this.strokeStyle, lineWidth: this.lineWidth }); }
  fill(rule) { this.ops.push({ type: 'fill', fillStyle: this.fillStyle, strokeStyle: this.strokeStyle, rule }); }
  rect(x, y, w, h) { this.ops.push({ type: 'rect', x, y, w, h }); }
  arc(x, y, r, startAngle, endAngle, anticlockwise) {
    this.ops.push({ type: 'arc', x, y, r, startAngle, endAngle, anticlockwise });
  }
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
    this._width = 0;
    this._height = 0;
    this._ctx = ctx;
    this._listeners = new Map();
  }
  get width() { return this._width; }
  set width(value) { this._width = Math.max(0, Math.floor(Number(value) || 0)); }
  get height() { return this._height; }
  set height(value) { this._height = Math.max(0, Math.floor(Number(value) || 0)); }
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

function loadMapClass(runtime = {}) {
  const source = fs.readFileSync(path.join(__dirname, 'static', 'map.js'), 'utf8');
  const context = {
    window: {
      devicePixelRatio: runtime.devicePixelRatio || 1,
      addEventListener: runtime.addWindowListener || (() => {}),
    },
    requestAnimationFrame: runtime.requestAnimationFrame || (() => 1),
    cancelAnimationFrame: runtime.cancelAnimationFrame || (() => {}),
    document: {
      createElement(tag) {
        assert.strictEqual(tag, 'canvas');
        return new FakeCanvas(new FakeContext());
      },
    },
    console,
  };
  vm.createContext(context);
  vm.runInContext(source, context, { filename: 'map.js' });
  return context.window.Go2WMap;
}

function createMap(opts = {}, runtime = {}) {
  const ctx = new FakeContext();
  const canvas = new FakeCanvas(ctx);
  const Go2WMap = loadMapClass(runtime);
  const map = new Go2WMap(canvas, opts);
  map._resize();
  return { map, ctx, canvas };
}

function fakeRafRuntime() {
  let nextId = 1;
  const callbacks = new Map();
  return {
    requestAnimationFrame(callback) {
      const id = nextId++;
      callbacks.set(id, callback);
      return id;
    },
    cancelAnimationFrame(id) { callbacks.delete(id); },
    flush() {
      const pending = Array.from(callbacks.values());
      callbacks.clear();
      for (const callback of pending) callback(0);
      return pending.length;
    },
    pending() { return callbacks.size; },
  };
}

function testDirtySchedulerStaysIdleAndCoalescesUpdates() {
  const raf = fakeRafRuntime();
  const { map } = createMap({}, raf);
  let draws = 0;
  const originalDraw = map._draw.bind(map);
  map._draw = () => { draws += 1; originalDraw(); };

  map.start();
  assert.strictEqual(raf.pending(), 1);
  raf.flush();
  assert.strictEqual(draws, 1);
  for (let i = 0; i < 100; i++) raf.flush();
  assert.strictEqual(draws, 1, 'idle RAF ticks must not redraw an unchanged map');

  map.update({ x: 1 });
  map.update({ y: 2 });
  map.setNavGoal({ x: 3, y: 4, yaw: 0, frame_id: 'map' });
  assert.strictEqual(raf.pending(), 1, 'many mutations before a frame must coalesce');
  raf.flush();
  assert.strictEqual(draws, 2);
  assert.strictEqual(raf.pending(), 0);
}

function testDirtySchedulerInvalidatesCachesInteractionAndResize() {
  const raf = fakeRafRuntime();
  let resizeListener = null;
  const { map, canvas } = createMap({}, {
    ...raf,
    addWindowListener(type, callback) { if (type === 'resize') resizeListener = callback; },
  });
  map.start();
  raf.flush();

  map._cmDirty = false;
  map._tf = { stale: true };
  map.update({ costmap: { w: 1, h: 1, vals: [100], ox: 0, oy: 0, res: 0.1 } });
  assert.strictEqual(map._cmDirty, true);
  assert.strictEqual(map._tf, null);
  assert.strictEqual(raf.pending(), 1);
  raf.flush();

  map._tf = { stale: true };
  map.update({ occupancy_map: { points: [[2, 3]], resolution: 0.1 } });
  assert.strictEqual(map._tf, null, 'occupancy changes must invalidate transform bounds');
  raf.flush();

  canvas.emit('mousedown', 100, 100);
  canvas.emit('mousemove', 140, 140);
  assert.strictEqual(raf.pending(), 1, 'drag feedback must invalidate the frame');
  raf.flush();

  map._fogCacheKey = 'old';
  map._tf = { stale: true };
  canvas.clientWidth = 900;
  resizeListener();
  assert.strictEqual(map._tf, null);
  assert.strictEqual(map._fogCacheKey, '');
  assert.strictEqual(raf.pending(), 1);
  raf.flush();
  assert.strictEqual(map.W, 900);

  map.update({ x: 9 });
  assert.strictEqual(raf.pending(), 1);
  map.stop();
  assert.strictEqual(raf.pending(), 0, 'stop must cancel a queued frame');
  map.start();
  assert.strictEqual(raf.pending(), 1, 'restart must create one fresh frame');
}

function testFractionalDevicePixelRatioDoesNotInvalidateEveryDirtyFrame() {
  const { map, canvas } = createMap({}, { devicePixelRatio: 1.25 });
  canvas.clientWidth = 801;
  canvas.clientHeight = 601;
  map._resize();
  assert.strictEqual(canvas.width, Math.round(801 * 1.25));
  assert.strictEqual(canvas.height, Math.round(601 * 1.25));

  const transform = { shouldSurvive: true };
  map._tf = transform;
  map._fogCacheKey = 'cached-for-current-viewport';
  map._resize();

  assert.strictEqual(map._tf, transform,
    'an unchanged fractional-DPR viewport must not invalidate transforms');
  assert.strictEqual(map._fogCacheKey, 'cached-for-current-viewport',
    'an unchanged fractional-DPR viewport must not rebuild fog caches');
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

function testPersistentSlamWallsRenderSeparatelyFromLiveCostmap() {
  const { map, ctx } = createMap();
  map.update({
    occupancy_map: {
      points: [[1.0, 2.0], [1.1, 2.0]],
      resolution: 0.1,
    },
  });

  map._draw();

  assert.strictEqual(map.slam.wallPoints.length, 2);
  assert(
    ctx.ops.some(op => op.type === 'fillRect'
      && op.fillStyle === 'rgba(207,216,220,0.82)'),
    'persistent SLAM walls should remain visible outside the local costmap',
  );
}

function testRoomSearchCoverageAndViewpointsRenderOnMap() {
  const { map, ctx } = createMap();
  map.update({
    room_search: {
      phase: 'DONE',
      room: '客厅',
      room_area: { origin_x: 1, origin_y: 2, width: 4, height: 3, spacing: 1 },
      candidate_viewpoints: [{ x: 2, y: 3 }, { x: 4, y: 4 }],
      visited_viewpoints: [{ x: 2, y: 3 }],
      observed_cells: [{ x: 1, y: 2 }, { x: 2, y: 2 }],
      coverage_ratio: 0.5,
      visual_coverage_ratio: 0.99,
      explored_ratio: 0.24,
      bounded_explored_ratio: 0.98,
      completion_reason: 'motion_trapped',
      completion_status: 'incomplete',
      global_search: {
        explainable_coverage_ratio: 0.96,
        traversable_opening_count: 1,
        completion_eligible: false,
      },
      coverage_threshold: 0.9,
      visual_range_m: 2.5,
    },
  });
  map._draw();

  assert.strictEqual(map.slam.roomSearch.coverageRatio, 0.5);
  assert.strictEqual(map.slam.roomSearch.visualCoverageRatio, 0.99);
  assert.strictEqual(map.slam.roomSearch.exploredRatio, 0.24);
  assert.strictEqual(map.slam.roomSearch.explainableCoverageRatio, 0.96);
  assert.strictEqual(map.slam.roomSearch.closureConfirmed, false);
  assert.strictEqual(map.slam.roomSearch.completionReason, 'motion_trapped');
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
    ctx.ops.some(op => op.type === 'fillText'
      && op.text.includes('未完成')
      && !op.text.includes('DONE')
      && op.text.includes('视觉 99%')
      && op.text.includes('全局 24%')
      && op.text.includes('闭合未确认')),
    'expected distinct visual/global/closure label',
  );
}

function testFrontierCoverageRendersWithoutCalibratedRoomRectangle() {
  const { map, ctx } = createMap();
  map.update({
    room_search: {
      phase: 'FRONTIER_DETECT',
      room: '__frontier__',
      observed_cells: [{ x: 0.25, y: 0.25 }, { x: 0.75, y: 0.25 }],
      coverage_cell_size_m: 0.5,
      coverage_ratio: 0.2,
      adaptive_step_m: 6.0,
    },
  });

  map._draw();

  assert(
    ctx.ops.some(op => op.type === 'drawImage' && op.image === map._fogCanvas),
    'expected the cached search fog layer to be composited once per frame',
  );
  const revealedCells = map._fogCtx.ops.filter(op => op.type === 'clearRect').slice(1);
  assert.strictEqual(revealedCells.length, 2,
    `expected two observed cells to be revealed, got ${JSON.stringify(revealedCells)}`);
  assert.strictEqual(map.slam.roomSearch.coverageCellSizeM, 0.5);
  assert.strictEqual(map.slam.roomSearch.adaptiveStepM, 6.0);
}

function testSearchCoverageUsesFogMaskWithObservedCellsCutOut() {
  const { map, ctx } = createMap();
  map.update({
    room_search: {
      phase: 'ACTIVE_SEARCH',
      observed_cells: [{ x: 0.25, y: 0.25 }, { x: 0.75, y: 0.25 }],
      coverage_cell_size_m: 0.5,
      coverage_ratio: 0.2,
    },
  });

  map._draw();

  const fog = map._fogCtx.ops.find(
    op => op.type === 'fillRect' && op.fillStyle === 'rgba(3,10,18,0.86)',
  );
  assert(fog, 'expected a high-contrast cached search fog overlay');
  assert(
    map._fogCtx.ops.filter(op => op.type === 'clearRect').length >= 3,
    'expected one canvas clear plus two observed-cell cut-outs',
  );
  assert.strictEqual(map._fogBuildCount, 1);

  map._draw();
  assert.strictEqual(map._fogBuildCount, 1,
    'unchanged fog must be reused instead of rebuilding thousands of cells at 60 FPS');

  map.update({ room_search: { observed_cells: [
    { x: 0.25, y: 0.25 }, { x: 0.75, y: 0.25 }, { x: 1.25, y: 0.25 },
  ] } });
  map._draw();
  assert.strictEqual(map._fogBuildCount, 2, 'new coverage must invalidate the fog cache');
}

function testPartialSearchProgressDoesNotEraseAccumulatedFogCoverage() {
  const { map } = createMap();
  map.update({
    room_search: {
      phase: 'FRONTIER_DETECT',
      observed_cells: [{ x: 0.25, y: 0.25 }],
      coverage_cell_size_m: 0.5,
      coverage_ratio: 0.2,
    },
  });
  map.update({ room_search: { phase: 'NAVIGATING', current_wp: 2 } });

  assert.strictEqual(map.slam.roomSearch.observedCells.length, 1,
    'phase-only progress must retain cumulative observed cells');
  assert.strictEqual(map.slam.roomSearch.observedCells[0].x, 0.25);
  assert.strictEqual(map.slam.roomSearch.observedCells[0].y, 0.25);
  assert.strictEqual(map.slam.roomSearch.coverageRatio, 0.2);
  assert.strictEqual(map.slam.roomSearch.coverageCellSizeM, 0.5);
}

function assertClose(actual, expected, message) {
  assert(Math.abs(actual - expected) < 1e-9,
    `${message}: expected ${expected}, got ${actual}`);
}

function testCameraFrustumGeometryUsesRobotYawAndExactCalibration() {
  const { map } = createMap();
  map.update({
    x: 0, y: 0, yaw: 0,
    room_search: {
      camera_hfov_deg: 60,
      camera_yaw_offset_deg: 0,
      visual_range_m: 2,
    },
  });

  const straight = map._cameraFrustumGeometry();
  assert(straight, 'complete finite calibration should produce frustum geometry');
  assertClose(straight.centerBearingDeg, 0, 'straight camera center bearing');
  assertClose(straight.leftEndpoint.x, Math.sqrt(3), 'left endpoint x');
  assertClose(straight.leftEndpoint.y, 1, 'left endpoint y');
  assertClose(straight.rightEndpoint.x, Math.sqrt(3), 'right endpoint x');
  assertClose(straight.rightEndpoint.y, -1, 'right endpoint y');

  map.update({
    yaw: Math.PI / 2,
    room_search: { camera_yaw_offset_deg: -10 },
  });
  const offset = map._cameraFrustumGeometry();
  assert(offset, 'partial progress must preserve the remaining valid calibration');
  assertClose(offset.centerBearingDeg, 80, 'camera yaw offset must rotate from robot yaw');
}

function testCameraFrustumHidesRatherThanGuessingInvalidCalibration() {
  const missing = createMap().map;
  missing.update({ room_search: {
    phase: 'ACTIVE_SEARCH', camera_hfov_deg: 60, visual_range_m: 2,
  } });
  assert.strictEqual(missing._cameraFrustumGeometry(), null,
    'a missing yaw offset must hide the geometric cone instead of guessing zero');

  const invalid = createMap().map;
  invalid.update({ room_search: {
    camera_hfov_deg: 'not-a-number',
    camera_yaw_offset_deg: 0,
    visual_range_m: 2,
  } });
  assert.strictEqual(invalid._cameraFrustumGeometry(), null,
    'invalid calibration must hide the geometric cone');
}

function testCameraFrustumUsesObstacleClippedCellsAndCorrectLayerOrder() {
  const { map, ctx } = createMap();
  map.update({
    x: 0, y: 0, yaw: 0,
    occupancy_map: { points: [[0.5, 0]], resolution: 0.1 },
    costmap: { w: 1, h: 1, res: 0.1, ox: 0, oy: 0, vals: [100] },
    room_search: {
      phase: 'ACTIVE_SEARCH',
      camera_hfov_deg: 77.4,
      camera_yaw_offset_deg: 0,
      visual_range_m: 2,
      coverage_cell_size_m: 0.5,
      visible_cells: [{ x: 0.25, y: 0.25 }, { x: 0.75, y: 0.25 }],
      observed_cells: [{ x: 0.25, y: 0.25 }, { x: 0.75, y: 0.25 }],
    },
  });

  map._draw();

  assert.strictEqual(map.slam.roomSearch.visibleCells.length, 2);
  const wallIndex = ctx.ops.findIndex(op => op.type === 'fillRect'
    && op.fillStyle === 'rgba(207,216,220,0.82)');
  const cellIndexes = ctx.ops.map((op, index) => ({ op, index }))
    .filter(({ op }) => op.type === 'fillRect'
      && op.fillStyle === 'rgba(0,229,255,0.18)')
    .map(({ index }) => index);
  const fogIndex = ctx.ops.findIndex(op => op.type === 'drawImage'
    && op.image === map._fogCanvas);
  const costmapIndex = ctx.ops.findIndex(op => op.type === 'drawImage'
    && op.image === map._cmCanvas);
  assert.strictEqual(cellIndexes.length, 2,
    'the current obstacle-clipped visibility buckets must be filled exactly');
  assert(fogIndex >= 0 && fogIndex < cellIndexes[0],
    'search fog must be drawn below the current frustum overlay');
  assert(wallIndex > cellIndexes[cellIndexes.length - 1],
    'persistent walls must remain visible above the frustum overlay');
  assert(costmapIndex > cellIndexes[cellIndexes.length - 1],
    'the live costmap must remain visible above the frustum overlay');
  assert(ctx.ops.some(op => op.type === 'arc'), 'frustum needs a range arc');
  assert(ctx.ops.some(op => op.type === 'fillText' && op.text === 'C13 77.4\u00b0'),
    'frustum label must identify the calibrated C13 horizontal field of view');
}

function testCameraFrustumFallsBackToWedgeOnlyWithCompleteCalibration() {
  const { map, ctx } = createMap();
  map.update({
    x: 0, y: 0, yaw: 0,
    room_search: {
      camera_hfov_deg: 60,
      camera_yaw_offset_deg: 0,
      visual_range_m: 2,
    },
  });

  map._draw();

  assert(ctx.ops.some(op => op.type === 'fill'
    && op.fillStyle === 'rgba(0,229,255,0.12)'),
  'without exact visible cells, complete calibration should draw a geometric wedge');
}

function testCameraFrustumExplicitEmptyVisibilityNeverFallsBackToWedgeFill() {
  const { map, ctx } = createMap();
  map.update({
    x: 0, y: 0, yaw: 0,
    room_search: {
      phase: 'ACTIVE_SEARCH',
      camera_hfov_deg: 60,
      camera_yaw_offset_deg: 0,
      visual_range_m: 2,
      visible_cells: [],
    },
  });
  map.update({ room_search: { phase: 'NAVIGATING' } });

  map._draw();

  assert.strictEqual(map.slam.roomSearch.visibleCellsAvailable, true,
    'partial progress must retain that the backend supplied exact visibility');
  assert(!ctx.ops.some(op => op.type === 'fill'
    && op.fillStyle === 'rgba(0,229,255,0.12)'),
  'an explicitly empty exact visibility set must not become a guessed wedge fill');
  assert(ctx.ops.some(op => op.type === 'arc'),
    'valid calibration may still draw the calibrated frustum outline');
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

function testStatusPollingRefreshesRoomSearchWhenWebSocketStateIsDropped() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const helper = panel.indexOf('function applyRoomNavigationState(roomNav)');
  assert(helper >= 0, 'room navigation state should have one shared renderer');
  const poll = panel.indexOf('function applyStatusSnapshot(snapshot');
  assert(poll >= 0, 'shared status rehydration helper missing');
  assert(
    panel.indexOf('applyRoomNavigationState(snapshot.room_nav)', poll) > poll,
    'HTTP status polling must refresh room-search mask when WebSocket updates are dropped',
  );
  assert(panel.includes('panelStatusPoller.start(0)'), 'self-scheduling status poll must start');
  assert(!panel.includes("setInterval(() => {\n  const statusEpoch"),
    'status polling must not use an overlapping fixed interval');
}

function testPanelFiltersDetectionResultsBelowEightyPercent() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  assert(
    panel.includes('const DETECTION_MIN_CONFIDENCE = 0.8;'),
    'panel detection threshold must be 0.8',
  );
  assert(
    panel.includes('function filterDetectionResults(detections)'),
    'panel should share one confidence filter across result views',
  );
  assert(
    panel.includes('updateMissionTargetMarkers(markers)'),
    'map markers must exclude low-confidence detections',
  );
}

function testPanelForwardsGenericTargetMarkerEventsIntoMap() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const branch = panel.indexOf("data.type === 'target_markers'");
  assert(branch >= 0, 'target_markers WebSocket branch missing');
  assert(
    panel.indexOf('updateMissionTargetMarkers(markers)', branch) > branch,
    'generic target markers must be forwarded to the map renderer',
  );
  const reportBranch = panel.indexOf("data.type === 'mission_report'");
  assert(
    panel.indexOf('target_markers: detections', reportBranch) > reportBranch,
    'mission report detections must be forwarded as generic target markers',
  );
}

function testPanelPreservesMissionTargetsAndExplainsSearchFailure() {
  const panel = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  assert(
    panel.includes('function preferredDetectionResults('),
    'mission results need a fallback when the current frame is empty',
  );
  assert(
    panel.includes('function updateMissionTargetMarkers('),
    'mission target markers need state separate from instantaneous detections',
  );
  assert(
    panel.includes("motion_trapped: '运动受困'"),
    'search failure must explain motion_trapped instead of generic nav2_aborted',
  );
}

testWorldScanPointsAreDrawnWithoutSecondTransform();
testDirtySchedulerStaysIdleAndCoalescesUpdates();
testDirtySchedulerInvalidatesCachesInteractionAndResize();
testFractionalDevicePixelRatioDoesNotInvalidateEveryDirtyFrame();
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
testPersistentSlamWallsRenderSeparatelyFromLiveCostmap();
testRoomSearchCoverageAndViewpointsRenderOnMap();
testFrontierCoverageRendersWithoutCalibratedRoomRectangle();
testSearchCoverageUsesFogMaskWithObservedCellsCutOut();
testPartialSearchProgressDoesNotEraseAccumulatedFogCoverage();
testCameraFrustumGeometryUsesRobotYawAndExactCalibration();
testCameraFrustumHidesRatherThanGuessingInvalidCalibration();
testCameraFrustumUsesObstacleClippedCellsAndCorrectLayerOrder();
testCameraFrustumFallsBackToWedgeOnlyWithCompleteCalibration();
testCameraFrustumExplicitEmptyVisibilityNeverFallsBackToWedgeFill();
testPanelForwardsSearchRoomProgressIntoMap();
testStatusPollingRefreshesRoomSearchWhenWebSocketStateIsDropped();
testPanelFiltersDetectionResultsBelowEightyPercent();
testPanelForwardsGenericTargetMarkerEventsIntoMap();
testPanelPreservesMissionTargetsAndExplainsSearchFailure();
console.log('map contract tests passed');
