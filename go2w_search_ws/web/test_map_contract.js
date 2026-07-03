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
  strokeRect(x, y, w, h) { this.ops.push({ type: 'strokeRect', x, y, w, h }); }
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
  }
  getContext() { return this._ctx; }
  addEventListener() {}
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

function createMap() {
  const ctx = new FakeContext();
  const canvas = new FakeCanvas(ctx);
  const Go2WMap = loadMapClass();
  const map = new Go2WMap(canvas);
  map._resize();
  return { map, ctx };
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

function testLocalLidarPointsAccumulateIntoObstacleMap() {
  const { map } = createMap();
  map.update({ x: 5, y: -5, yaw: Math.PI / 2, trail: [[5, -5]], scan: [], map: [] });
  assert.strictEqual(typeof map.addLocalObstaclePoints, 'function');

  map.addLocalObstaclePoints([[2, 0]]);

  const hasPoint = map.slam.lidarMapPoints.some(([x, y]) => near(x, 5) && near(y, -3));
  assert(hasPoint, `expected local MID360 point [2,0] to map to world point [5,-3], got ${JSON.stringify(map.slam.lidarMapPoints)}`);
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
      { x: 24, y: 0, position_quality: 'range_lidar' },
      { world_x: -18, world_y: 0, position_quality: 'bearing_only' },
    ],
  });

  map._draw();

  const labels = ctx.ops
    .filter(op => op.type === 'fillText')
    .map(op => op.text);
  assert(labels.includes('人1'), `expected person marker label 人1, got ${JSON.stringify(labels)}`);
  assert(labels.includes('人2'), `expected person marker label 人2, got ${JSON.stringify(labels)}`);
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

testWorldScanPointsAreDrawnWithoutSecondTransform();
testTransformRecomputesAfterSlamUpdate();
testLocalLidarPointsAccumulateIntoObstacleMap();
testPersonMarkersRenderWithLabelsAndQualityStyles();
console.log('map contract tests passed');
