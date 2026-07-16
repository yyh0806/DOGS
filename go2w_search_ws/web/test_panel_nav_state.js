const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((res, rej) => { resolve = res; reject = rej; });
  return { promise, resolve, reject };
}

function response(status, payload) {
  return {
    ok: status >= 200 && status < 300,
    status,
    async json() { return payload; },
  };
}

function loadNavigationStateMachine() {
  const html = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const start = html.indexOf('let map = null;');
  const end = html.indexOf('// ---- 地图初始化 ----', start);
  assert(start >= 0 && end > start, 'navigation state-machine block not found');

  let fetchImpl = () => Promise.reject(new Error('unexpected fetch'));
  const elements = new Map();
  const context = {
    console,
    AbortController,
    setTimeout,
    clearTimeout,
    fetch: (...args) => fetchImpl(...args),
    controlFetch: (...args) => fetchImpl(...args),
    document: {
      getElementById(id) {
        if (!elements.has(id)) elements.set(id, { className: '', textContent: '' });
        return elements.get(id);
      },
    },
  };
  vm.createContext(context);
  const exports = `
    this.__nav = {
      applyNavState,
      applyNavStateForEpoch,
      applyNavigationReadiness,
      sendNavGoal,
      resetDriveFault,
      resetNavOrderingForNewConnection,
      personMarkerEvidenceLabel,
      setMap(value) { map = value; },
      setFetch(value) { __setFetch(value); },
      state() {
        return { navRequestSerial, navRequestPending, lastNavGeneration, lastNavUpdatedMonotonic, navConnectionEpoch, latestNavigationReadiness };
      },
    };
  `;
  context.__setFetch = value => { fetchImpl = value; };
  vm.runInContext(html.slice(start, end) + exports, context, { filename: 'panel-nav-state.js' });
  return { nav: context.__nav, elements };
}

function fakeMap() {
  return {
    slam: { robotX: 0, robotY: 0, robotYaw: 0 },
    goals: [],
    clearCount: 0,
    setNavGoal(goal) { this.goals.push({ ...goal }); return true; },
    clearNavGoal() { this.clearCount += 1; },
  };
}

async function testIdleNullPoseClearsTarget() {
  const { nav } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  const accepted = nav.applyNavState({
    generation: 0,
    status: 'idle',
    x: null,
    y: null,
    yaw: null,
    updated_monotonic: 1,
  }, 'status');
  assert.strictEqual(accepted, true);
  assert.strictEqual(map.clearCount, 1);
  assert.strictEqual(map.goals.length, 0);
}

async function testReplacementDropsOldWsAndReportsNewestHttpFailure() {
  const { nav, elements } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  nav.applyNavigationReadiness({ ready: true, reason: 'ok' });
  nav.applyNavState({ generation: 0, status: 'idle', x: null, y: null, yaw: null, updated_monotonic: 1 }, 'status');

  const requests = [];
  nav.setFetch((url, options) => {
    const item = deferred();
    requests.push({ url, options, ...item });
    return item.promise;
  });

  const first = nav.sendNavGoal({ x: 1, y: 0 });
  const second = nav.sendNavGoal({ x: 2, y: 0 });
  assert.strictEqual(requests.length, 2);
  assert.strictEqual(map.goals.at(-1).x, 2);

  const oldWsAccepted = nav.applyNavState({
    generation: 1,
    status: 'active',
    x: 1,
    y: 0,
    yaw: 0,
    updated_monotonic: 2,
  }, 'ws');
  assert.strictEqual(oldWsAccepted, false);
  assert.strictEqual(map.goals.at(-1).x, 2, 'old goal must not replace the newest local reticle');
  assert.strictEqual(nav.state().navRequestPending, true);

  requests[0].resolve(response(202, {
    ok: true,
    generation: 1,
    goal: { x: 1, y: 0, yaw: 0, frame_id: 'map' },
  }));
  await first;
  assert.strictEqual(nav.state().navRequestPending, true, 'stale HTTP response must not finish request B');

  requests[1].resolve(response(409, { ok: false, msg: '定位不可用' }));
  await second;
  assert.strictEqual(nav.state().navRequestPending, false);
  assert.strictEqual(map.goals.at(-1).x, 2);
  assert.strictEqual(map.goals.at(-1).status, 'failed');
  assert(elements.get('navStatus').textContent.includes('定位不可用'));
}

async function testNewConnectionAllowsBackendGenerationReset() {
  const { nav } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  assert.strictEqual(nav.applyNavState({
    generation: 6, status: 'active', x: 6, y: 0, yaw: 0, updated_monotonic: 10,
  }, 'ws'), true);
  const staleEpoch = nav.state().navConnectionEpoch;
  const currentEpoch = nav.resetNavOrderingForNewConnection();
  assert.strictEqual(nav.applyNavStateForEpoch({
    generation: 6, status: 'active', x: 6, y: 0, yaw: 0, updated_monotonic: 11,
  }, 'status', staleEpoch), false, 'old in-flight status response must be rejected');
  assert.strictEqual(nav.applyNavStateForEpoch({
    generation: 0, status: 'active', x: 0.5, y: 1, yaw: 0.2, updated_monotonic: 1,
  }, 'status', currentEpoch), true);
  assert.strictEqual(map.goals.at(-1).x, 0.5);
  assert.strictEqual(nav.state().lastNavGeneration, 0);
}

async function testAbortErrorBecomesVisibleTimeoutFailure() {
  const { nav, elements } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  nav.applyNavigationReadiness({ ready: true, reason: 'ok' });
  const timeoutError = new Error('aborted');
  timeoutError.name = 'AbortError';
  nav.setFetch(() => Promise.reject(timeoutError));
  await nav.sendNavGoal({ x: 3, y: -1 });
  assert.strictEqual(map.goals.at(-1).status, 'failed');
  assert(elements.get('navStatus').textContent.includes('请求超时'));
}

async function testNavigationReadinessBlocksUnsafeMapClick() {
  const { nav, elements } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  let requests = 0;
  nav.setFetch(() => { requests += 1; return Promise.resolve(response(202, { ok: true })); });

  nav.applyNavigationReadiness({
    ready: false,
    reason: 'localization_stale',
    drive_fault_reset_available: false,
  });
  await nav.sendNavGoal({ x: 1, y: 0 });

  assert.strictEqual(requests, 0);
  assert.strictEqual(map.goals.length, 0);
  assert(elements.get('navStatus').textContent.includes('定位数据已失鲜'));
  assert.strictEqual(elements.get('resetDriveFaultBtn').disabled, true);
}

async function testPureTurnSafetyReasonsAreHumanReadable() {
  const { nav, elements } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  nav.applyNavigationReadiness({
    ready: false,
    reason: 'pure_turn_clearance',
    drive_fault_reset_available: false,
  });

  await nav.sendNavGoal({ x: 1, y: 0 });
  assert(elements.get('navStatus').textContent.includes('转向扫掠范围内有障碍'));

  nav.applyNavState({
    generation: 1,
    status: 'aborted',
    x: 1,
    y: 0,
    yaw: 0,
    reason: 'pure_turn_oscillation',
    updated_monotonic: 2,
  }, 'ws');
  assert(elements.get('navStatus').textContent.includes('左右转向振荡'));
}

async function testDriveFaultResetButtonUsesBackendGuard() {
  const { nav, elements } = loadNavigationStateMachine();
  const requests = [];
  nav.setFetch((url, options) => {
    requests.push({ url, options });
    return Promise.resolve(response(200, { ok: true }));
  });
  nav.applyNavigationReadiness({
    ready: false,
    reason: 'wheel_no_response',
    drive_fault_reset_available: true,
  });

  assert.strictEqual(elements.get('resetDriveFaultBtn').disabled, false);
  await nav.resetDriveFault();

  assert.strictEqual(requests.length, 1);
  assert.strictEqual(requests[0].url, '/api/reset_drive_fault');
  assert.strictEqual(requests[0].options.method, 'POST');
  assert.strictEqual(elements.get('resetDriveFaultBtn').disabled, true);
}

async function testPersonMarkerEvidenceLabelShows3dAndDedupProof() {
  const { nav } = loadNavigationStateMachine();

  const label = nav.personMarkerEvidenceLabel({
    world_x: 1.234,
    world_y: -2.345,
    world_z: 0.678,
    position_dimension: 3,
    height_source: 'mid360_pointcloud',
    observation_count: 3,
    dedup_method: 'appearance_spatial',
  });

  assert(label.includes('map (1.23, -2.35, 0.68)'));
  assert(label.includes('MID360 三维'));
  assert(label.includes('3 次观测'));
  assert(label.includes('外观+位置去重'));
}

(async () => {
  await testIdleNullPoseClearsTarget();
  await testReplacementDropsOldWsAndReportsNewestHttpFailure();
  await testNewConnectionAllowsBackendGenerationReset();
  await testAbortErrorBecomesVisibleTimeoutFailure();
  await testNavigationReadinessBlocksUnsafeMapClick();
  await testPureTurnSafetyReasonsAreHumanReadable();
  await testDriveFaultResetButtonUsesBackendGuard();
  await testPersonMarkerEvidenceLabelShows3dAndDedupProof();
  console.log('panel navigation state tests passed');
})().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
