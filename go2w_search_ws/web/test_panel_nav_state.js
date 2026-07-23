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

async function testParkedButActivatableNavigationMayStart() {
  const { nav } = loadNavigationStateMachine();
  const map = fakeMap();
  nav.setMap(map);
  let requests = 0;
  nav.setFetch(() => {
    requests += 1;
    return Promise.resolve(response(202, {
      ok: true,
      generation: 1,
      goal: { x: 1, y: 0, yaw: 0, frame_id: 'map' },
    }));
  });
  nav.applyNavigationReadiness({
    ready: false,
    activatable: true,
    reason: 'drive_session_parked',
  });

  await nav.sendNavGoal({ x: 1, y: 0 });

  assert.strictEqual(requests, 1, 'an intentionally parked drive session may be activated by navigation');
}

function loadPanelLifecycle(overrides = {}) {
  const html = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const start = html.indexOf('// ---- Panel lifecycle helpers ----');
  const end = html.indexOf('// ---- WebSocket ----', start);
  assert(start >= 0 && end > start, 'panel lifecycle helper block not found');
  const context = {
    console,
    Math,
    Date,
    AbortController,
    setTimeout,
    clearTimeout,
    requestAnimationFrame: callback => setTimeout(callback, 0),
    cancelAnimationFrame: clearTimeout,
    document: { getElementById: () => null },
    ...overrides,
  };
  vm.createContext(context);
  const exports = `
    this.__panelLifecycle = {
      createSocketLifecycle,
      createLatestFrameScheduler,
      createStatusPoller,
      computeWsReconnectDelay,
      applyStatusSnapshot,
      refreshStatusSnapshot,
      markRealtimeStateUpdate,
      wsMessageOverlapsStatus,
      cacheDetectionSnapshot,
    };
  `;
  vm.runInContext(html.slice(start, end) + exports, context, { filename: 'panel-lifecycle.js' });
  context.__panelLifecycle.context = context;
  return context.__panelLifecycle;
}

function loadManualMotionControls() {
  const html = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const start = html.indexOf('const SPEED =');
  const end = html.indexOf('function connect()', start);
  assert(start >= 0 && end > start, 'manual motion control block not found');

  const requests = [];
  const context = {
    console,
    controlFetch(url, options) {
      requests.push({ url, options });
      return Promise.resolve(response(200, { ok: true }));
    },
    setInterval: () => 1,
    clearInterval: () => {},
    clearTimeout: () => {},
  };
  vm.createContext(context);
  const exports = `
    this.__manualMotion = { move, stopMove };
  `;
  vm.runInContext(
    'let _moveInterval = null; let _locateLoopTimer = null;\n' +
      html.slice(start, end) + exports,
    context,
    { filename: 'panel-manual-motion.js' },
  );
  return { manualMotion: context.__manualMotion, requests };
}

function testIdleReleaseEventsCannotParkAutonomousNavigation() {
  const { manualMotion, requests } = loadManualMotionControls();

  manualMotion.stopMove();
  assert.deepStrictEqual(requests, [],
    'blur/leave while manual control is idle must not send manual_stop');

  manualMotion.move(0.4, 0, 0);
  manualMotion.stopMove();
  manualMotion.stopMove();
  assert.deepStrictEqual(requests.map(item => item.url), [
    '/api/move?vx=0.4&vy=0&vyaw=0',
    '/api/manual_stop',
  ], 'an active manual move must still stop exactly once on release');
}

function fakeTimers() {
  let now = 0;
  let serial = 0;
  const timeouts = new Map();
  const intervals = new Map();
  return {
    now: () => now,
    setTimeout(fn, delay) { const id = ++serial; timeouts.set(id, { fn, at: now + delay }); return id; },
    clearTimeout(id) { timeouts.delete(id); },
    setInterval(fn, delay) { const id = ++serial; intervals.set(id, { fn, delay, at: now + delay }); return id; },
    clearInterval(id) { intervals.delete(id); },
    advance(ms) {
      const target = now + ms;
      while (true) {
        const due = [
          ...[...timeouts].map(([id, item]) => ({ id, item, interval: false })),
          ...[...intervals].map(([id, item]) => ({ id, item, interval: true })),
        ].filter(entry => entry.item.at <= target).sort((a, b) => a.item.at - b.item.at)[0];
        if (!due) break;
        now = due.item.at;
        if (due.interval) {
          if (!intervals.has(due.id)) continue;
          due.item.at += due.item.delay;
        } else {
          timeouts.delete(due.id);
        }
        due.item.fn();
      }
      now = target;
    },
    timeoutCount: () => timeouts.size,
  };
}

function fakeWebSocketFactory() {
  const sockets = [];
  class FakeWebSocket {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;
    constructor(url) { this.url = url; this.readyState = 0; this.closeCount = 0; sockets.push(this); }
    open() { this.readyState = 1; this.onopen && this.onopen(); }
    message(data = '{}') { this.onmessage && this.onmessage({ data }); }
    close() { this.closeCount += 1; this.readyState = 3; this.onclose && this.onclose(); }
  }
  return { FakeWebSocket, sockets };
}

async function testSocketWatchdogReconnectAndBackoff() {
  const timers = fakeTimers();
  const { FakeWebSocket, sockets } = fakeWebSocketFactory();
  const lifecycle = loadPanelLifecycle().createSocketLifecycle({
    createSocket: url => new FakeWebSocket(url),
    url: 'ws://test',
    now: timers.now,
    setTimeoutFn: timers.setTimeout,
    clearTimeoutFn: timers.clearTimeout,
    setIntervalFn: timers.setInterval,
    clearIntervalFn: timers.clearInterval,
    random: () => 0.5,
  });

  lifecycle.connect();
  lifecycle.connect();
  assert.strictEqual(sockets.length, 1, 'CONNECTING socket must not be duplicated');
  sockets[0].open();
  lifecycle.connect();
  assert.strictEqual(sockets.length, 1, 'OPEN socket must not be duplicated');
  timers.advance(6000);
  assert.strictEqual(sockets[0].closeCount, 1, 'watchdog closes an OPEN socket after six seconds of silence');
  assert.strictEqual(timers.timeoutCount(), 1, 'close schedules exactly one reconnect');
  sockets[0].onclose();
  assert.strictEqual(timers.timeoutCount(), 1, 'duplicate close cannot schedule another reconnect');
  timers.advance(1000);
  assert.strictEqual(sockets.length, 2);

  assert(loadPanelLifecycle().computeWsReconnectDelay(99, 1) <= 30000, 'backoff is capped');
  sockets[1].open();
  assert.strictEqual(lifecycle.state().attempt, 0, 'successful open resets backoff attempt');
}

async function testStatusPollerNeverOverlaps() {
  const timers = fakeTimers();
  const requests = [];
  const poller = loadPanelLifecycle().createStatusPoller({
    fetchStatus: () => { const item = deferred(); requests.push(item); return item.promise; },
    applySnapshot: () => {},
    getEpoch: () => 1,
    setTimeoutFn: timers.setTimeout,
    clearTimeoutFn: timers.clearTimeout,
    timeoutMs: 10000,
    intervalMs: 3000,
  });
  poller.start(0);
  timers.advance(0);
  assert.strictEqual(requests.length, 1);
  timers.advance(9000);
  assert.strictEqual(requests.length, 1, 'no second poll starts while the first request is in flight');
  requests[0].resolve({ connected: true });
  await new Promise(resolve => setImmediate(resolve));
  timers.advance(2999);
  assert.strictEqual(requests.length, 1);
  timers.advance(1);
  assert.strictEqual(requests.length, 2, 'next poll is scheduled only after prior request settles');
  poller.stop();
}

async function testSlowHttpSnapshotCannotOverwriteNewerRealtimeState() {
  const pending = deferred();
  const applied = [];
  const lifecycle = loadPanelLifecycle({
    navConnectionEpoch: 4,
    fetch: () => pending.promise,
    updateDogStatus: value => applied.push(value),
    updatePoseInfo: () => {},
  });

  const request = lifecycle.refreshStatusSnapshot(4, true);
  lifecycle.markRealtimeStateUpdate();
  pending.resolve({
    ok: true,
    json: async () => ({ connected: false }),
  });
  await request;

  assert.deepStrictEqual(applied, [],
    'a slow HTTP response must not overwrite a newer WS update in the same epoch');

  await lifecycle.refreshStatusSnapshot(4, true);
  assert.deepStrictEqual(applied, [false],
    'HTTP state still applies when no newer realtime update arrived');
}

function testOnlyStatusDomainWsEventsInvalidateHttpSnapshots() {
  const lifecycle = loadPanelLifecycle();
  for (const type of ['status', 'tasks', 'nav_goal', 'detections', 'locate',
    'target_markers', 'person_markers', 'search_room', 'mission_report']) {
    assert.strictEqual(lifecycle.wsMessageOverlapsStatus(type), true,
      `${type} must supersede an older HTTP status snapshot`);
  }
  for (const type of ['gimbal', 'lidar', 'slam', 'costmap',
    'costmap_global', 'occupancy_map', 'plan', 'vlm']) {
    assert.strictEqual(lifecycle.wsMessageOverlapsStatus(type), false,
      `${type} must not disable HTTP status recovery`);
  }
}

async function testNewestOverlayGenerationWins() {
  const callbacks = [];
  const scheduler = loadPanelLifecycle().createLatestFrameScheduler({
    requestFrame: callback => { callbacks.push(callback); return callbacks.length; },
    cancelFrame: () => {},
  });
  const rendered = [];
  const first = scheduler.schedule('c13_vis', { id: 1 }, value => rendered.push(value.id));
  const second = scheduler.schedule('c13_vis', { id: 2 }, value => rendered.push(value.id));
  assert.strictEqual(callbacks.length, 1, 'only one RAF may be queued per source');
  assert(second > first);
  callbacks.shift()();
  assert.deepStrictEqual(rendered, [2], 'queued RAF paints only the newest payload');
  assert.strictEqual(scheduler.isCurrent('c13_vis', first), false, 'old image onload generation is stale');
  assert.strictEqual(scheduler.isCurrent('c13_vis', second), true);
}

function testHttpDetectionCacheInvalidatesQueuedOldOverlay() {
  const callbacks = [];
  const rendered = [];
  const lifecycle = loadPanelLifecycle({
    latestDetections: [],
    latestDetectionsBySource: {},
    DETECTION_VIEWS: {
      c13_vis: { overlayId: 'detectOverlay' },
      c13_ir: { overlayId: 'irDetectOverlay' },
    },
    filterDetectionResults: values => values || [],
    updateDetectionBadge: () => {},
    document: { getElementById: () => ({ innerHTML: 'old-box' }) },
  });
  const scheduler = lifecycle.createLatestFrameScheduler({
    requestFrame: callback => { callbacks.push(callback); return callbacks.length; },
    // Simulate a throttled/background RAF which is already dispatchable and
    // cannot be physically removed by cancelAnimationFrame.
    cancelFrame: () => {},
  });
  lifecycle.context.overlayFrameScheduler = scheduler;
  scheduler.schedule('detection:c13_vis', { id: 'old' }, value => rendered.push(value.id));

  lifecycle.cacheDetectionSnapshot([
    { id: 'new', source: 'c13_vis', confidence: 0.95 },
  ]);
  callbacks.shift()();

  assert.deepStrictEqual(rendered, [],
    'an already queued old detection RAF must stay invalid after HTTP recovery');
  assert.strictEqual(lifecycle.context.latestDetections[0].id, 'new');
}

async function testStatusSnapshotRehydratesAllStateAndRejectsOldEpoch() {
  const calls = [];
  const elements = new Map();
  const map = { update(value) { calls.push(['map', value]); } };
  const lifecycle = loadPanelLifecycle({
    navConnectionEpoch: 4,
    map,
    latestDetections: [],
    latestDetectionsBySource: {},
    DETECTION_VIEWS: {
      c13_vis: { overlayId: 'detectOverlay' },
      c13_ir: { overlayId: 'irDetectOverlay' },
    },
    overlayFrameScheduler: { cancel: source => calls.push(['overlay-cancel', source]) },
    filterDetectionResults: values => values || [],
    updateDogStatus: value => calls.push(['dog-status', value]),
    updateDogState: value => calls.push(['dog-state', value]),
    updateTasks: value => calls.push(['tasks', value]),
    applyNavigationReadiness: value => calls.push(['navigation', value]),
    applyNavStateForEpoch: (value, source, epoch) => calls.push(['point-nav', value, source, epoch]),
    applyRoomNavigationState: value => calls.push(['room-nav', value]),
    updatePoseInfo: (localization, odometry) => calls.push(['pose', localization, odometry]),
    updateDetList: value => calls.push(['detections', value]),
    updateDetectionBadge: () => calls.push(['detection-badge']),
    renderDetectionStreams: value => calls.push(['overlay-render', value]),
    document: {
      getElementById(id) {
        if (!elements.has(id)) elements.set(id, { className: '', title: '', dataset: {}, innerHTML: '' });
        return elements.get(id);
      },
    },
  });
  const snapshot = {
    connected: true,
    dog_state: 'STOOD',
    tasks: { pending: [] },
    navigation: { ready: true },
    point_nav: { generation: 2, status: 'active' },
    room_nav: { phase: 'ACTIVE_SEARCH' },
    localization: { healthy: true, x: 1, y: 2, yaw: 0 },
    odometry: { healthy: true, x: 1, y: 2, yaw: 0 },
    det_list: [{ class: 'person', confidence: 0.9 }],
    target_markers: [{ class: 'person', confidence: 0.9, x: 1, y: 2 }],
    services: { nav2: { state: 'active', color: 'green', label: 'Nav2' } },
    stats: { ws_connected_clients: 1, ws_reliable_depth: 0, ws_stream_replaced: 5 },
  };

  assert.strictEqual(lifecycle.applyStatusSnapshot(snapshot, 3), false);
  assert.strictEqual(calls.length, 0, 'stale connection epoch must not rehydrate the page');
  assert.strictEqual(lifecycle.applyStatusSnapshot(snapshot, 4), true);
  for (const kind of ['dog-status', 'dog-state', 'tasks', 'navigation', 'point-nav', 'room-nav', 'pose', 'detections', 'detection-badge', 'map']) {
    assert(calls.some(call => call[0] === kind), `missing ${kind} rehydration: ${JSON.stringify(calls)}`);
  }
  assert(!calls.some(call => call[0] === 'overlay-render'),
    'HTTP detections have no matching frame and must not paint boxes immediately');
  assert.strictEqual(lifecycle.context.latestDetections.length, 1,
    'HTTP detections must still restore the badge/cache used by the next image');
  assert(elements.get('svcBar').innerHTML.includes('Nav2'));
  assert.strictEqual(elements.get('nxDot').dataset.wsClients, '1');
  assert(elements.get('nxDot').title.includes('stream replacements 5'));
}

function testInlinePanelScriptParsesAsJavaScript() {
  const html = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const inlineScripts = [...html.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/gi)]
    .map(match => match[1].trim())
    .filter(Boolean);
  assert(inlineScripts.length > 0, 'panel inline script missing');
  for (const source of inlineScripts) new vm.Script(source, { filename: 'panel-inline.js' });
}

function testGimbalRenderingWaitsForTheNewImageOnload() {
  const html = fs.readFileSync(path.join(__dirname, 'static', 'panel.html'), 'utf8');
  const branch = html.split("data.type === 'gimbal'", 2)[1]
    .split("data.type === 'detections'", 1)[0];
  assert(branch.includes("setStreamImage('c13_vis'"));
  assert(branch.includes("setStreamImage('c13_ir'"));
  assert(!branch.includes('scheduleDetectionOverlay'),
    'gimbal handler must wait for image onload before painting cached detections');
  assert(!branch.includes('scheduleLocateOverlay'),
    'gimbal handler must wait for image onload before painting cached locate boxes');
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
  await testParkedButActivatableNavigationMayStart();
  await testPureTurnSafetyReasonsAreHumanReadable();
  await testDriveFaultResetButtonUsesBackendGuard();
  await testPersonMarkerEvidenceLabelShows3dAndDedupProof();
  testIdleReleaseEventsCannotParkAutonomousNavigation();
  await testSocketWatchdogReconnectAndBackoff();
  await testStatusPollerNeverOverlaps();
  await testSlowHttpSnapshotCannotOverwriteNewerRealtimeState();
  testOnlyStatusDomainWsEventsInvalidateHttpSnapshots();
  await testNewestOverlayGenerationWins();
  testHttpDetectionCacheInvalidatesQueuedOldOverlay();
  await testStatusSnapshotRehydratesAllStateAndRejectsOldEpoch();
  testInlinePanelScriptParsesAsJavaScript();
  testGimbalRenderingWaitsForTheNewImageOnload();
  console.log('panel navigation state tests passed');
})().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
