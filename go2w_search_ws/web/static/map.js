/**
 * Go2W 地图渲染模块 (Canvas 俯视图)
 *
 * 职责:
 *   1. 消费 WebSocket 推来的 slam 数据 (狗位姿/轨迹/地图/航点/检测点)
 *   2. 在 Canvas 上绘制: 网格/障碍点/扫描线/路径/轨迹/检测标记/狗箭头/起点/距离圈
 *   3. 鼠标拖框选搜索区域 → 回调返回世界坐标矩形
 *
 * 渲染逻辑源自 web/static/index.html 的 drawSlam(), 抽成独立模块 + 增强选区。
 *
 * 数据契约 (slam 消息, 对应 panel.py 的 WS 推送):
 *   { x, y, yaw,           // 狗当前位姿 (世界坐标, 米/弧度)
 *     trail: [[x,y],...],   // 已走轨迹
 *     map: [[x,y],...],     // 障碍栅格点
 *     scan: [[x,y],...],    // 本帧激光扫描点 (世界坐标)
 *     detections: [{x,y,class}],  // 检测目标位置
 *     waypoints: [{x,y}],   // 规划航点
 *     currentWP: int }
 */
class Go2WMap {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onSelectRegion = opts.onSelectRegion || null; // 选区回调 (worldRect) => {}
    this.onSelectGoal = opts.onSelectGoal || null;     // 点导航回调 ({x,y,frame_id}) => {}

    // 地图状态
    this.slam = {
      robotX: 0, robotY: 0, robotYaw: 0,
      trail: [], mapPoints: [], lidarMapPoints: [], wallPoints: [], scanPoints: [],
      detMarks: [], waypoints: [], currentWP: -1,
      targetMarkers: [],
      personMarkers: [], // compatibility alias for older panel/tests
      roomSearch: {
        missionId: '', phase: '', room: '', roomArea: null,
        candidateViewpoints: [], visitedViewpoints: [], observedCells: [], visibleCells: [],
        visibleCellsAvailable: false,
        coverageRatio: 0, coverageThreshold: 0.9, visualRangeM: 0,
        cameraHfovDeg: null, cameraYawOffsetDeg: null,
        coverageCellSizeM: 0.5, adaptiveStepM: 0,
        sceneComplexity: 1, forwardClearanceM: 0,
      },
      slamSource: '',
      costmap: null,
      costmapLocal: null,    // 2026-07-18 DWB 避障用 local_costmap (前端默认显示)
      costmapGlobal: null,   // 2026-07-18 Navfn 规划用 global_costmap (切换看 ghost 对比)
      plan: null,            // 2026-07-18 nav2 /plan 规划路线 {pts:[[x,y],...]}
    };
    this._cmDirty = false; this._cmCanvas = null; this._cmCtx = null;
    // The visibility mask may contain thousands of cells.  Build it only when
    // coverage or the world-to-screen transform changes; the 60 FPS loop then
    // composites a single cached bitmap.
    this._fogCanvas = null; this._fogCtx = null; this._fogCacheKey = '';
    this._fogCoverageRevision = 0; this._fogBuildCount = 0;
    this._showGlobalCostmap = false;  // 2026-07-18 切换 local(避障)/global(规划) costmap 显示
    this._lidarCells = new Map();
    // 用户选区 (世界坐标), 由 panel.html 设置 (表单输入时也同步显示)
    this.searchRegion = null; // {x, y, w, h}
    this.navGoal = null;      // {x, y, yaw, frame_id, generation, status, ...}

    // 鼠标拖框状态
    this._dragging = false;
    this._dragStart = null;   // 屏幕坐标
    this._dragCur = null;
    this._dragTransform = null;
    this._bindMouse();

    // 渲染循环
    // Draw only after a mutation; high-rate WS updates coalesce into one frame.
    this._running = false;
    this._rafId = null;
    this._frameGeneration = 0;
    this._inFrame = false;
    this._dirty = true;
    this._handleResize = () => {
      this._invalidateViewport();
      this._markDirty();
    };
    if (window && typeof window.addEventListener === 'function') {
      window.addEventListener('resize', this._handleResize);
    }
  }

  /** 更新 slam 数据 (来自 WS 的 type=slam 消息) */
  update(data) {
    if (data.x !== undefined) this.slam.robotX = data.x;
    if (data.y !== undefined) this.slam.robotY = data.y;
    if (data.yaw !== undefined) this.slam.robotYaw = data.yaw;
    if (data.trail) this.slam.trail = data.trail;
    if (data.detections) this.slam.detMarks = data.detections;
    if (data.target_markers !== undefined) {
      this.slam.targetMarkers = data.target_markers;
      this.slam.personMarkers = this.slam.targetMarkers;
    } else if (data.person_markers !== undefined) {
      this.slam.targetMarkers = data.person_markers;
      this.slam.personMarkers = this.slam.targetMarkers;
    }
    if (data.room_search !== undefined) this._updateRoomSearch(data.room_search);
    if (data.waypoints && data.waypoints.length) this.slam.waypoints = data.waypoints;
    if (data.currentWP !== undefined) this.slam.currentWP = data.currentWP;
    if (data.map) {
      this.slam.mapPoints = data.map.length > 2000 ? data.map.slice(-2000) : data.map;
    }
    if (data.occupancy_map !== undefined) {
      const occupancy = data.occupancy_map || {};
      this.slam.wallPoints = Array.isArray(occupancy.points)
        ? occupancy.points.slice(-5000) : [];
      this.slam.wallResolution = Number(occupancy.resolution) || 0.05;
      this._fogCacheKey = '';
    }
    if (data.scan) this.slam.scanPoints = data.scan;
    if (data.slam_source !== undefined) this.slam.slamSource = data.slam_source;
    if (data.costmap) {
      this.slam.costmapLocal = data.costmap;
      if (!this._showGlobalCostmap) { this.slam.costmap = data.costmap; this._cmDirty = true; }
    }
    if (data.costmap_global) {
      this.slam.costmapGlobal = data.costmap_global;
      if (this._showGlobalCostmap) { this.slam.costmap = data.costmap_global; this._cmDirty = true; }
    }
    if (data.plan) this.slam.plan = data.plan;
    this._tf = null;
    this._markDirty();
  }

  /** 切换前端显示的 costmap: false=local(避障,默认) / true=global(规划) */
  setShowGlobalCostmap(v) {
    this._showGlobalCostmap = !!v;
    const src = this._showGlobalCostmap ? this.slam.costmapGlobal : this.slam.costmapLocal;
    if (src) { this.slam.costmap = src; this._cmDirty = true; this._tf = null; }
    this._markDirty();
  }

  _updateRoomSearch(progress) {
    const src = progress && typeof progress === 'object' ? progress : {};
    const has = key => Object.prototype.hasOwnProperty.call(src, key);
    if (has('observed_cells') || has('room_area') || has('coverage_cell_size_m')) {
      this._fogCoverageRevision += 1;
    }
    const previous = this.slam.roomSearch || {};
    const incomingMissionId = has('mission_id') ? String(src.mission_id || '') : '';
    const missionChanged = !!incomingMissionId
      && incomingMissionId !== String(previous.missionId || '');
    const base = missionChanged ? {
      missionId: incomingMissionId,
      phase: '', room: '', roomArea: null,
      candidateViewpoints: [], visitedViewpoints: [], observedCells: [], visibleCells: [],
      visibleCellsAvailable: false,
      coverageRatio: 0, coverageThreshold: 0.9, visualRangeM: 0,
      cameraHfovDeg: null, cameraYawOffsetDeg: null,
      coverageCellSizeM: 0.5, adaptiveStepM: 0,
      sceneComplexity: 1, forwardClearanceM: 0,
    } : previous;
    const finitePointList = value => Array.isArray(value)
      ? value.map(point => ({ x: Number(point && point.x), y: Number(point && point.y) }))
        .filter(point => Number.isFinite(point.x) && Number.isFinite(point.y))
      : [];
    const area = has('room_area') && src.room_area && typeof src.room_area === 'object'
      ? {
          origin_x: Number(src.room_area.origin_x),
          origin_y: Number(src.room_area.origin_y),
          width: Number(src.room_area.width),
          height: Number(src.room_area.height),
          spacing: Number(src.room_area.spacing || 1),
        }
      : null;
    const parsedArea = area && Number.isFinite(area.origin_x) && Number.isFinite(area.origin_y)
      && Number.isFinite(area.width) && Number.isFinite(area.height)
      && area.width > 0 && area.height > 0 ? area : null;
    const finiteOr = (value, fallback) => Number.isFinite(Number(value)) ? Number(value) : fallback;
    const finiteCalibration = value => {
      if (value === null || value === undefined || value === '') return null;
      const parsed = Number(value);
      return Number.isFinite(parsed) ? parsed : null;
    };
    this.slam.roomSearch = {
      missionId: incomingMissionId || String(base.missionId || ''),
      phase: has('phase') ? String(src.phase || '') : String(base.phase || ''),
      room: has('room') ? String(src.room || '') : String(base.room || ''),
      roomArea: has('room_area') ? parsedArea : (base.roomArea || null),
      candidateViewpoints: has('candidate_viewpoints')
        ? finitePointList(src.candidate_viewpoints) : (base.candidateViewpoints || []),
      visitedViewpoints: has('visited_viewpoints')
        ? finitePointList(src.visited_viewpoints) : (base.visitedViewpoints || []),
      observedCells: has('observed_cells')
        ? finitePointList(src.observed_cells) : (base.observedCells || []),
      visibleCells: has('visible_cells')
        ? finitePointList(src.visible_cells) : (base.visibleCells || []),
      visibleCellsAvailable: has('visible_cells')
        ? true : base.visibleCellsAvailable === true,
      coverageRatio: Math.max(0, Math.min(1, has('coverage_ratio')
        ? finiteOr(src.coverage_ratio, 0) : finiteOr(base.coverageRatio, 0))),
      coverageThreshold: Math.max(0, Math.min(1, has('coverage_threshold')
        ? finiteOr(src.coverage_threshold, 0.9) : finiteOr(base.coverageThreshold, 0.9))),
      visualRangeM: Math.max(0, has('visual_range_m')
        ? finiteOr(src.visual_range_m, 0) : finiteOr(base.visualRangeM, 0)),
      cameraHfovDeg: has('camera_hfov_deg')
        ? finiteCalibration(src.camera_hfov_deg)
        : finiteCalibration(base.cameraHfovDeg),
      cameraYawOffsetDeg: has('camera_yaw_offset_deg')
        ? finiteCalibration(src.camera_yaw_offset_deg)
        : finiteCalibration(base.cameraYawOffsetDeg),
      coverageCellSizeM: Math.max(0.1, has('coverage_cell_size_m')
        ? finiteOr(src.coverage_cell_size_m, 0.5) : finiteOr(base.coverageCellSizeM, 0.5)),
      adaptiveStepM: Math.max(0, has('adaptive_step_m')
        ? finiteOr(src.adaptive_step_m, 0) : finiteOr(base.adaptiveStepM, 0)),
      sceneComplexity: Math.max(0, Math.min(1, has('scene_complexity')
        ? finiteOr(src.scene_complexity, 1) : finiteOr(base.sceneComplexity, 1))),
      forwardClearanceM: Math.max(0, has('forward_clearance_m')
        ? finiteOr(src.forward_clearance_m, 0) : finiteOr(base.forwardClearanceM, 0)),
    };
  }

  /** 设置/清除搜索区域 (世界坐标 {x,y,w,h}); 表单输入也调这个同步显示 */
  setRegion(region) { this.searchRegion = region; this._tf = null; this._markDirty(); }
  clearRegion() { this.searchRegion = null; this._tf = null; this._markDirty(); }

  /** 设置 Panel/Nav2 当前目标及状态；只接受 map 系有限坐标。 */
  setNavGoal(state) {
    if (state == null) {
      this.navGoal = null;
      this._tf = null;
      this._markDirty();
      return true;
    }
    const nested = state.goal && typeof state.goal === 'object' ? state.goal : {};
    const finiteValue = value => {
      if (value === null || value === undefined || value === '') return null;
      const normalized = Number(value);
      return Number.isFinite(normalized) ? normalized : null;
    };
    const x = finiteValue(state.x !== undefined ? state.x : nested.x);
    const y = finiteValue(state.y !== undefined ? state.y : nested.y);
    const rawYaw = state.yaw !== undefined ? state.yaw :
      (nested.yaw !== undefined ? nested.yaw : 0);
    const yaw = finiteValue(rawYaw);
    const frameId = String(state.frame_id || nested.frame_id || 'map');
    if (x === null || y === null || yaw === null || frameId !== 'map') {
      return false;
    }
    this.navGoal = { ...nested, ...state, x, y, yaw, frame_id: 'map' };
    delete this.navGoal.goal;
    this._tf = null;
    this._markDirty();
    return true;
  }

  clearNavGoal() { return this.setNavGoal(null); }

  /** 接收 MID360 局部点 (x前/y左, 米), 转成世界坐标并累积到左侧障碍栅格。 */
  addLocalObstaclePoints(points) {
    if (!Array.isArray(points) || !points.length) return;
    const cosY = Math.cos(this.slam.robotYaw);
    const sinY = Math.sin(this.slam.robotYaw);
    for (const pt of points) {
      if (!Array.isArray(pt) || pt.length < 2) continue;
      const lx = Number(pt[0]);
      const ly = Number(pt[1]);
      if (!Number.isFinite(lx) || !Number.isFinite(ly)) continue;
      const wx = cosY * lx - sinY * ly + this.slam.robotX;
      const wy = sinY * lx + cosY * ly + this.slam.robotY;
      this._addLidarObstacleCell(wx, wy);
    }
    this.slam.lidarMapPoints = Array.from(this._lidarCells.values());
    this._tf = null;
    this._markDirty();
  }

  _addLidarObstacleCell(wx, wy) {
    const qx = Math.round(wx * 10) / 10;
    const qy = Math.round(wy * 10) / 10;
    const x = Object.is(qx, -0) ? 0 : qx;
    const y = Object.is(qy, -0) ? 0 : qy;
    const key = `${x.toFixed(1)},${y.toFixed(1)}`;
    if (this._lidarCells.has(key)) this._lidarCells.delete(key);
    this._lidarCells.set(key, [x, y]);
    while (this._lidarCells.size > 5000) {
      this._lidarCells.delete(this._lidarCells.keys().next().value);
    }
  }

  _personMarkerPosition(marker) {
    if (!marker) return null;
    const x = marker.x !== undefined ? marker.x : marker.world_x;
    const y = marker.y !== undefined ? marker.y : marker.world_y;
    const nx = Number(x);
    const ny = Number(y);
    if (!Number.isFinite(nx) || !Number.isFinite(ny)) return null;
    return { x: nx, y: ny };
  }

  _targetMarkerLabel(marker, index) {
    const target = marker || {};
    const key = String(target.class || target.label || target.type || '')
      .trim().toLowerCase();
    const explicit = String(target.label_zh || '').trim();
    const number = index + 1;
    if (explicit) return `${explicit}${number}`;
    if (!key || key === 'person' || key === 'people' || key === 'person_marker') {
      return `人${number}`;
    }
    if (key === 'dining table' || key === 'table') return `桌子${number}`;
    const fallback = String(target.class || target.label || '目标').trim() || '目标';
    return `${fallback}${number}`;
  }

  _hitTestMarker(sx, sy) {
    // 屏幕 px 命中测试 targetMarkers (圆点半径 5, 命中阈值放大到 12 好点)。
    // 返回最近的 marker 对象 (含 photo_url/crop_url/frame_url 等字段, 供 onSelectMarker 弹截图)。
    if (!this.slam || !this.slam.targetMarkers || !this.slam.targetMarkers.length) return null;
    if (!this._tf) this._computeTransform();
    const toX = this._tf.toX, toY = this._tf.toY;
    const HIT_RADIUS = 12;
    let best = null, bestDist = HIT_RADIUS;
    for (let i = 0; i < this.slam.targetMarkers.length; i++) {
      const marker = this.slam.targetMarkers[i];
      const pos = this._personMarkerPosition(marker);
      if (!pos) continue;
      const dx = toX(pos.x) - sx, dy = toY(pos.y) - sy;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist <= bestDist) { bestDist = dist; best = marker; }
    }
    return best;
  }

  start() {
    if (this._running) return;
    this._running = true;
    this._dirty = true;
    this._scheduleFrame();
  }

  stop() {
    if (!this._running && this._rafId === null) return;
    this._running = false;
    this._frameGeneration += 1;
    if (this._rafId !== null) cancelAnimationFrame(this._rafId);
    this._rafId = null;
    this._inFrame = false;
  }

  _markDirty() {
    this._dirty = true;
    this._scheduleFrame();
  }

  _scheduleFrame() {
    if (!this._running || this._rafId !== null || this._inFrame || !this._dirty) return;
    const generation = this._frameGeneration;
    this._rafId = requestAnimationFrame(() => {
      // cancelAnimationFrame is not guaranteed to suppress a callback which is
      // already dispatching.  A generation guard keeps old stop/start frames
      // from clearing or repainting the current scheduler.
      if (!this._running || generation !== this._frameGeneration) return;
      this._rafId = null;
      if (!this._dirty) return;
      this._dirty = false;
      this._inFrame = true;
      try {
        this._resize();
        this._draw();
      } finally {
        this._inFrame = false;
      }
      this._scheduleFrame();
    });
  }

  _invalidateViewport() {
    this._tf = null;
    this._fogCacheKey = '';
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    // Canvas bitmap dimensions are integers in the DOM.  Comparing them with
    // fractional CSS*dpr values would invalidate every dirty frame at 125%
    // or 150% Windows scaling.
    const pixelWidth = Math.max(1, Math.round(w * dpr));
    const pixelHeight = Math.max(1, Math.round(h * dpr));
    if (this.canvas.width !== pixelWidth || this.canvas.height !== pixelHeight) {
      this.canvas.width = pixelWidth;
      this.canvas.height = pixelHeight;
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this._invalidateViewport();
      if (!this._inFrame) this._markDirty();
    }
    this.W = w; this.H = h;
  }

  // ---- costmap 离屏渲染 (cell → ImageData, 1px/cell; 主循环只 drawImage 一次) ----
  _renderCostmap() {
    const cm = this.slam.costmap;
    if (!cm || !cm.vals) return;
    if (!this._cmCanvas) {
      this._cmCanvas = document.createElement('canvas');
      this._cmCtx = this._cmCanvas.getContext('2d');
    }
    const w = cm.w, h = cm.h;
    this._cmCanvas.width = w; this._cmCanvas.height = h;
    const img = this._cmCtx.createImageData(w, h);
    // OccupancyGrid: data row 0 = 世界 y 最小 (origin). 前端 toY: 世界 y 大→屏幕上(y小).
    // 翻转 row: cell row i(世界y小) → img 底行, drawImage 正向贴时 img 顶=世界y大=屏幕上.
    for (let i = 0; i < h; i++) {
      const imgRow = h - 1 - i;
      const rowOff = imgRow * w * 4;
      const cellRowOff = i * w;
      for (let j = 0; j < w; j++) {
        const c = cm.vals[cellRowOff + j];
        const off = rowOff + j * 4;
        if (c < 0)            { img.data[off]=70;  img.data[off+1]=70;  img.data[off+2]=80;  img.data[off+3]=80;  } // unknown 灰
        else if (c >= 50)     { img.data[off]=255; img.data[off+1]=40; img.data[off+2]=40; img.data[off+3]=200; } // occupied 红
        else if (c > 0)       { img.data[off]=60;  img.data[off+1]=110;img.data[off+2]=255;img.data[off+3]=90;  } // inflation 蓝
        else                  { img.data[off+3]=0; }                                                              // free 透明
      }
    }
    this._cmCtx.putImageData(img, 0, 0);
    this._cmDirty = false;
  }

  _renderSearchFog(roomSearch, roomArea, coverageCellWorld, toX, toY, W, H) {
    const observedCells = roomSearch.observedCells || [];
    if (!roomSearch.phase && !observedCells.length) return false;
    const areaKey = roomArea
      ? [roomArea.origin_x, roomArea.origin_y, roomArea.width, roomArea.height].join(',')
      : 'viewport';
    const cacheKey = [
      W, H, this._fogCoverageRevision, areaKey, coverageCellWorld,
      this._tf.scale.toFixed(5), toX(0).toFixed(2), toY(0).toFixed(2),
    ].join('|');

    if (!this._fogCanvas) {
      this._fogCanvas = document.createElement('canvas');
      this._fogCtx = this._fogCanvas.getContext('2d');
    }
    if (cacheKey !== this._fogCacheKey) {
      if (this._fogCanvas.width !== W || this._fogCanvas.height !== H) {
        this._fogCanvas.width = W;
        this._fogCanvas.height = H;
      }
      const fogCtx = this._fogCtx;
      fogCtx.setTransform(1, 0, 0, 1, 0, 0);
      fogCtx.clearRect(0, 0, W, H);
      fogCtx.fillStyle = 'rgba(3,10,18,0.86)';
      if (roomArea) {
        fogCtx.fillRect(
          toX(roomArea.origin_x),
          toY(roomArea.origin_y + roomArea.height),
          roomArea.width * this._tf.scale,
          roomArea.height * this._tf.scale,
        );
      } else {
        fogCtx.fillRect(0, 0, W, H);
      }

      // Slight overlap removes hairline seams between adjacent coverage cells.
      const maskCellPx = Math.max(1, coverageCellWorld * this._tf.scale + 0.75);
      fogCtx.fillStyle = 'rgba(0,230,118,0.10)';
      for (const point of observedCells) {
        if (roomArea && (
          point.x < roomArea.origin_x || point.x > roomArea.origin_x + roomArea.width
          || point.y < roomArea.origin_y || point.y > roomArea.origin_y + roomArea.height
        )) continue;
        const x = toX(point.x) - maskCellPx / 2;
        const y = toY(point.y) - maskCellPx / 2;
        fogCtx.clearRect(x, y, maskCellPx, maskCellPx);
        fogCtx.fillRect(x, y, maskCellPx, maskCellPx);
      }
      this._fogCacheKey = cacheKey;
      this._fogBuildCount += 1;
    }
    this.ctx.drawImage(this._fogCanvas, 0, 0, W, H);
    return true;
  }

  // ---- 世界↔屏幕坐标变换 ----
  _computeTransform() {
    const SCAN_RANGE = 8.0;
    // Old obstacle cells can belong to a previous FAST_LIO origin.  They may
    // still be useful in storage, but must never collapse the click scale or
    // turn a nearby pixel into a kilometre-scale navigation goal.
    const MAX_VIEW_RADIUS = 50.0;
    const s = this.slam;
    let allX = [s.robotX], allY = [s.robotY];
    const include = (x, y) => {
      const nx = Number(x), ny = Number(y);
      if (!Number.isFinite(nx) || !Number.isFinite(ny)) return;
      if (Math.hypot(nx - s.robotX, ny - s.robotY) > MAX_VIEW_RADIUS) return;
      allX.push(nx); allY.push(ny);
    };
    if (this.searchRegion) {
      include(this.searchRegion.x, this.searchRegion.y);
      include(this.searchRegion.x + this.searchRegion.w,
        this.searchRegion.y + this.searchRegion.h);
    }
    if (this.navGoal && Number.isFinite(this.navGoal.x) && Number.isFinite(this.navGoal.y)) {
      include(this.navGoal.x, this.navGoal.y);
    }
    for (const p of s.mapPoints) include(p[0], p[1]);
    for (const p of s.lidarMapPoints) include(p[0], p[1]);
    for (const p of s.wallPoints) include(p[0], p[1]);
    for (const p of s.scanPoints) include(p[0], p[1]);
    for (const t of s.trail) include(t[0], t[1]);
    for (const wp of s.waypoints) include(wp.x, wp.y);
    for (const d of s.detMarks) include(d.x, d.y);
    for (const marker of s.targetMarkers) {
      const pos = this._personMarkerPosition(marker);
      if (!pos) continue;
      include(pos.x, pos.y);
    }
    const roomSearch = s.roomSearch || {};
    if (roomSearch.roomArea) {
      const area = roomSearch.roomArea;
      include(area.origin_x, area.origin_y);
      include(area.origin_x + area.width, area.origin_y + area.height);
    }
    for (const point of roomSearch.candidateViewpoints || []) include(point.x, point.y);
    for (const point of roomSearch.visitedViewpoints || []) include(point.x, point.y);
    for (const point of roomSearch.observedCells || []) include(point.x, point.y);
    for (const point of roomSearch.visibleCells || []) include(point.x, point.y);
    const frustum = this._cameraFrustumGeometry(roomSearch);
    if (frustum) {
      include(frustum.leftEndpoint.x, frustum.leftEndpoint.y);
      include(frustum.centerEndpoint.x, frustum.centerEndpoint.y);
      include(frustum.rightEndpoint.x, frustum.rightEndpoint.y);
    }

    const minX = Math.min(...allX), maxX = Math.max(...allX);
    const minY = Math.min(...allY), maxY = Math.max(...allY);
    const rangeX = Math.max(maxX - minX, SCAN_RANGE * 2);
    const rangeY = Math.max(maxY - minY, SCAN_RANGE * 2);
    const margin = 2.0;
    const scale = Math.min(this.W / (rangeX + margin * 2), this.H / (rangeY + margin * 2));
    // Keep click geometry robot-centric even when obstacles are distributed
    // asymmetrically.  The full bounds still determine zoom, but canvas center
    // is always the current trusted localization pose.
    const worldCX = s.robotX;
    const worldCY = s.robotY;
    const cx = this.W / 2 - worldCX * scale;
    const cy = this.H / 2 + worldCY * scale;
    this._tf = { toX: wx => cx + wx * scale, toY: wy => cy - wy * scale, scale, minX, maxX, minY, maxY };
  }

  /** 屏幕坐标 → 世界坐标 (供拖框选区用) */
  screenToWorld(sx, sy) {
    if (!this._tf) this._computeTransform();
    return this._screenToWorldWithTransform(sx, sy, this._tf);
  }

  _screenToWorldWithTransform(sx, sy, transform) {
    return {
      x: (sx - transform.toX(0)) / transform.scale,
      y: (transform.toY(0) - sy) / transform.scale,
    };
  }

  _bindMouse() {
    const c = this.canvas;
    c.addEventListener('mousedown', e => {
      this._markDirty();
      if (e.button !== undefined && e.button !== 0) return;
      const r = c.getBoundingClientRect();
      const sx = e.clientX - r.left, sy = e.clientY - r.top;
      // marker hit-test: 点 person 标记 → 触发 onSelectMarker 弹检测截图, 不启动拖框
      const hit = this._hitTestMarker(sx, sy);
      if (hit) {
        if (this.onSelectMarker) this.onSelectMarker(hit);
        return;
      }
      if (!this._tf) this._computeTransform();
      this._dragging = true;
      this._dragStart = { x: sx, y: sy };
      this._dragCur = { ...this._dragStart };
      // SLAM 数据可在一次手势中刷新；固定按下时的变换，避免目标坐标漂移。
      this._dragTransform = this._tf;
    });
    c.addEventListener('mousemove', e => {
      if (!this._dragging) return;
      const r = c.getBoundingClientRect();
      this._dragCur = { x: e.clientX - r.left, y: e.clientY - r.top };
      this._markDirty();
    });
    const resetDrag = () => {
      this._dragging = false;
      this._dragStart = null;
      this._dragCur = null;
      this._dragTransform = null;
    };
    const endDrag = e => {
      if (!this._dragging) return;
      if (e && e.button !== undefined && e.button !== 0) return;
      if (!this._dragStart || !this._dragTransform) { resetDrag(); return; }
      if (e && Number.isFinite(e.clientX) && Number.isFinite(e.clientY)) {
        const r = c.getBoundingClientRect();
        this._dragCur = { x: e.clientX - r.left, y: e.clientY - r.top };
      }
      if (!this._dragCur) { resetDrag(); return; }
      const a = this._dragStart, b = this._dragCur;
      const transform = this._dragTransform;
      if (Math.hypot(a.x - b.x, a.y - b.y) <= 8) {
        const goal = this._screenToWorldWithTransform(b.x, b.y, transform);
        resetDrag();
        this._markDirty();
        if (this.onSelectGoal) this.onSelectGoal({ ...goal, frame_id: 'map' });
        return;
      }
      const w1 = this._screenToWorldWithTransform(a.x, a.y, transform);
      const w2 = this._screenToWorldWithTransform(b.x, b.y, transform);
      const region = {
        x: Math.min(w1.x, w2.x),
        y: Math.min(w1.y, w2.y),
        w: Math.abs(w2.x - w1.x),
        h: Math.abs(w2.y - w1.y),
      };
      this.searchRegion = region;
      resetDrag();
      this._markDirty();
      if (this.onSelectRegion) this.onSelectRegion(region);
    };
    c.addEventListener('mouseup', endDrag);
    // 鼠标离开后释放位置不可靠：取消手势，绝不误发目标或选区。
    c.addEventListener('mouseleave', () => {
      const wasDragging = this._dragging;
      resetDrag();
      if (wasDragging) this._markDirty();
    });
  }

  _cameraFrustumGeometry(roomSearch = this.slam.roomSearch) {
    const source = roomSearch && typeof roomSearch === 'object' ? roomSearch : {};
    const requiredFinite = value => value !== null && value !== undefined && value !== ''
      && Number.isFinite(Number(value));
    if (!requiredFinite(source.cameraHfovDeg)
      || !requiredFinite(source.cameraYawOffsetDeg)
      || !requiredFinite(source.visualRangeM)) return null;
    const hfovDeg = Number(source.cameraHfovDeg);
    const yawOffsetDeg = Number(source.cameraYawOffsetDeg);
    const rangeM = Number(source.visualRangeM);
    const robotX = Number(this.slam.robotX);
    const robotY = Number(this.slam.robotY);
    const robotYaw = Number(this.slam.robotYaw);
    if (
      !Number.isFinite(hfovDeg) || hfovDeg <= 0 || hfovDeg > 360
      || !Number.isFinite(yawOffsetDeg)
      || !Number.isFinite(rangeM) || rangeM <= 0
      || !Number.isFinite(robotX) || !Number.isFinite(robotY)
      || !Number.isFinite(robotYaw)
    ) return null;

    const hfovRad = hfovDeg * Math.PI / 180;
    const centerBearingRad = robotYaw + yawOffsetDeg * Math.PI / 180;
    const leftBearingRad = centerBearingRad + hfovRad / 2;
    const rightBearingRad = centerBearingRad - hfovRad / 2;
    const endpoint = bearing => ({
      x: robotX + rangeM * Math.cos(bearing),
      y: robotY + rangeM * Math.sin(bearing),
    });
    return {
      origin: { x: robotX, y: robotY },
      rangeM,
      hfovDeg,
      hfovRad,
      centerBearingRad,
      centerBearingDeg: centerBearingRad * 180 / Math.PI,
      leftBearingRad,
      rightBearingRad,
      leftEndpoint: endpoint(leftBearingRad),
      centerEndpoint: endpoint(centerBearingRad),
      rightEndpoint: endpoint(rightBearingRad),
    };
  }

  _drawCameraFrustum(roomSearch, coverageCellWorld, toX, toY) {
    const source = roomSearch && typeof roomSearch === 'object' ? roomSearch : {};
    const visibleCells = Array.isArray(source.visibleCells) ? source.visibleCells : [];
    const visibleCellsAvailable = source.visibleCellsAvailable === true;
    const geometry = this._cameraFrustumGeometry(source);
    if (!visibleCellsAvailable && !geometry) return;

    const ctx = this.ctx;
    ctx.save();
    if (visibleCellsAvailable && visibleCells.length) {
      const cellWorld = Number.isFinite(coverageCellWorld) && coverageCellWorld > 0
        ? coverageCellWorld : 0.5;
      const cellPx = Math.max(1, cellWorld * this._tf.scale);
      ctx.fillStyle = 'rgba(0,229,255,0.18)';
      for (const cell of visibleCells) {
        ctx.fillRect(
          toX(cell.x - cellWorld / 2),
          toY(cell.y + cellWorld / 2),
          cellPx,
          cellPx,
        );
      }
    }

    if (!geometry) {
      ctx.restore();
      return;
    }

    const ox = toX(geometry.origin.x);
    const oy = toY(geometry.origin.y);
    const lx = toX(geometry.leftEndpoint.x);
    const ly = toY(geometry.leftEndpoint.y);
    const rx = toX(geometry.rightEndpoint.x);
    const ry = toY(geometry.rightEndpoint.y);
    const radiusPx = geometry.rangeM * this._tf.scale;
    const screenLeftAngle = -geometry.leftBearingRad;
    const screenRightAngle = -geometry.rightBearingRad;

    if (!visibleCellsAvailable) {
      ctx.fillStyle = 'rgba(0,229,255,0.12)';
      ctx.beginPath();
      ctx.moveTo(ox, oy);
      ctx.lineTo(lx, ly);
      ctx.arc(ox, oy, radiusPx, screenLeftAngle, screenRightAngle, false);
      ctx.closePath();
      ctx.fill();
    }

    ctx.strokeStyle = 'rgba(0,229,255,0.9)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(lx, ly);
    ctx.moveTo(ox, oy);
    ctx.lineTo(rx, ry);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(ox, oy, radiusPx, screenLeftAngle, screenRightAngle, false);
    ctx.stroke();

    const cx = toX(geometry.centerEndpoint.x);
    const cy = toY(geometry.centerEndpoint.y);
    ctx.strokeStyle = 'rgba(128,222,234,0.8)';
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(ox, oy);
    ctx.lineTo(cx, cy);
    ctx.stroke();
    ctx.setLineDash([]);

    const labelDistance = geometry.rangeM * 0.62;
    const labelX = toX(
      geometry.origin.x + labelDistance * Math.cos(geometry.centerBearingRad));
    const labelY = toY(
      geometry.origin.y + labelDistance * Math.sin(geometry.centerBearingRad));
    ctx.fillStyle = '#80deea';
    ctx.font = '10px sans-serif';
    ctx.fillText(`C13 ${geometry.hfovDeg.toFixed(1)}\u00b0`, labelX + 5, labelY - 5);
    ctx.restore();
  }

  _draw() {
    const ctx = this.ctx, W = this.W, H = this.H;
    if (!this._tf) this._computeTransform();
    const toX = this._tf.toX, toY = this._tf.toY;
    const s = this.slam;

    // 背景
    ctx.fillStyle = '#0a1520';
    ctx.fillRect(0, 0, W, H);

    // 网格
    ctx.strokeStyle = '#152230'; ctx.lineWidth = 1;
    const gMin = Math.floor(Math.min(this._tf.minX, 0) - 2);
    const gMax = Math.ceil(Math.max(this._tf.maxX, 0) + 2);
    for (let gx = gMin; gx <= gMax; gx++) { ctx.beginPath(); ctx.moveTo(toX(gx), 0); ctx.lineTo(toX(gx), H); ctx.stroke(); }
    for (let gy = gMin; gy <= gMax; gy++) { ctx.beginPath(); ctx.moveTo(0, toY(gy)); ctx.lineTo(W, toY(gy)); ctx.stroke(); }
    ctx.fillStyle = '#2a3a48'; ctx.font = '9px sans-serif';
    for (let gx = gMin; gx <= gMax; gx++) ctx.fillText(gx + 'm', toX(gx) + 2, toY(0) - 2);
    for (let gy = gMin; gy <= gMax; gy++) if (gy !== 0) ctx.fillText(gy + 'm', toX(0) + 2, toY(gy) - 2);

    // Visibility layer contract (bottom to top): fog, current camera frustum,
    // then authoritative walls/costmap. Obstacle boundaries stay unobscured.
    const roomSearch = s.roomSearch || {};
    const roomArea = roomSearch.roomArea;
    const coverageCellWorld = Number.isFinite(roomSearch.coverageCellSizeM)
      && roomSearch.coverageCellSizeM > 0
      ? roomSearch.coverageCellSizeM
      : (roomArea && Number.isFinite(roomArea.spacing) && roomArea.spacing > 0
        ? Math.min(roomArea.spacing, 1.0) : 0.5);
    this._renderSearchFog(
      roomSearch, roomArea, coverageCellWorld, toX, toY, W, H,
    );
    this._drawCameraFrustum(roomSearch, coverageCellWorld, toX, toY);

    // Persistent /map_frontier geometry. This display-only wall layer is
    // intentionally distinct from Nav2's live red/blue costmap authority.
    if (s.wallPoints.length) {
      const wallPx = Math.max(1.5, Math.min(
        4, (Number(s.wallResolution) || 0.05) * this._tf.scale + 0.75));
      ctx.fillStyle = 'rgba(207,216,220,0.82)';
      for (const point of s.wallPoints) {
        ctx.fillRect(
          toX(point[0]) - wallPx / 2,
          toY(point[1]) - wallPx / 2,
          wallPx,
          wallPx,
        );
      }
    }

    // 0. nav2 local_costmap 层 (离屏 canvas 预渲染, drawImage 一次, 不卡 60fps)
    if (s.costmap) {
      if (this._cmDirty) this._renderCostmap();
      if (this._cmCanvas) {
        const cm = s.costmap;
        const dx = toX(cm.ox);
        const dy = toY(cm.oy + cm.h * cm.res);   // 世界 y 最大→屏幕最上(y小)
        const dw = cm.w * cm.res * this._tf.scale;
        const dh = cm.h * cm.res * this._tf.scale;
        ctx.save();
        ctx.imageSmoothingEnabled = false;
        ctx.globalAlpha = 0.55;
        ctx.drawImage(this._cmCanvas, dx, dy, dw, dh);
        ctx.restore();
      }
    }

    // 0b. nav2 规划路径 /plan (青色虚线 polyline —— 显示狗要走的规划路线)
    if (s.plan && s.plan.pts && s.plan.pts.length > 1) {
      ctx.save();
      ctx.strokeStyle = 'rgba(0,229,255,0.9)';
      ctx.lineWidth = 2.5;
      ctx.setLineDash([7, 4]);
      ctx.lineJoin = 'round';
      ctx.beginPath();
      for (let i = 0; i < s.plan.pts.length; i++) {
        const px = toX(s.plan.pts[i][0]), py = toY(s.plan.pts[i][1]);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      }
      ctx.stroke();
      ctx.restore();
    }

    // 1. 障碍栅格点 [2026-07-18 关闭] mapPoints/lidarMapPoints 是 /map_frontier 建图
    //    历史累积点(满2000上限), 含 FastLIO 抖动留下的 ghost, 位置不对干扰判断.
    //    用户: "只有绿色scan点是正确扫描障碍, 黄(橙)和红都有问题". 只留 scan 绿 + costmap 红.
    // const obstaclePoints = s.mapPoints.concat(s.lidarMapPoints);
    // if (obstaclePoints.length) {
    //   ctx.fillStyle = 'rgba(255,145,0,0.7)';
    //   for (const p of obstaclePoints) ctx.fillRect(toX(p[0]) - 1, toY(p[1]) - 1, 2, 2);
    // }
    // 2. 实时扫描
    const rx = toX(s.robotX), ry = toY(s.robotY);
    if (s.scanPoints.length) {
      ctx.strokeStyle = 'rgba(0,230,118,0.12)'; ctx.lineWidth = 0.5;
      ctx.beginPath();
      for (const pt of s.scanPoints) {
        ctx.moveTo(rx, ry); ctx.lineTo(toX(pt[0]), toY(pt[1]));
      }
      ctx.stroke();
      ctx.fillStyle = 'rgba(0,255,136,0.8)';
      for (const pt of s.scanPoints) {
        ctx.beginPath(); ctx.arc(toX(pt[0]), toY(pt[1]), 2, 0, Math.PI * 2); ctx.fill();
      }
    }
    // 3. 产品房间搜索: 已观察覆盖、房间边界、候选/已访问视点
    if (roomArea) {
      ctx.strokeStyle = '#7e57c2'; ctx.lineWidth = 2;
      ctx.setLineDash([8, 4]);
      ctx.strokeRect(
        toX(roomArea.origin_x),
        toY(roomArea.origin_y + roomArea.height),
        roomArea.width * this._tf.scale,
        roomArea.height * this._tf.scale,
      );
      ctx.setLineDash([]);
      ctx.fillStyle = '#b39ddb'; ctx.font = '10px sans-serif';
      const pct = Math.round((roomSearch.coverageRatio || 0) * 100);
      ctx.fillText(
        `${roomSearch.room || '房间'} · ${roomSearch.phase || 'SEARCH'} · 覆盖 ${pct}%`,
        toX(roomArea.origin_x) + 4,
        toY(roomArea.origin_y + roomArea.height) + 13,
      );
    } else if (roomSearch.phase) {
      ctx.fillStyle = '#b39ddb'; ctx.font = '10px sans-serif';
      const pct = Math.round((roomSearch.coverageRatio || 0) * 100);
      ctx.fillText(
        `${roomSearch.phase} · 覆盖 ${pct}% · 步长 ${(roomSearch.adaptiveStepM || 0).toFixed(1)}m`,
        12,
        18,
      );
    }
    ctx.fillStyle = 'rgba(255,235,59,0.9)';
    for (const point of roomSearch.candidateViewpoints || []) {
      ctx.beginPath(); ctx.arc(toX(point.x), toY(point.y), 3, 0, Math.PI * 2); ctx.fill();
    }
    ctx.fillStyle = 'rgba(0,229,255,0.95)';
    for (const point of roomSearch.visitedViewpoints || []) {
      ctx.beginPath(); ctx.arc(toX(point.x), toY(point.y), 6, 0, Math.PI * 2); ctx.fill();
    }
    // 4. 搜索区域框 (用户选区) + 拖动中的虚框
    if (this.searchRegion) {
      const reg = this.searchRegion;
      ctx.strokeStyle = 'rgba(255,193,7,0.8)'; ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 4]);
      ctx.strokeRect(toX(reg.x), toY(reg.y), reg.w * this._tf.scale, -reg.h * this._tf.scale);
      ctx.setLineDash([]);
      ctx.fillStyle = 'rgba(255,193,7,0.08)';
      ctx.fillRect(toX(reg.x), toY(reg.y), reg.w * this._tf.scale, -reg.h * this._tf.scale);
      ctx.fillStyle = '#ffc107'; ctx.font = '10px sans-serif';
      ctx.fillText(`搜索区 ${reg.w.toFixed(1)}×${reg.h.toFixed(1)}m`, toX(reg.x) + 4, toY(reg.y) + 12);
    }
    if (this._dragging && this._dragStart && this._dragCur) {
      const a = this._dragStart, b = this._dragCur;
      ctx.strokeStyle = '#ffc107'; ctx.lineWidth = 1; ctx.setLineDash([4, 4]);
      ctx.strokeRect(Math.min(a.x, b.x), Math.min(a.y, b.y), Math.abs(a.x - b.x), Math.abs(a.y - b.y));
      ctx.setLineDash([]);
    }
    // 4. 规划路径
    const wps = s.waypoints;
    if (wps.length) {
      ctx.strokeStyle = '#2d4052'; ctx.lineWidth = 1.5; ctx.setLineDash([5, 5]);
      ctx.beginPath(); ctx.moveTo(toX(0), toY(0));
      for (const wp of wps) ctx.lineTo(toX(wp.x), toY(wp.y));
      ctx.stroke(); ctx.setLineDash([]);
      for (let i = 0; i < wps.length; i++) {
        ctx.fillStyle = '#3d5062'; ctx.beginPath(); ctx.arc(toX(wps[i].x), toY(wps[i].y), 3, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#90a4ae'; ctx.font = '9px sans-serif'; ctx.fillText(i + 1, toX(wps[i].x) + 4, toY(wps[i].y) - 4);
      }
      if (s.currentWP >= 0 && s.currentWP < wps.length) {
        ctx.strokeStyle = '#ff9100'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(toX(wps[s.currentWP].x), toY(wps[s.currentWP].y), 7, 0, Math.PI * 2); ctx.stroke();
      }
    }
    // 4.5 Panel 点导航目标：状态色圆形准星 + 最终朝向箭头。
    if (this.navGoal) {
      const goal = this.navGoal;
      const colors = {
        pending: '#ffb300', waiting_server: '#ffb300', waiting_health: '#ffb300',
        active: '#00e5ff', canceling: '#ffb300',
        succeeded: '#4caf50',
        rejected: '#ef5350', aborted: '#ef5350', failed: '#ef5350',
        timed_out: '#ef5350', canceled: '#ef5350', cancel_failed: '#ef5350',
        server_unavailable: '#ef5350', error: '#ef5350',
      };
      const color = colors[goal.status] || '#90a4ae';
      const gx = toX(goal.x), gy = toY(goal.y);
      const arrowLength = 18;
      const ax = gx + Math.cos(goal.yaw) * arrowLength;
      const ay = gy - Math.sin(goal.yaw) * arrowLength;
      ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gx, gy, 9, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx - 4, gy); ctx.lineTo(gx + 4, gy);
      ctx.moveTo(gx, gy - 4); ctx.lineTo(gx, gy + 4); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx, gy); ctx.lineTo(ax, ay); ctx.stroke();
      const wing = 5;
      const backAngle = goal.yaw + Math.PI;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(
        ax + Math.cos(backAngle + 0.55) * wing,
        ay - Math.sin(backAngle + 0.55) * wing,
      );
      ctx.lineTo(
        ax + Math.cos(backAngle - 0.55) * wing,
        ay - Math.sin(backAngle - 0.55) * wing,
      );
      ctx.closePath(); ctx.fill();
      ctx.fillStyle = color; ctx.font = '10px sans-serif';
      ctx.fillText(`NAV ${goal.status || '目标'}`, gx + 12, gy - 11);
    }
    // 5. 轨迹
    if (s.trail.length > 1) {
      ctx.strokeStyle = '#00e5ff'; ctx.lineWidth = 2.5; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(toX(s.trail[0][0]), toY(s.trail[0][1]));
      for (let i = 1; i < s.trail.length; i++) ctx.lineTo(toX(s.trail[i][0]), toY(s.trail[i][1]));
      ctx.stroke();
    }
    // 6. 检测目标 (红三角)
    for (const det of s.detMarks) {
      const dx = toX(det.x), dy = toY(det.y);
      ctx.fillStyle = '#ff1744';
      ctx.beginPath(); ctx.moveTo(dx, dy - 8); ctx.lineTo(dx - 6, dy + 5); ctx.lineTo(dx + 6, dy + 5); ctx.closePath(); ctx.fill();
      ctx.fillStyle = '#fff'; ctx.font = '10px sans-serif'; ctx.fillText(det.class, dx + 8, dy + 3);
    }
    for (let i = 0; i < s.targetMarkers.length; i++) {
      const marker = s.targetMarkers[i];
      const pos = this._personMarkerPosition(marker);
      if (!pos) continue;
      const px = toX(pos.x), py = toY(pos.y);
      const confirmed = marker.position_quality === 'range_lidar' || marker.position_quality === 'multi_view';
      ctx.fillStyle = confirmed ? '#ff1744' : '#ffc107';
      ctx.strokeStyle = confirmed ? '#ffffff' : '#263238';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = confirmed ? '#ffffff' : '#263238';
      ctx.font = '11px sans-serif';
      ctx.fillText(this._targetMarkerLabel(marker, i), px + 7, py - 7);
      const worldZ = Number(marker.world_z !== undefined ? marker.world_z : marker.z);
      if (marker.position_dimension === 3 && Number.isFinite(worldZ)) {
        ctx.font = '9px sans-serif';
        ctx.fillText(`z ${worldZ.toFixed(1)}m`, px + 7, py + 6);
      }
    }
    // 7. 狗 (箭头)
    ctx.save();
    ctx.translate(rx, ry); ctx.rotate(-s.robotYaw);
    ctx.fillStyle = '#00e5ff';
    ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-7, -7); ctx.lineTo(-4, 0); ctx.lineTo(-7, 7); ctx.closePath(); ctx.fill();
    ctx.restore();
    // 起点
    ctx.strokeStyle = '#00c853'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(toX(0), toY(0), 6, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#00c853'; ctx.font = '9px sans-serif'; ctx.fillText('START', toX(0) + 8, toY(0) + 3);
    // 距离圈
    ctx.strokeStyle = 'rgba(0,229,255,0.12)'; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    for (const r of [2, 4, 6, 8]) {
      ctx.beginPath(); ctx.arc(rx, ry, r * this._tf.scale, 0, Math.PI * 2); ctx.stroke();
    }
    ctx.setLineDash([]);
  }
}

window.Go2WMap = Go2WMap;
