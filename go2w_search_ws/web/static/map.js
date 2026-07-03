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

    // 地图状态
    this.slam = {
      robotX: 0, robotY: 0, robotYaw: 0,
      trail: [], mapPoints: [], lidarMapPoints: [], scanPoints: [],
      detMarks: [], waypoints: [], currentWP: -1,
      personMarkers: [],
      slamSource: '',
    };
    this._lidarCells = new Map();
    // 用户选区 (世界坐标), 由 panel.html 设置 (表单输入时也同步显示)
    this.searchRegion = null; // {x, y, w, h}

    // 鼠标拖框状态
    this._dragging = false;
    this._dragStart = null;   // 屏幕坐标
    this._dragCur = null;
    this._bindMouse();

    // 渲染循环
    this._running = false;
  }

  /** 更新 slam 数据 (来自 WS 的 type=slam 消息) */
  update(data) {
    if (data.x !== undefined) this.slam.robotX = data.x;
    if (data.y !== undefined) this.slam.robotY = data.y;
    if (data.yaw !== undefined) this.slam.robotYaw = data.yaw;
    if (data.trail) this.slam.trail = data.trail;
    if (data.detections) this.slam.detMarks = data.detections;
    if (data.person_markers !== undefined) this.slam.personMarkers = data.person_markers;
    if (data.waypoints && data.waypoints.length) this.slam.waypoints = data.waypoints;
    if (data.currentWP !== undefined) this.slam.currentWP = data.currentWP;
    if (data.map) this.slam.mapPoints = data.map;
    if (data.scan) this.slam.scanPoints = data.scan;
    if (data.slam_source !== undefined) this.slam.slamSource = data.slam_source;
    this._tf = null;
  }

  /** 设置/清除搜索区域 (世界坐标 {x,y,w,h}); 表单输入也调这个同步显示 */
  setRegion(region) { this.searchRegion = region; this._tf = null; }
  clearRegion() { this.searchRegion = null; this._tf = null; }

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

  start() {
    this._running = true;
    const loop = () => {
      if (!this._running) return;
      this._resize();
      this._draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }
  stop() { this._running = false; }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (this.canvas.width !== w * dpr || this.canvas.height !== h * dpr) {
      this.canvas.width = w * dpr;
      this.canvas.height = h * dpr;
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    this.W = w; this.H = h;
  }

  // ---- 世界↔屏幕坐标变换 ----
  _computeTransform() {
    const SCAN_RANGE = 8.0;
    const s = this.slam;
    let allX = [s.robotX, 0], allY = [s.robotY, 0];
    if (this.searchRegion) {
      allX.push(this.searchRegion.x, this.searchRegion.x + this.searchRegion.w);
      allY.push(this.searchRegion.y, this.searchRegion.y + this.searchRegion.h);
    }
    for (const p of s.mapPoints) { allX.push(p[0]); allY.push(p[1]); }
    for (const p of s.lidarMapPoints) { allX.push(p[0]); allY.push(p[1]); }
    for (const p of s.scanPoints) { allX.push(p[0]); allY.push(p[1]); }
    for (const t of s.trail) { allX.push(t[0]); allY.push(t[1]); }
    for (const wp of s.waypoints) { allX.push(wp.x); allY.push(wp.y); }
    for (const d of s.detMarks) { allX.push(d.x); allY.push(d.y); }
    for (const marker of s.personMarkers) {
      const pos = this._personMarkerPosition(marker);
      if (!pos) continue;
      allX.push(pos.x); allY.push(pos.y);
    }

    const minX = Math.min(...allX), maxX = Math.max(...allX);
    const minY = Math.min(...allY), maxY = Math.max(...allY);
    const rangeX = Math.max(maxX - minX, SCAN_RANGE * 2);
    const rangeY = Math.max(maxY - minY, SCAN_RANGE * 2);
    const margin = 2.0;
    const scale = Math.min(this.W / (rangeX + margin * 2), this.H / (rangeY + margin * 2));
    const worldCX = (minX + maxX) / 2;
    const worldCY = (minY + maxY) / 2;
    const cx = this.W / 2 - worldCX * scale;
    const cy = this.H / 2 + worldCY * scale;
    this._tf = { toX: wx => cx + wx * scale, toY: wy => cy - wy * scale, scale, minX, maxX, minY, maxY };
  }

  /** 屏幕坐标 → 世界坐标 (供拖框选区用) */
  screenToWorld(sx, sy) {
    if (!this._tf) this._computeTransform();
    const scale = this._tf.scale;
    // 反推: sx = cx + wx*scale → wx = (sx - cx)/scale; cy = H/2 + worldCY*scale
    const cx = this._tf.toX(0) - 0; // 不直接用, 用闭包反推
    // 更稳: 直接从 toX/toY 的定义反推
    // toX(wx)=cx+wx*scale, toY(wy)=cy-wy*scale
    // 已知 toX(0)=cx, toY(0)=cy → 取这两点
    const px = this._tf.toX(0);
    const py = this._tf.toY(0);
    const wx = (sx - px) / scale;
    const wy = (py - sy) / scale;
    return { x: wx, y: wy };
  }

  _bindMouse() {
    const c = this.canvas;
    c.addEventListener('mousedown', e => {
      const r = c.getBoundingClientRect();
      this._dragging = true;
      this._dragStart = { x: e.clientX - r.left, y: e.clientY - r.top };
      this._dragCur = { ...this._dragStart };
    });
    c.addEventListener('mousemove', e => {
      if (!this._dragging) return;
      const r = c.getBoundingClientRect();
      this._dragCur = { x: e.clientX - r.left, y: e.clientY - r.top };
    });
    const endDrag = () => {
      if (!this._dragging) return;
      this._dragging = false;
      if (!this._dragStart || !this._dragCur) return;
      const a = this._dragStart, b = this._dragCur;
      if (Math.abs(a.x - b.x) < 8 || Math.abs(a.y - b.y) < 8) { this._dragStart = null; return; } // 太小忽略
      const w1 = this.screenToWorld(Math.min(a.x, b.x), Math.min(a.y, b.y));
      const w2 = this.screenToWorld(Math.max(a.x, b.x), Math.max(a.y, b.y));
      const region = { x: w1.x, y: w1.y, w: w2.x - w1.x, h: w2.y - w1.y };
      this.searchRegion = region;
      if (this.onSelectRegion) this.onSelectRegion(region);
      this._dragStart = null;
    };
    c.addEventListener('mouseup', endDrag);
    c.addEventListener('mouseleave', endDrag);
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

    // 1. 障碍栅格点
    const obstaclePoints = s.mapPoints.concat(s.lidarMapPoints);
    if (obstaclePoints.length) {
      ctx.fillStyle = 'rgba(255,145,0,0.7)';
      for (const p of obstaclePoints) ctx.fillRect(toX(p[0]) - 1, toY(p[1]) - 1, 2, 2);
    }
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
    // 3. 搜索区域框 (用户选区) + 拖动中的虚框
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
    for (let i = 0; i < s.personMarkers.length; i++) {
      const marker = s.personMarkers[i];
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
      ctx.fillText(`人${i + 1}`, px + 7, py - 7);
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
