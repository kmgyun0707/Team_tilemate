window.TileInspect3D = (() => {
  let _scene, _camera, _renderer, _controls;
  let _meshes = [], _pointClouds = [], _arrows = [];
  let _worldGroup;
  let _autoRotate = false;
  let _wireframe = false;
  let _showPoints = true;
  let _showNormals = true;
  let _initialized = false;
  let _animId = null;
  let _grid;
  let _lastWall = null;

  const el = (id) => document.getElementById(id);

  function fmt(v, d = 2) {
    return Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '-';
  }

  function clearWorld() {
    if (!_worldGroup) return;
    while (_worldGroup.children.length > 0) {
      const obj = _worldGroup.children[0];
      _worldGroup.remove(obj);
      if (obj.geometry) obj.geometry.dispose?.();
      if (obj.material) {
        if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose?.());
        else obj.material.dispose?.();
      }
    }
    _meshes = [];
    _pointClouds = [];
    _arrows = [];
  }

function safeSizeFromPlane(obj) {
  // wall은 wall 실제 크기 그대로
  if (obj?.name === 'wall') {
    if (obj?.plane_size_m?.width && obj?.plane_size_m?.height) {
      return [Number(obj.plane_size_m.width), Number(obj.plane_size_m.height)];
    }
    return [0.60, 0.62];
  }

  // tile은 픽셀 size_uv를 쓰지 말고 wall 실제 크기 기준으로 계산
  // return getTileRealSizeFromWall();
  return [0.060, 0.060];
}

  function safeCentroidM(obj) {
    if (Array.isArray(obj?.centroid_m)) return obj.centroid_m.map(Number);
    if (Array.isArray(obj?.center_point_m)) return obj.center_point_m.map(Number);
    if (Array.isArray(obj?.plane_centroid_mm)) return obj.plane_centroid_mm.map(v => Number(v) / 1000.0);
    return [0, 0, 0];
  }
  function getTileRealSizeFromWall() {
    const wallW = Number(_lastWall?.plane_size_m?.width || 0);
    const wallH = Number(_lastWall?.plane_size_m?.height || 0);

    // wall 정보가 없으면 기본값
    if (wallW <= 0 || wallH <= 0) {
      return [0.085, 0.085];
    }

    // 3x3 타일 배치 기준
    // 사진처럼 줄눈/여백을 고려해서 0.90 정도로 축소
    const shrink = 0.90;

    const tileW = (wallW / 3.0) * shrink;
    const tileH = (wallH / 3.0) * shrink;

    return [tileW, tileH];
  }
  function safeNormal(obj) {
    const n = obj?.normal || obj?.plane_normal || [0, 0, 1];
    const v = new THREE.Vector3(Number(n[0] || 0), Number(n[1] || 0), Number(n[2] || 1));
    if (v.length() < 1e-9) v.set(0, 0, 1);
    return v.normalize();
  }

  function rpyQuaternion(rpyDeg) {
    const rx = THREE.MathUtils.degToRad(Number(rpyDeg?.[0] || 0));
    const ry = THREE.MathUtils.degToRad(Number(rpyDeg?.[1] || 0));
    const rz = THREE.MathUtils.degToRad(Number(rpyDeg?.[2] || 0));
    return new THREE.Quaternion().setFromEuler(new THREE.Euler(rx, ry, rz, 'XYZ'));
  }

  function normalQuaternion(normal, fallbackRpyDeg) {
    const qNormal = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 0, 1),
      normal.clone().normalize()
    );
    if (!fallbackRpyDeg) return qNormal;
    return qNormal.multiply(rpyQuaternion(fallbackRpyDeg));
  }

  function planeColor(obj, helpers) {
    if (obj?.name === 'wall') return 0x3b82f6;

    const centroidMm = Array.isArray(obj?.plane_centroid_mm)
      ? obj.plane_centroid_mm
      : Array.isArray(obj?.centroid_m)
        ? obj.centroid_m.map(v => Number(v) * 1000.0)
        : [0, 0, 0];

    const dist = Math.abs(helpers.signedDistanceToWallPlaneMm(centroidMm, helpers.getWallRef()));
    const judge = helpers.classify(dist);

    if (judge.cls === 'bad') return 0xef4444;
    if (judge.cls === 'warn') return 0xf59e0b;
    return 0x22c55e;
  }

  function addPlane(obj, helpers) {
    if (!_worldGroup || !obj) return;

    const [w, h] = safeSizeFromPlane(obj);
    const centroid = safeCentroidM(obj);
    const normal = safeNormal(obj);
    const quat = normalQuaternion(normal, obj.rpy_deg);
    const color = planeColor(obj, helpers);

    const geo = new THREE.PlaneGeometry(w, h, 1, 1);
    const mat = new THREE.MeshStandardMaterial({
      color,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: obj?.name === 'wall' ? 0.28 : 0.74,
      wireframe: _wireframe,
      metalness: 0.08,
      roughness: 0.72,
    });

const mesh = new THREE.Mesh(geo, mat);
mesh.position.set(centroid[0], centroid[1], centroid[2]);
mesh.quaternion.copy(quat);
_worldGroup.add(mesh);
_meshes.push(mesh);

if (obj?.name === 'wall') {
  const gridSize = Math.max(w, h);
  const gridDiv = 12;

  const wallGrid = new THREE.GridHelper(gridSize, gridDiv, 0xffffff, 0x475569);

  const qGridToXY = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(Math.PI / 2, 0, 0, 'XYZ')
  );

  wallGrid.position.copy(mesh.position);
  wallGrid.quaternion.copy(quat.clone().multiply(qGridToXY));

  _worldGroup.add(wallGrid);
}
  // --- local axis 표시 ---
    const axis = new THREE.AxesHelper(Math.max(w, h) * 0.5);
    axis.position.copy(mesh.position);
    axis.quaternion.copy(mesh.quaternion);
    _worldGroup.add(axis);

    const edge = new THREE.LineSegments(
      new THREE.EdgesGeometry(geo),
      new THREE.LineBasicMaterial({ color: 0xe2e8f0, transparent: true, opacity: 0.4 })
    );
    edge.position.copy(mesh.position);
    edge.quaternion.copy(mesh.quaternion);
    _worldGroup.add(edge);

    const arrow = new THREE.ArrowHelper(
      normal.clone(),
      new THREE.Vector3(...centroid),
      Math.max(w, h) * 0.55,
      0x93c5fd,
      Math.max(w, h) * 0.18,
      Math.max(w, h) * 0.12
    );
    arrow.visible = _showNormals;
    _worldGroup.add(arrow);
    _arrows.push(arrow);

    const pointsGeo = new THREE.BufferGeometry();
    const corners = [
      new THREE.Vector3(-w / 2, -h / 2, 0),
      new THREE.Vector3( w / 2, -h / 2, 0),
      new THREE.Vector3( w / 2,  h / 2, 0),
      new THREE.Vector3(-w / 2,  h / 2, 0),
      new THREE.Vector3(0, 0, 0),
    ].map(v => v.applyQuaternion(quat).add(new THREE.Vector3(...centroid)));

    pointsGeo.setFromPoints(corners);
    const points = new THREE.Points(
      pointsGeo,
      new THREE.PointsMaterial({ color: 0xffffff, size: 0.008 })
    );
    points.visible = _showPoints;
    _worldGroup.add(points);
    _pointClouds.push(points);
  }

  function fitCamera() {
    const box = new THREE.Box3().setFromObject(_worldGroup);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 0.2);

    _camera.near = 0.01;
    _camera.far = Math.max(20, radius * 30);
    _camera.updateProjectionMatrix();
    _camera.position.set(
      center.x + radius * 1.8,
      center.y - radius * 2.1,
      center.z + radius * 1.4
    );
    _controls.target.copy(center);
    _controls.update();
  }

  function buildSummaryHtml(data, helpers) {
    const wall = data?.wall || {};
    const tiles = Array.isArray(data?.tiles) ? data.tiles : [];
    let html = '';

    html += `
      <div class="insp-card">
        <h4>Wall</h4>
        <div class="insp-kv">
          <div class="k">name</div><div class="v">${wall.name ?? '-'}</div>
          <div class="k">normal</div><div class="v mono">[${(wall.normal || []).map(v => fmt(v, 4)).join(', ')}]</div>
          <div class="k">center</div><div class="v mono">[${(wall.center_point_m || []).map(v => fmt(v, 4)).join(', ')}] m</div>
          <div class="k">centroid</div><div class="v mono">[${(wall.centroid_m || []).map(v => fmt(v, 4)).join(', ')}] m</div>
          <div class="k">size</div><div class="v mono">${fmt(wall.plane_size_m?.width, 4)} × ${fmt(wall.plane_size_m?.height, 4)} m</div>
          <div class="k">rpy</div><div class="v mono">roll ${fmt(wall.orientation_deg?.roll, 2)}, pitch ${fmt(wall.orientation_deg?.pitch, 2)}, yaw ${fmt(wall.orientation_deg?.yaw, 2)}</div>
        </div>
      </div>
    `;

    for (const tile of tiles) {
      const p = tile.plane_centroid_mm || [];
      const dz = helpers.signedDistanceToWallPlaneMm(p, helpers.getWallRef());
      html += `
        <div class="insp-card">
          <h4>${tile.name ?? '-'}</h4>
          <div class="insp-kv">
            <div class="k">conf</div><div class="v">${fmt(tile.conf_score, 4)}</div>
            <div class="k">center_uv</div><div class="v mono">[${(tile.center_uv || []).map(v => fmt(v, 1)).join(', ')}]</div>
            <div class="k">centroid_mm</div><div class="v mono">[${p.map(v => fmt(v, 2)).join(', ')}]</div>
            <div class="k">wall dist</div><div class="v mono">${dz >= 0 ? '+' : ''}${fmt(dz, 2)} mm</div>
            <div class="k">tilt</div><div class="v mono">${fmt(helpers.tiltDeg(tile.rpy_deg), 2)} °</div>
            <div class="k">rpy_deg</div><div class="v mono">[${(tile.rpy_deg || []).map(v => fmt(v, 2)).join(', ')}]</div>
            <div class="k">normal</div><div class="v mono">[${(tile.normal || []).map(v => fmt(v, 4)).join(', ')}]</div>
          </div>
        </div>
      `;
    }
    return html;
  }

  async function init() {
    if (_initialized) return;
    _initialized = true;

    const container = el('insp-three-container');
    _scene = new THREE.Scene();
    _scene.background = new THREE.Color(0x0b1120);

    _camera = new THREE.PerspectiveCamera(
      50,
      Math.max(1, container.clientWidth) / Math.max(1, container.clientHeight),
      0.01,
      100
    );
    _camera.position.set(0.6, -0.6, 0.45);

    _renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    _renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    _renderer.setSize(Math.max(1, container.clientWidth), Math.max(1, container.clientHeight));
    container.appendChild(_renderer.domElement);

    _controls = new THREE.OrbitControls(_camera, _renderer.domElement);
    _controls.enableDamping = true;
    _controls.dampingFactor = 0.06;

    _scene.add(new THREE.AmbientLight(0xffffff, 0.92));

    const dir1 = new THREE.DirectionalLight(0xffffff, 0.9);
    dir1.position.set(1.6, -1.2, 2.4);
    _scene.add(dir1);

    const dir2 = new THREE.DirectionalLight(0x93c5fd, 0.4);
    dir2.position.set(-1.4, 1.0, 1.2);
    _scene.add(dir2);

  _grid = new THREE.GridHelper(1.0, 20, 0x334155, 0x1e293b);
  _grid.rotation.x = Math.PI / 2;
  _grid.visible = false;   // 월드 grid 숨김
  _scene.add(_grid);

    _scene.add(new THREE.AxesHelper(0.18));

    _worldGroup = new THREE.Group();
    _worldGroup.rotation.x = Math.PI;
    _scene.add(_worldGroup);

    function animate() {
      _animId = requestAnimationFrame(animate);
      if (_autoRotate && _worldGroup) {
        _worldGroup.rotation.z += 0.0035;
      }
      _controls.update();
      _renderer.render(_scene, _camera);
    }
    animate();

    window.addEventListener('resize', () => {
      if (!_renderer || !_camera) return;
      const w = Math.max(1, container.clientWidth);
      const h = Math.max(1, container.clientHeight);
      _camera.aspect = w / h;
      _camera.updateProjectionMatrix();
      _renderer.setSize(w, h);
    });

    el('insp-loading').style.display = 'none';
  }

  async function open() {
    el('inspection-modal').style.display = 'flex';
    if (!_initialized) {
      await new Promise(resolve => requestAnimationFrame(resolve));
      await init();
    } else {
      const container = el('insp-three-container');
      if (_renderer && container.clientWidth > 0) {
        _camera.aspect = container.clientWidth / container.clientHeight;
        _camera.updateProjectionMatrix();
        _renderer.setSize(container.clientWidth, container.clientHeight);
      }
    }
  }

  function close() {
    el('inspection-modal').style.display = 'none';
  }

function render(data, helpers) {
  if (!_initialized) return;

  clearWorld();
  _lastWall = data?.wall || null;

  if (data?.wall) addPlane(data.wall, helpers);
  for (const tile of (data?.tiles || [])) {
    addPlane(tile, helpers);
  }

  fitCamera();

  el('insp-badge-frame').textContent = `frame: ${data?.frame_id ?? '-'}`;
  el('insp-badge-tiles').textContent = `tiles: ${data?.tiles?.length ?? 0}`;
  el('insp-badge-time').textContent =
    `time: ${data?.timestamp_sec ? new Date(data.timestamp_sec * 1000).toLocaleTimeString() : '-'}`;

  el('insp-summary-content').innerHTML = buildSummaryHtml(data, helpers);
  el('insp-loading').style.display = 'none';
}

  function togglePoints() {
    _showPoints = !_showPoints;
    _pointClouds.forEach(p => p.visible = _showPoints);
    el('insp-btn-points').classList.toggle('insp-btn-active', _showPoints);
  }

  function toggleNormals() {
    _showNormals = !_showNormals;
    _arrows.forEach(a => a.visible = _showNormals);
    el('insp-btn-normals').classList.toggle('insp-btn-active', _showNormals);
  }

  function toggleWire() {
    _wireframe = !_wireframe;
    _meshes.forEach(m => {
      if (m.material) m.material.wireframe = _wireframe;
    });
    el('insp-btn-wire').classList.toggle('insp-btn-active', _wireframe);
  }

  function toggleRotate() {
    _autoRotate = !_autoRotate;
    el('insp-btn-rotate').classList.toggle('insp-btn-active', _autoRotate);
  }

  return {
    open,
    close,
    render,
    togglePoints,
    toggleNormals,
    toggleWire,
    toggleRotate,
    isOpen: () => el('inspection-modal').style.display === 'flex',
    isInitialized: () => _initialized,
  };
})();