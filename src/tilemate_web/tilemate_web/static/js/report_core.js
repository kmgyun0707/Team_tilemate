(() => {
  const LATEST_API_URL = '/api/inspect/latest';
  const DUMMY_API_URL = '/api/inspect/dummy';

  const state = {
    data: null,
    wallRef: null,
    lastSig: null,
    sortMode: 'dz', // dz | anomaly | tilt | name
  };

  const el = (id) => document.getElementById(id);
  const fmt = (v, d = 2) =>
    Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '-';

  function signature(data) {
    return JSON.stringify({
      ts: data?.timestamp_sec ?? null,
      tiles: (data?.tiles || []).map((t) => [
        t.name,
        t.center_uv,
        t.plane_centroid_mm,
        t.rpy_deg,
        t.anomaly_score,
      ]),
    });
  }

  function tiltDeg(rpy) {
    if (!Array.isArray(rpy)) return 0;
    const vals = rpy.map((v) => Math.abs(Number(v))).filter(Number.isFinite);
    return vals.length ? Math.max(...vals) : 0;
  }

  function classify(mm) {
    const warn = Number(el('warnMm')?.value || 3.0);
    const bad = Number(el('badMm')?.value || 6.0);

    if (mm >= bad) return { cls: 'bad', text: '불량', color: '#ef4444' };
    if (mm >= warn) return { cls: 'warn', text: '주의', color: '#f59e0b' };
    return { cls: 'good', text: '정상', color: '#22c55e' };
  }

  function anomalyThreshold() {
    return Number(el('anomThresh')?.value || 0.5);
  }

  function anomalyJudge(score) {
    const t = anomalyThreshold();

    if (!Number.isFinite(score)) {
      return { cls: 'none', label: '-', color: '#94a3b8' };
    }
    if (score >= t) {
      return { cls: 'bad', label: 'anomaly', color: '#ef4444' };
    }
    if (score >= t * 0.7) {
      return { cls: 'warn', label: 'warn', color: '#f59e0b' };
    }
    return { cls: 'good', label: 'normal', color: '#22c55e' };
  }

  function defectStatus(dzAbs, anomalyScore) {
    const badMm = Number(el('badMm')?.value || 6.0);
    const anomTh = Number(el('anomThresh')?.value || 0.5);

    const heightBad = Number.isFinite(dzAbs) && dzAbs >= badMm;
    const patternBad =
      Number.isFinite(anomalyScore) && anomalyScore >= anomTh;

    if (heightBad && patternBad) {
      return { cls: 'bad', text: '복합불량', color: '#ef4444' };
    }
    if (heightBad) {
      return { cls: 'warn', text: '단차불량', color: '#f97316' };
    }
    if (patternBad) {
      return { cls: 'warn', text: '무늬 불량', color: '#a855f7' };
    }
    return { cls: 'good', text: '정상', color: '#22c55e' };
  }

  function loadWallRefFromData(data) {
    if (!data?.wall) return null;

    const n =
      data.wall.normal ||
      data.wall.plane_normal ||
      data.wall.normal_vec ||
      [0, 0, 1];

    const c =
      data.wall.centroid_m ||
      data.wall.center_point_m ||
      data.wall.plane_centroid_m ||
      null;

    const cmm =
      data.wall.plane_centroid_mm ||
      (Array.isArray(c) ? c.map((v) => Number(v) * 1000.0) : null);

    if (!Array.isArray(n) || !Array.isArray(cmm)) return null;

    return {
      n: [Number(n[0]), Number(n[1]), Number(n[2])],
      p: [Number(cmm[0]), Number(cmm[1]), Number(cmm[2])],
    };
  }

  function signedDistanceToWallPlaneMm(pointMm, wallRef) {
    if (!wallRef || !Array.isArray(pointMm) || pointMm.length < 3) {
      return NaN;
    }

    const [nx, ny, nz] = wallRef.n;
    const [px, py, pz] = wallRef.p;

    const dx = Number(pointMm[0]) - px;
    const dy = Number(pointMm[1]) - py;
    const dz = Number(pointMm[2]) - pz;

    const nNorm = Math.hypot(nx, ny, nz) || 1;
    return (nx * dx + ny * dy + nz * dz) / nNorm;
  }

  function signedDistanceToWallPlaneSafe(p) {
    const d = signedDistanceToWallPlaneMm(p, state.wallRef);
    return Number.isFinite(d) ? d : 0;
  }

  function sortTiles(tiles) {
    const cloned = [...tiles];

    switch (state.sortMode) {
      case 'anomaly':
        return cloned.sort(
          (a, b) => (b.anomalyScore || -Infinity) - (a.anomalyScore || -Infinity)
        );

      case 'tilt':
        return cloned.sort((a, b) => (b.tilt || -Infinity) - (a.tilt || -Infinity));

      case 'name':
        return cloned.sort((a, b) =>
          (a.name || '').localeCompare(b.name || '')
        );

      case 'dz':
      default:
        return cloned.sort((a, b) => (b.dzAbs || -Infinity) - (a.dzAbs || -Infinity));
    }
  }

  function normalizeTiles(data) {
    const rawTiles = Array.isArray(data?.tiles) ? data.tiles : [];
    state.wallRef = loadWallRefFromData(data);

    const normalized = rawTiles.map((tile, idx) => {
      const p = Array.isArray(tile.plane_centroid_mm)
        ? [
            Number(tile.plane_centroid_mm[0]),
            Number(tile.plane_centroid_mm[1]),
            Number(tile.plane_centroid_mm[2]),
          ]
        : Array.isArray(tile.centroid_m)
          ? tile.centroid_m.map((v) => Number(v) * 1000.0)
          : [NaN, NaN, NaN];

      const z = p[2];
      const dist = signedDistanceToWallPlaneSafe(p);
      const dzAbs = Math.abs(dist);
      const tilt = tiltDeg(tile.rpy_deg);
      const judge = classify(dzAbs);
      const anomalyScore = Number(tile.anomaly_score ?? NaN);
      const anomaly = anomalyJudge(anomalyScore);
      const status = defectStatus(dzAbs, anomalyScore);

      return {
        ...tile,
        idx: idx + 1,
        z,
        dz: dist,
        dzAbs,
        tilt,
        judge,
        anomalyScore,
        anomaly,
        status,
      };
    });

    return sortTiles(normalized);
  }

  function updateSummary(tiles, data) {
    el('mTiles').textContent = String(tiles.length);
    el('mBase').textContent = state.wallRef ? 'inspection.wall' : '-';
    el('mMaxDz').textContent = tiles.length
      ? fmt(Math.max(...tiles.map((t) => t.dzAbs || 0)), 2)
      : '-';
    el('mAvgTilt').textContent = tiles.length
      ? fmt(tiles.reduce((s, t) => s + (t.tilt || 0), 0) / tiles.length, 2)
      : '-';
    el('mBad').textContent = String(
      tiles.filter((t) =>
        ['단차불량', '무늬 불량', '복합불량'].includes(t.status?.text)
      ).length
    );
    el('mTime').textContent = data?.timestamp_sec
      ? new Date(data.timestamp_sec * 1000).toLocaleTimeString()
      : '-';
  }

  function renderTable(tiles) {
    const tbody = el('tileTableBody');
    tbody.innerHTML = '';

    if (!tiles.length) {
      tbody.innerHTML =
        '<tr><td colspan="7"><div class="emptyHint">타일 데이터가 없습니다.</div></td></tr>';
      return;
    }

    for (const t of tiles) {
      const tr = document.createElement('tr');

      tr.innerHTML = `
        <td><strong>${t.name ?? `tile_${t.idx}`}</strong></td>
        <td>${fmt(t.conf_score, 4)}</td>
        <td class="mono" style="font-weight:800;color:${t.judge.color};">
          ${t.dz >= 0 ? '+' : ''}${fmt(t.dz, 2)}
        </td>
        <td class="mono">${fmt(t.tilt, 2)}</td>
        <td class="mono">[${fmt(t.rpy_deg?.[0], 2)}, ${fmt(t.rpy_deg?.[1], 2)}, ${fmt(t.rpy_deg?.[2], 2)}]</td>
        <td class="mono" style="font-weight:800;color:${t.anomaly.color};">
          ${Number.isFinite(t.anomalyScore) ? fmt(t.anomalyScore, 3) : '-'}
        </td>
        <td>
          <span
            class="badge ${t.status.cls}"
            style="background:${t.status.color}22;border:1px solid ${t.status.color}66;color:${t.status.color};"
          >
            ${t.status.text}
          </span>
        </td>
      `;

      tbody.appendChild(tr);
    }
  }

  function drawMap(tiles) {
    const canvas = el('mapCanvas');
    const ctx = canvas.getContext('2d');

    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.scale(dpr, dpr);

    const w = rect.width;
    const h = rect.height;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0e152a';
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = 'rgba(148,163,184,0.12)';
    ctx.lineWidth = 1;

    for (let x = 40; x < w; x += 60) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, h);
      ctx.stroke();
    }

    for (let y = 40; y < h; y += 60) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (!tiles.length) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '16px sans-serif';
      ctx.fillText('inspection 데이터를 기다리는 중...', 40, 42);
      return;
    }

    const valid = tiles.filter(
      (t) =>
        Number.isFinite(Number(t.center_uv?.[0])) &&
        Number.isFinite(Number(t.center_uv?.[1]))
    );

    if (!valid.length) return;

    const us = valid.map((t) => Number(t.center_uv[0]));
    const vs = valid.map((t) => Number(t.center_uv[1]));
    const minU = Math.min(...us);
    const maxU = Math.max(...us);
    const minV = Math.min(...vs);
    const maxV = Math.max(...vs);

    const leftPad = 80;
    const topPad = 80;
    const bottomPad = 110;
    const rightPad = 260;

    const sx = (u) =>
      leftPad +
      ((u - minU) / Math.max(1, maxU - minU)) * (w - leftPad - rightPad);

    const sy = (v) =>
      topPad +
      ((v - minV) / Math.max(1, maxV - minV)) * (h - topPad - bottomPad);

    ctx.strokeStyle = 'rgba(96,165,250,0.25)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    valid.forEach((t, i) => {
      const x = sx(Number(t.center_uv[0]));
      const y = sy(Number(t.center_uv[1]));
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    for (const t of valid) {
      const x = sx(Number(t.center_uv[0]));
      const y = sy(Number(t.center_uv[1]));
      const radius = 14 + Math.min(34, (t.dzAbs || 0) * 2.8);
      const textX = x + radius + 8;

      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fillStyle = t.judge.color + '33';
      ctx.fill();
      ctx.lineWidth = 3;
      ctx.strokeStyle = t.judge.color;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#f8fafc';
      ctx.fill();

      ctx.fillStyle = '#e5e7eb';
      ctx.font = 'bold 15px sans-serif';
      ctx.fillText(String(t.name ?? `tile_${t.idx}`), textX, y - 12);

      ctx.font = '13px sans-serif';
      ctx.fillStyle = '#e5e7eb';
      ctx.fillText(`벽 기준 ${t.dz >= 0 ? '+' : ''}${fmt(t.dz, 2)} mm`, textX, y + 6);
      ctx.fillText(`Tilt ${fmt(t.tilt, 2)}°`, textX, y + 22);

      ctx.fillStyle = t.anomaly.color;
      ctx.fillText(
        `Anom ${Number.isFinite(t.anomalyScore) ? fmt(t.anomalyScore, 3) : '-'}`,
        textX,
        y + 38
      );

      ctx.fillStyle = t.status.color;
      ctx.fillText(`Status ${t.status.text}`, textX, y + 54);
    }
  }

  function render(data) {
    state.data = data;
    const tiles = normalizeTiles(data);

    updateSummary(tiles, data);
    renderTable(tiles);
    drawMap(tiles);

    el('statusLine').textContent =
      `loaded ${tiles.length} tiles @ ${
        data?.timestamp_sec
          ? new Date(data.timestamp_sec * 1000).toLocaleString()
          : '-'
      }`;
  }

  async function fetchInspectionUrl(url, force = false) {
    if (!url) return;

    el('statusLine').textContent = `loading... ${url}`;

    const res = await fetch(url, { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);

    const data = await res.json();
    if (!Array.isArray(data?.tiles)) throw new Error('tiles 필드가 없습니다.');

    if (!data.wall) {
      try {
        const wallRes = await fetch('/wall.json', { cache: 'no-store' });
        if (wallRes.ok) {
          const wallData = await wallRes.json();
          if (wallData?.wall) data.wall = wallData.wall;
          else if (wallData?.plane_normal || wallData?.normal) data.wall = wallData;
        }
      } catch (e) {
        console.warn('[INSPECT] wall.json fetch 실패 (무시):', e);
      }
    }

    const sig = signature(data);
    if (!force && sig === state.lastSig) {
      el('statusLine').textContent = 'same payload, skipped';
      return;
    }

    state.lastSig = sig;
    render(data);
  }

  async function fetchFromInput(force = false) {
    const url = el('urlInput').value.trim();
    await fetchInspectionUrl(url, force);
  }

  async function fetchLatestOnly(force = false) {
    await fetchInspectionUrl(LATEST_API_URL, force);
  }

  async function loadDummyOnce() {
    const res = await fetch(DUMMY_API_URL, {
      method: 'POST',
      cache: 'no-store',
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    return await res.json();
  }

  const helpers = {
    tiltDeg,
    classify,
    signedDistanceToWallPlaneMm,
    getWallRef: () => state.wallRef,
  };

  el('fetchBtn')?.addEventListener('click', async () => {
    try {
      await fetchFromInput(true);
    } catch (err) {
      console.error(err);
      el('statusLine').textContent = `error: ${err.message}`;
    }
  });

  el('warnMm')?.addEventListener('change', () => {
    if (state.data) render(state.data);
  });

  el('badMm')?.addEventListener('change', () => {
    if (state.data) render(state.data);
  });

  el('anomThresh')?.addEventListener('change', () => {
    if (state.data) render(state.data);
  });

  el('sortMode')?.addEventListener('change', () => {
    state.sortMode = el('sortMode').value;
    if (state.data) render(state.data);
  });

  window.addEventListener('resize', () => {
    if (state.data) drawMap(normalizeTiles(state.data));
  });

  el('force3dBtn')?.addEventListener('click', async () => {
    if (!state.data) return;
    await TileInspect3D.open();
    TileInspect3D.render(state.data, helpers);
  });

  el('inspCloseBtn')?.addEventListener('click', TileInspect3D.close);

  el('inspection-modal')?.addEventListener('click', (e) => {
    if (e.target === el('inspection-modal')) TileInspect3D.close();
  });

  el('insp-btn-points')?.addEventListener('click', TileInspect3D.togglePoints);
  el('insp-btn-normals')?.addEventListener('click', TileInspect3D.toggleNormals);
  el('insp-btn-wire')?.addEventListener('click', TileInspect3D.toggleWire);
  el('insp-btn-rotate')?.addEventListener('click', TileInspect3D.toggleRotate);

  // SSE: 전역 연결(_globalEs)이 있으면 재사용, 없으면 새로 생성
  function _attachSSE(source) {
    source.addEventListener('inspect_updated', async () => {
      try {
        await fetchLatestOnly(false);
        if (state.data && TileInspect3D.isOpen() && TileInspect3D.isInitialized()) {
          TileInspect3D.render(state.data, helpers);
        }
      } catch (err) {
        console.error(err);
        if (el('statusLine')) el('statusLine').textContent = `error: ${err.message}`;
      }
    });
  }

  if (window._globalEs) {
    _attachSSE(window._globalEs);
  } else {
    try {
      const es = new EventSource('/api/inspect/events');
      es.onerror = (err) => console.error('SSE error:', err);
      _attachSSE(es);
    } catch (err) {
      console.warn('SSE not available', err);
    }
  }

  // 초기 데이터 로드 (보고서 모달이 열릴 때까지 미루기)
  // openModal() 호출 시 자동 fetch됨

  // ── 공개 API ──
  window.ReportCore = {
    /** 보고서 모달 열기 + 데이터 fetch/render */
    openModal: async () => {
      document.getElementById('report-modal').classList.add('show');
      try {
        await fetchFromInput(false);
      } catch (err) {
        if (el('statusLine')) el('statusLine').textContent = `error: ${err.message}`;
      }
    },
    /** 이미 데이터가 있으면 re-render만, 없으면 fetch */
    openModalWithData: async (data) => {
      document.getElementById('report-modal').classList.add('show');
      if (data) {
        render(data);
      } else {
        try { await fetchFromInput(false); } catch(err) {}
      }
    },
    closeModal: () => {
      document.getElementById('report-modal').classList.remove('show');
    },
    isOpen: () => document.getElementById('report-modal').classList.contains('show'),
    refresh: (force = true) => fetchFromInput(force),
    getState: () => state,
  };
})();