/* ============================================================
   TILING MONITOR — main.js
   두산 M0609
   ============================================================ */

/* ════════════════════════════════════════════════════════════
   전역 단일 SSE 연결 (중복 연결 방지)
   ════════════════════════════════════════════════════════════ */
window._globalEs = null;
try {
    window._globalEs = new EventSource('/api/inspect/events');
    window._globalEs.onerror = (e) => { console.warn('[SSE] 연결 오류, 재연결 대기 중...', e); };
} catch (err) {
    console.warn('[SSE] 사용 불가:', err);
}

/* ════════════════════════════════════════════════════════════
   Firebase 초기화
   ════════════════════════════════════════════════════════════ */
let database;

const firebaseConfig = {
    apiKey: "AIzaSyDPPouqUOm8Ib6blpADMf-VQ_5r9-_IyEk",
    authDomain: "co1-tiling.firebaseapp.com",
    databaseURL: "https://co1-tiling-default-rtdb.asia-southeast1.firebasedatabase.app",
    projectId: "co1-tiling",
    storageBucket: "co1-tiling.firebasestorage.app",
    messagingSenderId: "925424727446",
    appId: "1:925424727446:web:85cc99cd5a478fa50759b7"
};

try {
    firebase.initializeApp(firebaseConfig);
    database = firebase.database();
    console.log('Firebase initialized successfully!');
} catch (error) {
    console.error('Firebase initialization error:', error);
}

/* ════════════════════════════════════════════════════════════
   상수 & 전역 상태
   ════════════════════════════════════════════════════════════ */
const TOTAL_TILES = 9; // 3x3
const COLS = 3;
const ROWS = 3;

let timerInterval    = null;
let startTime        = null;
let elapsedSeconds   = 0;
let prevCompletedJobs = 0;
let isStarted        = false;
let isEstopped       = false;

// 각 타일의 고유 색상 (red/black) 추적 — null이면 기본 상태색 사용
const tileColors = new Array(TOTAL_TILES + 1).fill(null); // 1-indexed
const tileState  = new Array(TOTAL_TILES + 1).fill('empty'); // 1-indexed
let currentDesign = 0; // 현재 선택된 디자인 (1/2/3)

// 디자인별 3x3 타일 색상 (1-indexed, R=빨강 B=검정 W=흰색)
const DESIGN_COLORS = {
    1: ['W','B','W','B','W','B','W','B','W'],  // WBWB 체커보드
    2: ['W','W','W','B','B','B','W','W','W'],  // W줄/B줄/W줄
    3: ['B','W','B','W','W','W','B','W','B'],  // 데코
};

function getTileColor(design, tileIdx) {
    const pattern = DESIGN_COLORS[design];
    if (!pattern) return null;
    const c = pattern[tileIdx - 1];
    if (c === 'B') return 0x212121;
    return 0xffffff; // W = 흰색
}

/* ════════════════════════════════════════════════════════════
   2D 타일 그리드 생성 (9칸: 3행 3열)
   ════════════════════════════════════════════════════════════ */
const tileMap = document.getElementById('tile-map');
const activeTimers = {};
for (let i = 1; i <= TOTAL_TILES; i++) {
    const tile = document.createElement('div');
    tile.classList.add('tile');
    tile.id = `tile-${i}`;
    tileMap.appendChild(tile);
}

/* ════════════════════════════════════════════════════════════
   Three.js 3D 설정 (메인 뷰)
   ════════════════════════════════════════════════════════════ */
const container = document.getElementById('three-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf0f2f5);

const camera = new THREE.PerspectiveCamera(40, container.clientWidth / container.clientHeight, 0.1, 1000);
camera.position.set(0, 0, 7);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.shadowMap.enabled = true;
container.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);

// 조명
scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const light = new THREE.DirectionalLight(0xffffff, 0.6);
light.position.set(4, 6, 8);
light.castShadow = true;
scene.add(light);

// 벽 (타일이 붙는 면, z=0 평면)
const tileGap  = 0.9;
const tileSize = 0.8;
const wallW = (COLS - 1) * tileGap + tileSize + 0.3;
const wallH = (ROWS - 1) * tileGap + tileSize + 0.3;

const wallMat  = new THREE.MeshStandardMaterial({ color: 0xd4c5a9 });
const wallMesh = new THREE.Mesh(new THREE.BoxGeometry(wallW, wallH, 0.15), wallMat);
wallMesh.position.set(0, 0, -0.12);
scene.add(wallMesh);

// 바닥
const floorMat   = new THREE.MeshStandardMaterial({ color: 0xb0bec5 });
const floorDepth = 1.2;
const floorMesh  = new THREE.Mesh(new THREE.BoxGeometry(wallW, 0.12, floorDepth), floorMat);
floorMesh.position.set(0, -(wallH / 2) - 0.06, -0.045 + floorDepth / 2);
scene.add(floorMesh);

// 3D 타일 생성 (3x3)
const tiles3D  = [];
const tileGeom = new THREE.BoxGeometry(tileSize, tileSize, 0.08);
const startX   = -((COLS - 1) * tileGap) / 2;
const startY   =  ((ROWS - 1) * tileGap) / 2;

for (let i = 0; i < TOTAL_TILES; i++) {
    const mesh = new THREE.Mesh(
        tileGeom,
        new THREE.MeshStandardMaterial({ color: 0xaaaaaa })
    );
    const row = Math.floor(i / COLS);
    const col = i % COLS;
    mesh.position.set(
        startX + col * tileGap,
        startY - row * tileGap,
        -0.045 + 0.04
    );
    scene.add(mesh);
    tiles3D.push(mesh);
}

// 그라우팅 선 (타일 사이 줄눈)
const groutMat = new THREE.LineBasicMaterial({ color: 0x888888, linewidth: 1 });
for (let r = 0; r <= ROWS; r++) {
    const y = startY + tileGap * 0.5 - (r * tileGap);
    const pts = [
        new THREE.Vector3(startX - tileSize / 2, y, -0.005 + 0.045),
        new THREE.Vector3(-startX + tileSize / 2, y, -0.005 + 0.045),
    ];
    scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), groutMat));
}
for (let c = 0; c <= COLS; c++) {
    const x = startX - tileGap * 0.5 + (c * tileGap);
    const pts = [
        new THREE.Vector3(x, startY + tileSize / 2, -0.005 + 0.045),
        new THREE.Vector3(x, -startY - tileSize / 2, -0.005 + 0.045),
    ];
    scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), groutMat));
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
});

/* ════════════════════════════════════════════════════════════
   파이차트 부드러운 보간 애니메이션
   ════════════════════════════════════════════════════════════ */
let _pieTarget  = 0.0;
let _pieCurrent = 0.0;
let _pieEl      = null;
let _pieColor   = '';

(function _pieLoop() {
    requestAnimationFrame(_pieLoop);
    if (!_pieEl) return;
    if (Math.abs(_pieCurrent - _pieTarget) < 0.001) {
        _pieCurrent = _pieTarget;
    } else {
        _pieCurrent += (_pieTarget - _pieCurrent) * 0.08;
    }
    const deg = (_pieCurrent * 360).toFixed(1);
    _pieEl.style.backgroundImage = `conic-gradient(${_pieColor} ${deg}deg, transparent 0deg)`;
})();

/* ════════════════════════════════════════════════════════════
   Firebase: robot_status 실시간 수신
   ════════════════════════════════════════════════════════════ */
database.ref('robot_status').on('value', (snapshot) => {
    const data = snapshot.val();
    if (!data) return;

    // ── 1. 센서 데이터 및 로봇 상태 업데이트 ──
    const rawJointSpeed = data.joint_speed || 0;
    document.getElementById('robot-speed').innerText = rawJointSpeed.toFixed(3);
    document.getElementById('robot-state').innerText = data.state || '대기';
    document.getElementById('job-count').innerText   = data.completed_jobs || 0;

    // ── 시멘트 대기 모달 (2단계) ──
    const cementModal = document.getElementById('cement-modal');
    const phasePick   = document.getElementById('cement-phase-pick');
    const phaseCement = document.getElementById('cement-phase-cement');
    const cs = data.cement_state || '';
    if (cs === 'waiting_pick') {
        phasePick.style.display   = 'block';
        phaseCement.style.display = 'none';
        cementModal.classList.add('show');
        updateCementSttUI('pick', data.stt_mic_state || '');
    } else if (cs === 'waiting_cement') {
        phasePick.style.display   = 'none';
        phaseCement.style.display = 'block';
        cementModal.classList.add('show');
        updateCementSttUI('cement', data.stt_mic_state || '');
    } else {
        cementModal.classList.remove('show');
    }

    const forceZ     = data.force_z     !== undefined ? parseFloat(data.force_z).toFixed(2)     : '0.00';
    const forceTotal = data.force_total !== undefined ? parseFloat(data.force_total).toFixed(2) : '0.00';
    document.getElementById('force-z').innerText     = forceZ;
    document.getElementById('force-total').innerText = forceTotal;

    if (data.ext_torque) {
        const t = data.ext_torque;
        document.getElementById('ext-torque-display').innerHTML =
            `J1: ${(t['0']||0).toFixed(2)} &nbsp; J2: ${(t['1']||0).toFixed(2)} &nbsp; J3: ${(t['2']||0).toFixed(2)} &nbsp; J4: ${(t['3']||0).toFixed(2)} &nbsp; J5: ${(t['4']||0).toFixed(2)} &nbsp; J6: ${(t['5']||0).toFixed(2)}`;
    }

    // 충돌 감지 비상 정지 로직
    if (data.state && data.state.includes('충돌 감지')) {
        const overlay = document.getElementById('estop-overlay');
        if (overlay.style.display !== 'flex') {
            overlay.style.display = 'flex';
            isEstopped = true;
            pauseTimer();
        }
        const resumeBtn = document.getElementById('resume-btn');
        if (resumeBtn) resumeBtn.style.display = 'none';
        const info = document.getElementById('collision-info');
        if (data.collision_joint !== undefined && data.collision_torque !== undefined) {
            info.style.display = 'block';
            if (data.collision_joint === 0) {
                info.innerText = `⚠ TCP 합력 ${parseFloat(data.collision_torque).toFixed(2)} N 초과`;
            } else if (data.collision_joint === -1) {
                info.innerText = `⚠ TCP Fz ${parseFloat(data.collision_torque).toFixed(2)} N 초과`;
            } else {
                info.innerText = `⚠ J${data.collision_joint} 관절 외부토크 ${parseFloat(data.collision_torque).toFixed(2)} Nm 초과`;
            }
        }
        const resetGuide = document.getElementById('collision-reset-guide');
        if (resetGuide) resetGuide.style.display = 'block';
    }

    if (isEstopped) return;

    // ── 2. 핵심 Enum 상태 변수 ──
    const overallStep = data.current_step || 0; // 0: READY, 1: WORK, 2: FINISH
    const tileStep    = data.tile_step    || 0; // 0~6
    if (data.design !== undefined && data.design !== null) currentDesign = parseInt(data.design);
    const currentJobs    = data.completed_jobs || 0;
    const workingTileId  = (data.working_tile !== undefined) ? parseInt(data.working_tile) + 1 : currentJobs + 1;
    _currentWorkingTileId = workingTileId;

    if (!isStarted && (overallStep === 1 || overallStep === 2 || currentJobs > 0)) {
        isStarted = true;
        if (currentDesign >= 1 && currentDesign <= 2) showPatternPreview(currentDesign);
        if (!timerInterval) startTimer();
    }

    // 전체 공정 진행률 업데이트
    const overallProgress = data.overall_progress || 0.0;
    document.getElementById('overall-progress-bar').style.width = (overallProgress * 100) + '%';
    document.getElementById('overall-progress-text').innerText  = (overallProgress * 100).toFixed(1) + '%';

    // ── 3. 상단 Process Flow UI 업데이트 ──
    let activeUiStep = 0;
    if (overallStep === 1) {
        if      (tileStep === 1) activeUiStep = 1; // PICK
        else if (tileStep === 2) activeUiStep = 2; // COWORK
        else if (tileStep === 3) activeUiStep = 3; // PLACE
        else if (tileStep === 4) activeUiStep = 4; // INSPECT
        else if (tileStep === 5) activeUiStep = 5; // COMPACT
    } else if (overallStep === 2) {
        activeUiStep = 0;
    }
    updateProcessStep(activeUiStep);

    // ── 4. 현재 작업 중인 타일의 렌더링 로직 ──
    if (isStarted && overallStep === 1 && workingTileId >= 1 && workingTileId <= TOTAL_TILES) {
        const tileEl  = document.getElementById(`tile-${workingTileId}`);
        const tColor  = getTileColor(currentDesign, workingTileId);
        const isBlack = (tColor === 0x212121);

        // COMPACT(5) 진입 시
        if (tileStep === 5 && _lastInspTileStep === 4) {
            _pieEl = null; _pieCurrent = 0.0; _pieTarget = 0.0;
            if (tileEl) tileEl.style.backgroundImage = '';
            closeInspectionModalWithResult();

        // INSPECT(4)
        } else if (tileStep === 4) {
            _pieEl = null; _pieCurrent = 0.0; _pieTarget = 0.0;
            if (_lastInspTileStep !== 4) {
                for (let i = 1; i <= Math.max(currentJobs, workingTileId); i++) {
                    const el = document.getElementById(`tile-${i}`);
                    if (!el) continue;
                    const tc = getTileColor(currentDesign, i);
                    const ib = (tc === 0x212121);
                    el.classList.remove('working', 'running', 'finished');
                    el.style.backgroundImage = '';
                    el.style.color = ib ? '#ffffff' : '#555555';
                    el.innerText = '검수 중...';
                    tileState[i] = 'checking';
                }
                openInspectionModal();
            }

        // 나머지: 보호 조건 차단
        } else {
            const _protectedStates = ['checking','result-good','result-rmse-bad','result-rmse-warn','result-tilt-bad','result-both-bad'];
            if (tileEl && tileState[workingTileId] !== 'running' && tileState[workingTileId] !== 'finished' && !_protectedStates.includes(tileState[workingTileId])) {

                tileEl.style.backgroundColor = isBlack ? '#212121' : '#ffffff';
                tileEl.style.borderColor     = isBlack ? '#000000' : '#cccccc';
                tiles3D[workingTileId - 1].material.color.setHex(tColor !== null ? tColor : 0xffffff);

                // PICK / COWORK / PLACE
                if (tileStep === 1 || tileStep === 2 || tileStep === 3) {
                    tileEl.classList.add('working');
                    tileEl.style.color = isBlack ? '#ffffff' : '#333333';
                    if      (tileStep === 1) tileEl.innerText = '파지 중';
                    else if (tileStep === 2) tileEl.innerText = '시멘트';
                    else                    tileEl.innerText = '부착 중';
                    tileState[workingTileId] = 'working';
                    const tileProgress   = data.tile_progress || 0.0;
                    const overlayColor   = isBlack ? 'rgba(255,255,255,0.4)' : 'rgba(0,0,0,0.2)';
                    if (_pieEl !== tileEl) { _pieCurrent = 0.0; }
                    _pieEl = tileEl; _pieColor = overlayColor; _pieTarget = tileProgress;

                // COMPACT (4에서 온 게 아닌 경우)
                } else if (tileStep === 5) {
                    tileEl.classList.remove('working');
                    tileEl.style.backgroundImage = '';
                    _pieEl = null; _pieCurrent = 0.0; _pieTarget = 0.0;
                    tileEl.style.color  = isBlack ? '#aaffaa' : '#2e7d32';
                    tileEl.innerText    = '압착 중';
                    tileState[workingTileId] = 'pressing';
                }
            }
        }
    }

    _lastInspTileStep = tileStep;

    // ── 5. 작업이 완료(DONE)된 타일들 타이머 시작 처리 ──
    if (currentJobs > prevCompletedJobs) {
        for (let i = prevCompletedJobs + 1; i <= currentJobs; i++) {
            if (!['running', 'finished'].includes(tileState[i])) {
                const el = document.getElementById(`tile-${i}`);
                if (el) {
                    el.style.backgroundImage = '';
                    el.innerText = '완료';
                }
                tileState[i] = 'finished';
                if (el) { el.classList.add('finished'); el.innerText = '완료'; }
                const colorJob = getTileColor(currentDesign, i);
                tiles3D[i - 1].material.color.setHex(colorJob !== null ? colorJob : 0xffffff);
            }
        }
    }
    prevCompletedJobs = currentJobs;

    // ── 6. 전체 공정 완료 (OVERALL_FINISHED) ──
    if (overallStep === 2 && isStarted && !document.getElementById('finish-popup').classList.contains('show')) {
        document.getElementById('finish-popup').classList.add('show');
    }
});

/* ════════════════════════════════════════════════════════════
   Process Flow UI
   ════════════════════════════════════════════════════════════ */
function updateProcessStep(stepNumber) {
    for (let i = 1; i <= 5; i++) {
        const el = document.getElementById('step' + i);
        if (el) el.classList.remove('active');
    }
    if (stepNumber > 0 && stepNumber <= 5) {
        const el = document.getElementById('step' + stepNumber);
        if (el) el.classList.add('active');
    }
}

/* ════════════════════════════════════════════════════════════
   패턴 프리뷰
   ════════════════════════════════════════════════════════════ */
const PATTERNS = {
    1: ['W','B','W', 'B','W','B', 'W','B','W'],
    2: ['W','W','W', 'B','B','B', 'W','W','W'],
    3: ['B','W','B', 'W','W','W', 'B','W','B'],
};

let selectedDesign = null;

function buildPreviews() {
    [1, 2, 3].forEach(id => {
        const grid = document.getElementById('preview-' + id);
        grid.innerHTML = '';
        PATTERNS[id].forEach(c => {
            const cell = document.createElement('div');
            cell.className = 'preview-cell ' + (c === 'B' ? 'pc-black' : '');
            grid.appendChild(cell);
        });
    });
}
buildPreviews();

const patternMiniGrid = document.getElementById('pattern-mini-grid');
for (let i = 0; i < 9; i++) {
    const cell = document.createElement('div');
    cell.id = 'pmini-' + i;
    cell.style.cssText = 'width:20px; height:20px; border-radius:3px; border:1px solid #ccc; background:#e0e0e0;';
    patternMiniGrid.appendChild(cell);
}

function showPatternPreview(design) {
    const pattern = PATTERNS[design];
    if (!pattern) return;
    for (let i = 0; i < 9; i++) {
        const cell = document.getElementById('pmini-' + i);
        if (!cell) continue;
        if (pattern[i] === 'B') {
            cell.style.background = '#212121'; cell.style.border = '1px solid #000';
        } else {
            cell.style.background = '#ffffff'; cell.style.border = '1px solid #bbb';
        }
    }
    document.getElementById('pattern-preview').style.display = 'block';
}

function showCustomPatternPreview(values) {
    for (let i = 0; i < 9; i++) {
        const cell = document.getElementById('pmini-' + i);
        if (!cell) continue;
        if (values[i] === 'B') {
            cell.style.background = '#212121'; cell.style.border = '1px solid #000';
        } else {
            cell.style.background = '#ffffff'; cell.style.border = '1px solid #bbb';
        }
    }
    document.getElementById('pattern-preview').style.display = 'block';
}

function hidePatternPreview() {
    document.getElementById('pattern-preview').style.display = 'none';
}

/* ════════════════════════════════════════════════════════════
   디자인 팝업
   ════════════════════════════════════════════════════════════ */
function openDesignPopup() {
    selectedDesign = null;
    [1,2,3].forEach(i => document.getElementById('card-'+i).classList.remove('selected'));
    document.getElementById('design-popup').classList.add('show');
}

function closePopup() {
    document.getElementById('design-popup').classList.remove('show');
    document.getElementById('custom-pattern-popup').classList.remove('show');
}

function selectDesign(n) {
    selectedDesign = n;
    [1,2,3].forEach(i => document.getElementById('card-'+i).classList.remove('selected'));
    document.getElementById('card-'+n).classList.add('selected');
}

function speakTTS(text) {
    database.ref('robot_status/tts_start_b64').once('value', (snap) => {
        const b64 = snap.val();
        if (b64) {
            const audio = new Audio('data:audio/mp3;base64,' + b64);
            audio.play().catch(() => _speakFallback(text));
        } else {
            _speakFallback(text);
        }
    });
}

function _speakFallback(text) {
    if (!window.speechSynthesis) return;
    window.speechSynthesis.cancel();
    const utter = new SpeechSynthesisUtterance(text);
    utter.lang = 'ko-KR';
    window.speechSynthesis.speak(utter);
}

function confirmDesign() {
    if (!selectedDesign) { alert('패턴을 선택해주세요!'); return; }
    if (selectedDesign === 3) {
        const allOnes = '1,1,1,1,1,1,1,1,1';
        closePopup();
        database.ref('robot_command').set({ action: 'start', design: 3, custom_pattern: allOnes, is_resume: false, timestamp: Date.now() });
        DESIGN_COLORS[3] = new Array(9).fill('W');
        const allA = new Array(9).fill('A');
        isStarted = true; showCustomPatternPreview(allA);
        speakTTS('작업을 시작합니다');
        if (!timerInterval) startTimer();
        return;
    }
    closePopup();
    database.ref('robot_command').set({ action: 'start', design: selectedDesign, is_resume: false, timestamp: Date.now() });
    isStarted = true; showPatternPreview(selectedDesign);
    speakTTS('작업을 시작합니다');
    if (!timerInterval) startTimer();
}

/* ════════════════════════════════════════════════════════════
   커스텀 패턴 팝업
   ════════════════════════════════════════════════════════════ */
const customValues = new Array(9).fill('A');

function openCustomPatternPopup() {
    customValues.fill('A');
    const grid = document.getElementById('custom-input-grid');
    grid.innerHTML = '';
    for (let i = 0; i < 9; i++) {
        const wrapper = document.createElement('div');
        wrapper.style.cssText = 'display:flex; flex-direction:column; align-items:center; gap:6px;';
        const label = document.createElement('div');
        label.style.cssText = 'font-size:0.8em; color:#888;';
        label.innerText = `타일 ${i+1}`;
        const btn = document.createElement('button');
        btn.id = `custom-btn-${i}`;
        btn.style.cssText = `width:64px; height:64px; font-size:0.88em; font-weight:bold; border-radius:10px; cursor:pointer; border:3px solid; transition:0.2s; flex:unset; padding:0;`;
        applyCustomBtnStyle(btn, customValues[i]);
        btn.onclick = () => {
            const cycle = { 'A': 'B', 'B': 'C', 'C': 'A' };
            customValues[i] = cycle[customValues[i]] || 'A';
            applyCustomBtnStyle(btn, customValues[i]);
            updateCustomPreview();
        };
        wrapper.appendChild(label);
        wrapper.appendChild(btn);
        grid.appendChild(wrapper);
    }
    updateCustomPreview();
    document.getElementById('custom-pattern-popup').classList.add('show');
}

function applyCustomBtnStyle(btn, val) {
    if (val === 'B') {
        btn.style.background = '#212121'; btn.style.color = '#fff'; btn.style.borderColor = '#000';
        btn.innerText = '검정';
    } else if (val === 'C') {
        btn.style.background = '#d4a96a'; btn.style.color = '#fff'; btn.style.borderColor = '#a0784a';
        btn.innerText = '데코';
    } else {
        btn.style.background = '#ffffff'; btn.style.color = '#333'; btn.style.borderColor = '#aaa';
        btn.innerText = '흰색';
    }
}

function updateCustomPreview() {
    const pg = document.getElementById('custom-preview-grid');
    pg.innerHTML = '';
    customValues.forEach(v => {
        const cell = document.createElement('div');
        cell.className = 'preview-cell';
        if (v === 'B')      cell.style.background = '#212121';
        else if (v === 'C') cell.style.background = '#d4a96a';
        else                cell.style.background = '#ffffff';
        pg.appendChild(cell);
    });
}

function confirmCustomPattern() {
    const patternStr = customValues.join(',');
    closePopup();
    database.ref('robot_command').set({ action: 'start', design: 3, custom_pattern: patternStr, is_resume: false, timestamp: Date.now() });
    DESIGN_COLORS[3] = customValues.map(v => v === 'B' ? 'B' : 'W');
    isStarted = true; showCustomPatternPreview(customValues);
    speakTTS('작업을 시작합니다');
    if (!timerInterval) startTimer();
}

function backToDesignPopup() {
    document.getElementById('custom-pattern-popup').classList.remove('show');
    document.getElementById('design-popup').classList.add('show');
}

/* ════════════════════════════════════════════════════════════
   로봇 커맨드
   ════════════════════════════════════════════════════════════ */
function sendManualConfirm() {
    database.ref('robot_status').update({ manual_confirm: true });
}

function sendCommand(cmd) {
    database.ref('robot_command').set({ action: cmd, timestamp: Date.now() });
    if (cmd === 'start') {
        isStarted = true; if (!timerInterval) startTimer();
    } else if (cmd === 'stop') {
        pauseTimer(); isEstopped = true;
        document.getElementById('estop-overlay').style.display = 'flex';
    }
}

function resumeRobot() {
    database.ref('robot_status').once('value', (snapshot) => {
        const status = snapshot.val() || {};
        database.ref('robot_command').set({
            action: 'resume',
            current_step: status.current_step || 0,
            completed_jobs: status.completed_jobs || 0,
            timestamp: Date.now()
        });
    });
    document.getElementById('estop-overlay').style.display = 'none';
    isEstopped = false; startTimer();
}

/* ════════════════════════════════════════════════════════════
   타이머
   ════════════════════════════════════════════════════════════ */
function startTimer() {
    if (timerInterval) return;
    startTime = Date.now() - (elapsedSeconds * 1000);
    timerInterval = setInterval(() => {
        elapsedSeconds = Math.floor((Date.now() - startTime) / 1000);
        updateTimerDisplay();
    }, 1000);
}

function updateTimerDisplay() {
    const h = Math.floor(elapsedSeconds / 3600);
    const m = Math.floor((elapsedSeconds % 3600) / 60);
    const s = elapsedSeconds % 60;
    document.getElementById('timer').innerText =
        `${String(h).padStart(2,'0')} : ${String(m).padStart(2,'0')} : ${String(s).padStart(2,'0')}`;
}

function pauseTimer() {
    if (timerInterval) { clearInterval(timerInterval); timerInterval = null; }
}

/* ════════════════════════════════════════════════════════════
   시스템 초기화
   ════════════════════════════════════════════════════════════ */
function resetSystem() {
    pauseTimer();
    elapsedSeconds = 0; isStarted = false; isEstopped = false; prevCompletedJobs = 0;
    document.getElementById('timer').innerText = '00 : 00 : 00';
    for (let id in activeTimers) clearInterval(activeTimers[id]);
    Object.keys(activeTimers).forEach(key => delete activeTimers[key]);
    document.getElementById('estop-overlay').style.display = 'none';
    document.getElementById('finish-popup').classList.remove('show');
    document.getElementById('overall-progress-bar').style.width  = '0%';
    document.getElementById('overall-progress-text').innerText   = '0.0%';
    document.getElementById('cement-modal').classList.remove('show');
    closeInspectionModal();
    _lastInspTileStep = -1;
    for (let i = 1; i <= TOTAL_TILES; i++) {
        tileState[i] = 'empty';
        const el = document.getElementById(`tile-${i}`);
        if (el) {
            el.classList.remove('working','coated','running','finished','result-good','result-rmse-bad','result-rmse-warn','result-tilt-bad','result-both-bad');
            el.style.backgroundColor = ''; el.style.borderColor = '';
            el.innerText = ''; el.style.backgroundImage = '';
        }
    }
    tiles3D.forEach(mesh => {
        mesh.material.color.setHex(0xaaaaaa);
        mesh.rotation.set(0, 0, 0);
        mesh.position.z = -0.005;
    });
    updateProcessStep(0); hidePatternPreview();
    database.ref('robot_status').update({
        completed_jobs: 0, current_step: 0, state: '대기',
        inspect_no: 0, tile_level: 0, press_no: 0,
        overall_progress: 0.0, tile_progress: 0.0,
        tile_step: 0, working_tile: 0,
        cement_state: 'idle',
    });
    database.ref('robot_command').set({ action: 'reset', timestamp: Date.now() });
}

function startTileTimer(id, sec) {
    let left = sec;
    const el = document.getElementById(`tile-${id}`);
    if (!el || el.classList.contains('finished')) return;
    el.classList.remove('working','coated','finished'); el.classList.add('running');
    tileState[id] = 'running';
    activeTimers[id] = setInterval(() => {
        left--;
        const m = Math.floor(left / 60); const s = left % 60;
        el.innerText = `${m}:${s < 10 ? '0' + s : s}`;
        if (left <= 0) {
            clearInterval(activeTimers[id]);
            el.innerText = '완료';
            el.classList.replace('running', 'finished');
            tileState[id] = 'finished';
        }
    }, 1000);
}

/* ════════════════════════════════════════════════════════════
   WebSocket 카메라 스트림
   중요: 로봇에서 웹 켜면 192.168.10.39 로 변경!
   ════════════════════════════════════════════════════════════ */
//const WS_URL = 'ws://192.168.10.39:8765';
//const WS_URL = 'ws://172.20.10.2:8765';
const WS_URL = 'ws://192.168.10.39:8765';

let ws = null;
let wsReconnectTimer = null;

let colorFrameCount = 0, depthFrameCount = 0;
let colorFpsInterval = null, depthFpsInterval = null;

let colorSubscribing = false;
let depthSubscribing = false;
let pc3dSubscribing  = false;

function wsConnect() {
    if (ws && ws.readyState <= 1) return;
    ws = new WebSocket(WS_URL);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        console.log('[WS] 연결 성공:', WS_URL);
        if (wsReconnectTimer) { clearTimeout(wsReconnectTimer); wsReconnectTimer = null; }
    };
    ws.onclose = () => {
        console.warn('[WS] 연결 끊김 → 3초 후 재연결');
        wsReconnectTimer = setTimeout(wsConnect, 3000);
    };
    ws.onerror = (e) => { console.error('[WS] 오류:', e); };

    ws.onmessage = (event) => {
        const buf    = event.data;
        const view   = new Uint8Array(buf);
        const header = String.fromCharCode(view[0], view[1], view[2], view[3], view[4]);
        const payload = buf.slice(5);

        if      (header === 'COLOR' && colorSubscribing) { colorFrameCount++; renderColorImageWS(payload); }
        else if (header === 'DEPTH' && depthSubscribing) { depthFrameCount++; renderDepthImageWS(payload); }
        else if (header === 'POINT' && pc3dSubscribing)  { renderPointCloud(JSON.parse(new TextDecoder().decode(payload))); }
    };
}

function renderColorImageWS(payload) {
    const blob = new Blob([payload], { type: 'image/jpeg' });
    const url  = URL.createObjectURL(blob);
    const img  = new Image();
    img.onload = () => {
        const canvas = document.getElementById('color-canvas');
        canvas.width  = img.naturalWidth;
        canvas.height = img.naturalHeight;
        canvas.getContext('2d').drawImage(img, 0, 0);
        document.getElementById('color-placeholder').style.display = 'none';
        canvas.style.display = 'block';
        const infoBar = document.getElementById('color-info-bar');
        infoBar.style.display = 'flex';
        document.getElementById('color-resolution').innerText =
            `해상도: ${img.naturalWidth} × ${img.naturalHeight}`;
        URL.revokeObjectURL(url);
    };
    img.src = url;
}

function renderDepthImageWS(payload) {
    const meta  = new DataView(payload);
    const d_min = meta.getUint32(0, true);
    const d_max = meta.getUint32(4, true);
    const imgBuf = payload.slice(8);
    const blob = new Blob([imgBuf], { type: 'image/png' });
    const url  = URL.createObjectURL(blob);
    const img  = new Image();
    img.onload = () => {
        const canvas = document.getElementById('depth-canvas');
        canvas.width  = img.naturalWidth;
        canvas.height = img.naturalHeight;
        canvas.getContext('2d').drawImage(img, 0, 0);
        document.getElementById('pc-placeholder').style.display = 'none';
        canvas.style.display = 'block';
        document.getElementById('legend-min').innerText      = `${d_min}mm`;
        document.getElementById('legend-max').innerText      = `${d_max}mm`;
        document.getElementById('pc-depth-range').innerText  = `깊이: ${d_min}~${d_max}mm`;
        const legend = document.getElementById('depth-legend');
        if (legend) legend.style.display = 'flex';
        URL.revokeObjectURL(url);
    };
    img.src = url;
}

/* 컬러 카메라 구독 버튼 */
function toggleColorSubscription() { colorSubscribing ? stopColorSubscription() : startColorSubscription(); }
function startColorSubscription() {
    colorSubscribing = true;
    wsConnect();
    document.getElementById('color-subscribe-btn').innerText     = '수신 중지';
    document.getElementById('color-subscribe-btn').style.background = '#c62828';
    document.getElementById('color-status').innerHTML            = '<span style="color:#4caf50;">● 수신 중</span>';
    colorFpsInterval = setInterval(() => {
        document.getElementById('color-fps').innerText = `${colorFrameCount} FPS`;
        colorFrameCount = 0;
    }, 1000);
}
function stopColorSubscription() {
    colorSubscribing = false;
    if (colorFpsInterval) { clearInterval(colorFpsInterval); colorFpsInterval = null; }
    document.getElementById('color-subscribe-btn').innerText     = '수신 시작';
    document.getElementById('color-subscribe-btn').style.background = '#1565c0';
    document.getElementById('color-status').innerHTML            = '● 대기 중';
    document.getElementById('color-fps').innerText               = '';
}

/* 깊이 이미지 구독 버튼 */
function toggleDepthSubscription() { depthSubscribing ? stopDepthSubscription() : startDepthSubscription(); }
function startDepthSubscription() {
    depthSubscribing = true;
    wsConnect();
    document.getElementById('pc-subscribe-btn').innerText     = '수신 중지';
    document.getElementById('pc-subscribe-btn').style.background = '#c62828';
    document.getElementById('pc-status').innerHTML            = '<span style="color:#4caf50;">● 수신 중</span>';
    depthFpsInterval = setInterval(() => {
        document.getElementById('pc-fps').innerText = `${depthFrameCount} FPS`;
        depthFrameCount = 0;
    }, 1000);
}
function stopDepthSubscription() {
    depthSubscribing = false;
    if (depthFpsInterval) { clearInterval(depthFpsInterval); depthFpsInterval = null; }
    document.getElementById('pc-subscribe-btn').innerText     = '수신 시작';
    document.getElementById('pc-subscribe-btn').style.background = '#1565c0';
    document.getElementById('pc-status').innerHTML            = '● 대기 중';
}

/* ════════════════════════════════════════════════════════════
   Point Cloud 3D Viewer
   ════════════════════════════════════════════════════════════ */
let pc3dScene, pc3dCamera, pc3dRenderer, pc3dControls, pc3dPoints;

function initPC3D() {
    const container = document.getElementById('pc3d-container');
    if (!container || pc3dRenderer) return;
    pc3dScene = new THREE.Scene();
    pc3dScene.background = new THREE.Color(0x060d1a);
    pc3dCamera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.01, 100);
    pc3dCamera.position.set(0, 0, 2);
    pc3dRenderer = new THREE.WebGLRenderer({ antialias: true });
    pc3dRenderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(pc3dRenderer.domElement);
    pc3dControls = new THREE.OrbitControls(pc3dCamera, pc3dRenderer.domElement);
    pc3dControls.enableDamping = true;
    pc3dScene.add(new THREE.AxesHelper(0.3));
    (function animate() {
        requestAnimationFrame(animate);
        if (pc3dControls) pc3dControls.update();
        if (pc3dRenderer && pc3dScene && pc3dCamera) pc3dRenderer.render(pc3dScene, pc3dCamera);
    })();
}

function togglePCSubscription() { pc3dSubscribing ? stopPCSubscription() : startPCSubscription(); }
function startPCSubscription() {
    initPC3D();
    pc3dSubscribing = true;
    wsConnect();
    const btn = document.getElementById('pc3d-subscribe-btn');
    btn.innerText = '수신 중지'; btn.style.background = '#c62828';
    document.getElementById('pc3d-status').innerHTML = '<span style="color:#66bb6a;">● 수신 중</span>';
}
function stopPCSubscription() {
    pc3dSubscribing = false;
    const btn = document.getElementById('pc3d-subscribe-btn');
    btn.innerText = '수신 시작'; btn.style.background = '#1565c0';
    document.getElementById('pc3d-status').innerHTML = '● 대기 중';
}

function openPointCloudViewer() { document.getElementById('pointcloud-popup').classList.add('show'); }
function closeDepthViewer() {
    document.getElementById('pointcloud-popup').classList.remove('show');
    stopColorSubscription(); stopDepthSubscription(); stopPCSubscription();
    currentRole = 'user';
    document.getElementById('role-toggle').classList.remove('admin-mode');
    document.getElementById('btn-user').classList.add('active-user');
    document.getElementById('btn-user').classList.remove('active-admin');
    document.getElementById('btn-admin').classList.remove('active-admin');
}

function renderPointCloud(data) {
    const xs = data.x, ys = data.y, zs = data.z;
    const rs = data.r, gs = data.g, bs = data.b;
    const n  = xs.length;

    document.getElementById('pc3d-placeholder').style.display = 'none';
    document.getElementById('pc3d-count').innerText = `${n.toLocaleString()} pts`;

    const positions = new Float32Array(n * 3);
    const colors    = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
        positions[i*3]   = xs[i];
        positions[i*3+1] = -ys[i];
        positions[i*3+2] = -zs[i];
        colors[i*3]   = rs[i] / 255;
        colors[i*3+1] = gs[i] / 255;
        colors[i*3+2] = bs[i] / 255;
    }

    if (pc3dPoints) { pc3dScene.remove(pc3dPoints); pc3dPoints.geometry.dispose(); }
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setAttribute('color',    new THREE.BufferAttribute(colors, 3));
    pc3dPoints = new THREE.Points(geom, new THREE.PointsMaterial({ size: 0.005, vertexColors: true }));
    pc3dScene.add(pc3dPoints);
}

/* ════════════════════════════════════════════════════════════
   음성 패턴 선택 (STT + Keyword Extraction)
   ════════════════════════════════════════════════════════════ */
let voiceLayout = null;
let voiceCountdownTimer = null;
let voicePollingTimer   = null;

function openVoicePopup() {
    voiceLayout = null;
    resetVoiceUI();
    document.getElementById('voice-popup').classList.add('show');
}

function closeVoicePopup() {
    document.getElementById('voice-popup').classList.remove('show');
    clearVoiceTimers();
    stopVoiceFFT();
}

function switchToManualPopup() { closeVoicePopup(); openDesignPopup(); }

function resetVoiceUI() {
    clearVoiceTimers();
    ['voice-idle','voice-calibrating','voice-recording','voice-processing','voice-done','voice-error']
        .forEach(id => { const el = document.getElementById(id); if (el) el.style.display = 'none'; });
    document.getElementById('voice-idle').style.display       = 'flex';
    document.getElementById('voice-record-btn').style.display = 'block';
    document.getElementById('voice-confirm-btn').style.display = 'none';
}

function clearVoiceTimers() {
    if (voiceCountdownTimer) { clearInterval(voiceCountdownTimer); voiceCountdownTimer = null; }
    if (voicePollingTimer)   { clearInterval(voicePollingTimer);   voicePollingTimer   = null; }
}

/* ════════════════════════════════════════════════════════════
   공통 FFT 드로어
   ════════════════════════════════════════════════════════════ */
function _fftDraw(ctx, W, H, dataArr, bufLen, sampleRate, isCalib) {
    ctx.fillStyle = '#0a0a14';
    ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = isCalib ? '#ef6c0022' : '#c6282822';
    ctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
        ctx.beginPath(); ctx.moveTo(0, H*i/4); ctx.lineTo(W, H*i/4); ctx.stroke();
    }

    const displayBins = Math.min(Math.floor(8000 / (sampleRate/2) * bufLen), bufLen);
    const barW  = W / displayBins;
    const vLow  = Math.floor(80   / (sampleRate/2) * bufLen);
    const vHigh = Math.floor(3400 / (sampleRate/2) * bufLen);
    let peakIdx = vLow, peakVal = 0;
    for (let i = vLow; i < vHigh; i++) {
        if (dataArr[i] > peakVal) { peakVal = dataArr[i]; peakIdx = i; }
    }

    for (let i = 0; i < displayBins; i++) {
        const v    = dataArr[i] / 255;
        const barH = v * H;
        const ratio = i / displayBins;
        let r, g, b;
        if (isCalib) {
            r = Math.round(180 + ratio*50); g = Math.round(80 + ratio*80); b = 20;
        } else {
            if      (ratio < 0.3)  { r=Math.round(20+ratio/0.3*30);        g=Math.round(100+ratio/0.3*155); b=230; }
            else if (ratio < 0.65) { const t=(ratio-0.3)/0.35;  r=Math.round(50+t*180);  g=Math.round(255-t*50);  b=Math.round(255-t*200); }
            else                   { const t=(ratio-0.65)/0.35; r=Math.round(230+t*25);  g=Math.round(205-t*180); b=Math.round(55+t*150); }
        }
        const isPeak = peakVal > 60 && Math.abs(i - peakIdx) <= 5;
        ctx.fillStyle = `rgba(${r},${g},${b},${isPeak ? 1.0 : 0.5 + v*0.5})`;
        ctx.fillRect(i*barW, H-barH, Math.max(barW-0.5,0.5), barH);
        if (barH > 2) {
            ctx.fillStyle = `rgba(255,255,255,${v*0.5})`;
            ctx.fillRect(i*barW, H-barH, Math.max(barW-0.5,0.5), 1.5);
        }
    }
    if (peakVal > 60) {
        const px = (peakIdx/displayBins)*W;
        const grd = ctx.createLinearGradient(px,0,px,H);
        grd.addColorStop(0,'rgba(255,255,255,0.7)'); grd.addColorStop(1,'rgba(255,255,255,0)');
        ctx.strokeStyle=grd; ctx.lineWidth=1.5;
        ctx.beginPath(); ctx.moveTo(px,0); ctx.lineTo(px,H); ctx.stroke();
    }
}

/* voice-popup FFT 더미 애니메이션 */
let _vFft = { animId:null, mode:'calib', phase:0, data:null };

function startVoiceFFT(mode) {
    _vFft.mode = mode;
    if (!_vFft.data) _vFft.data = new Uint8Array(256);
    if (!_vFft.animId) _drawVoiceFFT();
}
function switchVoiceFFTMode(mode) { _vFft.mode = mode; }
function stopVoiceFFT() {
    if (_vFft.animId) { cancelAnimationFrame(_vFft.animId); _vFft.animId = null; }
    ['fft-canvas-calib','fft-canvas'].forEach(id => {
        const c = document.getElementById(id);
        if (c) c.getContext('2d').clearRect(0,0,c.width,c.height);
    });
}
function _drawVoiceFFT() {
    _vFft.animId = requestAnimationFrame(_drawVoiceFFT);
    const isCalib = _vFft.mode === 'calib';
    const canvas  = document.getElementById(isCalib ? 'fft-canvas-calib' : 'fft-canvas');
    const other   = document.getElementById(isCalib ? 'fft-canvas' : 'fft-canvas-calib');
    if (!canvas) return;
    if (other) other.getContext('2d').clearRect(0, 0, other.width, other.height);
    _vFft.phase += isCalib ? 0.018 : 0.032;
    const bins = _vFft.data.length;
    for (let i = 0; i < bins; i++) {
        const freq = i / bins;
        const base = Math.max(0, 90 - freq * 160);
        const wave = Math.sin(_vFft.phase * 1.3 + i * 0.18) * 28
                   + Math.sin(_vFft.phase * 2.1 + i * 0.07) * 18
                   + Math.sin(_vFft.phase * 0.7 + i * 0.31) * 14;
        const amp   = isCalib ? 0.55 : 1.0;
        const noise = (Math.random() - 0.5) * (isCalib ? 12 : 22);
        _vFft.data[i] = Math.max(0, Math.min(255, Math.round((base + wave + noise) * amp)));
    }
    _fftDraw(canvas.getContext('2d'), canvas.width, canvas.height, _vFft.data, _vFft.data.length, 44100, isCalib);
}

/* cement-modal FFT 더미 애니메이션 */
let _cFft = { ctx:null, analyser:null, stream:null, animId:null, data:null, canvasId:null };
let _cFftDummyData  = null;
let _cFftDummyPhase = 0;

function _initCementDummyData() {
    if (_cFftDummyData) return;
    _cFftDummyData = new Uint8Array(256);
}
function _updateCementDummyData(isCalib) {
    _initCementDummyData();
    _cFftDummyPhase += isCalib ? 0.018 : 0.032;
    const bins = _cFftDummyData.length;
    for (let i = 0; i < bins; i++) {
        const freq  = i / bins;
        const base  = Math.max(0, 90 - freq * 160);
        const wave  = Math.sin(_cFftDummyPhase * 1.3 + i * 0.18) * 28
                    + Math.sin(_cFftDummyPhase * 2.1 + i * 0.07) * 18
                    + Math.sin(_cFftDummyPhase * 0.7 + i * 0.31) * 14;
        const amp   = isCalib ? 0.55 : 1.0;
        const noise = (Math.random() - 0.5) * (isCalib ? 12 : 22);
        _cFftDummyData[i] = Math.max(0, Math.min(255, Math.round((base + wave + noise) * amp)));
    }
}

function startCementFFT(canvasId) {
    _cFft.canvasId = canvasId;
    _initCementDummyData();
    if (!_cFft.animId) _drawCementFFT();
}
function switchCementFFTCanvas(canvasId) {
    _cFft.canvasId = canvasId;
    if (!_cFft.animId) _drawCementFFT();
}
function stopCementFFT() {
    if (_cFft.animId) { cancelAnimationFrame(_cFft.animId); _cFft.animId = null; }
    _cFft.canvasId = null;
    ['cement-fft-pick-calib','cement-fft-pick-listen','cement-fft-cement-calib','cement-fft-cement-listen'].forEach(id => {
        const c = document.getElementById(id); if (c) c.getContext('2d').clearRect(0, 0, c.width, c.height);
    });
}
function _drawCementFFT() {
    _cFft.animId = requestAnimationFrame(_drawCementFFT);
    if (!_cFft.canvasId) return;
    const canvas = document.getElementById(_cFft.canvasId);
    if (!canvas) return;
    ['cement-fft-pick-calib','cement-fft-pick-listen','cement-fft-cement-calib','cement-fft-cement-listen'].forEach(id => {
        if (id !== _cFft.canvasId) { const c = document.getElementById(id); if (c) c.getContext('2d').clearRect(0, 0, c.width, c.height); }
    });
    const isCalib = _cFft.canvasId.includes('calib');
    _updateCementDummyData(isCalib);
    _fftDraw(canvas.getContext('2d'), canvas.width, canvas.height, _cFftDummyData, _cFftDummyData.length, 44100, isCalib);
}

/* ════════════════════════════════════════════════════════════
   음성 상태 UI
   ════════════════════════════════════════════════════════════ */
function showVoiceState(state) {
    ['voice-idle','voice-calibrating','voice-recording','voice-processing','voice-done','voice-error']
        .forEach(id => { document.getElementById(id).style.display = 'none'; });
    const el = document.getElementById('voice-' + state);
    if (el) el.style.display = 'flex';
    if      (state === 'calibrating') startVoiceFFT('calib');
    else if (state === 'recording')   switchVoiceFFTMode('record');
    else                              stopVoiceFFT();
}

function startVoiceRecording() {
    database.ref('robot_command').set({ action: 'stt_trigger', timestamp: Date.now() });
    showVoiceState('calibrating');
    document.getElementById('voice-record-btn').style.display = 'none';

    let elapsed = 0;
    const calibDuration = 1000;
    const calibInterval = 50;
    const calib = setInterval(() => {
        elapsed += calibInterval;
        const pct = Math.min(100, (elapsed / calibDuration) * 100);
        document.getElementById('calib-bar').style.width = pct + '%';
        const remaining = ((calibDuration - elapsed) / 1000).toFixed(1);
        document.getElementById('calib-countdown').innerText =
            elapsed < calibDuration ? `${remaining}초 후 녹음 시작` : '대기 중...';
        if (elapsed >= calibDuration) {
            clearInterval(calib);
            showVoiceState('recording');
            startPollingForSTTResult();
        }
    }, calibInterval);
}

function startPollingForSTTResult() {
    let attempts = 0;
    const maxAttempts = 60;

    voicePollingTimer = setInterval(() => {
        attempts++;
        database.ref('robot_status').once('value', (snap) => {
            const status   = snap.val() || {};
            const sttState = status.stt_state || '';
            const val      = status.stt_layout;

            if (sttState === 'extracting') showVoiceState('processing');

            if (val && Array.isArray(val) && val.length === 9) {
                clearInterval(voicePollingTimer); voicePollingTimer = null;
                voiceLayout = val;
                showVoiceResult(val);
            } else if (sttState === 'error' || attempts >= maxAttempts) {
                clearInterval(voicePollingTimer); voicePollingTimer = null;
                showVoiceError('음성 인식에 실패했습니다.\n다시 시도해 주세요.');
                document.getElementById('voice-record-btn').style.display = 'block';
                document.getElementById('voice-record-btn').innerText = '🎙️ 다시 녹음';
            }
        });
    }, 500);
}

function showVoiceResult(layout) {
    showVoiceState('done');
    const allWhite = layout.every(v => v === 1);
    const allBlack = layout.every(v => v === 2);
    let desc = '커스텀 패턴';
    if      (allWhite) desc = '전체 흰색';
    else if (allBlack) desc = '전체 검정';
    else {
        const isCheckW = JSON.stringify(layout) === JSON.stringify([1,2,1,2,1,2,1,2,1]);
        const isCheckB = JSON.stringify(layout) === JSON.stringify([2,1,2,1,2,1,2,1,2]);
        if      (isCheckW) desc = '흰색 체크무늬';
        else if (isCheckB) desc = '검정 체크무늬';
    }
    document.getElementById('voice-result-text').innerText = `인식된 패턴: ${desc}`;

    const grid = document.getElementById('voice-pattern-preview-popup');
    grid.innerHTML = '';
    layout.forEach(v => {
        const cell = document.createElement('div');
        cell.style.cssText = `width:32px; height:32px; border-radius:4px; border:1px solid #ccc; background:${v === 2 ? '#212121' : '#ffffff'};`;
        grid.appendChild(cell);
    });
    document.getElementById('voice-confirm-btn').style.display = 'block';
}

function showVoiceError(msg) {
    showVoiceState('error');
    document.getElementById('voice-error-text').innerText = msg;
}

function confirmVoicePattern() {
    if (!voiceLayout) return;
    const patternStr   = voiceLayout.map(v => v === 2 ? 'B' : (v === 3 ? 'C' : 'A')).join(',');
    const designColors = voiceLayout.map(v => v === 2 ? 'B' : (v === 3 ? 'D' : 'W'));
    closeVoicePopup();
    database.ref('robot_command').set({ action: 'start', design: 3, custom_pattern: patternStr, is_resume: false, timestamp: Date.now() });
    database.ref('robot_status/stt_layout').remove();
    DESIGN_COLORS[3] = designColors;
    isStarted = true;
    const voiceValues = voiceLayout.map(v => v === 2 ? 'B' : 'A');
    showCustomPatternPreview(voiceValues);
    speakTTS('작업을 시작합니다');
    if (!timerInterval) startTimer();
}

/* ════════════════════════════════════════════════════════════
   시멘트 모달 STT 상태 UI 업데이트
   ════════════════════════════════════════════════════════════ */
let _cementCalibTimer = null;
let _lastSttMicState  = '';

function updateCementSttUI(phase, micState) {
    const states = ['idle','tts_speaking','calibrating','listening','retry'];
    states.forEach(s => {
        const el = document.getElementById(`cement-stt-${phase}-${s}`);
        if (el) el.style.display = 'none';
    });

    const show = (s) => {
        const el = document.getElementById(`cement-stt-${phase}-${s}`);
        if (el) el.style.display = 'flex';
    };

    if (micState === 'calibrating' && _lastSttMicState !== 'calibrating') {
        startCementCalibAnim(phase);
    } else if (micState !== 'calibrating') {
        stopCementCalibAnim();
    }
    _lastSttMicState = micState;

    if      (micState === 'calibrating') startCementFFT(`cement-fft-${phase}-calib`);
    else if (micState === 'listening')   switchCementFFTCanvas(`cement-fft-${phase}-listen`);
    else                                 stopCementFFT();

    if      (micState === 'tts_speaking') show('tts_speaking');
    else if (micState === 'calibrating')  show('calibrating');
    else if (micState === 'listening')    show('listening');
    else if (micState === 'retry')        show('retry');
    else                                  show('idle');
}

function startCementCalibAnim(phase) {
    stopCementCalibAnim();
    const bar = document.getElementById(`cement-calib-bar-${phase}`);
    if (!bar) return;
    bar.style.width = '0%';
    let pct = 0;
    _cementCalibTimer = setInterval(() => {
        pct = Math.min(100, pct + 5);
        bar.style.width = pct + '%';
        if (pct >= 100) stopCementCalibAnim();
    }, 50);
}

function stopCementCalibAnim() {
    if (_cementCalibTimer) { clearInterval(_cementCalibTimer); _cementCalibTimer = null; }
}

/* ════════════════════════════════════════════════════════════
   Role Switcher
   ════════════════════════════════════════════════════════════ */
let currentRole = 'user';

function setRole(role) {
    currentRole = role;
    const toggle   = document.getElementById('role-toggle');
    const btnUser  = document.getElementById('btn-user');
    const btnAdmin = document.getElementById('btn-admin');

    if (role === 'admin') {
        toggle.classList.add('admin-mode');
        btnAdmin.classList.add('active-admin');
        btnAdmin.classList.remove('active-user');
        btnUser.classList.remove('active-user');
        openPointCloudViewer();
    } else {
        toggle.classList.remove('admin-mode');
        btnUser.classList.add('active-user');
        btnUser.classList.remove('active-admin');
        btnAdmin.classList.remove('active-admin');
        closeDepthViewer();
    }
}

function openDepthViewerPage() { openPointCloudViewer(); }

/* ════════════════════════════════════════════════════════════
   단차 검수 3D 뷰어 모달 (inspection-modal)
   ════════════════════════════════════════════════════════════ */
let _inspScene, _inspCamera, _inspRenderer, _inspControls;
let _inspMeshes = [], _inspPointClouds = [], _inspArrows = [];
let _inspWorldGroup;
let _inspAutoRotate  = false;
let _inspWireframe   = false;
let _inspShowPoints  = true;
let _inspShowNormals = true;
let _inspInitialized = false;
let _inspAnimId      = null;
let _lastInspTileStep       = -1;
let _currentWorkingTileId   = 1;
let _lastInspectionData     = null;

// 단차 판정 기준 (mm / degrees)
const DZ_BAD   = 6.0;
const DZ_WARN  = 3.0;
const TILT_BAD = 5.0;
const TILT_WARN = 2.0;

function _tilt(rpy_deg) {
    if (!Array.isArray(rpy_deg)) return 0;
    return Math.max(Math.abs(rpy_deg[0] || 0), Math.abs(rpy_deg[1] || 0));
}

async function openInspectionModal() {
    const modal = document.getElementById('inspection-modal');
    modal.style.display = 'flex';
    if (!_inspInitialized) {
        await new Promise(resolve => requestAnimationFrame(resolve));
        await _initInsp3D();
    }
    await _fetchAndApplyInspection();
    if (_lastInspectionData) renderInspectionData(_lastInspectionData);
}

function closeInspectionModal() {
    const modal = document.getElementById('inspection-modal');
    if (modal) modal.style.display = 'none';
}

function closeInspectionModalWithResult() {
    closeInspectionModal();
    if (!_lastInspectionData) return;

    const DZ_BAD_LOCAL   = 6.0;
    const DZ_WARN_LOCAL  = 3.0;
    const TILT_BAD_LOCAL = 5.0;
    const TILT_WARN_LOCAL = 2.0;

    (_lastInspectionData.tiles || []).forEach((tile, idx) => {
        const tileId = idx + 1;
        const tileEl = document.getElementById(`tile-${tileId}`);
        if (!tileEl) return;

        const p   = Array.isArray(tile.plane_centroid_mm) ? tile.plane_centroid_mm.map(Number) : [NaN,NaN,NaN];
        let dz = NaN;
        if (_lastInspectionData.wall) {
            const n    = (_lastInspectionData.wall.normal || _lastInspectionData.wall.plane_normal || [0,0,1]).map(Number);
            const wp   = (_lastInspectionData.wall.plane_centroid_mm || [0,0,0]).map(Number);
            const nNorm = Math.hypot(...n) || 1;
            dz = (n[0]*(p[0]-wp[0]) + n[1]*(p[1]-wp[1]) + n[2]*(p[2]-wp[2])) / nNorm;
        }
        const dzAbs   = Number.isFinite(dz) ? Math.abs(dz) : NaN;
        const tilt    = _tilt(tile.rpy_deg);

        const dzBad   = Number.isFinite(dzAbs) && dzAbs >= DZ_BAD_LOCAL;
        const dzWarn  = Number.isFinite(dzAbs) && dzAbs >= DZ_WARN_LOCAL;
        const tiltBad = tilt >= TILT_BAD_LOCAL;
        const tiltWarn = tilt >= TILT_WARN_LOCAL;

        tileEl.classList.remove('working','coated','running','finished','result-good','result-rmse-bad','result-rmse-warn','result-tilt-bad','result-both-bad');
        tileEl.style.backgroundImage = ''; tileEl.style.backgroundColor = '';
        tileEl.style.borderColor = '';     tileEl.style.color = '';

        let cls, label;
        if (dzBad && tiltBad) {
            cls = 'result-both-bad';
            label = `복합 불량\ndz:${Number.isFinite(dz)?dz.toFixed(1):'-'}mm ${tilt.toFixed(1)}°`;
        } else if (dzBad) {
            cls = 'result-rmse-bad';
            label = `단차 불량\n${Number.isFinite(dz)?dz.toFixed(1):'-'}mm`;
        } else if (tiltBad) {
            cls = 'result-tilt-bad';
            label = `기울기 불량\n${tilt.toFixed(1)}°`;
        } else if (dzWarn || tiltWarn) {
            cls = 'result-rmse-warn';
            const parts = [];
            if (dzWarn)   parts.push(`dz:${Number.isFinite(dz)?dz.toFixed(1):'-'}mm`);
            if (tiltWarn) parts.push(`${tilt.toFixed(1)}°`);
            label = `주의\n${parts.join(' ')}`;
        } else {
            cls = 'result-good';
            label = `정상\n${Number.isFinite(dz)?dz.toFixed(1):'-'}mm`;
        }

        tileEl.classList.add(cls);
        tileEl.innerText = label;
        tileState[tileId] = cls;

        const mesh3d = tiles3D[tileId - 1];
        if (mesh3d) {
            const DEG = Math.PI / 180;
            const rpy = tile.rpy_deg || [0, 0, 0];
            mesh3d.rotation.order = 'YXZ';
            mesh3d.rotation.x = -Number(rpy[0]) * DEG;
            mesh3d.rotation.y = -Number(rpy[1]) * DEG;
            mesh3d.rotation.z = 0;
            const dzScene = Number.isFinite(dz) ? dz * 0.0008 : 0;
            mesh3d.position.z = dzScene;
        }
    });
}

async function _initInsp3D() {
    if (_inspInitialized) return;
    _inspInitialized = true;

    const container = document.getElementById('insp-three-container');
    _inspScene = new THREE.Scene();
    _inspScene.background = new THREE.Color(0x0b1120);

    const grid = new THREE.GridHelper(1.2, 20, 0x1e3a5f, 0x0f1f35);
    grid.position.y = -0.22;
    _inspScene.add(grid);
    _inspScene.add(new THREE.AxesHelper(0.08));

    _inspScene.add(new THREE.AmbientLight(0xffffff, 0.9));
    const d1 = new THREE.DirectionalLight(0xffffff, 1.0); d1.position.set(1, 1, 1); _inspScene.add(d1);
    const d2 = new THREE.DirectionalLight(0xffffff, 0.45); d2.position.set(-1, 0.5, -0.5); _inspScene.add(d2);

    _inspCamera = new THREE.PerspectiveCamera(55, container.clientWidth / container.clientHeight, 0.001, 50);
    _inspCamera.position.set(0.35, 0.25, 0.7);

    _inspRenderer = new THREE.WebGLRenderer({ antialias: true });
    _inspRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    _inspRenderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(_inspRenderer.domElement);

    _inspControls = new THREE.OrbitControls(_inspCamera, _inspRenderer.domElement);
    _inspControls.enableDamping = true;
    _inspControls.dampingFactor = 0.06;

    _inspWorldGroup = new THREE.Group();
    _inspWorldGroup.rotation.x = Math.PI;
    _inspScene.add(_inspWorldGroup);

    _animateInsp();

    window.addEventListener('resize', () => {
        if (!_inspRenderer) return;
        _inspCamera.aspect = container.clientWidth / container.clientHeight;
        _inspCamera.updateProjectionMatrix();
        _inspRenderer.setSize(container.clientWidth, container.clientHeight);
    });
}

function _animateInsp() {
    _inspAnimId = requestAnimationFrame(_animateInsp);
    if (_inspAutoRotate && _inspWorldGroup) _inspWorldGroup.rotation.y += 0.003;
    if (_inspControls)  _inspControls.update();
    if (_inspRenderer && _inspScene && _inspCamera) _inspRenderer.render(_inspScene, _inspCamera);
}

function _clearInspWorld() {
    if (!_inspWorldGroup) return;
    while (_inspWorldGroup.children.length > 0) {
        const child = _inspWorldGroup.children[0];
        _inspWorldGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) {
            if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
            else child.material.dispose();
        }
    }
    _inspMeshes = []; _inspPointClouds = []; _inspArrows = [];
}

function _inspMakeMesh(vertices, indices, color) {
    const geometry = new THREE.BufferGeometry();
    const flat = new Float32Array(vertices.flat());
    geometry.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    const mat = new THREE.MeshPhongMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        side: THREE.DoubleSide, transparent: true, opacity: 0.58,
        wireframe: _inspWireframe,
    });
    const mesh = new THREE.Mesh(geometry, mat);
    _inspMeshes.push(mesh);
    return mesh;
}

function _inspMakePoints(points, color) {
    const geometry = new THREE.BufferGeometry();
    const flat = new Float32Array(points.flat());
    geometry.setAttribute('position', new THREE.BufferAttribute(flat, 3));
    const mat = new THREE.PointsMaterial({ color: new THREE.Color(color[0], color[1], color[2]), size: 0.008, sizeAttenuation: true });
    const pts = new THREE.Points(geometry, mat);
    pts.visible = _inspShowPoints;
    _inspPointClouds.push(pts);
    return pts;
}

function _inspMakeArrow(centroid, normal, color, length = 0.06) {
    const origin = new THREE.Vector3(...centroid);
    const dir    = new THREE.Vector3(...normal).normalize();
    const hex    = new THREE.Color(color[0], color[1], color[2]).getHex();
    const arrow  = new THREE.ArrowHelper(dir, origin, length, hex, 0.016, 0.009);
    arrow.visible = _inspShowNormals;
    _inspArrows.push(arrow);
    return arrow;
}

function _inspAddPlane(obj) {
    if (!obj) return;
    const group = new THREE.Group();

    // patch_vertices 포맷 (구 버전)
    if (obj.patch_vertices) {
        const color = obj.color || [0.13, 0.77, 0.36];
        group.add(_inspMakeMesh(obj.patch_vertices, obj.patch_indices, color));
        if (obj.sample_points) group.add(_inspMakePoints(obj.sample_points, color));
        if (obj.centroid && obj.normal) group.add(_inspMakeArrow(obj.centroid, obj.normal, color));
        if (obj.center_point) {
            const sg = new THREE.SphereGeometry(0.006, 16, 16);
            const sm = new THREE.MeshStandardMaterial({ color: new THREE.Color(color[0], color[1], color[2]) });
            const sphere = new THREE.Mesh(sg, sm);
            sphere.position.set(...obj.center_point);
            group.add(sphere);
        }
        _inspWorldGroup.add(group);
        return;
    }

    // 새 포맷: plane_centroid_mm (mm → m 변환)
    if (!obj.plane_centroid_mm && !obj.plane_normal) return;

    const isWall = (obj.name === 'wall');
    const color  = isWall ? [1.0, 0.75, 0.1] : [0.13, 0.77, 0.36];

    let su, sv;
    if (obj.plane_size_m?.width && obj.plane_size_m?.height) {
        su = Number(obj.plane_size_m.width);
        sv = Number(obj.plane_size_m.height);
    } else if (Array.isArray(obj.size_uv)) {
        su = Math.max(0.05, Number(obj.size_uv[0]) / 1000);
        sv = Math.max(0.05, Number(obj.size_uv[1]) / 1000);
    } else {
        su = isWall ? 0.30 : 0.09;
        sv = isWall ? 0.22 : 0.09;
    }

    let c;
    if      (Array.isArray(obj.centroid_m))       c = obj.centroid_m.map(Number);
    else if (Array.isArray(obj.center_point_m))   c = obj.center_point_m.map(Number);
    else c = (obj.plane_centroid_mm || [0,0,0]).map(v => Number(v) / 1000);

    const nArr  = obj.plane_normal || obj.normal || [0, 0, 1];
    const nVec  = new THREE.Vector3(Number(nArr[0]||0), Number(nArr[1]||0), Number(nArr[2]||1)).normalize();
    const qNormal = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), nVec);
    let qFinal = qNormal;
    if (obj.rpy_deg) {
        const DEG = Math.PI / 180;
        const e = new THREE.Euler(obj.rpy_deg[0]*DEG, obj.rpy_deg[1]*DEG, obj.rpy_deg[2]*DEG, 'XYZ');
        qFinal = qNormal.multiply(new THREE.Quaternion().setFromEuler(e));
    }

    const geo = new THREE.PlaneGeometry(su, sv);
    const mat = new THREE.MeshPhongMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        side: THREE.DoubleSide, transparent: true,
        opacity: isWall ? 0.28 : 0.65,
        wireframe: _inspWireframe,
    });
    const plane = new THREE.Mesh(geo, mat);
    plane.quaternion.copy(qFinal);
    plane.position.set(...c);
    _inspMeshes.push(plane);
    group.add(plane);

    const edge = new THREE.LineSegments(
        new THREE.EdgesGeometry(geo),
        new THREE.LineBasicMaterial({ color: isWall ? 0xffd54f : 0x69f0ae, transparent: true, opacity: 0.5 })
    );
    edge.quaternion.copy(qFinal);
    edge.position.set(...c);
    group.add(edge);

    group.add(_inspMakeArrow(c, nArr, color, Math.max(su, sv) * 0.5));

    const sg = new THREE.SphereGeometry(isWall ? 0.006 : 0.004, 12, 12);
    const sm = new THREE.MeshStandardMaterial({ color: new THREE.Color(color[0], color[1], color[2]) });
    const sphere = new THREE.Mesh(sg, sm);
    sphere.position.set(...c);
    group.add(sphere);

    _inspWorldGroup.add(group);
}

function _inspFitCamera() {
    const box = new THREE.Box3().setFromObject(_inspWorldGroup);
    if (box.isEmpty()) return;
    const size   = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z, 0.08);
    const dist   = maxDim * 2.6;
    _inspCamera.position.set(center.x + dist, center.y + dist * 0.75, center.z + dist);
    _inspCamera.lookAt(center);
    _inspControls.target.copy(center);
    _inspControls.update();
}

function _fmt(v, d = 3) {
    if (v === null || v === undefined || Number.isNaN(v)) return '-';
    return Number(v).toFixed(d);
}

function _buildInspSummaryHtml(data) {
    const _v    = (val, d=2) => (val !== undefined && val !== null && !isNaN(val)) ? Number(val).toFixed(d) : '-';
    const _arr  = (arr, d=3) => Array.isArray(arr) ? arr.map(v => Number(v).toFixed(d)).join(', ') : '-';
    const _tc   = (maxTilt) => maxTilt >= 5.0 ? '#f87171' : maxTilt >= 2.0 ? '#fbbf24' : '#4ade80';
    const _tl   = (maxTilt) => maxTilt >= 5.0 ? '불량' : maxTilt >= 2.0 ? '주의' : '정상';

    let html = '';

    if (data.wall) {
        const w = data.wall;
        const rpy = w.rpy_deg || [0, 0, 0];
        const maxTilt = Math.max(Math.abs(rpy[0]||0), Math.abs(rpy[1]||0));
        html += `
            <div class="insp-section">
                <h3>🧱 Wall</h3>
                <div class="insp-kv">
                    <div class="k">Normal</div><div class="v">[${_arr(w.plane_normal)}]</div>
                    <div class="k">Centroid</div><div class="v">[${_arr(w.plane_centroid_mm, 1)}] mm</div>
                    <div class="k">Roll</div><div class="v">${_v(rpy[0])}°</div>
                    <div class="k">Pitch</div><div class="v">${_v(rpy[1])}°</div>
                    <div class="k">Yaw</div><div class="v">${_v(rpy[2])}°</div>
                </div>
            </div>`;
    }

    html += '<div class="insp-section"><h3>🔲 Tiles</h3>';
    for (const tile of (data.tiles || [])) {
        const rpy = tile.rpy_deg || [0, 0, 0];
        const maxTilt = Math.max(Math.abs(rpy[0]||0), Math.abs(rpy[1]||0), Math.abs(rpy[2]||0));
        html += `
            <div class="insp-tile-card">
                <div class="insp-tile-name">${tile.name || 'tile'}</div>
                <div class="insp-kv">
                    <div class="k">판정</div><div class="v" style="color:${_tc(maxTilt)};font-weight:700;">${_tl(maxTilt)} (${maxTilt.toFixed(1)}°)</div>
                    <div class="k">Roll</div><div class="v">${_v(rpy[0])}°</div>
                    <div class="k">Pitch</div><div class="v">${_v(rpy[1])}°</div>
                    <div class="k">Yaw</div><div class="v">${_v(rpy[2])}°</div>
                    <div class="k">Normal</div><div class="v">[${_arr(tile.plane_normal)}]</div>
                    <div class="k">Centroid</div><div class="v">[${_arr(tile.plane_centroid_mm, 1)}] mm</div>
                    <div class="k">Conf</div><div class="v">${_v(tile.conf_score, 4)}</div>
                </div>
            </div>`;
    }
    html += '</div>';
    return html;
}

function renderInspectionData(data) {
    if (!_inspInitialized) return;
    _clearInspWorld();
    if (data.wall) _inspAddPlane(data.wall);
    for (const tile of (data.tiles || [])) _inspAddPlane(tile);
    _inspFitCamera();

    document.getElementById('insp-badge-frame').textContent = `frame: ${data.frame_id ?? '-'}`;
    document.getElementById('insp-badge-tiles').textContent = `tiles: ${data.tiles?.length ?? 0}`;
    const ts = data.timestamp_sec ? new Date(data.timestamp_sec * 1000).toLocaleTimeString() : '-';
    document.getElementById('insp-badge-time').textContent  = `time: ${ts}`;
    document.getElementById('insp-summary-content').innerHTML = _buildInspSummaryHtml(data);
    document.getElementById('insp-loading').style.display   = 'none';
}

/* 툴바 버튼 토글 */
function inspTogglePoints()  { _inspShowPoints  = !_inspShowPoints;  _inspPointClouds.forEach(p => p.visible = _inspShowPoints);  document.getElementById('insp-btn-points').classList.toggle('insp-btn-active', _inspShowPoints); }
function inspToggleNormals() { _inspShowNormals = !_inspShowNormals; _inspArrows.forEach(a => a.visible = _inspShowNormals);        document.getElementById('insp-btn-normals').classList.toggle('insp-btn-active', _inspShowNormals); }
function inspToggleWire()    { _inspWireframe   = !_inspWireframe;   _inspMeshes.forEach(m => m.material.wireframe = _inspWireframe); document.getElementById('insp-btn-wire').classList.toggle('insp-btn-active', _inspWireframe); }
function inspToggleRotate()  { _inspAutoRotate  = !_inspAutoRotate;  document.getElementById('insp-btn-rotate').classList.toggle('insp-btn-active', _inspAutoRotate); }

/* API inspection_result 실시간 감시 */
async function _fetchAndApplyInspection() {
    try {
        const res = await fetch('/api/inspect/latest', { cache: 'no-store' });
        if (!res.ok) return;
        const data = await res.json();
        if (!data || !Array.isArray(data.tiles)) { _lastInspectionData = null; return; }

        if (!data.wall) {
            try {
                const wallRes = await fetch('/wall.json', { cache: 'no-store' });
                if (wallRes.ok) {
                    const wallData = await wallRes.json();
                    if      (wallData?.wall)         data.wall = wallData.wall;
                    else if (wallData?.plane_normal)  data.wall = wallData;
                }
            } catch (e) { console.warn('[INSPECT] wall.json fetch 실패 (무시):', e); }
        }

        _lastInspectionData = data;
        const modal = document.getElementById('inspection-modal');
        if (modal.style.display === 'flex' && _inspInitialized) renderInspectionData(data);
    } catch (err) {
        console.warn('[INSPECT] fetch 실패:', err);
    }
}

_fetchAndApplyInspection();

if (window._globalEs) {
    window._globalEs.addEventListener('inspect_updated', () => { _fetchAndApplyInspection(); });
}

/* ════════════════════════════════════════════════════════════
   단차 보고서 모달 로직
   ════════════════════════════════════════════════════════════ */
const rpState = { data: null, wallRef: null, lastSig: null };
const rpEl    = (id) => document.getElementById(id);
const rpFmt   = (v, d = 2) => Number.isFinite(Number(v)) ? Number(v).toFixed(d) : '-';

function rpTiltDeg(rpy) {
    return Math.max(Math.abs(Number(rpy?.[0] ?? 0)), Math.abs(Number(rpy?.[1] ?? 0)));
}

function rpClassify(mm) {
    const warn = Number(rpEl('rp-warnMm').value || 3.0);
    const bad  = Number(rpEl('rp-badMm').value  || 6.0);
    if (mm >= bad)  return { cls: 'rp-bad',  text: '불량', color: '#ef4444' };
    if (mm >= warn) return { cls: 'rp-warn', text: '주의', color: '#f59e0b' };
    return { cls: 'rp-good', text: '정상', color: '#22c55e' };
}

function rpLoadWallRef(data) {
    if (!data?.wall) return null;
    const n   = data.wall.normal || data.wall.plane_normal || [0, 0, 1];
    const cmm = data.wall.plane_centroid_mm || null;
    if (!Array.isArray(n) || !Array.isArray(cmm)) return null;
    return { n: n.map(Number), p: cmm.map(Number) };
}

function rpSignedDist(pointMm, wallRef) {
    if (!wallRef || !Array.isArray(pointMm) || pointMm.length < 3) return NaN;
    const [nx, ny, nz] = wallRef.n;
    const [px, py, pz] = wallRef.p;
    const dx = Number(pointMm[0]) - px, dy = Number(pointMm[1]) - py, dz = Number(pointMm[2]) - pz;
    const nNorm = Math.hypot(nx, ny, nz) || 1;
    return (nx * dx + ny * dy + nz * dz) / nNorm;
}

function rpNormalizeTiles(data) {
    const tiles = Array.isArray(data?.tiles) ? data.tiles : [];
    rpState.wallRef = rpLoadWallRef(data);
    return tiles.map((tile, idx) => {
        const p = Array.isArray(tile.plane_centroid_mm) ? tile.plane_centroid_mm.map(Number) : [NaN,NaN,NaN];
        let distSafe = 0;
        if (rpState.wallRef) {
            const dist = rpSignedDist(p, rpState.wallRef);
            distSafe = Number.isFinite(dist) ? dist : 0;
        }
        const rpy  = tile.rpy_deg || [0,0,0];
        const tilt = Math.max(Math.abs(rpy[0]), Math.abs(rpy[1]));
        const judge = rpClassify(Math.abs(distSafe));
        return { ...tile, idx: idx + 1, z: p[2], dz: distSafe, dzAbs: Math.abs(distSafe), tilt, judge };
    }).sort((a, b) => (b.dzAbs || 0) - (a.dzAbs || 0));
}

function rpUpdateSummary(tiles, data) {
    rpEl('rp-mTiles').textContent   = String(tiles.length);
    rpEl('rp-mBase').textContent    = rpState.wallRef ? 'inspection.wall' : '-';
    rpEl('rp-mMaxDz').textContent   = tiles.length ? rpFmt(Math.max(...tiles.map(t => t.dzAbs || 0))) : '-';
    rpEl('rp-mAvgTilt').textContent = tiles.length
        ? rpFmt(tiles.reduce((s, t) => s + (t.tilt || 0), 0) / tiles.length)
        : '-';
    rpEl('rp-mBad').textContent  = String(tiles.filter(t => t.judge.cls === 'rp-bad').length);
    rpEl('rp-mTime').textContent = data?.timestamp_sec
        ? new Date(data.timestamp_sec * 1000).toLocaleTimeString() : '-';
}

function rpRenderTable(tiles) {
    const tbody = rpEl('rp-tileTableBody');
    tbody.innerHTML = '';
    if (!tiles.length) {
        tbody.innerHTML = '<tr><td colspan="8"><div class="rp-emptyHint">타일 데이터가 없습니다.</div></td></tr>';
        return;
    }
    for (const t of tiles) {
        const tr = document.createElement('tr');
        tr.innerHTML = `
            <td><strong>${t.name ?? `tile_${t.idx}`}</strong></td>
            <td>${rpFmt(t.conf_score, 4)}</td>
            <td class="rp-mono">(${rpFmt(t.center_uv?.[0], 1)}, ${rpFmt(t.center_uv?.[1], 1)})</td>
            <td class="rp-mono">${rpFmt(t.z, 2)}</td>
            <td class="rp-mono" style="font-weight:800;color:${t.judge.color};">${t.dz >= 0 ? '+' : ''}${rpFmt(t.dz, 2)}</td>
            <td class="rp-mono">${rpFmt(t.tilt, 2)}</td>
            <td class="rp-mono">[${rpFmt(t.rpy_deg?.[0], 2)}, ${rpFmt(t.rpy_deg?.[1], 2)}, ${rpFmt(t.rpy_deg?.[2], 2)}]</td>
            <td><span class="rp-badge ${t.judge.cls}">${t.judge.text}</span></td>
        `;
        tbody.appendChild(tr);
    }
}

function rpDrawMap(tiles) {
    const canvas = rpEl('rpMapCanvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0e152a'; ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = 'rgba(148,163,184,0.12)'; ctx.lineWidth = 1;
    for (let x = 40; x < w; x += 60) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); }
    for (let y = 40; y < h; y += 60) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }

    if (!tiles.length) {
        ctx.fillStyle = '#94a3b8'; ctx.font = '16px sans-serif';
        ctx.fillText('inspection 데이터를 기다리는 중...', 40, 42);
        return;
    }

    const valid = tiles.filter(t => Number.isFinite(Number(t.center_uv?.[0])) && Number.isFinite(Number(t.center_uv?.[1])));
    if (!valid.length) return;

    const us = valid.map(t => Number(t.center_uv[0]));
    const vs = valid.map(t => Number(t.center_uv[1]));
    const minU = Math.min(...us), maxU = Math.max(...us);
    const minV = Math.min(...vs), maxV = Math.max(...vs);
    const pad = 70;
    const sx = u => pad + ((u - minU) / Math.max(1, maxU - minU)) * (w - 2 * pad);
    const sy = v => pad + ((v - minV) / Math.max(1, maxV - minV)) * (h - 2 * pad);

    for (const t of valid) {
        const x = sx(Number(t.center_uv[0])), y = sy(Number(t.center_uv[1]));
        const radius = 14 + Math.min(34, (t.dzAbs || 0) * 2.8);
        ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fillStyle = t.judge.color + '33'; ctx.fill();
        ctx.lineWidth = 3; ctx.strokeStyle = t.judge.color; ctx.stroke();
        ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.fillStyle = '#f8fafc'; ctx.fill();
        ctx.fillStyle = '#e5e7eb'; ctx.font = 'bold 15px sans-serif';
        ctx.fillText(String(t.name ?? `tile_${t.idx}`), x + radius + 8, y - 12);
        ctx.font = '13px sans-serif';
        ctx.fillText(`벽 기준 ${t.dz >= 0 ? '+' : ''}${rpFmt(t.dz)} mm`, x + radius + 8, y + 6);
        ctx.fillText(`Tilt ${rpFmt(t.tilt)}°`, x + radius + 8, y + 22);
    }
}

function rpRender(data) {
    rpState.data = data;
    const tiles = rpNormalizeTiles(data);
    rpUpdateSummary(tiles, data);
    rpRenderTable(tiles);
    rpDrawMap(tiles);
    rpEl('rp-statusLine').textContent =
        `loaded ${tiles.length} tiles @ ${data?.timestamp_sec ? new Date(data.timestamp_sec * 1000).toLocaleString() : '-'}`;
}

async function rpFetchLatest(force = false) {
    const url = rpEl('rp-urlInput').value.trim();
    const res = await fetch(url, { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    if (!Array.isArray(data?.tiles)) throw new Error('tiles 필드가 없습니다.');
    rpRender(data);
}

function openReportModal() {
    document.getElementById('report-modal').classList.add('show');
    if (rpState.data) {
        rpRender(rpState.data);
    } else {
        rpFetchLatest(true).catch(err => { rpEl('rp-statusLine').textContent = `error: ${err.message}`; });
    }
}

function closeReportModal() {
    document.getElementById('report-modal').classList.remove('show');
}

// 보고서 모달 버튼 이벤트
rpEl('rp-fetchBtn').addEventListener('click', () => {
    rpFetchLatest(true).catch(err => rpEl('rp-statusLine').textContent = `error: ${err.message}`);
});

rpEl('rp-loadDummyBtn').addEventListener('click', async () => {
    try {
        rpEl('rp-statusLine').textContent = 'loading dummy...';
        await fetch('/api/inspect/dummy', { method: 'POST', cache: 'no-store' });
        await rpFetchLatest(true);
    } catch (err) {
        rpEl('rp-statusLine').textContent = `error: ${err.message}`;
    }
});

rpEl('rp-warnMm').addEventListener('change', () => { if (rpState.data) rpRender(rpState.data); });
rpEl('rp-badMm').addEventListener('change',  () => { if (rpState.data) rpRender(rpState.data); });

rpEl('rp-open3dBtn').addEventListener('click', async () => {
    if (!rpState.data) return;
    closeReportModal();
    const modal = document.getElementById('inspection-modal');
    modal.style.display = 'flex';
    if (!_inspInitialized) {
        await new Promise(resolve => requestAnimationFrame(resolve));
        await _initInsp3D();
    }
    await _fetchAndApplyInspection();
    if (_lastInspectionData) renderInspectionData(_lastInspectionData);
});

rpEl('rp-force3dBtn').addEventListener('click', async () => {
    if (!rpState.data) return;
    closeReportModal();
    const modal = document.getElementById('inspection-modal');
    modal.style.display = 'flex';
    if (!_inspInitialized) {
        await new Promise(resolve => requestAnimationFrame(resolve));
        await _initInsp3D();
    }
    await _fetchAndApplyInspection();
    if (_lastInspectionData) renderInspectionData(_lastInspectionData);
});

// 보고서 모달 바깥 클릭 시 닫기
document.getElementById('report-modal').addEventListener('click', (e) => {
    if (e.target === document.getElementById('report-modal')) closeReportModal();
});

// 페이지 로드 시 보고서 버튼 표시
document.getElementById('btn-report').style.display = 'block';

// SSE로 inspection 업데이트 시 보고서도 자동 갱신
if (window._globalEs) {
    window._globalEs.addEventListener('inspect_updated', async () => {
        try {
            await rpFetchLatest(true);
            if (document.getElementById('report-modal').classList.contains('show') && rpState.data) {
                rpRender(rpState.data);
            }
        } catch (err) { console.error('report SSE error:', err); }
    });
}
