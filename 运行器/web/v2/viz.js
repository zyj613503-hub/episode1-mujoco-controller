/* ==========================================================================
   上位机 V2 · 3D 可视化与编程（viz.js）
   原生 JS，无依赖。后端 /api/v2 不可用时优雅降级为静态演示数据，不白屏。
   ========================================================================== */
(function () {
  'use strict';

  var API = '/api/v2';

  function $(sel, root) { return (root || document).querySelector(sel); }
  function $$(sel, root) { return Array.prototype.slice.call((root || document).querySelectorAll(sel)); }

  function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
  function num(v, fb) { var n = parseFloat(v); return isFinite(n) ? n : (fb || 0); }
  function fx(v, d) { return (Math.round(v * Math.pow(10, d)) / Math.pow(10, d)).toFixed(d); }
  function clampGrip(v) { return clamp(num(v, 0), 0, 110); }
  /* 旧数据的 'open'/'close' 仍然认：open=0°，close=110° */
  function gripDeg(p) {
    var g = p && p.grip;
    if (g === 'open') return 0;
    if (g === 'close') return 110;
    return clampGrip(g);
  }
  function degText(v) {
    var s = fx(clampGrip(v), 1);
    return s.replace(/\.0$/, '') + '\u00b0';
  }
  function el(tag, cls, txt) {
    var n = document.createElement(tag);
    if (cls) n.className = cls;
    if (txt != null) n.textContent = txt;
    return n;
  }

  /* ---------------- 状态 ---------------- */
  var OFFLINE = false;
  var failures = 0;

  var JOINT_RANGES = [[0, 340], [0, 180], [0, 163], [0, 335], [0, 220], [0, 335]];
  var JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

  var defaults = {
    points: [
      /* 初始路点：与 scripts 里的演示一致（黄球上方 → 抓取高度 → 盘子） */
      { x: 0.30, y: 0.08, z: 0.15, dwell: 0.5, grip: 0 },
      { x: 0.30, y: 0.08, z: 0.062, dwell: 0.3, grip: 110 },
      { x: 0.34, y: 0.28, z: 0.14, dwell: 0.8, grip: 0 }
    ],
    /* 首屏 = home（与约定 OFFSET 一致）：滑块值、预览姿态、FK 读数三者统一 */
    joints: [180.0, 90.0, 83.0, 30.0, 110.0, 30.0],
    targets: [180.0, 90.0, 83.0, 30.0, 110.0, 30.0],
    duration: 0,
    speed: 1.0
  };

  var S = {
    points: JSON.parse(JSON.stringify(defaults.points)),
    joints: defaults.joints.slice(),
    targets: defaults.targets.slice(),
    duration: defaults.duration,
    reachable: true,
    playing: false,
    loop: false,
    speed: 1.0,
    t: 0,
    grip: 0,
    /* 用户刚拖过的夹爪目标角：/state 的实测值在电机缓动期间会落后，
       这里短暂压住实测值，避免滑块/新路点被中间角度带偏 */
    gripSet: null,
    gripSetAt: 0,
    cam: { az: 138, el: -19, dist: 1.22, lookat: [0.20, 0.10, 0.13] }
  };

  /* ---------------- 请求 ---------------- */
  function post(path, body) {
    return fetch(API + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    }).then(function (r) {
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return r.json();
    }).then(function (j) {
      failures = 0; OFFLINE = false;
      return j;
    });
  }

  function get(path) {
    return fetch(API + path, { headers: { Accept: 'application/json' } })
      .then(function (r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      });
  }

  function soft(promise, label) {
    return promise.catch(function (e) {
      failures += 1;
      if (failures >= 2 && !OFFLINE) {
        OFFLINE = true;
        document.body.setAttribute('data-api', 'offline');
        var live = $('#roLiveText');
        if (live) live.textContent = '本地预览 · 离线';
        var dot = $('#roDot');
        if (dot) dot.classList.add('is-off');
      }
      if (window.console && console.debug) console.debug('[viz:' + label + ']', e.message);
      return null;
    });
  }

  /* ---------------- Toast ---------------- */
  var toastHost = $('#toastHost');
  function toast(msg, kind) {
    if (!toastHost) return;
    var t = el('div', 'vi-toast' + (kind ? ' is-' + kind : ''), msg);
    toastHost.appendChild(t);
    requestAnimationFrame(function () { t.classList.add('is-in'); });
    setTimeout(function () {
      t.classList.remove('is-in');
      setTimeout(function () { if (t.parentNode) t.parentNode.removeChild(t); }, 200);
    }, 2600);
  }

  /* ---------------- 轨迹点 ---------------- */
  var wpList = $('#wpList');
  var wpCount = $('#wpCount');

  function wpPayload() {
    return S.points.map(function (p) {
      return [p.x, p.y, p.z, p.dwell, gripDeg(p)];
    });
  }

  function renderPoints() {
    wpList.textContent = '';
    S.points.forEach(function (p, i) {
      var row = el('div', 'wp-row');
      row.setAttribute('data-i', String(i));

      var line = el('div', 'wp-line');
      line.appendChild(el('span', 'wp-idx', '#' + (i + 1)));
      ['x', 'y', 'z'].forEach(function (k) {
        var inp = el('input', 'wp-in');
        inp.type = 'text';
        inp.setAttribute('inputmode', 'decimal');
        inp.setAttribute('data-k', k);
        inp.setAttribute('data-num', '');
        inp.setAttribute('data-step', '0.01');
        inp.setAttribute('aria-label', '轨迹点 ' + (i + 1) + ' ' + k.toUpperCase());
        inp.value = fx(p[k], 2);
        line.appendChild(inp);
      });
      var del = el('button', 'wp-del', '\u00d7');
      del.type = 'button';
      del.setAttribute('data-act', 'del');
      del.setAttribute('aria-label', '删除轨迹点 ' + (i + 1));
      line.appendChild(del);
      row.appendChild(line);

      var meta = el('div', 'wp-meta');
      meta.appendChild(el('span', 'wp-dwell', '停留'));
      var dwell = el('input', 'wp-in wp-in--dwell');
      dwell.type = 'text';
      dwell.setAttribute('inputmode', 'decimal');
      dwell.setAttribute('data-k', 'dwell');
      dwell.setAttribute('data-num', '');
      dwell.setAttribute('data-step', '0.1');
      dwell.setAttribute('aria-label', '轨迹点 ' + (i + 1) + ' 停留秒');
      dwell.value = fx(p.dwell, 1);
      meta.appendChild(dwell);
      meta.appendChild(el('span', 'wp-unit', 's'));

      var gripBox = el('span', 'wp-grip-edit');
      var gripDisp = el('button', 'wp-grip-num', degText(gripDeg(p)));
      gripDisp.type = 'button';
      gripDisp.setAttribute('data-act', 'grip');
      gripDisp.setAttribute('aria-label', '轨迹点 ' + (i + 1) + ' 夹爪开合角，点击编辑');
      gripBox.appendChild(gripDisp);
      var gripInp = el('input', 'wp-in wp-in--grip');
      gripInp.type = 'text';
      gripInp.setAttribute('inputmode', 'decimal');
      gripInp.setAttribute('data-k', 'grip');
      gripInp.setAttribute('data-num', '');
      gripInp.setAttribute('data-step', '1');
      gripInp.setAttribute('aria-label', '轨迹点 ' + (i + 1) + ' 夹爪开合角（0-110）');
      gripInp.value = fx(gripDeg(p), 1);
      gripBox.appendChild(gripInp);
      meta.appendChild(gripBox);

      var edit = el('button', 'wp-edit', '编辑');
      edit.type = 'button';
      edit.setAttribute('data-act', 'edit');
      meta.appendChild(edit);
      row.appendChild(meta);

      wpList.appendChild(row);
    });
    if (wpCount) wpCount.textContent = S.points.length + ' 个 · 米';
  }

  var planTimer = 0;
  function schedulePlan() {
    clearTimeout(planTimer);
    planTimer = setTimeout(plan, 220);
  }

  function setCmdMeta() {
    var meta = $('#cmdMeta');
    if (!meta) return;
    var n = S.points.length;
    var dur = S.duration > 0 ? (fx(S.duration, 1) + ' S') : '—';
    meta.textContent = n + ' 路点 · ' + dur + ' · ' + (S.reachable ? '全部可达' : '存在不可达');
  }

  function plan() {
    setCmdMeta();
    if (!$('#cmdMeta')) return;
    soft(post('/plan', { waypoints: wpPayload(), speed: S.speed }), 'plan').then(function (j) {
      if (!j) return;
      var d = j.duration != null ? j.duration : (j.total_duration != null ? j.total_duration : null);
      if (d != null) { S.duration = num(d, S.duration); }
      if (j.reachable != null) S.reachable = !!j.reachable;
      if (Array.isArray(j.waypoints)) {
        S.reachable = j.waypoints.every(function (w) { return !w || w.status == null || w.status === 'ok'; });
      }
      setCmdMeta();
      if (j.ok !== false && Array.isArray(j.wp)) {
        if (setTraj(j.wp, j.duration)) {
          S.t = 0;
          applyPreviewFrame();
        }
      }
      renderSeek();
    });
  }

  wpList.addEventListener('focusin', function (e) {
    var inp = e.target;
    if (inp.tagName !== 'INPUT') return;
    inp.dataset.prev = inp.value;
  });

  wpList.addEventListener('input', function (e) {
    var inp = e.target;
    if (inp.tagName !== 'INPUT') return;
    var row = inp.closest('.wp-row');
    if (!row) return;
    var i = parseInt(row.getAttribute('data-i'), 10);
    var k = inp.getAttribute('data-k');
    if (!S.points[i] || !k) return;
    S.points[i][k] = k === 'grip' ? clampGrip(inp.value) : num(inp.value, S.points[i][k]);
    schedulePlan();
  });

  function commitGrip(inp) {
    var row = inp.closest('.wp-row');
    var i = row ? parseInt(row.getAttribute('data-i'), 10) : -1;
    var p = S.points[i];
    if (!p) return;
    p.grip = clampGrip(inp.value);
    inp.value = fx(p.grip, 1);
    var box = inp.closest('.wp-grip-edit');
    if (box) {
      box.classList.remove('is-editing');
      var disp = box.querySelector('.wp-grip-num');
      if (disp) disp.textContent = degText(p.grip);
    }
  }

  wpList.addEventListener('change', function (e) {
    var inp = e.target;
    if (inp.tagName !== 'INPUT') return;
    var row = inp.closest('.wp-row');
    if (!row) return;
    var i = parseInt(row.getAttribute('data-i'), 10);
    var k = inp.getAttribute('data-k');
    if (!S.points[i] || !k) return;
    if (k === 'grip') {
      commitGrip(inp);
    } else {
      S.points[i][k] = num(inp.value, S.points[i][k]);
      inp.value = fx(S.points[i][k], k === 'dwell' ? 1 : 2);
    }
    plan();
  });

  wpList.addEventListener('focusout', function (e) {
    var inp = e.target;
    if (inp.tagName === 'INPUT' && inp.getAttribute('data-k') === 'grip') commitGrip(inp);
  });

  wpList.addEventListener('keydown', function (e) {
    if (e.key === 'Enter' && e.target.tagName === 'INPUT') {
      e.preventDefault();
      e.target.blur();
    }
  });

  wpList.addEventListener('click', function (e) {
    var btn = e.target.closest('button[data-act]');
    if (!btn) return;
    var row = btn.closest('.wp-row');
    var i = row ? parseInt(row.getAttribute('data-i'), 10) : -1;
    var act = btn.getAttribute('data-act');
    if (act === 'del') {
      if (S.points.length <= 1) { toast('至少保留一个轨迹点', 'warn'); return; }
      S.points.splice(i, 1);
      renderPoints();
      plan();
    } else if (act === 'grip') {
      var box = row ? row.querySelector('.wp-grip-edit') : null;
      var ginp = row ? row.querySelector('.wp-in--grip') : null;
      if (box && ginp) {
        box.classList.add('is-editing');
        ginp.focus();
        ginp.select();
      }
    } else if (act === 'edit') {
      var first = row ? row.querySelector('.wp-in') : null;
      if (first) { first.focus(); first.select(); }
    }
  });

  $('#wpAdd').addEventListener('click', function () {
    var last = S.points[S.points.length - 1] || { x: 0.35, y: 0.10, z: 0.45, dwell: 0.5, grip: 0 };
    S.points.push({
      x: num(last.x, 0.35) + 0.02,
      y: num(last.y, 0.10),
      z: num(last.z, 0.45),
      dwell: 0.5,
      grip: clampGrip(S.grip)   /* 点「+」时把右侧滑块当前开合角一起记下 */
    });
    renderPoints();
    plan();
  });

  /* ---------------- 脚本库 ---------------- */
  var libList = $('#libList');
  var libCount = $('#libCount');
  var LOCAL_SCRIPTS = [
    { name: 'pick_and_place.py', waypoints: 3, duration: 12.4 },
    { name: 'stack_3box.py', waypoints: 6, duration: 21.8 }
  ];

  function renderScripts(items) {
    var list = Array.isArray(items) && items.length ? items : LOCAL_SCRIPTS;
    libList.textContent = '';
    list.forEach(function (it) {
      var row = el('div', 'lib-row');
      row.appendChild(el('span', 'lib-name', it.name || 'untitled.py'));
      var w = it.waypoints != null ? (Array.isArray(it.waypoints) ? it.waypoints.length : it.waypoints) : 0;
      var d = it.duration != null ? num(it.duration, 0) : 0;
      row.appendChild(el('span', 'lib-sub', w + ' 路点 · ' + fx(d, 1) + ' s'));
      var open = el('button', 'lib-open', '打开');
      open.type = 'button';
      open.addEventListener('click', function (e) {
        e.stopPropagation();
        openScript(it);
      });
      row.appendChild(open);
      row.addEventListener('click', function () { openScript(it); });
      libList.appendChild(row);
    });
    if (libCount) libCount.textContent = list.length + ' 个文件';
  }

  function openScript(it) {
    soft(post('/play', { action: 'select', script: it.name }), 'select').then(function (j) {
      if (!j) toast('后端未连接，无法载入 ' + it.name, 'warn');
      else toast('已载入 ' + it.name, 'ok');
    });
  }

  function loadScripts() {
    soft(get('/scripts'), 'scripts').then(function (j) {
      if (!j || !Array.isArray(j.items)) return;
      renderScripts(j.items);
    });
  }

  $('#openDir').addEventListener('click', function () {
    soft(get('/scripts'), 'scripts').then(function (j) {
      if (!j) toast('后端未连接', 'warn');
      else toast('脚本目录：' + (j.path || 'scripts/'), 'ok');
    });
  });

  /* ---------------- SCENE ---------------- */
  $('#sceneApply').addEventListener('click', function () {
    var model = $('#modelXml').value.trim() || 'model/episode1.xml';
    var scene = $('#sceneXml').value.trim() || 'model/scene_episode1.xml';
    toast('正在布置场景…');
    soft(post('/scene', { model: model, scene: scene }), 'scene').then(function (j) {
      if (!j) toast('场景布置失败：后端未连接', 'warn');
      else toast('场景已布置 · ' + scene.split('/').pop(), 'ok');
    });
  });

  $('#teachAdd').addEventListener('click', function () {
    toast('示教录制需连接真机', 'warn');
  });

  /* ---------------- 本地三维预览（three.js · 本地 FK；不联网、不推流） ---------------- */
  var stage = $('#vpStage');
  var previewEl = $('#preview');
  var view = null;
  var VIEW_ERR = '';

  if (previewEl && window.LocalArmPreview) {
    try {
      view = new window.LocalArmPreview(previewEl, {
        onReady: function () {
          /* 同步 XHR 会在构造函数里就绪，推到下一拍，等下面所有控件都定义完 */
          setTimeout(function () {
            syncPreview();
            renderReadout();
            applyPreviewFrame();
            if (TRAJ.degs.length) renderSeek();
          }, 0);
        }
      });
    } catch (e) {
      view = null;
      VIEW_ERR = (e && e.message) || String(e);
    }
  } else if (previewEl) {
    VIEW_ERR = 'js/preview.js 未加载';
  }
  if (VIEW_ERR && previewEl) {
    previewEl.innerHTML = '<div class="pv-missing">本地预览未加载：' + VIEW_ERR + '</div>';
  }
  if (view && view.getErrors && view.getErrors().length) {
    VIEW_ERR = view.getErrors().join('; ');
    if (window.console) console.warn('[preview]', VIEW_ERR);
  }

  function viewOk() { return !!(view && view.ready); }

  /* 关节角 / 夹爪 → 本地预览（无网络、无节流） */
  function syncPreview() {
    if (!viewOk()) return;
    view.setJointsDeg(S.joints);
    view.setGripperDeg(S.grip);
  }

  function paintJointRows() {
    $$('.joint-row', jointBody).forEach(function (row, i) {
      var box = row.querySelector('.jbox');
      if (box && document.activeElement !== box) box.value = fx(S.joints[i], 1);
      var rng = row.querySelector('input[type="range"]');
      if (rng && document.activeElement !== rng) rng.value = String(S.joints[i]);
      paintJoint(row, i);
    });
  }

  /* 本地 FK 位姿 → 「运动学状态」数字（MM / 度 ZYX） */
  function writePose(frame, pose) {
    ['mm', 'deg'].forEach(function (ax) {
      var vals = ax === 'mm' ? pose.mm : pose.deg;
      var list = $$('.pose-in[data-frame="' + frame + '"][data-axis="' + ax + '"]', kinFields);
      list.forEach(function (inp, i) {
        if (document.activeElement !== inp && vals[i] != null) inp.value = fx(vals[i], 1);
      });
    });
  }

  function renderReadout() {
    if (!viewOk()) return;
    var fl = view.flangePose();
    var tp = view.tcpPose();
    var roT = $('#roTcp');
    if (roT) roT.textContent = 'TCP ' + tp.mm.map(function (v) { return fx(v, 1); }).join(' ') + ' MM';
    var roF = $('#roFlange');
    if (roF) roF.textContent = '法兰 ' + fl.mm.map(function (v) { return fx(v, 1); }).join(' ') + ' MM';
    writePose('flange', fl);
    writePose('tcp', tp);
  }

  /* 读回真机时用的平滑跟随（400ms） */
  var twinTimer = 0;
  function tweenJointsTo(target, ms) {
    if (!Array.isArray(target) || target.length < 6) return;
    var from = S.joints.slice();
    ms = ms || 400;
    clearInterval(twinTimer);
    var t0 = nowMs();
    twinTimer = setInterval(function () {
      var k = clamp((nowMs() - t0) / ms, 0, 1);
      var e = k < 0.5 ? 2 * k * k : 1 - Math.pow(-2 * k + 2, 2) / 2;
      for (var i = 0; i < 6; i++) S.joints[i] = from[i] + (num(target[i], from[i]) - from[i]) * e;
      syncPreview();
      paintJointRows();
      renderReadout();
      if (k >= 1) clearInterval(twinTimer);
    }, 16);
  }

  function nowMs() {
    return (window.performance && window.performance.now) ? window.performance.now() : Date.now();
  }

  /* 视口工具：网格 / 坐标轴 / 复位视角（全部本地） */
  var chkGrid = $('#chkGrid');
  var chkAxes = $('#chkAxes');
  if (chkGrid) chkGrid.addEventListener('change', function () { if (view) view.setGridVisible(chkGrid.checked); });
  if (chkAxes) chkAxes.addEventListener('change', function () { if (view) view.setAxesVisible(chkAxes.checked); });
  var viewBtn = $('#viewBtn');
  if (viewBtn) viewBtn.addEventListener('click', function () {
    if (!view) return;
    view.resetView();
    toast('视角已复位', 'ok');
  });

  /* ---------------- 路点轨迹：本页预演（不上网） ---------------- */
  var TRAJ = { degs: [], times: [], grip: [], duration: 0, start: null };

  function setTraj(wps, duration) {
    var okWp = (wps || []).filter(function (w) {
      return w && Array.isArray(w.degs) && w.degs.length >= 6;
    });
    if (!okWp.length) {
      TRAJ.degs = []; TRAJ.times = []; TRAJ.grip = [];
      return false;
    }
    TRAJ.start = S.joints.slice();
    TRAJ.degs = okWp.map(function (w) {
      return w.degs.slice(0, 6).map(function (v) { return num(v, 0); });
    });
    TRAJ.times = okWp.map(function (w) { return num(w.t, 0); });
    TRAJ.grip = okWp.map(function (w) { return num(w.grip, S.grip); });
    TRAJ.duration = num(duration, TRAJ.times[TRAJ.times.length - 1] || 0);
    return true;
  }

  function mix6(a, b, k) {
    k = clamp(k, 0, 1);
    var out = new Array(6);
    for (var i = 0; i < 6; i++) out[i] = num(a[i], 0) + (num(b[i], 0) - num(a[i], 0)) * k;
    return out;
  }

  function trajJointsAt(t) {
    if (!TRAJ.degs.length) return null;
    var times = TRAJ.times, degs = TRAJ.degs;
    var start = TRAJ.start || degs[0];
    if (t <= 0) return start.slice();
    if (t <= times[0]) return mix6(start, degs[0], t / Math.max(1e-6, times[0]));
    for (var i = 0; i < degs.length - 1; i++) {
      if (t <= times[i + 1]) {
        var span = Math.max(1e-6, times[i + 1] - times[i]);
        return mix6(degs[i], degs[i + 1], (t - times[i]) / span);
      }
    }
    return degs[degs.length - 1].slice();
  }

  function trajGripAt(t) {
    var g = TRAJ.grip.length ? TRAJ.grip[0] : S.grip;
    for (var i = 0; i < TRAJ.times.length; i++) {
      if (t >= TRAJ.times[i] - 1e-9) g = TRAJ.grip[i];
    }
    return clamp(num(g, S.grip), 0, 110);
  }

  function applyPreviewFrame() {
    if (!TRAJ.degs.length) return;
    var degs = trajJointsAt(S.t);
    if (!degs) return;
    for (var i = 0; i < 6; i++) S.joints[i] = degs[i];
    S.grip = trajGripAt(S.t);
    syncPreview();
    paintJointRows();
    paintGrip();
    renderReadout();
  }

  var playTimer = 0;
  var playWall = 0;
  var playBase = 0;

  function stopLocalPlay(keep) {
    if (playTimer) { clearInterval(playTimer); playTimer = 0; }
    if (!keep) { S.playing = false; renderSeek(); }
  }

  function startLocalPlay() {
    if (!TRAJ.degs.length) { toast('没有可预演的轨迹：先规划路点', 'warn'); return; }
    stopLocalPlay(true);
    if (S.t >= TRAJ.duration) S.t = 0;
    S.playing = true;
    playBase = S.t;
    playWall = nowMs();
    renderSeek();
    playTimer = setInterval(function () {
      S.t = playBase + (nowMs() - playWall) / 1000 * S.speed;
      if (S.t >= TRAJ.duration) {
        if (S.loop) { playBase = 0; playWall = nowMs(); S.t = 0; }
        else { S.t = TRAJ.duration; stopLocalPlay(); }
      }
      applyPreviewFrame();
      renderSeek();
    }, 16);
  }

  /* 服务端 /play 不可用时：按 50Hz 逐帧下发（契约 §5） */
  var streamTimer = 0;
  function stopLocalStream() {
    if (streamTimer) { clearInterval(streamTimer); streamTimer = 0; }
  }
  function startLocalStream() {
    if (!TRAJ.degs.length) { toast('没有可下发的轨迹', 'warn'); return; }
    stopLocalStream();
    stopLocalPlay();
    var base = 0;
    var t0 = nowMs();
    toast('服务端 /play 不可用，改为 50Hz 逐帧下发', 'warn');
    streamTimer = setInterval(function () {
      var t = base + (nowMs() - t0) / 1000 * S.speed;
      if (t >= TRAJ.duration) { t = TRAJ.duration; stopLocalStream(); }
      S.t = t;
      var degs = trajJointsAt(t);
      if (!degs) { stopLocalStream(); return; }
      for (var i = 0; i < 6; i++) S.joints[i] = degs[i];
      S.grip = trajGripAt(t);
      syncPreview();
      paintJointRows();
      paintGrip();
      renderReadout();
      renderSeek();
      post('/joints', { deg: degs }).catch(function () {});
    }, 20);
  }

  /* ---------------- 关节 ---------------- */
  var jointBody = $('#jointBody');

  function renderJoints() {
    jointBody.textContent = '';
    S.joints.forEach(function (val, i) {
      var r = JOINT_RANGES[i] || [0, 360];
      var row = el('div', 'joint-row');

      var label = el('span', 'jlabel');
      label.appendChild(el('span', 'jname', JOINT_NAMES[i]));
      label.appendChild(el('span', 'jrange', r[0] + '\u2013' + r[1] + '\u00b0'));
      row.appendChild(label);

      var slider = el('div', 'slider slider--joint');
      slider.appendChild(el('div', 'slider-track'));
      var fill = el('div', 'slider-fill');
      slider.appendChild(fill);
      slider.appendChild(el('div', 'slider-thumb'));
      var range = el('input');
      range.type = 'range';
      range.min = String(r[0]);
      range.max = String(r[1]);
      range.step = '0.1';
      range.value = String(val);
      range.setAttribute('aria-label', JOINT_NAMES[i] + ' 关节角度');
      range.setAttribute('data-j', String(i));
      slider.appendChild(range);
      row.appendChild(slider);

      var wrap = el('div', 'jbox-wrap');
      var box = el('input', 'jbox');
      box.type = 'text';
      box.setAttribute('inputmode', 'decimal');
      box.setAttribute('data-num', '');
      box.setAttribute('data-step', '1');
      box.setAttribute('data-range', String(r[1] - r[0]));
      box.setAttribute('data-j', String(i));
      box.value = fx(val, 1);
      box.setAttribute('aria-label', JOINT_NAMES[i] + ' 角度值');
      wrap.appendChild(box);
      var tgt = el('span', 'jtarget', '目标 ' + fx(S.targets[i], 1));
      wrap.appendChild(tgt);
      row.appendChild(wrap);

      jointBody.appendChild(row);
      paintJoint(row, i);
    });
  }

  function paintJoint(row, i) {
    var r = JOINT_RANGES[i] || [0, 360];
    var pct = clamp((S.joints[i] - r[0]) / (r[1] - r[0]), 0, 1) * 100;
    var fill = row.querySelector('.slider-fill');
    var thumb = row.querySelector('.slider-thumb');
    if (fill) fill.style.width = pct + '%';
    if (thumb) thumb.style.left = pct + '%';
  }

  jointBody.addEventListener('input', function (e) {
    var inp = e.target;
    var i = parseInt(inp.getAttribute('data-j'), 10);
    if (isNaN(i)) return;
    S.joints[i] = num(inp.value, S.joints[i]);
    var row = inp.closest('.joint-row');
    if (row) {
      var box = row.querySelector('.jbox');
      if (box && box !== inp) box.value = fx(S.joints[i], 1);
      paintJoint(row, i);
    }
    syncPreview();
    renderReadout();
  });

  jointBody.addEventListener('change', function (e) {
    var inp = e.target;
    var i = parseInt(inp.getAttribute('data-j'), 10);
    if (isNaN(i)) return;
    var r = JOINT_RANGES[i] || [0, 360];
    S.joints[i] = clamp(num(inp.value, S.joints[i]), r[0], r[1]);
    inp.value = fx(S.joints[i], 1);
    var row = inp.closest('.joint-row');
    if (row) {
      var box = row.querySelector('.jbox');
      if (box && box !== inp) box.value = fx(S.joints[i], 1);
      paintJoint(row, i);
    }
    syncPreview();
    renderReadout();
  });

  $('#jointCopy').addEventListener('click', function () {
    var txt = S.joints.map(function (v) { return fx(v, 1); }).join(', ');
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(txt).then(function () {
        toast('已复制关节角：' + txt, 'ok');
      }, function () {
        toast('复制失败：' + txt, 'warn');
      });
    } else {
      toast('关节角：' + txt);
    }
  });

  $('#jointPaste').addEventListener('click', function () {
    if (!navigator.clipboard || !navigator.clipboard.readText) {
      toast('浏览器不支持读取剪贴板', 'warn');
      return;
    }
    navigator.clipboard.readText().then(function (txt) {
      var arr = (txt.match(/-?\d+(\.\d+)?/g) || []).slice(0, 6).map(parseFloat);
      if (arr.length < 6) { toast('剪贴板里没有 6 个关节角', 'warn'); return; }
      S.joints = arr.map(function (v, i) {
        var r = JOINT_RANGES[i] || [0, 360];
        return clamp(v, r[0], r[1]);
      });
      renderJoints();
      syncPreview();
      renderReadout();
      toast('已粘贴 6 个关节角', 'ok');
    }, function () {
      toast('读取剪贴板被拒绝', 'warn');
    });
  });

  $('#jointHome').addEventListener('click', function () {
    var home = [180, 90, 83, 30, 110, 30];
    tweenJointsTo(home, 400);
    soft(post('/home', {}), 'home').then(function (j) {
      if (!j) { toast('回默认位仅预览：后端未连接', 'warn'); return; }
      if (j.ok === false) { toast('回默认位失败：' + (j.error || '未知错误'), 'warn'); return; }
      toast('已回默认位', 'ok');
    });
  });

  /* 「读取真机」：GET /state 拉回读关节角 → 平滑 400ms 跟到预览 */
  $('#jointRead').addEventListener('click', function () {
    soft(get('/state'), 'state:read').then(function (j) {
      if (!j) { toast('读取真机失败：后端未连接', 'warn'); return; }
      var st = j.state || j;
      if (!st || !st.joints || !Array.isArray(st.joints.deg)) {
        toast('读取真机失败：回包里没有关节角', 'warn');
        return;
      }
      var degs = st.joints.deg.slice(0, 6).map(function (v) { return num(v, 0); });
      tweenJointsTo(degs, 400);
      if (st.gripper && st.gripper.deg != null) {
        S.grip = clamp(num(st.gripper.deg, S.grip), 0, 110);
        paintGrip();
        if (view) view.setGripperDeg(S.grip);
      }
      toast('已读取真机：' + degs.map(function (v) { return fx(v, 1); }).join(' '), 'ok');
    });
  });

  /* 「应用关节角度」：这一步才真正下发到 MuJoCo（契约 §5） */
  $('#jointApply').addEventListener('click', function () {
    var btn = this;
    btn.disabled = true;
    soft(post('/joints', { deg: S.joints }), 'joints').then(function (j) {
      btn.disabled = false;
      if (!j) { toast('下发失败：后端未连接', 'warn'); return; }
      if (j.ok === false) { toast('下发失败：' + (j.error || '未知错误'), 'warn'); return; }
      toast('已下发关节角度：' + S.joints.map(function (v) { return fx(v, 1); }).join(' '), 'ok');
    });
  });

  /* ---------------- 运动学 ---------------- */
  var kinFields = $('#kinFields');
  var kinDot = $('#kinDot');
  var kinStatus = $('#kinStatus');
  var kinIk = $('#kinIk');

  var IK_KIND = {
    ok: ['is-ok', '可达'],
    self_collision: ['is-accent', '自我干涉'],
    wrist_flip: ['is-warn', '不可达（需翻腕）'],
    discontinuity: ['is-warn', '不可达（关节突跳）'],
    unreachable: ['is-accent', '不可达（够不到）']
  };

  function setIk(status, text, err) {
    var k = IK_KIND[status] || ['is-idle', text || '待机'];
    kinDot.className = 'kdot ' + k[0];
    kinStatus.textContent = text || k[1];
    var e = err != null ? num(err, 0) : null;
    if (e != null) kinIk.textContent = 'IK ' + fx(e, 1) + ' MM';
    var ro = $('#roIk');
    if (ro && e != null) ro.textContent = 'IK 误差 ' + fx(e, 1) + ' MM';
  }

  function poseRows() {
    return $$('.pose-in', kinFields);
  }

  function gatherPose(frame) {
    var out = { mm: [], deg: [] };
    poseRows().filter(function (i) { return i.getAttribute('data-frame') === frame; })
      .forEach(function (i) {
        out[i.getAttribute('data-axis')].push(num(i.value, 0));
      });
    return { frame: frame, mm: out.mm, deg: out.deg };
  }

  function submitPose(frame) {
    var body = gatherPose(frame);
    var rows = poseRows().filter(function (i) { return i.getAttribute('data-frame') === frame; });
    soft(post('/pose', body), 'pose').then(function (j) {
      if (!j) { toast('解 IK 失败：后端未连接', 'warn'); return; }
      rows.forEach(function (i) { i.classList.remove('is-bad'); });
      var st = j.state || {};
      if (st.ik) setIk(st.ik.status, st.ik.text, st.ik.pos_err_mm);
      else if (j.ok === false) setIk('unreachable', null, null);
      if (j.ok === false) {
        rows.forEach(function (i) { i.classList.add('is-bad'); });
        setIk((j.ik && j.ik.status) || 'unreachable', null, j.ik && j.ik.pos_err_mm);
        toast('IK 解不出：' + (j.error || (j.ik && j.ik.text) || '不可达'), 'warn');
        return;
      }
      var tgt = (st.joints && (st.joints.target || st.joints.deg)) || null;
      if (Array.isArray(tgt) && tgt.length >= 6) {
        tweenJointsTo(tgt.slice(0, 6).map(function (v) { return num(v, 0); }), 400);
        toast('IK 已解出并下发：' + tgt.slice(0, 6).map(function (v) { return fx(num(v, 0), 1); }).join(' '), 'ok');
      }
    });
  }

  kinFields.addEventListener('keydown', function (e) {
    if (e.key === 'Enter' && e.target.classList.contains('pose-in')) {
      e.preventDefault();
      submitPose(e.target.getAttribute('data-frame'));
    }
  });

  kinFields.addEventListener('change', function (e) {
    if (!e.target.classList.contains('pose-in')) return;
    e.target.value = fx(num(e.target.value, 0), 1);
  });

  /* ---------------- 夹爪 ---------------- */
  var gripRange = $('#grip');
  var gripValue = $('#gripValue');
  var gripWrap = $('#gripSlider');

  function paintGrip() {
    var pct = clamp(S.grip / 110, 0, 1) * 100;
    var fill = gripWrap.querySelector('.slider-fill');
    var thumb = gripWrap.querySelector('.slider-thumb');
    if (fill) fill.style.width = pct + '%';
    if (thumb) thumb.style.left = pct + '%';
    gripRange.value = String(Math.round(S.grip));
    if (gripValue) gripValue.value = fx(S.grip, 1);
  }

  var gripTimer = 0;
  function pushGrip() {
    clearTimeout(gripTimer);
    gripTimer = setTimeout(function () {
      soft(post('/gripper', { deg: S.grip }), 'gripper');
    }, 60);
  }

  function markGripSet(v) {
    S.grip = clampGrip(v);
    S.gripSet = S.grip;
    S.gripSetAt = nowMs();
  }

  gripRange.addEventListener('input', function () {
    markGripSet(gripRange.value);
    paintGrip();
    pushGrip();
  });
  gripValue.addEventListener('change', function () {
    markGripSet(num(gripValue.value, S.grip));
    paintGrip();
    pushGrip();
  });

  /* ---------------- 播放 ---------------- */
  var playBtn = $('#playBtn');
  var seek = $('#seek');
  var playTime = $('#playTime');
  var seekSlider = $('#seekSlider');
  var icPlay = $('.ic-play', playBtn);
  var icPause = $('.ic-pause', playBtn);

  function renderSeek() {
    var total = S.duration > 0 ? S.duration : 0;
    var pct = clamp(S.t / total, 0, 1);
    seek.value = String(Math.round(pct * 1000));
    var fill = seekSlider.querySelector('.slider-fill');
    var thumb = seekSlider.querySelector('.slider-thumb');
    if (fill) fill.style.width = (pct * 100) + '%';
    if (thumb) thumb.style.left = (pct * 100) + '%';
    playTime.textContent = fx(S.t, 1) + ' S / ' + (total > 0 ? (fx(total, 1) + ' S') : '—');
    if (icPlay && icPause) {
      if (S.playing) { icPlay.setAttribute('hidden', ''); icPause.removeAttribute('hidden'); }
      else { icPause.setAttribute('hidden', ''); icPlay.removeAttribute('hidden'); }
      playBtn.setAttribute('aria-label', S.playing ? '暂停' : '播放');
    }
  }

  playBtn.addEventListener('click', function () {
    if (S.playing) stopLocalPlay();
    else startLocalPlay();
  });

  seek.addEventListener('input', function () {
    var total = S.duration > 0 ? S.duration : 0;
    S.t = num(seek.value, 0) / 1000 * total;
    applyPreviewFrame();
    renderSeek();
  });

  $('#chkLoop').addEventListener('change', function () {
    S.loop = this.checked;
  });

  $('#speedBtn').addEventListener('click', function () {
    var steps = [0.5, 1.0, 1.5, 2.0];
    var i = steps.indexOf(S.speed);
    S.speed = steps[(i + 1) % steps.length];
    this.textContent = S.speed.toFixed(1) + '\u00d7';
    if (S.playing) startLocalPlay();
  });

  /* 「应用」：把当前预演位姿一次性下发；「运行」：走服务端 /play，拿不到就 50Hz 逐帧下发 */
  $('#trajApply').addEventListener('click', function () {
    var btn = this;
    btn.disabled = true;
    soft(post('/joints', { deg: S.joints }), 'joints:apply').then(function (j) {
      btn.disabled = false;
      if (!j) { toast('下发失败：后端未连接', 'warn'); return; }
      if (j.ok === false) { toast('下发失败：' + (j.error || '未知错误'), 'warn'); return; }
      toast('已下发当前位姿（t=' + fx(S.t, 2) + ' s）', 'ok');
    });
  });

  $('#trajRun').addEventListener('click', function () {
    if (!TRAJ.degs.length) { toast('先规划路点再运行', 'warn'); return; }
    var btn = this;
    btn.disabled = true;
    soft(post('/play', { action: 'play', speed: S.speed, loop: S.loop }), 'play:run').then(function (j) {
      btn.disabled = false;
      if (j && j.ok !== false) {
        toast('已在 MuJoCo 运行规划轨迹（' + fx(TRAJ.duration, 1) + ' s）', 'ok');
        return;
      }
      if (j && j.error) toast('服务端 /play 拒绝：' + j.error + '，改为逐帧下发', 'warn');
      else if (!j) toast('服务端未连接，改为逐帧下发', 'warn');
      startLocalStream();
    });
  });

  /* ---------------- 通用数值键 ---------------- */
  document.addEventListener('keydown', function (e) {
    var elx = e.target;
    if (!elx || elx.tagName !== 'INPUT' || !elx.hasAttribute('data-num')) return;
    var step = num(elx.getAttribute('data-step'), 1);
    var range = num(elx.getAttribute('data-range'), 0);
    var d = 0;
    if (e.key === 'ArrowUp') d = e.shiftKey ? step * 10 : step;
    else if (e.key === 'ArrowDown') d = -(e.shiftKey ? step * 10 : step);
    else if (e.key === 'PageUp') d = range > 0 ? range / 10 : step * 10;
    else if (e.key === 'PageDown') d = -(range > 0 ? range / 10 : step * 10);
    else if (e.key === 'Enter') {
      e.preventDefault();
      elx.blur();
      return;
    }
    if (!d) return;
    e.preventDefault();
    elx.value = String(num(elx.value, 0) + d);
    elx.dispatchEvent(new Event('input', { bubbles: true }));
    elx.dispatchEvent(new Event('change', { bubbles: true }));
  });

  /* ---------------- 右栏滚动提示 ---------------- */
  var rightScroll = $('#rightScroll');
  var rightBar = $('#rightBar');
  var rightThumb = $('#rightThumb');

  function syncScrollbar() {
    var ch = rightScroll.clientHeight;
    var sh = rightScroll.scrollHeight;
    if (sh - ch < 4) { rightBar.hidden = true; return; }
    rightBar.hidden = false;
    var h = Math.max(24, Math.round(ch * ch / sh));
    var top = Math.round(rightScroll.scrollTop / (sh - ch) * (ch - h));
    rightThumb.style.height = h + 'px';
    rightThumb.style.marginTop = top + 'px';
  }
  rightScroll.addEventListener('scroll', syncScrollbar);
  window.addEventListener('resize', syncScrollbar);

  /* ---------------- 实时状态 ---------------- */
  var pollTimer = 0;
  var pollBusy = false;

  function applyState(st) {
    if (!st) return;
    /* 关节角 / 位姿只由本地预览（本地 FK）决定：/state 只同步「目标」与 IK 状态，
       不再覆盖 S.joints 和「运动学状态」输入框（契约 V2.1 §5） */
    if (st.joints && Array.isArray(st.joints.target)) {
      st.joints.target.forEach(function (v, i) { if (i < 6) S.targets[i] = num(v, S.targets[i]); });
    }
    $$('.joint-row', jointBody).forEach(function (row, i) {
      var tgt = row.querySelector('.jtarget');
      if (tgt) tgt.textContent = '目标 ' + fx(S.targets[i], 1);
    });
    if (st.ik) setIk(st.ik.status, st.ik.text, st.ik.pos_err_mm);
    if (st.gripper && st.gripper.deg != null && document.activeElement !== gripValue && document.activeElement !== grip) {
      var actual = num(st.gripper.deg, S.grip);
      var settling = S.gripSet != null && Math.abs(S.grip - S.gripSet) < 0.6 &&
                     (nowMs() - S.gripSetAt) < 1500;
      if (!settling || Math.abs(actual - S.gripSet) < 0.6) {
        S.grip = actual;
        if (settling) S.gripSet = null;
        paintGrip();
      }
    }
    if (st.fps != null) {
      var f = $$('.vp-fps')[0];
      if (f) f.textContent = Math.round(num(st.fps, 30)) + ' FPS';
    }
    if (st.mode) {
      var live = $('#roLiveText');
      if (live) live.textContent = '本地 FK · 不上传画面';
    }
  }

  function poll() {
    if (pollBusy || OFFLINE) return;
    pollBusy = true;
    soft(get('/state'), 'state').then(function (j) {
      pollBusy = false;
      if (j && j.ok !== false) applyState(j.state || j);
    });
  }

  document.addEventListener('visibilitychange', function () {
    if (document.hidden) { clearInterval(pollTimer); pollTimer = 0; }
    else if (!pollTimer) { poll(); pollTimer = setInterval(poll, 250); }
  });

  /* ---------------- 生成脚本 ---------------- */
  $('#cmdConfirm').addEventListener('click', function () {
    var name = ($('#fileName').value || 'pick_and_place.py').trim();
    if (name.indexOf('.py') < 0) name += '.py';
    var btn = this;
    btn.disabled = true;
    soft(post('/generate', { name: name, waypoints: wpPayload() }), 'generate').then(function (j) {
      btn.disabled = false;
      if (!j) { toast('生成失败：后端未连接', 'warn'); return; }
      if (j.ok === false) { toast('生成失败：' + (j.error || '未知错误'), 'warn'); return; }
      toast('已生成 scripts/' + name, 'ok');
      loadScripts();
    });
  });

  $('#cmdCancel').addEventListener('click', function () {
    $('#fileName').value = 'pick_and_place.py';
    toast('已取消');
  });

  $('#estopBtn').addEventListener('click', function () {
    soft(post('/estop', {}), 'estop').then(function (j) {
      if (!j) { toast('急停指令未送达（后端未连接）', 'warn'); return; }
      toast('已急停', 'warn');
    });
  });

  /* ---------------- 初始化 ---------------- */
  renderPoints();
  renderJoints();
  paintGrip();
  syncPreview();
  renderReadout();
  renderSeek();
  renderScripts(LOCAL_SCRIPTS);
  setCmdMeta();
  syncScrollbar();
  setTimeout(syncScrollbar, 60);

  poll();
  pollTimer = setInterval(poll, 250);
  /* 首屏自动规划一次：把「路点 · 时长 · 是否全部可达」填成真实值（只算不动） */
  setTimeout(function () { if (!S._autoPlanned) { S._autoPlanned = true; plan(); } }, 1500);
  soft(get('/scripts'), 'scripts').then(function (j) {
    if (j && Array.isArray(j.items) && j.items.length) renderScripts(j.items);
  });
})();
