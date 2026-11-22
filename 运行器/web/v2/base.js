/* ==========================================================================
   V2 · 基础控制页  /v2/base
   原生 JS，无依赖。契约见 运行器/UI/上位机设计/接口契约.md。
   - SSE /api/v2/events（10 Hz）刷新状态与日志，断线带 since 续传，失败退回轮询
   - 控制一律 POST /api/v2/*
   - 后端未启动时：不抛错、不白屏，界面进入「未连接」态
   - ?demo=1：仅用于与 A-基础控制.png 做版式对照，注入静态样例数据，不发任何网络请求
   ========================================================================== */
(function () {
  'use strict';

  var DEMO = /[?&]demo=1\b/.test(location.search);
  var DASH = '\u2014';       // —
  var MINUS = '\u2212';      // 真减号，与设计稿一致
  var API = '';               // 同源；页面由 server/app.py 提供
  var JOINT_RANGE = [[0, 340], [0, 180], [0, 163], [0, 335], [0, 220], [0, 335]];
  var ROW_H = 28;

  var app = {
    mode: 'sim',
    channel: 1,
    port: 47999,
    connected: false,
    estop: false,
    logEnabled: true,
    ranges: JOINT_RANGE.map(function (r) { return r.slice(); }),
    actual: [null, null, null, null, null, null],
    target: [null, null, null, null, null, null],
    limits: [false, false, false, false, false, false],
    gripper: { deg: null, target: null, range: [0, 110] },
    app: { running: false, pid: null, port: 47999 },
    lastSeq: 0,
    dirty: {},
    totalLines: 0,
    offlineNoted: false
  };

  var el = {};
  var rows = [];
  var es = null;
  var retryTimer = 0;
  var failures = 0;

  /* ---------- 小工具 ---------- */
  function $(sel, root) { return (root || document).querySelector(sel); }
  function $$(sel, root) { return Array.prototype.slice.call((root || document).querySelectorAll(sel)); }

  function setText(node, text) {
    if (node && node.textContent !== text) node.textContent = text;
  }

  function fmt(v, digits) {
    if (v === null || v === undefined || !isFinite(v)) return DASH;
    var d = digits === undefined ? 1 : digits;
    return (v < 0 ? MINUS : '') + Math.abs(v).toFixed(d);
  }

  function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

  function timeLabel(t) {
    var d = null;
    if (typeof t === 'number' && isFinite(t)) d = new Date(t > 1e12 ? t : t * 1000);
    else if (typeof t === 'string' && t) {
      var m = t.match(/(\d{2}:\d{2}:\d{2})/);
      if (m) return m[1];
      d = new Date(t);
    } else d = new Date();
    if (!d || isNaN(d.getTime())) return '--:--:--';
    return [d.getHours(), d.getMinutes(), d.getSeconds()]
      .map(function (n) { return String(n).padStart(2, '0'); }).join(':');
  }

  function toast(text, kind) {
    if (!el.toastHost) return;
    var node = document.createElement('div');
    node.className = 'toast toast--' + (kind || 'info');
    node.innerHTML = '<span class="toast-dot"></span><span class="toast-text"></span>';
    node.querySelector('.toast-text').textContent = text;
    el.toastHost.appendChild(node);
    setTimeout(function () {
      node.style.opacity = '0';
      setTimeout(function () { node.remove(); }, 200);
    }, 3200);
  }

  /* ---------- 网络 ---------- */
  function getJSON(path, timeoutMs) {
    var ctrl = typeof AbortController !== 'undefined' ? new AbortController() : null;
    var timer = setTimeout(function () { if (ctrl) ctrl.abort(); }, timeoutMs || 2500);
    return fetch(API + path, { signal: ctrl ? ctrl.signal : undefined })
      .then(function (res) {
        if (!res.ok) throw new Error('HTTP ' + res.status);
        return res.json();
      })
      .then(function (data) { clearTimeout(timer); return data; })
      .catch(function (err) { clearTimeout(timer); throw err; });
  }

  function post(path, body) {
    return fetch(API + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    }).then(function (res) {
      if (!res.ok) return { ok: false, error: 'HTTP ' + res.status };
      return res.json().catch(function () { return { ok: true }; });
    }).catch(function () {
      return { ok: false, error: '网络不可达 · 后端未启动' };
    });
  }

  /* ---------- 日志 ---------- */
  function appendLog(items) {
    if (!items || !items.length) return;
    var list = el.logList;
    var stick = list.scrollTop + list.clientHeight >= list.scrollHeight - 4;

    items.forEach(function (item) {
      app.totalLines += 1;
      if (!app.logEnabled) return;
      var level = item.level || 'info';
      var row = document.createElement('div');
      row.className = 'log-row' + (level === 'info' ? '' : ' is-' + level);
      var t = document.createElement('span');
      t.className = 'log-t';
      t.textContent = timeLabel(item.t);
      var x = document.createElement('span');
      x.className = 'log-x';
      x.textContent = item.text == null ? '' : String(item.text);
      row.appendChild(t);
      row.appendChild(x);
      if (level === 'error') {
        var dot = document.createElement('span');
        dot.className = 'log-dot';
        row.appendChild(dot);
      }
      list.appendChild(row);
    });

    while (list.children.length > 400) list.removeChild(list.firstChild);
    if (stick) list.scrollTop = list.scrollHeight;
    paintLogMeta();
  }

  function paintLogMeta() {
    setText(el.logMeta, app.totalLines + ' 行 · ' + (app.logEnabled ? '自动滚动' : '日志暂停'));
    var scrolled = Math.round(el.logList.scrollTop / ROW_H);
    setText(el.logCount, 'SCROLL ' + scrolled + ' / ' + app.totalLines);
  }

  function localLog(level, text) {
    appendLog([{ t: Date.now() / 1000, level: level, text: text }]);
  }

  /* ---------- 状态绘制 ---------- */
  function paintStatus() {
    var text, cls;
    if (app.estop) { text = '急停已触发'; cls = 'is-estop'; }
    else if (app.connected) {
      text = app.mode === 'sim' ? '仿真应用已连接' : '真机已连接';
      cls = 'is-ok';
    } else if (app.app && app.app.running) { text = '仿真应用就绪 · 未连接'; cls = 'is-idle'; }
    else { text = '未连接'; cls = 'is-idle'; }
    setText(el.statusText, text);
    el.statusDot.className = 'status-dot ' + cls;
    setText(el.stateMeta, app.mode === 'sim'
      ? (app.app && app.app.running ? 'MUJOCO · PID ' + (app.app.pid || '外部') : 'SIMULATION')
      : 'REAL');
    setText(el.hintMeta, (app.mode === 'sim' ? 'SIM' : 'REAL') + ' · CHANNEL USB' + app.channel);
  }

  function paintPose(s) {
    var pose = (s && s.tcp) || (s && s.flange) || null;
    var mm = pose && pose.mm;
    var deg = pose && pose.deg;
    setText(el.poseX, mm ? fmt(mm[0], 1) : DASH);
    setText(el.poseY, mm ? fmt(mm[1], 1) : DASH);
    setText(el.poseZ, mm ? fmt(mm[2], 1) : DASH);
    setText(el.poseRx, deg ? fmt(deg[0], 1) + '\u00b0' : DASH);
    setText(el.poseRy, deg ? fmt(deg[1], 1) + '\u00b0' : DASH);
    setText(el.poseRz, deg ? fmt(deg[2], 1) + '\u00b0' : DASH);
  }

  function paintJointReadout() {
    for (var i = 0; i < 6; i++) {
      setText(el.jv[i], fmt(app.actual[i], 1));
      setText(el.jt[i], '目标 ' + fmt(app.target[i], 1));
    }
  }

  function sliderWidth(sliderEl) {
    var w = sliderEl.clientWidth;
    if (!w) w = parseFloat(getComputedStyle(sliderEl).width) || 0;
    return w;
  }

  function paintSlider(sliderEl) {
    var input = $('input', sliderEl);
    if (!input) return;
    var min = parseFloat(input.min);
    var max = parseFloat(input.max);
    var v = parseFloat(input.value);
    if (!isFinite(v)) v = min;
    var frac = max > min ? clamp((v - min) / (max - min), 0, 1) : 0;
    var w = sliderWidth(sliderEl);
    $('.s-fill', sliderEl).style.width = (frac * w).toFixed(2) + 'px';
    $('.s-thumb', sliderEl).style.left = (frac * w).toFixed(2) + 'px';
  }

  function rowValue(row) {
    var v = parseFloat(row.num.value);
    return isFinite(v) ? v : null;
  }

  function paintRow(row, i) {
    var range = app.ranges[i] || JOINT_RANGE[i];
    var lo = range[0], hi = range[1];
    var input = row.slider.querySelector('input');
    if (parseFloat(input.min) !== lo) input.min = lo;
    if (parseFloat(input.max) !== hi) input.max = hi;
    setText(row.rangeText, lo + '\u2013' + hi + '\u00b0');
    var target = app.target[i];
    var actual = app.actual[i];
    var dirty = !!app.dirty[i];

    if (!dirty && target !== null && target !== undefined) {
      if (parseFloat(input.value) !== target) input.value = target;
      setText(row.target, '目标 ' + fmt(target, 1));
    } else if (dirty) {
      var pending = rowValue(row);
      setText(row.target, '目标 ' + fmt(pending, 1));
    }

    if (!dirty && document.activeElement !== row.num) {
      var shown = actual === null || actual === undefined ? DASH : fmt(actual, 1);
      if (row.num.value !== shown) row.num.value = shown;
    }

    var atLimit = !!app.limits[i];
    if (dirty) {
      var pending2 = rowValue(row);
      atLimit = pending2 !== null && (pending2 <= lo + 0.05 || pending2 >= hi - 0.05);
    }
    row.numbox.classList.toggle('is-limit', atLimit);
    paintSlider(row.slider);
  }

  function paintGripper() {
    var g = app.gripper;
    var range = g.range || [0, 110];
    var input = el.gripperInput;
    if (parseFloat(input.min) !== range[0]) input.min = range[0];
    if (parseFloat(input.max) !== range[1]) input.max = range[1];
    setText(el.gripperRange, range[0] + '\u2013' + range[1] + '\u00b0');
    if (!app.dirty.grip && g.target !== null && g.target !== undefined) {
      if (parseFloat(input.value) !== g.target) input.value = g.target;
    }
    if (!app.dirty.grip && document.activeElement !== el.gripperNum) {
      var shown = g.deg === null || g.deg === undefined ? DASH : fmt(g.deg, 1);
      if (el.gripperNum.value !== shown) el.gripperNum.value = shown;
    }
    paintSlider(el.gripperSlider);
  }

  function paint() {
    paintStatus();
    paintJointReadout();
    rows.forEach(paintRow);
    paintGripper();
  }

  function applyState(s) {
    if (!s || typeof s !== 'object') return;
    if (s.mode) app.mode = s.mode;
    app.connected = !!s.connected;
    app.estop = !!s.estop;
    if (s.app && typeof s.app === 'object') {
      app.app = {
        running: !!s.app.running,
        pid: s.app.pid == null ? null : s.app.pid,
        port: s.app.port || app.port
      };
      if (el.btnAppStop) el.btnAppStop.disabled = !app.app.running;
    }

    var j = s.joints || {};
    if (Array.isArray(j.deg) && j.deg.length >= 6) app.actual = j.deg.slice(0, 6).map(Number);
    if (Array.isArray(j.target) && j.target.length >= 6) app.target = j.target.slice(0, 6).map(Number);
    if (Array.isArray(j.range) && j.range.length >= 6) app.ranges = j.range.slice(0, 6);
    if (Array.isArray(j.limit) && j.limit.length >= 6) app.limits = j.limit.slice(0, 6);

    var g = s.gripper || {};
    if (isFinite(g.deg)) app.gripper.deg = Number(g.deg);
    if (isFinite(g.target)) app.gripper.target = Number(g.target);
    if (Array.isArray(g.range)) app.gripper.range = g.range;

    paintPose(s);
    paint();
  }

  function setOffline(reason) {
    app.connected = false;
    paint();
    if (!app.offlineNoted) {
      app.offlineNoted = true;
      localLog('error', '未连接 · ' + (reason || '后端未启动') + ' · 上位机 ' + (location.host || '127.0.0.1:8770'));
    }
  }

  /* ---------- 关节控制 ---------- */
  function markDirty(i, value) {
    app.dirty[i] = true;
    var row = rows[i];
    var range = app.ranges[i] || JOINT_RANGE[i];
    var v = clamp(value, range[0], range[1]);
    row.num.value = v.toFixed(1);
    row.slider.querySelector('input').value = v;
    paintRow(row, i);
  }

  function stepJoint(i, delta) {
    var row = rows[i];
    var range = app.ranges[i] || JOINT_RANGE[i];
    var base = rowValue(row);
    if (base === null) base = app.actual[i] !== null ? app.actual[i] : range[0];
    markDirty(i, base + delta);
  }

  function applyJoints() {
    var deg = [];
    for (var i = 0; i < 6; i++) {
      var range = app.ranges[i] || JOINT_RANGE[i];
      var v = rowValue(rows[i]);
      if (v === null) v = app.target[i] !== null ? app.target[i] : app.actual[i];
      if (v === null) v = range[0];
      deg.push(Number(clamp(v, range[0], range[1]).toFixed(1)));
    }
    return post('/api/v2/joints', { deg: deg }).then(function (res) {
      if (res && res.ok) {
        app.dirty = {};
        if (res.state) applyState(res.state);
        else paint();
        localLog('info', '关节目标已下发 · ' + deg.map(function (d) { return d.toFixed(1); }).join(' / '));
        toast('关节目标已下发', 'ok');
      } else {
        toast('下发失败 · ' + ((res && (res.error || (res.ik && res.ik.text))) || '未知错误'), 'err');
      }
      return res;
    });
  }

  function bindRows() {
    rows = $$('.jrow').map(function (rowEl, i) {
      var row = {
        el: rowEl,
        slider: $('.slider', rowEl),
        num: $('.num', rowEl),
        target: $('.target', rowEl),
        numbox: $('.numbox', rowEl),
        rangeText: $('[data-range]', rowEl)
      };
      var input = row.slider.querySelector('input');
      input.addEventListener('input', function () {
        app.dirty[i] = true;
        if (row.num.value !== fmt(parseFloat(input.value), 1)) row.num.value = fmt(parseFloat(input.value), 1);
        paintRow(row, i);
      });
      input.addEventListener('change', function () { paintRow(row, i); });

      row.num.addEventListener('input', function () { app.dirty[i] = true; });
      row.num.addEventListener('focus', function () {
        var v = rowValue(row);
        if (v !== null) row.num.value = v.toFixed(1);
        row.num.select();
      });
      row.num.addEventListener('blur', function () {
        if (row.suppress) return;
        var v = rowValue(row);
        if (v !== null) markDirty(i, v);
        else { delete app.dirty[i]; }
        paintRow(row, i);
      });
      row.num.addEventListener('keydown', function (ev) {
        var range = app.ranges[i] || JOINT_RANGE[i];
        if (ev.key === 'ArrowUp' || ev.key === 'ArrowDown') {
          ev.preventDefault();
          stepJoint(i, (ev.key === 'ArrowUp' ? 1 : -1) * (ev.shiftKey ? 10 : 1));
        } else if (ev.key === 'PageUp' || ev.key === 'PageDown') {
          ev.preventDefault();
          var span = (range[1] - range[0]) / 10;
          stepJoint(i, (ev.key === 'PageUp' ? span : -span));
        } else if (ev.key === 'Enter') {
          ev.preventDefault();
          row.num.blur();
          applyJoints();
        } else if (ev.key === 'Escape') {
          ev.preventDefault();
          delete app.dirty[i];
          row.suppress = true;
          row.num.value = app.actual[i] === null || app.actual[i] === undefined ? DASH : fmt(app.actual[i], 1);
          row.num.blur();
          row.suppress = false;
          paintRow(row, i);
        }
      });
      return row;
    });
  }

  /* ---------- 夹爪 ---------- */
  function applyGripper() {
    var range = app.gripper.range || [0, 110];
    var v = parseFloat(el.gripperNum.value);
    if (!isFinite(v)) v = app.gripper.target !== null ? app.gripper.target : 0;
    v = clamp(v, range[0], range[1]);
    return post('/api/v2/gripper', { deg: Number(v.toFixed(1)) }).then(function (res) {
      if (res && res.ok) {
        app.dirty.grip = false;
        if (res.state) applyState(res.state); else paintGripper();
        localLog('info', '夹爪目标已下发 · ' + v.toFixed(1) + '°');
        toast('夹爪已下发 · ' + v.toFixed(1) + '°', 'ok');
      } else {
        toast('夹爪下发失败 · ' + ((res && res.error) || '未知错误'), 'err');
      }
      return res;
    });
  }

  /* ---------- 连接 ---------- */
  function selectMode(mode) {
    if (mode === 'real' && app.mode !== 'real') {
      var ok = window.confirm('真机模式需二次确认：将连接真实机械臂，确认继续？');
      if (!ok) return;
    }
    app.mode = mode;
    $$('.seg-btn').forEach(function (b) {
      var on = b.dataset.mode === mode;
      b.classList.toggle('is-on', on);
      b.setAttribute('aria-pressed', on ? 'true' : 'false');
    });
    paintStatus();
    if (app.connected) connect();
  }

  function connect() {
    var body = app.mode === 'sim'
      ? { mode: 'sim', port: app.port }
      : { mode: 'real', channel: app.channel, port: app.port };
    if (app.mode === 'sim') {
      localLog('info', '连接请求 · 仿真 · 127.0.0.1:' + app.port + '（必要时自动拉起仿真应用）');
    } else {
      localLog('info', '连接请求 · 真机 · CHANNEL USB' + app.channel + ' · 端口 ' + app.port);
    }
    return post('/api/v2/connect', body).then(function (res) {
      if (res && res.ok) {
        if (res.state) applyState(res.state);
        if (app.mode === 'sim') {
          localLog('info', '已连接 MuJoCo 仿真应用 · 127.0.0.1:' + app.port +
            (app.app && app.app.pid ? ' · PID ' + app.app.pid : ''));
          toast('已连接 MuJoCo 仿真应用', 'ok');
        } else {
          toast('已连接 · 真机', 'ok');
        }
        startStream();
      } else {
        toast('连接失败 · ' + ((res && res.error) || '未知错误'), 'err');
        localLog('error', '连接失败 · ' + ((res && res.error) || '后端未响应'));
      }
      return res;
    });
  }

  function disconnect() {
    return post('/api/v2/disconnect', {}).then(function (res) {
      if (res && res.state) applyState(res.state);
      localLog('info', app.mode === 'sim'
        ? '已断开 MuJoCo 仿真应用（应用进程仍在运行）'
        : '已断开真机连接');
      toast('已停止', 'info');
      return res;
    });
  }

  /* 关闭独立仿真应用进程：断开后它会真的退出，下次「启动服务器」会重新拉起 */
  function stopApp() {
    if (!window.confirm('关闭 MuJoCo 仿真应用？上位机会同时断开连接。')) return Promise.resolve({ ok: false });
    return post('/api/v2/app', { action: 'stop' }).then(function (res) {
      if (res && res.state) applyState(res.state);
      if (res && res.ok) {
        localLog('info', 'MuJoCo 仿真应用已关闭');
        toast('仿真应用已关闭', 'info');
      } else {
        localLog('error', '关闭仿真应用失败 · ' + ((res && res.error) || '未知错误'));
        toast('关闭失败 · ' + ((res && res.error) || '未知错误'), 'err');
      }
      return res;
    });
  }

  function estop() {
    var release = !!app.estop;   // 已在急停中 → 同一个按钮变成解除
    return post('/api/v2/estop', { on: !release }).then(function (res) {
      if (res && res.state) applyState(res.state);
      if (res && res.ok === false) {
        toast((release ? '解除急停失败 · ' : '急停失败 · ') + ((res && res.error) || '未知错误'), 'err');
        return res;
      }
      if (release) {
        localLog('info', '急停已解除 · 可以重新下发目标');
        toast('急停已解除', 'ok');
      } else {
        localLog('warn', '急停触发 · 仿真应用目标冻结在当前位置');
        toast('急停已触发', 'warn');
      }
      return res;
    });
  }

  function home() {
    return post('/api/v2/home', {}).then(function (res) {
      if (res && res.ok) {
        app.dirty = {};
        if (res.state) applyState(res.state); else paint();
        localLog('info', '回默认位 · convention.OFFSET');
        toast('已回默认位', 'ok');
      } else {
        toast('回零失败 · ' + ((res && res.error) || '未知错误'), 'err');
      }
      return res;
    });
  }

  /* ---------- SSE ---------- */
  function startStream() {
    if (DEMO) return;
    if (location.protocol === 'file:') {
      setOffline('本地文件打开 · 后端未启动');
      return;
    }
    if (typeof EventSource === 'undefined') { startPolling(); return; }
    if (es) { try { es.close(); } catch (e) { /* noop */ } es = null; }
    var url = API + '/api/v2/events' + (app.lastSeq ? '?since=' + app.lastSeq : '');
    var src;
    try { src = new EventSource(url); } catch (e) { startPolling(); return; }
    es = src;
    src.onopen = function () { failures = 0; app.offlineNoted = false; };
    src.onmessage = function (ev) {
      var data;
      try { data = JSON.parse(ev.data); } catch (e) { return; }
      if (typeof data.seq === 'number') app.lastSeq = data.seq;
      if (data.state) applyState(data.state);
      if (Array.isArray(data.log)) appendLog(data.log);
    };
    src.onerror = function () {
      try { src.close(); } catch (e) { /* noop */ }
      es = null;
      failures += 1;
      if (failures >= 2) { setOffline('事件流中断'); startPolling(); }
      clearTimeout(retryTimer);
      retryTimer = setTimeout(startStream, failures >= 2 ? 4000 : 1200);
    };
  }

  var pollTimer = 0;
  function startPolling() {
    if (pollTimer) return;
    pollTimer = setInterval(function () {
      getJSON('/api/v2/state', 2000).then(function (s) {
        if (s && s.ok !== false) applyState(s);
      }).catch(function () { /* 保持未连接态 */ });
    }, 900);
  }

  /* ---------- 启动 ---------- */
  function bindUI() {
    el.statusDot = $('#statusDot');
    el.statusText = $('#statusText');
    el.stateMeta = $('#stateMeta');
    el.logList = $('#logList');
    el.logMeta = $('#logMeta');
    el.logCount = $('#logCount');
    el.poseX = $('#poseX'); el.poseY = $('#poseY'); el.poseZ = $('#poseZ');
    el.poseRx = $('#poseRx'); el.poseRy = $('#poseRy'); el.poseRz = $('#poseRz');
    el.gripperInput = $('#gripperInput');
    el.gripperNum = $('#gripperNum');
    el.gripperSlider = $('#gripperSlider');
    el.gripperRange = $('#gripperRange');
    el.hintMeta = $('#hintMeta');
    el.btnAppStop = $('#btnAppStop');
    el.toastHost = $('#toastHost');
    el.jv = [0, 1, 2, 3, 4, 5].map(function (i) { return $('#jv' + i); });
    el.jt = [0, 1, 2, 3, 4, 5].map(function (i) { return $('#jt' + i); });

    bindRows();

    $$('.seg-btn').forEach(function (btn) {
      btn.addEventListener('click', function () { selectMode(btn.dataset.mode); });
    });

    $('#channel').addEventListener('change', function (ev) {
      app.channel = parseInt(ev.target.value, 10) || 1;
      paintStatus();
    });
    $('#port').addEventListener('change', function (ev) {
      var v = parseInt(ev.target.value, 10);
      app.port = isFinite(v) && v > 0 ? v : 47999;
      ev.target.value = String(app.port);
    });

    $('#logSwitch').addEventListener('click', function () {
      app.logEnabled = !app.logEnabled;
      var sw = $('#logSwitch');
      sw.classList.toggle('is-on', app.logEnabled);
      sw.setAttribute('aria-checked', app.logEnabled ? 'true' : 'false');
      if (app.logEnabled) {
        el.logList.scrollTop = el.logList.scrollHeight;
      }
      paintLogMeta();
    });

    $('#logClear').addEventListener('click', function () {
      el.logList.innerHTML = '';
      app.totalLines = 0;
      paintLogMeta();
    });
    el.logList.addEventListener('scroll', paintLogMeta);

    if (el.btnAppStop) el.btnAppStop.addEventListener('click', stopApp);
    $('#btnConnect').addEventListener('click', connect);
    $('#btnDisconnect').addEventListener('click', disconnect);
    $('#btnEstop').addEventListener('click', estop);
    $('#estopBtn').addEventListener('click', estop);
    $('#btnHome').addEventListener('click', home);
    $('#btnApply').addEventListener('click', applyJoints);

    el.gripperInput.addEventListener('input', function () {
      app.dirty.grip = true;
      el.gripperNum.value = fmt(parseFloat(el.gripperInput.value), 1);
      paintSlider(el.gripperSlider);
    });
    el.gripperInput.addEventListener('change', function () { applyGripper(); });

    el.gripperNum.addEventListener('input', function () { app.dirty.grip = true; });
    el.gripperNum.addEventListener('keydown', function (ev) {
      var range = app.gripper.range || [0, 110];
      var cur = parseFloat(el.gripperNum.value);
      if (!isFinite(cur)) cur = app.gripper.target !== null ? app.gripper.target : 0;
      if (ev.key === 'ArrowUp' || ev.key === 'ArrowDown') {
        ev.preventDefault();
        var v = clamp(cur + (ev.key === 'ArrowUp' ? 1 : -1) * (ev.shiftKey ? 10 : 1), range[0], range[1]);
        el.gripperNum.value = v.toFixed(1);
        el.gripperInput.value = v;
        paintSlider(el.gripperSlider);
      } else if (ev.key === 'PageUp' || ev.key === 'PageDown') {
        ev.preventDefault();
        var span = (range[1] - range[0]) / 10;
        var v2 = clamp(cur + (ev.key === 'PageUp' ? span : -span), range[0], range[1]);
        el.gripperNum.value = v2.toFixed(1);
        el.gripperInput.value = v2;
        paintSlider(el.gripperSlider);
      } else if (ev.key === 'Enter') {
        ev.preventDefault();
        applyGripper();
      }
    });
    el.gripperNum.addEventListener('blur', function () {
      var v = parseFloat(el.gripperNum.value);
      if (isFinite(v)) el.gripperInput.value = v;
      paintSlider(el.gripperSlider);
    });

    $('#btnGripper').addEventListener('click', function () {
      localLog('info', '负压吸盘启动 · 目标 ' + fmt(parseFloat(el.gripperInput.value), 1) + '°');
      applyGripper().then(function (res) {
        if (res && res.ok) toast('负压吸盘已启动', 'ok');
      });
    });

    window.addEventListener('resize', function () {
      rows.forEach(function (r) { paintSlider(r.slider); });
      paintSlider(el.gripperSlider);
    });
  }

  function loadLimits() {
    return getJSON('/api/v2/limits', 2500).then(function (data) {
      if (data && Array.isArray(data.joints)) {
        data.joints.forEach(function (j, i) {
          if (i < 6 && Array.isArray(j.range)) app.ranges[i] = j.range;
        });
      }
      if (data && data.gripper && Array.isArray(data.gripper.range)) {
        app.gripper.range = data.gripper.range;
      }
      paint();
    }).catch(function () { /* 用契约默认量程 */ });
  }

  /* ---------- 设计对照数据（仅 ?demo=1，不发请求） ---------- */
  var DEMO_STATE = {
    ok: true, mode: 'sim', connected: true, busy: false, estop: false,
    app: { running: true, pid: 41207, port: 47999 },
    engine: { name: 'MuJoCo', version: '3.17.0' },
    joints: {
      deg: [227.6, 135.2, 79.5, 210.0, 63.8, 118.3],
      target: [225.0, 134.0, 80.0, 208.0, 62.0, 118.0],
      range: JOINT_RANGE.map(function (r) { return r.slice(); }),
      limit: [false, false, true, false, false, false]
    },
    gripper: { deg: 48.0, target: 48.0, range: [0, 110] },
    tcp: { mm: [302.0, 118.0, 465.0], deg: [-178.2, 0.4, 90.0], offset_mm: 120 },
    ik: { status: 'ok', text: '可达', pos_err_mm: 0.4, ori_err_deg: 0.6 },
    fps: 29.8
  };

  var DEMO_LOG = [
    { t: 1732029972, level: 'info', text: '系统启动 · 配置已加载' },
    { t: 1732029974, level: 'info', text: 'CAN 初始化 · 通道 USB1 · 500 kbps' },
    { t: 1732029980, level: 'info', text: '桥已连接 127.0.0.1:47999' },
    { t: 1732030022, level: 'info', text: '载入 model/episode1.xml · 6 关节 + 夹爪' },
    { t: 1732030140, level: 'info', text: '仿真引擎就绪 · MuJoCo 3.17.0' },
    { t: 1732030298, level: 'error', text: '错误 CAN 初始化失败 0x1400 · 未找到适配器' },
    { t: 1732030300, level: 'info', text: '已切换至仿真模式 · 通道 USB1' },
    { t: 1732030324, level: 'info', text: '软限位已载入 · 速度上限 180°/s' },
    { t: 1732030350, level: 'info', text: '等待指令 · 空闲' }
  ];

  function runDemo() {
    applyState(DEMO_STATE);
    appendLog(DEMO_LOG);
    el.logList.scrollTop = 0;
    paintLogMeta();
  }

  function boot() {
    bindUI();
    paint();
    if (DEMO) { runDemo(); return; }

    getJSON('/api/v2/state', 2500).then(function (s) {
      if (s && s.ok !== false) {
        applyState(s);
        loadLimits();
        startStream();
      } else {
        setOffline('后端返回异常');
      }
    }).catch(function () {
      setOffline(location.protocol === 'file:' ? '本地文件打开 · 后端未启动' : '后端未启动');
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', boot);
  } else {
    boot();
  }
})();
