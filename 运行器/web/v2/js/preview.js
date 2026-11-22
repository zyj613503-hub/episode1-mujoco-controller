/* preview.js — 上位机页面「本地三维预览」
 *
 * 契约：运行器/UI/上位机设计/接口契约-V2.1.md §4
 *   window.LocalArmPreview(container, { onReady, descUrl })
 *     setJointsDeg([6]) / setGripperDeg(0..110) / flangePose() / tcpPose(offsetMm)
 *     resetView() / setAxesVisible(on)
 *
 * 硬约束：本文件只做「本地几何 FK」——不联网、不建仿真、不吃任何画面帧。
 * 关节角 -> 位姿全部在浏览器里算：rad = (deg - OFFSET[i]) * DEG2RAD。
 */
(function (global) {
  'use strict';

  var DEG2RAD = Math.PI / 180;
  var RAD2DEG = 180 / Math.PI;
  var AXIS_LEN = 0.26;          // 世界坐标轴长度（米）
  var GRID_SIZE = 2;
  var GRID_DIV = 20;
  var COL_BG = 0x22262b;
  /* 默认取景：把 6 个 link + 夹爪整体框在画面里（参考 app 的 scene.js 视角，略拉远 + 略微抬高 target） */
  var VIEW_POS = [0.80, -0.80, 0.66];
  var VIEW_TARGET = [0.10, 0.0, 0.26];
  var COL_GRID = 0x557799;
  var COL_GRID2 = 0x3a4148;

  function isNum(v) { return typeof v === 'number' && isFinite(v); }
  function num(v, d) { var n = parseFloat(v); return isFinite(n) ? n : d; }

  /* 同步读取（预览默认走这条路：页面 load 事件前几何就全部就位，
     headless 截图/自测不会拍到「还没加载完」的画布）。
     sync XHR 不支持 responseType='arraybuffer'，用 x-user-defined 逐字节取。 */
  function xhrJsonSync(url) {
    var x = new XMLHttpRequest();
    x.open('GET', url, false);
    x.send(null);
    if (x.status !== 200) throw new Error('HTTP ' + x.status + ' ' + url);
    return JSON.parse(x.responseText);
  }

  function xhrBinarySync(url) {
    var x = new XMLHttpRequest();
    x.open('GET', url, false);
    x.overrideMimeType('text/plain; charset=x-user-defined');
    x.send(null);
    if (x.status !== 200) throw new Error('HTTP ' + x.status + ' ' + url);
    var s = x.responseText, n = s.length, buf = new Uint8Array(n);
    for (var i = 0; i < n; i++) buf[i] = s.charCodeAt(i) & 0xff;
    return buf.buffer;
  }

  /* ---------------------------------------------------------------- 数学 */

  // URDF/MuJoCo 的 rpy（roll-pitch-yaw）＝ R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
  // 注意：desc 里的 rpy 是 URDF 原值，单位【弧度】（不是度）；角度只在关节角上按 deg 传。
  function rpyToQuat(THREE, rpy) {
    var rx = num(rpy && rpy[0], 0);
    var ry = num(rpy && rpy[1], 0);
    var rz = num(rpy && rpy[2], 0);
    var m = new THREE.Matrix4().makeRotationZ(rz)
      .multiply(new THREE.Matrix4().makeRotationY(ry))
      .multiply(new THREE.Matrix4().makeRotationX(rx));
    return new THREE.Quaternion().setFromRotationMatrix(m);
  }

  // 与 robot/ik.py:matrix_to_euler_xyz_deg 完全一致（R = Rz(rz)@Ry(ry)@Rx(rx)）
  function matrixToEulerXYZDeg(THREE, m4) {
    var e = new THREE.Matrix4().extractRotation(m4).elements;
    // three 列主序：e[0]=R00 e[4]=R01 e[8]=R02 / e[1]=R10 e[9]=R21 e[10]=R22 ...
    var R20 = e[2], R21 = e[6], R22 = e[10], R10 = e[1], R00 = e[0];
    var sy = Math.max(-1, Math.min(1, -R20));
    var ry = Math.asin(sy);
    var rx, rz;
    if (Math.abs(sy) < 0.999999) {
      rx = Math.atan2(R21, R22);
      rz = Math.atan2(R10, R00);
    } else {
      rx = Math.atan2(-e[9], e[5]);
      rz = 0;
    }
    return [rx * RAD2DEG, ry * RAD2DEG, rz * RAD2DEG];
  }

  /* ---------------------------------------------------------------- 文字标签 */

  function textSprite(THREE, text, color, sizeAt1M) {
    var pad = 12, fs = 62;
    var cv = document.createElement('canvas');
    var ctx = cv.getContext('2d');
    ctx.font = '600 ' + fs + 'px "Helvetica Neue", Arial, sans-serif';
    var w = Math.ceil(ctx.measureText(text).width) + pad * 2;
    var h = fs + pad * 2;
    cv.width = w; cv.height = h;
    ctx = cv.getContext('2d');
    ctx.font = '600 ' + fs + 'px "Helvetica Neue", Arial, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = color;
    ctx.fillText(text, w / 2, h / 2 + 2);
    var tex = new THREE.CanvasTexture(cv);
    if (THREE.sRGBEncoding !== undefined) tex.encoding = THREE.sRGBEncoding;
    var mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false, depthWrite: false });
    var sp = new THREE.Sprite(mat);
    sp.renderOrder = 1000;
    sp.userData.sizeAt1M = sizeAt1M || 0.055;
    sp.userData.aspect = w / h;
    sp.scale.set(sp.userData.sizeAt1M * sp.userData.aspect, sp.userData.sizeAt1M, 1);
    return sp;
  }

  /* ---------------------------------------------------------------- 关节树 */

  // 按 desc（links/joints）建一棵 Three 的 Group 树；不关心几何以外的东西。
  function Tree(THREE, desc, opts) {
    opts = opts || {};
    this.THREE = THREE;
    this.desc = desc;
    this.nodes = {};     // link 名 -> Group
    this.joints = {};    // joint 名 -> { node, axis, type }
    this.meshes = [];
    this.pending = 0;
    this.errors = [];
    this.onDone = opts.onDone || null;
    this.getBuffer = opts.getBuffer || null;   // 给了就同步加载（截图/自测友好）
    this.root = new THREE.Group();
    this.root.name = opts.name || 'robot';
    this._materialCache = {};
    this._build();
  }

  Tree.prototype._material = function (color) {
    var THREE = this.THREE;
    var key = (color || [0.82, 0.82, 0.84]).join(',');
    if (!this._materialCache[key]) {
      var c = new THREE.Color();
      c.setRGB(color[0], color[1], color[2]);
      this._materialCache[key] = new THREE.MeshStandardMaterial({
        color: c, roughness: 0.55, metalness: 0.35
      });
    }
    return this._materialCache[key];
  };

  Tree.prototype._build = function () {
    var THREE = this.THREE, self = this, desc = this.desc;
    var links = desc.links || [], joints = desc.joints || [];
    var childOf = {};
    joints.forEach(function (j) { childOf[j.child] = true; });

    links.forEach(function (l) {
      var g = new THREE.Group();
      g.name = 'link:' + l.name;
      self.nodes[l.name] = g;
    });

    // 根 link 挂到 root
    links.forEach(function (l) {
      if (!childOf[l.name] && self.nodes[l.name]) self.root.add(self.nodes[l.name]);
    });

    joints.forEach(function (j) {
      var parent = self.nodes[j.parent], child = self.nodes[j.child];
      if (!parent || !child) {
        self.errors.push('关节 ' + j.name + ' 的 link 缺失：' + j.parent + ' -> ' + j.child);
        return;
      }
      var jg = new THREE.Group();
      jg.name = 'joint:' + j.name;
      jg.position.set(num(j.origin && j.origin.xyz[0], 0),
        num(j.origin && j.origin.xyz[1], 0),
        num(j.origin && j.origin.xyz[2], 0));
      jg.quaternion.copy(rpyToQuat(THREE, (j.origin && j.origin.rpy) || [0, 0, 0]));
      parent.add(jg);
      jg.add(child);
      var ax = j.axis || [0, 0, 1];
      var v = new THREE.Vector3(num(ax[0], 0), num(ax[1], 0), num(ax[2], 1));
      if (v.lengthSq() < 1e-12) v.set(0, 0, 1);
      self.joints[j.name] = { node: child, axis: v.normalize(), type: j.type || 'fixed' };
    });

    // 网格：给了 getBuffer 就同步解析，否则异步加载
    var loader = new THREE.STLLoader();
    links.forEach(function (l) {
      if (!l.mesh) return;
      if (self.getBuffer) {
        try {
          var geo = loader.parse(self.getBuffer(l.mesh));
          self._attachMesh(l, geo);
        } catch (e) {
          self.errors.push('网格加载失败：' + l.mesh + ' (' + (e && e.message || e) + ')');
        }
        return;
      }
      self.pending += 1;
      loader.load(l.mesh, function (geometry) {
        geometry.computeVertexNormals();
        var mesh = new THREE.Mesh(geometry, self._material(l.color));
        var vo = l.visual_origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
        var sc = l.scale || [1, 1, 1];
        mesh.scale.set(num(sc[0], 1), num(sc[1], 1), num(sc[2], 1));
        mesh.position.set(num(vo.xyz[0], 0), num(vo.xyz[1], 0), num(vo.xyz[2], 0));
        mesh.quaternion.copy(rpyToQuat(THREE, vo.rpy || [0, 0, 0]));
        mesh.castShadow = false;
        mesh.receiveShadow = false;
        mesh.name = 'mesh:' + l.name;
        self.nodes[l.name].add(mesh);
        self.meshes.push(mesh);
        self._done();
      }, undefined, function (err) {
        self.errors.push('网格加载失败：' + l.mesh + ' (' + (err && err.message || err) + ')');
        self._done();
      });
    });
    if (this.getBuffer && this.onDone) {
      var fn0 = this.onDone;
      this.onDone = null;
      setTimeout(function () { fn0(self.errors); }, 0);
    }
  };

  Tree.prototype._attachMesh = function (l, geometry) {
    var THREE = this.THREE;
    geometry.computeVertexNormals();
    var mesh = new THREE.Mesh(geometry, this._material(l.color));
    var vo = l.visual_origin || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
    var sc = l.scale || [1, 1, 1];
    mesh.scale.set(num(sc[0], 1), num(sc[1], 1), num(sc[2], 1));
    mesh.position.set(num(vo.xyz[0], 0), num(vo.xyz[1], 0), num(vo.xyz[2], 0));
    mesh.quaternion.copy(rpyToQuat(THREE, vo.rpy || [0, 0, 0]));
    mesh.name = 'mesh:' + l.name;
    this.nodes[l.name].add(mesh);
    this.meshes.push(mesh);
    return mesh;
  };

  Tree.prototype._done = function () {
    this.pending -= 1;
    if (this.pending <= 0 && this.onDone) {
      var fn = this.onDone;
      this.onDone = null;
      fn(this.errors);
    }
  };

  /* ---------------------------------------------------------------- 预览主体 */

  function LocalArmPreview(container, opts) {
    opts = opts || {};
    this.container = (typeof container === 'string') ? document.getElementById(container) : container;
    this.opts = opts;
    this.descUrl = opts.descUrl || 'assets/robot_desc.json';
    this.sync = opts.async !== true;
    this.offsets = null;
    this.ranges = null;
    this.jointsDeg = [0, 0, 0, 0, 0, 0];
    this.gripperDeg = 0;
    this.ready = false;
    this.disposed = false;
    this._meshDone = false;
    this._descDone = false;
    this._onReady = typeof opts.onReady === 'function' ? opts.onReady : null;
    this._readyFired = false;
    this._errors = [];
    this._labels = [];
    this._axesVisible = true;
    this._gridVisible = true;

    if (!this.container) throw new Error('LocalArmPreview: 容器不存在');
    if (!global.THREE) {
      this._error('three.js 未加载');
      this.container.innerHTML = '<div class="pv-missing">三维预览未加载（vendor/three.min.js 缺失）</div>';
      return;
    }
    this._initScene();
    this._loadDesc();
  }

  LocalArmPreview.prototype._error = function (msg) {
    this._errors.push(msg);
    if (global.console) console.warn('[preview] ' + msg);
  };

  /* --- 场景：照参考实现 scene.js --- */
  LocalArmPreview.prototype._initScene = function () {
    var THREE = global.THREE, self = this;
    this.three = THREE;
    var w = Math.max(1, this.container.clientWidth || this.container.offsetWidth || 672);
    var h = Math.max(1, this.container.clientHeight || this.container.offsetHeight || 456);

    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(global.devicePixelRatio || 1, 1.5));
    renderer.setSize(w, h, false);
    renderer.domElement.style.display = 'block';
    renderer.domElement.style.width = '100%';
    renderer.domElement.style.height = '100%';
    if ('outputColorSpace' in renderer && THREE.SRGBColorSpace) {
      renderer.outputColorSpace = THREE.SRGBColorSpace;
    } else if ('outputEncoding' in renderer && THREE.sRGBEncoding !== undefined) {
      renderer.outputEncoding = THREE.sRGBEncoding;
    }
    while (this.container.firstChild) this.container.removeChild(this.container.firstChild);
    this.container.appendChild(renderer.domElement);

    var scene = new THREE.Scene();
    scene.background = new THREE.Color(COL_BG);

    var camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 50);
    camera.position.set(VIEW_POS[0], VIEW_POS[1], VIEW_POS[2]);
    camera.up.set(0, 0, 1);
    camera.lookAt(VIEW_TARGET[0], VIEW_TARGET[1], VIEW_TARGET[2]);

    var controls = null;
    if (THREE.OrbitControls) {
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.target.set(VIEW_TARGET[0], VIEW_TARGET[1], VIEW_TARGET[2]);
      controls.enableDamping = false;
      controls.mouseButtons = {
        LEFT: THREE.MOUSE.ROTATE,
        MIDDLE: THREE.MOUSE.PAN,
        RIGHT: null
      };
      controls.saveState();
    }

    scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.9));
    var dir = new THREE.DirectionalLight(0xffffff, 0.7);
    dir.position.set(0.5, -0.8, 1.2);
    scene.add(dir);

    // 地面网格（Z-up：绕 X 转 90°）
    var grid = new THREE.GridHelper(GRID_SIZE, GRID_DIV, COL_GRID, COL_GRID2);
    grid.rotation.x = Math.PI / 2;
    grid.name = 'grid';
    scene.add(grid);
    this._grid = grid;

    // 世界坐标轴 + 文字标签
    var axes = new THREE.Group();
    axes.name = 'axes';
    var triad = [
      { dir: new THREE.Vector3(1, 0, 0), color: 0xc0442a, label: 'X' },
      { dir: new THREE.Vector3(0, 1, 0), color: 0x3f6b4a, label: 'Y' },
      { dir: new THREE.Vector3(0, 0, 1), color: 0x3a5f82, label: 'Z' }
    ];
    triad.forEach(function (t) {
      axes.add(self._axisArrow(t.dir, t.color, AXIS_LEN));
      var sp = textSprite(THREE, t.label, '#' + t.color.toString(16).padStart(6, '0'), 0.055);
      sp.position.copy(t.dir).multiplyScalar(AXIS_LEN * 1.18);
      axes.add(sp);
      self._labels.push(sp);
    });
    scene.add(axes);
    this._axes = axes;

    this.renderer = renderer;
    this.scene = scene;
    this.camera = camera;
    this.controls = controls;

    var ro = global.ResizeObserver ? new global.ResizeObserver(function () { self.resize(); }) : null;
    if (ro) ro.observe(this.container);
    this._onResize = function () { self.resize(); };
    global.addEventListener('resize', this._onResize);

    this._onDblClick = function () { self.resetView(); };
    this.container.addEventListener('dblclick', this._onDblClick);
    renderer.domElement.addEventListener('dblclick', this._onDblClick);

    this._tick = function () {
      if (self.disposed) return;
      self._raf = global.requestAnimationFrame(self._tick);
      self._updateLabels();
      if (self.controls) self.controls.update();
      self.renderer.render(self.scene, self.camera);
    };
    this._tick();
  };

  LocalArmPreview.prototype._axisArrow = function (dir, color, len) {
    var THREE = this.three;
    var k = len / AXIS_LEN;
    var shaftR = 0.006 * k, headLen = 0.05 * k, headR = 0.017 * k;
    var shaftLen = Math.max(0.001, len - headLen);
    var mat = new THREE.MeshBasicMaterial({ color: color });
    var g = new THREE.Group();
    var shaft = new THREE.Mesh(new THREE.CylinderGeometry(shaftR, shaftR, shaftLen, 12), mat);
    shaft.position.y = shaftLen / 2;
    var head = new THREE.Mesh(new THREE.ConeGeometry(headR, headLen, 16), mat);
    head.position.y = shaftLen + headLen / 2;
    g.add(shaft, head);
    var q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
    g.quaternion.copy(q);
    g.traverse(function (o) {
      if (o.isMesh) { o.renderOrder = 999; o.material.depthTest = false; o.material.depthWrite = false; }
    });
    return g;
  };

  LocalArmPreview.prototype._updateLabels = function () {
    if (!this._labels.length || !this.camera) return;
    var camPos = this.camera.position;
    for (var i = 0; i < this._labels.length; i++) {
      var sp = this._labels[i];
      var d = camPos.distanceTo(sp.position);
      var h = sp.userData.sizeAt1M * Math.max(0.15, d);
      sp.scale.set(h * sp.userData.aspect, h, 1);
    }
  };

  /* --- 模型描述 + 机器人 --- */
  LocalArmPreview.prototype._loadDesc = function () {
    var self = this;
    var url = this.descUrl;
    var done = function (desc) { self._buildRobot(desc); };
    var fail = function (err) {
      self._error('robot_desc.json 加载失败：' + url + ' (' + err + ')');
      self._descDone = true;
      self._maybeReady();
    };
    if (this.sync) {
      try {
        done(xhrJsonSync(url));
      } catch (e) {
        fail(e && e.message || e);
      }
      return;
    }
    if (global.fetch) {
      global.fetch(url, { cache: 'no-cache' }).then(function (r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      }).then(done).catch(fail);
    } else if (global.THREE && global.THREE.FileLoader) {
      new global.THREE.FileLoader().load(url, function (txt) {
        try { done(JSON.parse(txt)); } catch (e) { fail(e); }
      }, undefined, fail);
    } else {
      fail('浏览器不支持 fetch');
    }
  };

  LocalArmPreview.prototype._buildRobot = function (desc) {
    var THREE = this.three, self = this;
    this.desc = desc;
    this.offsets = (desc.joint_offsets_deg || [0, 0, 0, 0, 0, 0]).slice(0, 6).map(function (v) { return num(v, 0); });
    this.ranges = desc.joint_ranges_deg || null;
    this._descDone = true;

    var arm = new Tree(THREE, { links: desc.links, joints: desc.joints }, {
      name: 'episode1',
      getBuffer: self.sync ? xhrBinarySync : null,
      onDone: function () { self._meshDone = true; self._maybeReady(); }
    });
    this.arm = arm;
    this.robot = arm.root;
    var wo = desc.world_offset || { pos: [0, 0, 0], rpy: [0, 0, 0] };
    this.robot.position.set(num(wo.pos[0], 0), num(wo.pos[1], 0), num(wo.pos[2], 0));
    this.robot.quaternion.copy(rpyToQuat(THREE, wo.rpy || [0, 0, 0]));
    this.scene.add(this.robot);

    this.flangeLink = arm.nodes[desc.flange_link || 'link6'];
    this.flangeOffsetMm = num(desc.flange_offset_mm, 22);
    this.tcpSiteOffsetMm = num(desc.tcp_site_offset_mm, 68);
    this.tcpOffsetMm = num(desc.tcp_offset_mm, 120);

    // 夹爪：挂在法兰上
    if (desc.gripper && this.flangeLink) {
      this._buildGripper(desc.gripper);
    }
    if (!arm.pending) { this._meshDone = true; }
    this.setJointsDeg(this.jointsDeg);
    this.setGripperDeg(this.gripperDeg);
    this._maybeReady();
  };

  LocalArmPreview.prototype._buildGripper = function (g) {
    var THREE = this.three, self = this;
    var mount = g.mount || { pos: [0, 0, 0], rpy: [0, 0, 0] };
    var tree = new Tree(THREE, { links: g.links, joints: g.joints }, {
      name: 'gripper',
      getBuffer: self.sync ? xhrBinarySync : null,
      onDone: function () { self._meshDone = true; self._maybeReady(); }
    });
    tree.root.position.set(num(mount.pos[0], 0), num(mount.pos[1], 0), num(mount.pos[2], 0));
    tree.root.quaternion.copy(rpyToQuat(THREE, mount.rpy || [0, 0, 0]));
    this.flangeLink.add(tree.root);
    this.gripper = tree;
    this.gripperDesc = g;
    this.drive = (g.drive && g.drive.joint) ? g.drive.joint : 'joint_gear1';
    this.driveMax = num(g.drive && g.drive.ctrl_max, 1.70);
    this.coupling = g.coupling || [];
    this.gripperRange = g.range_deg || [0, 110];
  };

  LocalArmPreview.prototype._maybeReady = function () {
    if (this._readyFired) return;
    if (!this._descDone) return;
    if (this.arm && this.arm.pending > 0) return;
    if (this.gripper && this.gripper.pending > 0) return;
    this._readyFired = true;
    this.ready = true;
    var fn = this._onReady;
    if (fn) setTimeout(function () { fn(); }, 0);
  };

  /* --- 关节 --- */
  LocalArmPreview.prototype.setJointsDeg = function (degs) {
    var THREE = this.three;
    if (!THREE) return;
    var self = this;
    var list = [];
    if (Array.isArray(degs)) list = degs.slice(0, 6);
    else if (degs && typeof degs === 'object') {
      for (var i = 0; i < 6; i++) list.push(degs['joint' + (i + 1)]);
    }
    for (var i = 0; i < 6; i++) {
      if (isNum(list[i]) || (list[i] !== undefined && isNum(parseFloat(list[i])))) {
        this.jointsDeg[i] = num(list[i], this.jointsDeg[i]);
      }
    }
    if (!this.arm) return;
    for (var k = 0; k < 6; k++) {
      var jt = this.arm.joints['joint' + (k + 1)];
      if (!jt) continue;
      var rad = (this.jointsDeg[k] - (self.offsets ? self.offsets[k] : 0)) * DEG2RAD;
      jt.node.quaternion.setFromAxisAngle(jt.axis, rad);
    }
  };

  LocalArmPreview.prototype.getJointsDeg = function () { return this.jointsDeg.slice(); };

  /* --- 夹爪：伺服角 -> 闭环联动（照抄 MuJoCo equality） --- */
  LocalArmPreview.prototype.setGripperDeg = function (deg) {
    if (!this.three) return;
    var r0 = this.gripperRange ? this.gripperRange[0] : 0;
    var r1 = this.gripperRange ? this.gripperRange[1] : 110;
    var d = Math.max(r0, Math.min(r1, num(deg, 0)));
    this.gripperDeg = d;
    if (!this.gripper) return;
    var gear = this.driveMax * (d - r0) / Math.max(1e-6, (r1 - r0));
    var q = {};
    q[this.drive] = gear;
    var self = this;
    // 5 条 equality 是链式的（gear1 -> gear2 -> ...），迭代几轮把它们解到位
    for (var pass = 0; pass < 4; pass++) {
      this.coupling.forEach(function (c) {
        var x = isNum(q[c.driver]) ? q[c.driver] : 0;
        var p = c.polycoef || [0, 1, 0, 0, 0];
        var v = 0, acc = 1;
        for (var i = 0; i < p.length; i++) { v += num(p[i], 0) * acc; acc *= x; }
        q[c.joint] = v;
      });
    }
    Object.keys(this.gripper.joints).forEach(function (name) {
      var jt = self.gripper.joints[name];
      if (!jt || jt.type === 'fixed') return;
      var ang = isNum(q[name]) ? q[name] : 0;
      jt.node.quaternion.setFromAxisAngle(jt.axis, ang);
    });
  };

  /* --- 位姿 --- */
  LocalArmPreview.prototype._poseAt = function (offsetMm) {
    var THREE = this.three;
    if (!THREE || !this.flangeLink) return null;
    this.robot.updateMatrixWorld(true);
    var m = new THREE.Matrix4().multiplyMatrices(
      this.flangeLink.matrixWorld,
      new THREE.Matrix4().makeTranslation(0, 0, num(offsetMm, 0) / 1000)
    );
    var p = new THREE.Vector3().setFromMatrixPosition(m);
    var deg = matrixToEulerXYZDeg(THREE, m);
    return { mm: [p.x * 1000, p.y * 1000, p.z * 1000], m: [p.x, p.y, p.z], deg: deg };
  };

  LocalArmPreview.prototype.flangePose = function () {
    return this._poseAt(this.flangeOffsetMm || 22);
  };

  /* TCP 默认偏移 = 模型 site tcp 在 link6 系下的 68mm（site flange 是 22mm，
     所以 TCP 在法兰外 46mm），与 GET /api/v2/state 的 tcp.mm 同源一致。
     传 offsetMm 可显式指定（例如按 convention.GRIPPER_TCP_MM=120 评估真实夹爪尖端）。 */
  LocalArmPreview.prototype.tcpPose = function (offsetMm) {
    var off = isNum(offsetMm) ? num(offsetMm, 0) : this.tcpSiteOffsetMm;
    return this._poseAt(off);
  };
  LocalArmPreview.prototype.tcpDefaults = function () {
    return { site_mm: this.tcpSiteOffsetMm - this.flangeOffsetMm, gripper_mm: this.tcpOffsetMm };
  };

  /* --- 视图 --- */
  LocalArmPreview.prototype.resize = function () {
    if (!this.renderer) return;
    var w = Math.max(1, this.container.clientWidth || 1);
    var h = Math.max(1, this.container.clientHeight || 1);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  };

  LocalArmPreview.prototype.resetView = function () {
    var THREE = this.three;
    if (this.controls) {
      this.controls.reset();
      this.camera.up.set(0, 0, 1);
      this.controls.target.set(VIEW_TARGET[0], VIEW_TARGET[1], VIEW_TARGET[2]);
      this.controls.update();
    } else if (this.camera) {
      this.camera.position.set(VIEW_POS[0], VIEW_POS[1], VIEW_POS[2]);
      this.camera.up.set(0, 0, 1);
      this.camera.lookAt(VIEW_TARGET[0], VIEW_TARGET[1], VIEW_TARGET[2]);
    }
  };

  LocalArmPreview.prototype.setAxesVisible = function (on) {
    this._axesVisible = !!on;
    if (this._axes) this._axes.visible = !!on;
  };

  LocalArmPreview.prototype.setGridVisible = function (on) {
    this._gridVisible = !!on;
    if (this._grid) this._grid.visible = !!on;
  };

  LocalArmPreview.prototype.getErrors = function () { return this._errors.slice(); };

  LocalArmPreview.prototype.dispose = function () {
    this.disposed = true;
    if (this._raf) global.cancelAnimationFrame(this._raf);
    if (this._onResize) global.removeEventListener('resize', this._onResize);
    if (this.controls && this.controls.dispose) this.controls.dispose();
    if (this.renderer) this.renderer.dispose();
  };

  LocalArmPreview.VERSION = '1.0.0';
  global.LocalArmPreview = LocalArmPreview;
})(window);
