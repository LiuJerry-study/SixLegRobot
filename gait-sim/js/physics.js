/* ============================================================
 * physics.js — Rapier 3D 物理引擎封装
 *
 * 功能:
 *   - 异步加载 Rapier WASM (compat 版本, 内嵌 base64)
 *   - 创建物理世界 (重力, 地面, 机器人刚体)
 *   - 提供力施加 / 状态查询接口
 *   - 替代 main.js 中的手写 Euler 积分 + 库仑摩擦
 *
 * 单位: 厘米 / 秒 / 千克 (与项目一致)
 * 全局挂载: window.Physics
 * ============================================================ */
(function (g) {
  'use strict';

  /* ---- 物理常量 ---- */
  const GRAVITY   = { x: 0, y: -980, z: 0 };   // cm/s²
  const BODY_MASS = 2.0;                          // kg
  const BODY_RADIUS = 6.8;                        // cm (六棱柱外接圆)
  const BODY_HALF_H = 1.0;                        // cm (半厚度)
  const GROUND_FRIC = 0.8;                        // 地面摩擦
  const LINEAR_DAMP = 0.3;                        // 线性阻尼
  const ANGULAR_DAMP = 2.5;                       // 角阻尼

  /* ---- 内部状态 ---- */
  let RAPIER     = null;
  let world      = null;
  let robotBody  = null;
  let ready      = false;

  /* ---- 初始化 ---- */
  async function init() {
    if (ready) return true;
    try {
      // compat 版本内嵌 WASM (base64), 无需单独加载 .wasm
      var mod = await import(
        'https://cdn.jsdelivr.net/npm/@dimforge/rapier3d-compat@0.11.2/rapier.es.js'
      );
      RAPIER = mod.default || mod;
      await RAPIER.init();

      // 物理世界
      world = new RAPIER.World(GRAVITY);
      world.timestep = 1 / 120;   // 固定 120 Hz 子步

      // ---- 地面 (静态半空间, 法线 +Y) ----
      var gBody = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
      var gColl = RAPIER.ColliderDesc.halfspace({ x: 0, y: 1, z: 0 });
      gColl.setFriction(GROUND_FRIC);
      world.createCollider(gColl, gBody);

      // ---- 机器人刚体 (动态) ----
      var rDesc = RAPIER.RigidBodyDesc.dynamic();
      rDesc.setAdditionalMass(BODY_MASS);
      rDesc.setLinearDamping(LINEAR_DAMP);
      rDesc.setAngularDamping(ANGULAR_DAMP);
      rDesc.setCanSleep(false);
      robotBody = world.createRigidBody(rDesc);

      // 碰撞体: 圆柱 (半高, 半径) — 简化六棱柱
      var rColl = RAPIER.ColliderDesc.cylinder(BODY_HALF_H, BODY_RADIUS);
      rColl.setFriction(GROUND_FRIC);
      rColl.setRestitution(0.05);
      world.createCollider(rColl, robotBody);

      ready = true;
      return true;
    } catch (e) {
      console.error('[Physics] Rapier 初始化失败:', e);
      return false;
    }
  }

  /* ---- 世界步进 ---- */
  function step() {
    if (!world) return;
    world.step();
  }

  /* ---- 机器人位姿 ---- */
  function setPosition(x, y, z) {
    if (!robotBody) return;
    robotBody.setTranslation({ x: x, y: y, z: z }, true);
    robotBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    robotBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
  }

  function getPosition() {
    if (!robotBody) return { x: 0, y: 7.8, z: 0 };
    var p = robotBody.translation();
    return { x: p.x, y: p.y, z: p.z };
  }

  function getRotation() {
    if (!robotBody) return { x: 0, y: 0, z: 0, w: 1 };
    var r = robotBody.rotation();
    return { x: r.x, y: r.y, z: r.z, w: r.w };
  }

  function getLinvel() {
    if (!robotBody) return { x: 0, y: 0, z: 0 };
    var v = robotBody.linvel();
    return { x: v.x, y: v.y, z: v.z };
  }

  function getAngvel() {
    if (!robotBody) return { x: 0, y: 0, z: 0 };
    var v = robotBody.angvel();
    return { x: v.x, y: v.y, z: v.z };
  }

  /* ---- 力 ---- */
  function addForce(fx, fy, fz) {
    if (!robotBody) return;
    robotBody.addForce({ x: fx, y: fy, z: fz }, true);
  }

  function addForceAtPoint(fx, fy, fz, px, py, pz) {
    if (!robotBody) return;
    robotBody.addForceAtPoint(
      { x: fx, y: fy, z: fz },
      { x: px, y: py, z: pz },
      true
    );
  }

  /* ---- 暴露 API ---- */
  g.Physics = {
    init: init,
    step: step,
    setPosition: setPosition,
    getPosition: getPosition,
    getRotation: getRotation,
    getLinvel: getLinvel,
    getAngvel: getAngvel,
    addForce: addForce,
    addForceAtPoint: addForceAtPoint,
    ready: function () { return ready; }
  };
})(typeof window !== 'undefined' ? window : globalThis);
