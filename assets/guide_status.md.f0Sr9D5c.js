import { _ as _export_sfc, o as openBlock, c as createElementBlock, a3 as createStaticVNode } from "./chunks/framework.Cj9_MUzw.js";
const __pageData = JSON.parse('{"title":"📋 实现进度与已知限制","description":"","frontmatter":{"title":"📋 实现进度与已知限制","outline":[2,4]},"headers":[],"relativePath":"guide/status.md","filePath":"guide/status.md","lastUpdated":1788858789000}');
const _sfc_main = { name: "guide/status.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="📋-实现进度与已知限制" tabindex="-1">📋 实现进度与已知限制 <a class="header-anchor" href="#📋-实现进度与已知限制" aria-label="Permalink to &quot;📋 实现进度与已知限制&quot;">​</a></h1><p><a href="/rokae_ws/JOINT_TRAJECTORY_UPDATE">查看完整来源文档</a></p><h2 id="📊-实现状态" tabindex="-1">📊 实现状态 <a class="header-anchor" href="#📊-实现状态" aria-label="Permalink to &quot;📊 实现状态&quot;">​</a></h2><table tabindex="0"><thead><tr><th>功能</th><th>本地状态</th></tr></thead><tbody><tr><td>Python 定时 ServoJ 轨迹</td><td>已实现，含启动、反馈、到位检查</td></tr><tr><td>单臂/双臂初始化上电服务</td><td>已实现并编译，未在本次修改中执行硬件测试</td></tr><tr><td><code>Joints.msg</code>、<code>JointTrajectory.srv</code></td><td>已定义并编译</td></tr><tr><td>新轨迹请求构造与参数校验</td><td>已实现，4 项离线测试通过</td></tr><tr><td>新 <code>JointTrajectory</code> 运动服务端</td><td><strong>尚未实现</strong></td></tr><tr><td>新接口逐点时间戳、同步/异步执行</td><td><strong>仅定义契约，尚未接入执行端</strong></td></tr></tbody></table><p>原 <code>MoveJByPath</code> 服务及 <code>movej_by_path_client.py</code> 执行路径保持兼容。 不要把新服务类型生成成功理解为运动服务已经上线。</p><h2 id="🛡️-已知限制" tabindex="-1">🛡️ 已知限制 <a class="header-anchor" href="#🛡️-已知限制" aria-label="Permalink to &quot;🛡️ 已知限制&quot;">​</a></h2><ul><li>轨迹退出时一次 powerState=on 不能保证之后持续上电。</li><li>已观察到退出检查通过、下一次运行却未上电的情况；根因尚未确认， 不能断言切换控制模式必然导致下电。</li><li>编译和离线测试不替代真实机器人安全验证。</li><li>曾出现多个进程处于内核 D 状态、等待 rtnl_lock/netlink_dump_start； 这不等于内存耗尽。旧进程未退出时不要重复启动驱动。</li></ul><p>原接口总览见 <a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">ROS 2 接口参考</a>。</p>', 8)
  ])]);
}
const status = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  status as default
};
