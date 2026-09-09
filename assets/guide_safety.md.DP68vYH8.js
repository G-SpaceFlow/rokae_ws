import { _ as _export_sfc, o as openBlock, c as createElementBlock, a3 as createStaticVNode } from "./chunks/framework.Cj9_MUzw.js";
const __pageData = JSON.parse('{"title":"🛡️ 安全使用要求","description":"","frontmatter":{"title":"🛡️ 安全使用要求","outline":[2,4]},"headers":[],"relativePath":"guide/safety.md","filePath":"guide/safety.md","lastUpdated":1788858789000}');
const _sfc_main = { name: "guide/safety.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="🛡️-安全使用要求" tabindex="-1">🛡️ 安全使用要求 <a class="header-anchor" href="#🛡️-安全使用要求" aria-label="Permalink to &quot;🛡️ 安全使用要求&quot;">​</a></h1><p><a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">查看完整来源文档</a></p><ol><li>确认本机网卡已经配置 <code>192.168.4.10</code> 和 <code>192.168.2.10</code>，并确认左右臂 IP 没有互换。</li><li>第一次调用只使用单臂、小位移、低速度、<code>zone_mm=0</code>。</li><li>清空工作空间，操作员保持急停可触及。</li><li>MoveAbsJ 前确认关节顺序、弧度单位和控制器软限位。</li><li>MoveL 前确认外部参考坐标系、TCP、工具负载、臂角和姿态约定。</li><li>拖动示教会切换为手动模式并下电；启动前确认工具负载，优先使用默认的按键保护。</li><li>ServoJ 必须从当前关节反馈开始连续发送，不能直接发布远离当前位置的目标。</li><li>ServoL 输入是 TCP 相对外部参考系位姿；驱动转换到法兰相对基坐标系后发送。</li><li>不要同时运行绕过 ROS 驱动的 SDK 控制程序。</li><li>YAML 中的限制值只是软件请求边界，不能替代控制器安全配置、碰撞检测或 风险评估。</li></ol>', 3)
  ])]);
}
const safety = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  safety as default
};
