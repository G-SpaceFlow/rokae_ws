import { _ as _export_sfc, o as openBlock, c as createElementBlock, a3 as createStaticVNode } from "./chunks/framework.Cj9_MUzw.js";
const __pageData = JSON.parse('{"title":"🚙 底盘桥接","description":"","frontmatter":{"title":"🚙 底盘桥接","outline":[2,4]},"headers":[],"relativePath":"api/chassis.md","filePath":"api/chassis.md","lastUpdated":null}');
const _sfc_main = { name: "api/chassis.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="🚙-底盘桥接" tabindex="-1">🚙 底盘桥接 <a class="header-anchor" href="#🚙-底盘桥接" aria-label="Permalink to &quot;🚙 底盘桥接&quot;">​</a></h1><p><a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">查看完整来源文档</a></p><p><code>start_chassis_navigation=true</code> 时启动：</p><table tabindex="0"><thead><tr><th>名称</th><th>类型</th><th>方向/作用</th></tr></thead><tbody><tr><td><code>/scheduler/cmd/chassis</code></td><td><code>std_msgs/msg/String</code></td><td>订阅 <code>LM1</code>、<code>LM2</code>、<code>LM3</code></td></tr><tr><td><code>/chassis/state</code></td><td><code>std_msgs/msg/String</code></td><td>发布到站或失败状态</td></tr><tr><td><code>/bt_navigation_server/cancel_navigation</code></td><td><code>std_srvs/srv/Trigger</code></td><td>取消当前导航</td></tr><tr><td><code>/seer/navigate</code></td><td><code>seer_interfaces/action/Navigate</code></td><td>桥接器调用的 Seer Action</td></tr></tbody></table><p>状态映射：</p><table tabindex="0"><thead><tr><th>命令</th><th>成功状态</th></tr></thead><tbody><tr><td><code>LM1</code></td><td><code>ARRIVE_HOME</code></td></tr><tr><td><code>LM2</code></td><td><code>ARRIVE_A</code></td></tr><tr><td><code>LM3</code></td><td><code>ARRIVE_B</code></td></tr></tbody></table><p>失败状态为 <code>NAVIGATION_FAILED</code>，取消状态为 <code>NAVIGATION_CANCELED</code>。</p>', 7)
  ])]);
}
const chassis = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  chassis as default
};
