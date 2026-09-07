import { _ as _export_sfc, o as openBlock, c as createElementBlock, ag as createStaticVNode } from "./chunks/framework.47_HgVV0.js";
const __pageData = JSON.parse('{"title":"📘 概述","description":"","frontmatter":{"title":"📘 概述","outline":[2,4]},"headers":[],"relativePath":"guide/overview.md","filePath":"guide/overview.md","lastUpdated":null}');
const _sfc_main = { name: "guide/overview.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="📘-概述" tabindex="-1">📘 概述 <a class="header-anchor" href="#📘-概述" aria-label="Permalink to &quot;📘 概述&quot;">​</a></h1><p><a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">查看完整来源文档</a></p><p>本文档描述 <code>rokae_ws</code> 当前对外提供的 ROS 2 接口，重点是 <code>ros_dual_arm_driver</code> 中的机械臂、状态、初始化和灵巧手接口，并附带默认 bringup 中启动的视觉目标服务和可选底盘桥接接口。</p><p>接口分层如下：</p><div class="language-text vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">text</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>应用与行为树</span></span>\n<span class="line"><span>  ├── 视觉目标服务</span></span>\n<span class="line"><span>  ├── 底盘桥接</span></span>\n<span class="line"><span>  └── 机械臂客户端</span></span>\n<span class="line"><span>          ↓ ROS 2</span></span>\n<span class="line"><span>ros_dual_arm_driver</span></span>\n<span class="line"><span>  ├── MoveAbsJ Action</span></span>\n<span class="line"><span>  ├── MoveL Services</span></span>\n<span class="line"><span>  ├── ServoJ Realtime Topics</span></span>\n<span class="line"><span>  ├── State Topics</span></span>\n<span class="line"><span>  ├── Initialization Service</span></span>\n<span class="line"><span>  └── Linker Hand Services</span></span>\n<span class="line"><span>          ↓ xCoreSDK</span></span>\n<span class="line"><span>左臂 ArRobot                 右臂 ArRobot</span></span></code></pre></div><p>当前驱动同时提供非实时 Move 接口和 ServoJ 实时关节位置流接口。<code>ServoL</code>、 阻抗和力矩控制尚未作为 ROS 接口发布，参见<a href="/rokae_ws/ROS2_INTERFACE_REFERENCE#_13-未实现接口">未实现接口</a>。</p>', 6)
  ])]);
}
const overview = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  overview as default
};
