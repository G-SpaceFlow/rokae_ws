import { _ as _export_sfc, o as openBlock, c as createElementBlock, ag as createStaticVNode } from "./chunks/framework.47_HgVV0.js";
const __pageData = JSON.parse('{"title":"🔒 控制权与并发","description":"","frontmatter":{"title":"🔒 控制权与并发","outline":[2,4]},"headers":[],"relativePath":"guide/concurrency.md","filePath":"guide/concurrency.md","lastUpdated":null}');
const _sfc_main = { name: "guide/concurrency.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="🔒-控制权与并发" tabindex="-1">🔒 控制权与并发 <a class="header-anchor" href="#🔒-控制权与并发" aria-label="Permalink to &quot;🔒 控制权与并发&quot;">​</a></h1><p><a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">查看完整来源文档</a></p><p>统一驱动进程只创建两个 <code>ArRobot</code>：左、右臂各一个。每只手臂有两类内部锁：</p><div class="language-text vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">text</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>commandMutex：控制任务级独占</span></span>\n<span class="line"><span>sdkMutex：单次 SDK 函数调用串行化</span></span></code></pre></div><p><code>MoveAbsJ</code>、所有 <code>MoveL</code>、ServoJ、回原和初始化使用 <code>commandMutex</code>。同一只 手臂一次只允许一个控制任务，左右臂可以并行；双臂 ServoJ 和双臂回原同时 取得两侧锁。状态发布和灵巧手只短暂使用 <code>sdkMutex</code>。</p><p>内部控制锁目前没有发布为 ROS topic，SDK 的 <code>operationState()</code> 也没有独立状态 topic。因此：</p><ul><li><code>operationState=moving</code> 表示控制器在运动，不等价于 ROS 控制权归属。</li><li><code>commandMutex</code> 能阻止统一进程内的命令冲突，但无法阻止外部程序直接创建 <code>ArRobot</code> 并绕过驱动。</li><li>不应在统一驱动运行时并行启动 <code>cxl/servol.cpp</code> 等直接 SDK 控制程序。</li></ul><p>后续若增加 <code>control_state</code>，应同时表达 <code>occupied</code>、<code>owner</code>、SDK operation state、实际运动状态、控制模式和错误，而不是只发布一个 <code>moving</code> 布尔值。</p>', 8)
  ])]);
}
const concurrency = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  concurrency as default
};
