import { _ as _export_sfc, o as openBlock, c as createElementBlock, a3 as createStaticVNode } from "./chunks/framework.Cj9_MUzw.js";
const __pageData = JSON.parse('{"title":"🐍 Python 轨迹配置","description":"","frontmatter":{"title":"🐍 Python 轨迹配置","outline":[2,4]},"headers":[],"relativePath":"guide/trajectory-client.md","filePath":"guide/trajectory-client.md","lastUpdated":null}');
const _sfc_main = { name: "guide/trajectory-client.md" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, [..._cache[0] || (_cache[0] = [
    createStaticVNode('<h1 id="🐍-python-轨迹配置" tabindex="-1">🐍 Python 轨迹配置 <a class="header-anchor" href="#🐍-python-轨迹配置" aria-label="Permalink to &quot;🐍 Python 轨迹配置&quot;">​</a></h1><p><a href="/rokae_ws/JOINT_TRAJECTORY_UPDATE">查看完整来源文档</a></p><p>文件：<code>src/test/movej_by_path_client.py</code>。</p><table tabindex="0"><thead><tr><th>配置</th><th>作用</th></tr></thead><tbody><tr><td><code>ARM_SELECTION</code></td><td>1 左臂、2 右臂、3 双臂</td></tr><tr><td><code>EXECUTE</code></td><td>是否真实执行</td></tr><tr><td><code>POWER_ON_BEFORE_MOTION</code></td><td>是否在运动前显式请求所选臂上电</td></tr><tr><td><code>TOTAL_MOTION_TIME_S</code></td><td>正数使用定时 ServoJ；None 使用原非实时服务</td></tr><tr><td><code>CORNER_BLEND_RAD</code></td><td>拐角圆滑偏差，0 恢复逐点停顿</td></tr></tbody></table><p>上电开关开启时使用 power_on 服务；失败或响应超时不执行轨迹。 程序不在轨迹结束或异常后自动补上电。</p><p>运行前仍需加载 ROS 和已编译工作区环境：</p><div class="language-bash vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">source</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> /opt/ros/humble/setup.bash</span></span>\n<span class="line"><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">source</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> /home/niic/rokae_ws/install/local_setup.bash</span></span></code></pre></div>', 7)
  ])]);
}
const trajectoryClient = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  trajectoryClient as default
};
