import{_ as s,o as n,c as e,a3 as p}from"./chunks/framework.BmGVitc1.js";const v=JSON.parse('{"title":"📘 概述","description":"","frontmatter":{"title":"📘 概述","outline":[2,4]},"headers":[],"relativePath":"guide/overview.md","filePath":"guide/overview.md","lastUpdated":1788858789000}'),i={name:"guide/overview.md"};function t(l,a,o,r,c,d){return n(),e("div",null,[...a[0]||(a[0]=[p(`<h1 id="📘-概述" tabindex="-1">📘 概述 <a class="header-anchor" href="#📘-概述" aria-label="Permalink to &quot;📘 概述&quot;">​</a></h1><p><a href="/rokae_ws/ROS2_INTERFACE_REFERENCE">查看完整来源文档</a></p><p>本文档描述 <code>rokae_ws</code> 当前对外提供的 ROS 2 接口，重点是 <code>ros_dual_arm_driver</code> 中的机械臂、状态、初始化和灵巧手接口，并附带默认 bringup 中启动的视觉目标服务和可选底盘桥接接口。</p><p>接口分层如下：</p><div class="language-text vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">text</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span>应用与行为树</span></span>
<span class="line"><span>  ├── 视觉目标服务</span></span>
<span class="line"><span>  ├── 底盘桥接</span></span>
<span class="line"><span>  └── 机械臂客户端</span></span>
<span class="line"><span>          ↓ ROS 2</span></span>
<span class="line"><span>ros_dual_arm_driver</span></span>
<span class="line"><span>  ├── FK / IK Services</span></span>
<span class="line"><span>  ├── MoveAbsJ Action</span></span>
<span class="line"><span>  ├── MoveL Services</span></span>
<span class="line"><span>  ├── ServoJ Realtime Topics</span></span>
<span class="line"><span>  ├── ServoL Realtime Topics</span></span>
<span class="line"><span>  ├── State Topics</span></span>
<span class="line"><span>  ├── Initialization Service</span></span>
<span class="line"><span>  └── Linker Hand Services</span></span>
<span class="line"><span>          ↓ xCoreSDK</span></span>
<span class="line"><span>左臂 ArRobot                 右臂 ArRobot</span></span></code></pre></div><p>当前驱动同时提供非实时 Move、FK/IK 运动学计算、ServoJ 实时关节位置流和 ServoL 实时笛卡尔位姿流接口。阻抗和力矩控制尚未作为 ROS 接口发布，参见 <a href="/rokae_ws/ROS2_INTERFACE_REFERENCE#_13-未实现接口">未实现接口</a>。</p>`,6)])])}const S=s(i,[["render",t]]);export{v as __pageData,S as default};
