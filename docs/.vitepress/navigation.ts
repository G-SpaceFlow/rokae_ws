// Functional navigation, separate from interface content and runtime code.
const reference = '/ROS2_INTERFACE_REFERENCE'
const trajectory = '/JOINT_TRAJECTORY_UPDATE'

export default {
  aside: 'right' as const,
  outline: { level: [2, 3] as [number, number], label: '📑 本页导航' },
  nav: [
    { text: '📘 开发指南', link: reference },
    { text: '🦾 轨迹与上电', link: trajectory },
    { text: 'GitHub ↗', link: 'https://github.com/G-SpaceFlow/rokae_ws' },
  ],
  sidebar: [
    {
      text: '📘 开发指南',
      items: [
        { text: '概述', link: '/' },
        { text: '完整 API 文档', link: reference },
        { text: '启动与发现', link: `${reference}#_4-启动与发现` },
      ],
    },
    {
      text: '🦾 上肢控制', collapsed: false,
      items: [
        { text: '轨迹与上电 · 总览', link: trajectory },
        { text: '关节轨迹 · 接口契约', link: `${trajectory}#joint-trajectory` },
        { text: '左臂 / 右臂 / 双臂', link: `${trajectory}#arm-modes` },
        { text: '初始化上电', link: `${trajectory}#initialize` },
        { text: '按需上电', link: `${trajectory}#power-on` },
        { text: 'Python 配置', link: `${trajectory}#python-client` },
      ],
    },
    {
      text: '📡 ROS 接口', collapsed: false,
      items: [
        { text: '📊 状态 Topics', link: `${reference}#_5-状态-topics` },
        { text: '⚡ ServoJ 实时控制', link: `${reference}#_6-servoj-实时控制-topics` },
        { text: '🎯 MoveAbsJ 动作', link: `${reference}#_7-moveabsj-action` },
        { text: '📦 MoveL 服务', link: `${reference}#_8-movel-services` },
        { text: '🏠 初始化与回原', link: `${reference}#_9-初始化与回原服务` },
        { text: '🖐️ 灵巧手', link: `${reference}#_10-linker-hand-服务` },
      ],
    },
    {
      text: '🧩 应用与集成', collapsed: true,
      items: [
        { text: '📷 视觉目标', link: `${reference}#_14-上层视觉目标接口` },
        { text: '🚙 底盘桥接', link: `${reference}#_15-可选底盘桥接接口` },
        { text: '🗂️ 实现位置', link: `${reference}#_17-实现位置` },
      ],
    },
    {
      text: '🛡️ 配置与安全', collapsed: true,
      items: [
        { text: '参数参考', link: `${reference}#_11-参数参考` },
        { text: '控制权与并发', link: `${reference}#_12-控制权、并发和状态语义` },
        { text: '实现状态', link: `${trajectory}#implementation-status` },
        { text: '已知限制', link: `${trajectory}#known-limits` },
      ],
    },
  ],
}
