export default {
  aside: 'right' as const,
  outline: { level: [2, 2] as [number, number], label: '本页导航' },
  nav: [
    { text: '📘 开发指南', link: '/guide/overview' },
    { text: '📦 完整 API', link: '/ROS2_INTERFACE_REFERENCE' },
    { text: 'GitHub ↗', link: 'https://github.com/G-SpaceFlow/rokae_ws' },
  ],
  sidebar: [
    { text: '📘 开发指南', items: [
      { text: '概述', link: '/guide/overview' },
      { text: '快速开始', link: '/guide/getting-started' },
      { text: '单位与坐标系', link: '/guide/conventions' },
    ] },
    { text: '📡 ROS API', collapsed: false, items: [
      { text: '完整 API 文档', link: '/ROS2_INTERFACE_REFERENCE' },
    ] },
    { text: '🧩 接口契约与示例', collapsed: false, items: [
      { text: '数据类型与接口契约', link: '/api/joint-trajectory' },
      { text: 'Python 开发示例', link: '/guide/trajectory-client' },
      { text: '实现进度与已知限制', link: '/guide/status' },
    ] },
    { text: '🛡️ 配置与安全', collapsed: true, items: [
      { text: '参数参考', link: '/guide/parameters' },
      { text: '控制权与并发', link: '/guide/concurrency' },
      { text: '安全使用要求', link: '/guide/safety' },
    ] },
  ],
}
