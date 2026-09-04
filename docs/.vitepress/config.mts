export default {
  lang: 'zh-CN',
  title: 'Rokae ROS 2 开发指南',
  description: 'Rokae 双臂 ROS 2 Topic、Service 与 Action 接口文档',
  base: '/rokae_ws/',
  cleanUrls: true,
  lastUpdated: true,

  themeConfig: {
    nav: [
      { text: '接口文档', link: '/ROS2_INTERFACE_REFERENCE' },
      { text: 'GitHub', link: 'https://github.com/G-SpaceFlow/rokae_ws' },
    ],

    sidebar: [
      {
        text: '开始',
        items: [
          { text: '文档首页', link: '/' },
          {
            text: '接口范围',
            link: '/ROS2_INTERFACE_REFERENCE#_1-文档范围',
          },
          {
            text: '启动与发现',
            link: '/ROS2_INTERFACE_REFERENCE#_4-启动与发现',
          },
        ],
      },
      {
        text: '上肢 API',
        collapsed: false,
        items: [
          {
            text: 'API 快速查询',
            link: '/ROS2_INTERFACE_REFERENCE#_2-api-快速查询',
          },
          {
            text: 'Topics（6）',
            link: '/ROS2_INTERFACE_REFERENCE#_2-1-上肢状态-topics-6',
          },
          {
            text: 'Actions（2）',
            link: '/ROS2_INTERFACE_REFERENCE#_2-2-上肢运动-actions-2',
          },
          {
            text: 'Services（11）',
            link: '/ROS2_INTERFACE_REFERENCE#_2-3-底层控制-services-11',
          },
          {
            text: '状态数据详解',
            link: '/ROS2_INTERFACE_REFERENCE#_5-状态-topics',
          },
          {
            text: 'MoveAbsJ',
            link: '/ROS2_INTERFACE_REFERENCE#_6-moveabsj-action',
          },
          {
            text: 'MoveL',
            link: '/ROS2_INTERFACE_REFERENCE#_7-movel-services',
          },
          {
            text: '初始化',
            link: '/ROS2_INTERFACE_REFERENCE#_8-初始化服务',
          },
          {
            text: '灵巧手',
            link: '/ROS2_INTERFACE_REFERENCE#_9-linker-hand-服务',
          },
        ],
      },
      {
        text: '配置与安全',
        collapsed: false,
        items: [
          {
            text: '单位与坐标系',
            link: '/ROS2_INTERFACE_REFERENCE#_3-约定',
          },
          {
            text: '参数参考',
            link: '/ROS2_INTERFACE_REFERENCE#_10-参数参考',
          },
          {
            text: '控制权与并发',
            link: '/ROS2_INTERFACE_REFERENCE#_11-控制权、并发和状态语义',
          },
          {
            text: '未实现接口',
            link: '/ROS2_INTERFACE_REFERENCE#_12-未实现接口',
          },
          {
            text: '安全要求',
            link: '/ROS2_INTERFACE_REFERENCE#_15-安全使用要求',
          },
        ],
      },
      {
        text: '应用接口',
        collapsed: true,
        items: [
          {
            text: '视觉目标',
            link: '/ROS2_INTERFACE_REFERENCE#_13-上层视觉目标接口',
          },
          {
            text: '底盘桥接',
            link: '/ROS2_INTERFACE_REFERENCE#_14-可选底盘桥接接口',
          },
          {
            text: '实现位置',
            link: '/ROS2_INTERFACE_REFERENCE#_16-实现位置',
          },
        ],
      },
    ],

    outline: {
      level: [2, 4],
      label: '本页导航',
    },
    search: {
      provider: 'local',
    },
    docFooter: {
      prev: '上一节',
      next: '下一节',
    },
    lastUpdated: {
      text: '最后更新',
      formatOptions: {
        dateStyle: 'medium',
        timeStyle: 'short',
      },
    },
    socialLinks: [
      { icon: 'github', link: 'https://github.com/G-SpaceFlow/rokae_ws' },
    ],
  },
}
