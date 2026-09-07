<script setup lang="ts">
import { useData } from 'vitepress'
const { page } = useData()
// Fixed page-level navigation. Never derive its contents from the URL hash.
const groups = [
  { title: '📦 Services', items: [
    { title: '🦾 上肢运动服务', hash: '_8-movel-services' },
    { title: '🔌 初始化与回原', hash: '_9-初始化与回原服务' },
    { title: '🖐️ 灵巧手服务', hash: '_10-linker-hand-服务' },
    { title: '📷 视觉目标服务', hash: '_14-上层视觉目标接口' },
  ] },
  { title: '📡 Topics', items: [
    { title: '🦾 上肢状态话题', hash: '_5-状态-topics' },
    { title: '⚡ 实时控制话题', hash: '_6-servoj-实时控制-topics' },
    { title: '🚙 底盘桥接话题', hash: '_15-可选底盘桥接接口' },
  ] },
  { title: '🎯 Actions', items: [
    { title: '🦾 上肢运动动作', hash: '_7-moveabsj-action' },
  ] },
]
</script>

<template>
  <nav v-if="page.relativePath === 'ROS2_INTERFACE_REFERENCE.md'"
       class="api-module-outline" aria-label="本页导航">
    <strong>本页导航</strong>
    <ul>
      <li><a href="#_2-api-快速查询">📑 API 目录</a></li>
      <li v-for="group in groups" :key="group.title">
        <span class="api-outline-group">{{ group.title }}</span>
        <ul>
          <li v-for="item in group.items" :key="item.hash">
            <a :href="`#${item.hash}`">{{ item.title }}</a>
          </li>
        </ul>
      </li>
    </ul>
  </nav>
</template>
