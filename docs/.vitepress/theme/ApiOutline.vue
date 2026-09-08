<script setup lang="ts">
import { useData } from 'vitepress'
import { computed } from 'vue'
const { page } = useData()
const counts = computed(() => page.value.frontmatter.interfaceCounts || {})
// Fixed page-level navigation. Never derive its contents from the URL hash.
const groups = [
  { title: '📡 Topics', count: 'topics', items: [
    { title: '上肢状态', hash: '_2-1-上肢状态-topics-6' },
    { title: 'ServoJ 实时控制', hash: '_2-2-servoj-实时控制-topics-3' },
    { title: 'ServoL 实时控制', hash: '_2-3-servol-实时控制-topics-10' },
  ] },
  { title: '🎯 Actions', count: 'actions', items: [
    { title: '上肢运动', hash: '_2-4-上肢运动-actions-2' },
  ] },
  { title: '📦 Services', count: 'services', items: [
    { title: '底层控制', hash: '_2-5-底层控制-services-26' },
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
        <span class="api-outline-group">{{ group.title }}（{{ counts[group.count] ?? '—' }}）</span>
        <ul>
          <li v-for="item in group.items" :key="item.hash">
            <a :href="`#${item.hash}`">{{ item.title }}</a>
          </li>
        </ul>
      </li>
    </ul>
    <p class="api-count-note">点击表格中的 Type 查看字段与调用示例。</p>
  </nav>
</template>
