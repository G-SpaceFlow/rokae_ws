<script setup lang="ts">
import { useData } from 'vitepress'
import { computed } from 'vue'
const { page } = useData()
const counts = computed(() => page.value.frontmatter.interfaceCounts || {})
// Fixed page-level navigation. Never derive its contents from the URL hash.
const groups = [
  { title: '📡 Topics', count: 'topics', open: true, items: [
    { title: '上肢状态反馈', count: 'stateTopics', hash: '_2-1-上肢状态-topics-8' },
    { title: '上肢 ServoJ', count: 'servoJTopics', hash: '_2-2-servoj-实时控制-topics-3' },
    { title: '上肢 ServoL', count: 'servoLTopics', hash: '_2-3-servol-实时控制-topics-10' },
    { title: '摄像头与视觉', count: 'visionTopics', hash: 'topic-vision' },
    { title: '移动底盘', count: 'chassisTopics', hash: 'topic-chassis' },
  ] },
  { title: '📦 Services', count: 'services', open: true, items: [
    { title: '上肢运动与状态', count: 'motionServices', hash: 'service-motion' },
    { title: '笛卡尔拖动示教', count: 'teachServices', hash: 'service-teach' },
    { title: '初始化与上电', count: 'powerServices', hash: 'service-power' },
    { title: '回原', count: 'homeServices', hash: 'service-home' },
    { title: '灵巧手', count: 'handServices', hash: 'service-hand' },
    { title: 'FK 正运动学', count: 'fkServices', hash: 'service-fk' },
    { title: 'IK 逆运动学', count: 'ikServices', hash: 'service-ik' },
    { title: '摄像头与视觉', count: 'visionServices', hash: 'service-vision' },
    { title: '移动底盘', count: 'chassisServices', hash: 'service-chassis' },
  ] },
  { title: '🎯 Actions', count: 'actions', open: false, items: [
    { title: '上肢 MoveAbsJ', count: 'moveActions', hash: '_2-4-上肢运动-actions-2' },
    { title: '底盘导航', count: 'chassisActions', hash: 'action-chassis' },
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
        <details class="api-outline-section" :open="group.open">
          <summary>{{ group.title }}<span>{{ counts[group.count] ?? '—' }}</span></summary>
          <ul>
            <li v-for="item in group.items" :key="item.hash">
              <a :href="`#${item.hash}`">
                <span>{{ item.title }}</span>
                <small>{{ counts[item.count] ?? '—' }}</small>
              </a>
            </li>
          </ul>
        </details>
      </li>
    </ul>
    <p class="api-count-note">数量是文档收录的接口数。Type 可查看字段与调用示例。</p>
  </nav>
</template>
