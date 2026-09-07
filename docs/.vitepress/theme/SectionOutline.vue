<script setup lang="ts">
import { nextTick, onMounted, onBeforeUnmount, ref, watch } from 'vue'
import { useRoute } from 'vitepress'

type Item = { id: string; title: string; depth: number }
const route = useRoute()
const items = ref<Item[]>([])
const title = ref('本页导航')
const active = ref('')
let scope: HTMLElement | null = null
let headings: HTMLElement[] = []

function label(element: HTMLElement) {
  return (element.textContent || '').replace(/[\u200b#]/g, '').trim()
}
function decoratedLabel(element: HTMLElement) {
  const text = label(element)
  if (/^[^\p{L}\p{N}]/u.test(text)) return text
  const icon = /Topics|消息|状态/.test(text) ? '📡' : /Services|接口/.test(text) ? '📦'
    : /Action|Goal|Feedback|Result/.test(text) ? '🎯' : /参数|约定|单位/.test(text) ? '⚙️'
    : /示例|启动|调用/.test(text) ? '▶️' : '📄'
  return `${icon} ${text}`
}
function level(element: HTMLElement) { return Number(element.tagName.slice(1)) }
function children(element: HTMLElement) {
  const start = headings.indexOf(element)
  let end = start + 1
  while (end < headings.length && level(headings[end]) > level(element)) end++
  return headings.slice(start + 1, end)
}
function targetHeading(hash: string) {
  let id: string
  try { id = decodeURIComponent(hash.replace(/^#/, '')) } catch { return null }
  if (!id) return null
  const target = document.getElementById(id)
  if (!target) return null
  if (headings.includes(target)) return target
  // Explicit stable anchors precede headings in the trajectory document.
  return headings.find(el => !!(target.compareDocumentPosition(el) & Node.DOCUMENT_POSITION_FOLLOWING)) || null
}
function render() {
  const list = scope ? children(scope) : headings.filter(el => level(el) <= 3)
  title.value = scope ? label(scope) : '本页导航'
  const base = scope ? level(scope) + 1 : 2
  items.value = list.map(el => ({ id: el.id, title: decoratedLabel(el), depth: level(el) - base }))
  active.value = targetHeading(location.hash)?.id || ''
}
function fromHash() {
  const target = targetHeading(location.hash)
  if (target && (!scope || (target !== scope && !children(scope).includes(target)))) {
    const index = headings.indexOf(target)
    scope = headings.slice(0, index + 1).reverse().find(el => level(el) === 2) || null
  }
  render()
}
async function refresh() {
  await nextTick()
  headings = Array.from(document.querySelectorAll<HTMLElement>('.vp-doc h2[id], .vp-doc h3[id], .vp-doc h4[id]'))
  scope = null
  fromHash()
}
function sidebarClick(event: MouseEvent) {
  const outlineLink = (event.target as Element)?.closest<HTMLAnchorElement>('.section-outline a')
  if (outlineLink) active.value = decodeURIComponent(outlineLink.hash.slice(1))
  const link = (event.target as Element)?.closest<HTMLAnchorElement>('.VPSidebar a')
  if (!link || event.ctrlKey || event.metaKey || event.shiftKey || event.altKey) return
  const url = new URL(link.href)
  if (url.pathname !== location.pathname) return
  scope = targetHeading(url.hash)
  render()
}
let stopWatch: (() => void) | undefined
onMounted(() => {
  refresh()
  stopWatch = watch(() => route.path, refresh)
  window.addEventListener('hashchange', fromHash)
  document.addEventListener('click', sidebarClick)
})
onBeforeUnmount(() => {
  stopWatch?.()
  window.removeEventListener('hashchange', fromHash)
  document.removeEventListener('click', sidebarClick)
})
</script>

<template>
  <nav class="section-outline" aria-label="当前功能子导航">
    <div class="section-outline-label">本页导航</div>
    <div v-if="title !== '本页导航'" class="section-outline-scope">{{ title }}</div>
    <ul v-if="items.length">
      <li v-for="item in items" :key="item.id" :style="{ paddingLeft: `${Math.max(0, item.depth) * 14}px` }">
        <a :href="`#${item.id}`" :class="{ active: active === item.id }" :aria-current="active === item.id ? 'location' : undefined">{{ item.title }}</a>
      </li>
    </ul>
    <p v-else class="section-outline-empty">此功能暂无子章节</p>
  </nav>
</template>
