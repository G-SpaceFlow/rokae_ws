// Mechanical extraction. Full reference documents remain the source of truth.
// Run: node docs/.vitepress/split-pages.mjs [docs-directory]
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'
const root = resolve(process.argv[2] || fileURLToPath(new URL('..', import.meta.url)))
const reference = readFileSync(resolve(root, '.vitepress/ROS2_INTERFACE_DETAILS.md'), 'utf8')
const update = readFileSync(resolve(root, 'JOINT_TRAJECTORY_UPDATE.md'), 'utf8')
const sections = [...reference.matchAll(/^## (\d+)\. .*$/gm)]
function section(number) {
  const index = sections.findIndex(match => Number(match[1]) === number)
  if (index < 0) throw new Error(`Missing section ${number}`)
  return reference.slice(sections[index].index, sections[index + 1]?.index ?? reference.length).trim()
}
function between(start, end) {
  const first = update.indexOf(start)
  const last = end ? update.indexOf(end, first + start.length) : update.length
  if (first < 0 || last < 0) throw new Error(`Missing section ${start}`)
  return update.slice(first + start.length, last).trim()
}
function page(path, title, body, source, promote = true) {
  if (promote) body = body.replace(/^## [^\n]+\n/, '')
  let fenced = false
  body = body.split('\n').map(line => {
    if (/^\s*(```|~~~)/.test(line)) fenced = !fenced
    if (!fenced) {
      if (promote) line = line.replace(/^(#{3,6}) /, (_, hashes) => hashes.slice(1) + ' ')
      line = line.replace(/\]\(#([^)]*)\)/g, `](${source}#$1)`)
      line = line.replace(/\]\(\.\/ROS2_INTERFACE_REFERENCE\.md\)/g, '](/ROS2_INTERFACE_REFERENCE)')
    }
    return line
  }).join('\n')
  const output = resolve(root, path + '.md')
  mkdirSync(dirname(output), { recursive: true })
  writeFileSync(output, `---\ntitle: ${title}\noutline: [2, 4]\n---\n\n# ${title}\n\n[查看完整来源文档](${source})\n\n${body.trim()}\n`)
}
const pages = [
  ['guide/overview', '📘 概述', 1],
  ['guide/getting-started', '🚀 快速开始', 4],
  ['guide/conventions', '📐 单位与坐标系', 3],
  ['api/state', '📊 状态反馈', 5],
  ['api/servoj', '⚡ ServoJ 与 ServoL 实时控制', 6],
  ['api/moveabsj', '🎯 MoveAbsJ 与非实时路径', 7],
  ['api/movel', '📦 MoveL 服务', 8],
  ['api/home', '🏠 回原与兼容初始化', 9],
  ['api/hand', '🖐️ 灵巧手', 10],
  ['guide/parameters', '⚙️ 参数参考', 11],
  ['guide/concurrency', '🔒 控制权与并发', 12],
  ['api/vision', '📷 视觉目标', 14],
  ['api/chassis', '🚙 底盘桥接', 15],
  ['guide/safety', '🛡️ 安全使用要求', 16],
  ['guide/implementation', '🗂️ 实现位置', 17],
  ['api/kinematics', '🧮 FK / IK 运动学计算', 18],
]
for (const [path, title, number] of pages) page(path, title, section(number), '/ROS2_INTERFACE_REFERENCE')
page('api/joint-trajectory', '🦾 关节轨迹接口契约',
  '> **运动服务端尚未实现。** 以下为接口契约，不代表可以直接调用运动。\n\n' +
  between('<a id="joint-trajectory"></a>', '<a id="initialize"></a>'), '/JOINT_TRAJECTORY_UPDATE', false)
page('api/power', '🔌 初始化上电',
  between('<a id="initialize"></a>', '<a id="python-client"></a>'), '/JOINT_TRAJECTORY_UPDATE')
page('guide/trajectory-client', '🐍 Python 轨迹配置',
  between('<a id="python-client"></a>', '<a id="known-limits"></a>'), '/JOINT_TRAJECTORY_UPDATE')
page('guide/status', '📋 实现进度与已知限制',
  between('<a id="implementation-status"></a>', '<a id="joint-trajectory"></a>') + '\n\n' +
  between('<a id="known-limits"></a>', null), '/JOINT_TRAJECTORY_UPDATE', false)
console.log(`Generated ${pages.length + 4} function pages in ${root}`)
