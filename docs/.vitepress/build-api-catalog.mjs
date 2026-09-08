// Generate the compact API catalogue from the maintained detailed reference.
import { readFileSync, writeFileSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..')
const sourcePath = resolve(root, '.vitepress/ROS2_INTERFACE_DETAILS.md')
const outputPath = resolve(root, 'ROS2_INTERFACE_REFERENCE.md')
const source = readFileSync(sourcePath, 'utf8')
const detailStart = source.indexOf('\n## 3. ')

if (detailStart < 0) throw new Error('Cannot find detailed reference section 3')

function slug(type) {
  return type.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '')
}

function typePage(type) {
  if (type.includes('/msg/')) return 'message-types'
  if (type.includes('/srv/')) return 'service-types'
  if (type.includes('/action/')) return 'action-types'
  throw new Error(`Unsupported ROS interface type: ${type}`)
}

let catalogue = source.slice(0, detailStart).trim()
catalogue = catalogue.replace(
  /^\| Type \| `([^`]+)` \|$/gm,
  (_, type) => `| Type | [\`${type}\`](/reference/${typePage(type)}#${slug(type)}) |`,
)
catalogue = catalogue.replace(
  /更完整的数据字段、参数约束与调用\n示例见后续章节。/,
  '字段定义、请求响应和调用示例请点击每张卡片中的 `Type`。',
)
catalogue = catalogue
  .replace('[未实现接口](#_13-未实现接口)', '[实现进度与已知限制](/guide/status)')
  .replace('[上层视觉目标接口](#_14-上层视觉目标接口)', '[上层视觉目标接口示例](/api/vision)')
  .replace('[可选底盘桥接接口](#_15-可选底盘桥接接口)', '[可选底盘桥接接口示例](/api/chassis)')

const header = `---\ntitle: 完整 API 文档\noutline: false\n---\n\n`
const footer = `

## 接口类型与示例

完整 API 页面只负责快速查找接口。字段结构、请求/响应以及调用方式分别收录在：

- [Message Type · 消息类型](/reference/message-types)
- [Service Type · 服务类型](/reference/service-types)
- [Action Type · 动作类型](/reference/action-types)

具体运动流程、参数限制和安全说明请从左侧“接口示例”进入对应功能页面。
`

writeFileSync(outputPath, header + catalogue + footer, 'utf8')
console.log(`Generated compact API catalogue: ${outputPath}`)
