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

function detailLink(name) {
  if (name.startsWith('/aide/upperlimb/')) {
    const operation = name.split('/')[3]
    const links = {
      joint_states: '/api/state#_5-1-joint-states', tcp_pose: '/api/state#_5-2-tcp-pose',
      tcp_state: '/api/state#_5-3-tcp-state', jacobian: '/api/state#_5-4-jacobian',
      servoj: '/api/servoj#_6-1-消息结构',
      servol: '/api/servoj#_6-4-servol-消息与坐标系', move_absj: '/api/moveabsj',
      movej_by_path: '/api/moveabsj#_7-6-movej-by-path-service',
      move_l: '/api/movel#_8-1-绝对-movel', move_l_relative: '/api/movel#_8-2-相对-movel',
      get_cartesian_state: '/api/movel#_8-3-读取笛卡尔状态',
      cartesian_teach: '/api/cartesian-teach',
      initialize: '/api/power', power_on: '/api/power', go_home: '/api/home',
      control_hand: '/api/hand', fk: '/api/kinematics#_18-1-forwardkinematics',
      ik: '/api/kinematics#_18-2-inversekinematics',
    }
    if (!links[operation]) throw new Error(`Missing detail link: ${name}`)
    return links[operation]
  }
  return /^(\/scheduler\/|\/chassis\/|\/bt_navigation_server\/|\/seer\/)/.test(name)
    ? '/api/chassis' : '/api/vision'
}

function section(number) {
  return source.match(new RegExp(`^## ${number}\\. .*\\n([\\s\\S]*?)(?=^## |$(?![\\s\\S]))`, 'm'))[1]
}
function card(name, type, description, note = '') {
  const field = type.includes('/srv/') ? 'Service' : type.includes('/action/') ? 'Action' : 'Topic'
  return `\n##### ${name}\n\n| 字段 | 值 |\n| --- | --- |\n| ${field} Name | \`${name}\` |\n| Type | \`${type}\` |\n| Description | ${description} |\n${note ? `| Note | ${note} |\n` : ''}`
}
const vision = section(14)
const visionServices = [...vision.matchAll(/^\| `([^`]+)` \| `([^`]+\/srv\/[^`]+)` \| (.+) \|$/gm)]
const visionTopics = [...vision.matchAll(/^(\/\S+)\s+(\S+\/msg\/\S+)$/gm)]
const chassis = [...section(15).matchAll(/^\| `([^`]+)` \| `([^`]+)` \| (.+) \|$/gm)]
function chassisCards(kind) {
  return chassis.filter(row => row[2].includes(`/${kind}/`)).map(row => card(row[1], row[2], row[3], '可选底盘桥接；需启用 start_chassis_navigation。')).join('')
}

let catalogue = source.slice(0, detailStart).trim()
catalogue += `\n\n### 2.6 摄像头与视觉 Topics（${visionTopics.length}） {#topic-vision}\n\n视觉检测依赖的话题；部分名称可通过参数覆盖。\n` +
  visionTopics.map(row => card(row[1], row[2], '视觉检测模块的数据或触发话题', '具体数据格式与使用方法见 Type 链接。')).join('') +
  `\n### 2.7 移动底盘 Topics（2） {#topic-chassis}\n` + chassisCards('msg') +
  `\n### 2.8 摄像头与视觉 Services（${visionServices.length}） {#service-vision}\n` +
  visionServices.map(row => card(row[1], row[2], row[3], '由视觉目标服务提供。')).join('') +
  `\n### 2.9 移动底盘 Services（1） {#service-chassis}\n` + chassisCards('srv') +
  `\n### 2.10 底盘导航 Actions（1） {#action-chassis}\n` + chassisCards('action')
catalogue = catalogue.replace(
  /(^\| (?:Topic|Service|Action) Name \| `([^`]+)` \|\r?\n)\| Type \| `([^`]+)` \|/gm,
  (_, nameRow, name, type) => `${nameRow}| Type | [\`${type}\`](${detailLink(name)}) |`,
)
catalogue = catalogue.replace(
  /更完整的数据字段、参数约束与调用\n示例见后续章节。/,
  '字段定义、请求响应和调用示例请点击每张卡片中的 `Type`。',
)
catalogue = catalogue
  .replace('[未实现接口](#_13-未实现接口)', '[实现进度与已知限制](/guide/status)')
  .replace('[上层视觉目标接口](#_14-上层视觉目标接口)', '[摄像头与视觉](#topic-vision)')
  .replace('[可选底盘桥接接口](#_15-可选底盘桥接接口)', '[移动底盘](#topic-chassis)')

const header = `---\ntitle: 完整 API 文档\noutline: false\n---\n\n`
const footer = `

点击卡片中的 Type 可查看该接口的字段、调用示例与使用限制，也可展开左侧“接口说明与示例”按功能查阅。
`

writeFileSync(outputPath, header + catalogue + footer, 'utf8')
console.log(`Generated compact API catalogue: ${outputPath}`)
