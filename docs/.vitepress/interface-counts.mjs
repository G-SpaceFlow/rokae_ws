// Count documented endpoint names once; never infer runtime availability.
export function interfaceCounts(markdown) {
  const section = number => {
    const match = markdown.match(new RegExp(`^## ${number}\\. .*\\n([\\s\\S]*?)(?=^## |$(?![\\s\\S]))`, 'm'))
    if (!match) throw new Error(`Missing interface section ${number}`)
    return match[1]
  }
  const names = (text, field) => [...new Set(
    [...text.matchAll(new RegExp(`\\| ${field} \\| \\x60([^\\x60]+)\\x60`, 'g'))].map(m => m[1]))]
  const quick = section(2)
  const services = names(quick, 'Service Name')
  const topics = names(quick, 'Topic Name')
  return {
    motion: services.filter(n => /\/(move_l|move_l_relative|move_l_target|movej_by_path|get_cartesian_state)(\/|$)/.test(n)).length,
    power: services.filter(n => /\/(initialize(?:_robots)?|power_on|go_home)(\/|$)/.test(n)).length,
    kinematics: services.filter(n => /\/(fk|ik)(\/|$)/.test(n)).length,
    hand: services.filter(n => /\/control_hand(\/|$)/.test(n)).length,
    vision: new Set([...section(14).matchAll(/\| `([^`]+)` \| `[^`]*\/srv\/[^`]+`/g)].map(m => m[1])).size,
    state: topics.filter(n => !/\/(servoj|servol)(\/|$)/.test(n)).length,
    servo: topics.filter(n => /\/(servoj|servol)(\/|$)/.test(n)).length,
    chassis: new Set([...section(15).matchAll(/\| `([^`]+)` \| `[^`]*\/msg\/[^`]+`/g)].map(m => m[1])).size,
    action: names(quick, 'Action Name').length,
  }
}
