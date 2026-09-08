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
  const actions = names(quick, 'Action Name')
  const matching = (items, pattern) => items.filter(name => pattern.test(name)).length
  return {
    topics: topics.length,
    services: services.length,
    actions: actions.length,
    stateTopics: matching(topics, /\/(joint_states|tcp_pose|jacobian)\//),
    servoJTopics: matching(topics, /\/servoj\//),
    servoLTopics: matching(topics, /\/servol\//),
    moveActions: matching(actions, /\/move_absj\//),
    motionServices: matching(services, /\/(movej_by_path|move_l|move_l_relative|move_l_target|get_cartesian_state)\//),
    powerServices: matching(services, /\/(initialize|power_on)(\/|$)/),
    homeServices: matching(services, /\/go_home\//),
    handServices: matching(services, /\/control_hand\//),
    fkServices: matching(services, /\/fk\//),
    ikServices: matching(services, /\/ik\//),
    visionTopics: matching(topics, /^\/(yolo_vision|aruco|tool|box|box_grab_points|small_box)(\/|$)/),
    visionServices: matching(services, /^\/bt_target_server\//),
    chassisTopics: matching(topics, /^\/(scheduler|chassis)\//),
    chassisServices: matching(services, /^\/bt_navigation_server\//),
    chassisActions: matching(actions, /^\/seer\//),
  }
}
