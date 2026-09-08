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
    topics: topics.length,
    services: services.length,
    actions: names(quick, 'Action Name').length,
  }
}
