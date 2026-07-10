export const linkedCursorGroup = 'mems-ins-linked-cursor'

export function formatTime(seconds: number) {
  if (!Number.isFinite(seconds)) return '--'
  const minutes = Math.floor(seconds / 60)
  const remain = seconds - minutes * 60
  return `${minutes}:${remain.toFixed(1).padStart(4, '0')}`
}

export function findNearestIndex(time: number[], target: number) {
  if (!time.length) return -1
  let lo = 0
  let hi = time.length - 1

  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2)
    if (time[mid] < target) lo = mid + 1
    else hi = mid
  }

  if (lo > 0 && Math.abs(time[lo - 1] - target) < Math.abs(time[lo] - target)) return lo - 1
  return lo
}

export function unwrapDegrees(values: number[]) {
  if (!values.length) return values
  const result = [values[0]]
  let offset = 0

  for (let index = 1; index < values.length; index += 1) {
    const previous = values[index - 1]
    const current = values[index]
    const delta = current - previous
    if (delta > 180) offset -= 360
    if (delta < -180) offset += 360
    result.push(current + offset)
  }

  return result
}
