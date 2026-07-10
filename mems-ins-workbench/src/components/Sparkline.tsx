interface SparklineProps {
  values: number[]
  color: string
}

export function Sparkline({ values, color }: SparklineProps) {
  const sampleCount = 34
  const step = Math.max(1, Math.floor(values.length / sampleCount))
  const sampled = values.filter((_, index) => index % step === 0).slice(0, sampleCount)
  const min = Math.min(...sampled)
  const max = Math.max(...sampled)
  const span = max - min || 1
  const points = sampled
    .map((value, index) => {
      const x = (index / Math.max(1, sampled.length - 1)) * 72
      const y = 24 - ((value - min) / span) * 20 - 2
      return `${x.toFixed(1)},${y.toFixed(1)}`
    })
    .join(' ')

  return (
    <svg className="sparkline" viewBox="0 0 72 26" preserveAspectRatio="none" aria-hidden="true">
      <polyline fill="none" stroke={color} strokeWidth="1.6" points={points} />
    </svg>
  )
}
