interface MetricCardProps {
  label: string
  value: string | number
  unit?: string
  tone?: 'blue' | 'green' | 'red' | 'orange' | 'purple' | 'gray'
  hint?: string
}

export function MetricCard({ label, value, unit, tone = 'blue', hint }: MetricCardProps) {
  return (
    <article className={`metric-card metric-card--${tone}`}>
      <div className="metric-card__label">{label}</div>
      <div className="metric-card__value">{value}</div>
      <div className="metric-card__footer">
        <span>{unit}</span>
        {hint ? <span>{hint}</span> : null}
      </div>
    </article>
  )
}
