import { X } from 'lucide-react'
import type { StatsSelection } from '../types'

interface StatsPopoverProps {
  stats: StatsSelection | null
  onClose: () => void
}

function fmt(value: number) {
  if (!Number.isFinite(value)) return '--'
  const abs = Math.abs(value)
  return abs > 100 ? value.toFixed(1) : abs > 10 ? value.toFixed(2) : value.toFixed(3)
}

export function StatsPopover({ stats, onClose }: StatsPopoverProps) {
  if (!stats) return null
  return (
    <aside className="stats-popover">
      <header>
        <strong>区间统计</strong>
        <span className="mono">
          {stats.range[0].toFixed(1)} - {stats.range[1].toFixed(1)} s
        </span>
        <button type="button" className="icon-button icon-button--light" onClick={onClose}>
          <X size={14} />
        </button>
      </header>
      <div className="stats-table">
        <div className="stats-table__head">
          <span>曲线</span>
          <span>均值</span>
          <span>Std</span>
          <span>峰峰</span>
          <span>斜率</span>
        </div>
        {stats.rows.map((row) => (
          <div className="stats-table__row" key={row.name}>
            <span style={{ '--row-color': row.color } as React.CSSProperties}>{row.name}</span>
            <span className="mono">{fmt(row.mean)}</span>
            <span className="mono">{fmt(row.std)}</span>
            <span className="mono">{fmt(row.peakToPeak)}</span>
            <span className="mono">{fmt(row.slope)}</span>
          </div>
        ))}
      </div>
    </aside>
  )
}
