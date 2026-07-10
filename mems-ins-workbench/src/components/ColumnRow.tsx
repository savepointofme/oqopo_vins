import type { SignalColumn } from '../types'
import { Sparkline } from './Sparkline'

interface ColumnRowProps {
  signal: SignalColumn
  onAdd: (signalId: string) => void
  onDragStart: (signalId: string) => void
  selected: boolean
}

function formatValue(value: number, unit: string) {
  if (!Number.isFinite(value)) return '--'
  const abs = Math.abs(value)
  const numeric = abs >= 1000 ? value.toFixed(0) : abs >= 10 ? value.toFixed(2) : value.toFixed(3)
  return unit ? `${numeric} ${unit}` : numeric
}

export function ColumnRow({ signal, onAdd, onDragStart, selected }: ColumnRowProps) {
  const color = signal.semantic.color
  return (
    <button
      type="button"
      className={`column-row ${selected ? 'column-row--selected' : ''}`}
      draggable
      onDragStart={(event) => {
        event.dataTransfer.setData('application/x-signal-id', signal.id)
        event.dataTransfer.effectAllowed = 'copy'
        onDragStart(signal.id)
      }}
      onDoubleClick={() => onAdd(signal.id)}
      title={`${signal.rawName} · ${signal.semantic.unit || '无单位'} · 解析置信度 ${Math.round(signal.semantic.confidence * 100)}%`}
    >
      <span className="column-row__stripe" style={{ backgroundColor: color }} />
      <span className="column-row__main">
        <strong>{signal.semantic.displayName}</strong>
        <small>{signal.rawName}</small>
      </span>
      <span className="source-chip" style={{ borderColor: color, color }}>
        {signal.semantic.source}
      </span>
      <span className="mono column-row__value">{formatValue(signal.latestValue, signal.semantic.unit)}</span>
      <Sparkline values={signal.values} color={color} />
    </button>
  )
}
