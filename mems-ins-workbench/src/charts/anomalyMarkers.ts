import type { AnomalyPoint, SignalColumn } from '../types'

export function toMarkPoints(signal: SignalColumn, maxMarkers = 18) {
  return signal.anomalies.slice(0, maxMarkers).map((point) => ({
    name: point.label,
    coord: [point.time, point.value],
    value: point.label,
    itemStyle: { color: '#d33a35' },
    symbol: point.kind === 'jump' ? 'pin' : 'rect',
    symbolSize: point.kind === 'jump' ? 30 : 18,
  }))
}

export function summarizeAnomalies(signals: SignalColumn[]) {
  return signals.reduce(
    (summary, signal) => {
      for (const anomaly of signal.anomalies) {
        summary.total += 1
        summary.byKind[anomaly.kind] = (summary.byKind[anomaly.kind] ?? 0) + 1
      }
      return summary
    },
    { total: 0, byKind: {} as Record<AnomalyPoint['kind'], number> },
  )
}
