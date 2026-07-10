import type { AxisKind, Quantity, SourceKind } from '../semantics/types'
import type { FlightDataset, SignalColumn } from '../types'

interface SignalQuery {
  quantity?: Quantity
  source?: SourceKind
  axis?: AxisKind
  derived?: boolean
  rawIncludes?: string
  tableId?: string
}

export function findSignals(dataset: FlightDataset, query: SignalQuery) {
  return Object.values(dataset.signals).filter((signal) => {
    if (query.quantity && signal.semantic.quantity !== query.quantity) return false
    if (query.source && signal.semantic.source !== query.source) return false
    if (query.axis && signal.semantic.axis !== query.axis) return false
    if (typeof query.derived === 'boolean' && signal.semantic.isDerived !== query.derived) return false
    if (query.rawIncludes && !signal.rawName.toLowerCase().includes(query.rawIncludes.toLowerCase())) return false
    if (query.tableId && signal.tableId !== query.tableId) return false
    return true
  })
}

export function findSignal(dataset: FlightDataset, query: SignalQuery) {
  return findSignals(dataset, query)[0]
}

export function nearestValue(signal: SignalColumn, time: number) {
  if (!signal.time.length) return 0
  let lo = 0
  let hi = signal.time.length - 1
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2)
    if (signal.time[mid] < time) lo = mid + 1
    else hi = mid
  }
  const index = lo > 0 && Math.abs(signal.time[lo - 1] - time) < Math.abs(signal.time[lo] - time) ? lo - 1 : lo
  return signal.values[index] ?? 0
}

export function makeVirtualSignal(base: SignalColumn, id: string, displayName: string, values: number[], color: string, unit = base.semantic.unit) {
  return {
    ...base,
    id,
    rawName: id,
    values,
    typicalValue: values[Math.floor(values.length / 2)] ?? 0,
    latestValue: values[values.length - 1] ?? 0,
    anomalies: [],
    semantic: {
      ...base.semantic,
      displayName,
      rawName: id,
      unit,
      color,
      source: 'Derived' as const,
      isDerived: true,
    },
  }
}
