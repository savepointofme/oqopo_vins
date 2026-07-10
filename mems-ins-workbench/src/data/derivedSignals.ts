import { parseColumn } from '../semantics/parser'
import type { FlightTable, SignalColumn } from '../types'

function median(values: number[]) {
  const finite = values.filter(Number.isFinite).slice().sort((a, b) => a - b)
  if (!finite.length) return 0
  const mid = Math.floor(finite.length / 2)
  return finite.length % 2 ? finite[mid] : (finite[mid - 1] + finite[mid]) / 2
}

function standardDeviation(values: number[]) {
  const finite = values.filter(Number.isFinite)
  if (finite.length < 2) return 0
  const mean = finite.reduce((sum, value) => sum + value, 0) / finite.length
  const variance = finite.reduce((sum, value) => sum + (value - mean) ** 2, 0) / (finite.length - 1)
  return Math.sqrt(variance)
}

export function detectAnomalies(signal: Pick<SignalColumn, 'semantic' | 'time' | 'values'>) {
  const anomalies: SignalColumn['anomalies'] = []
  const diffs: number[] = []

  for (let index = 1; index < signal.values.length; index += 1) {
    diffs.push(signal.values[index] - signal.values[index - 1])
  }

  if (!diffs.length) return anomalies

  const diffMedian = median(diffs)
  const diffStd = standardDeviation(diffs)
  const absDiffs = diffs.map(Math.abs)
  const absMedian = median(absDiffs)
  const threshold = Math.max(absMedian * 8, diffStd * 5)

  for (let index = 1; index < signal.values.length; index += 1) {
    const diff = signal.values[index] - signal.values[index - 1]
    const time = signal.time[index]
    const value = signal.values[index]

    if (signal.semantic.quantity === 'time' && Math.abs(diff - diffMedian) > Math.max(0.08, Math.abs(diffMedian) * 3)) {
      anomalies.push({ time, value, kind: 'time-step', label: '时间步长异常' })
    } else if (signal.semantic.quantity === 'counter' && Math.abs(diff - diffMedian) > 1.5) {
      anomalies.push({ time, value, kind: 'counter', label: '计数器异常' })
    } else if (
      signal.semantic.quantity !== 'time' &&
      signal.semantic.quantity !== 'counter' &&
      threshold > 0 &&
      Math.abs(diff) > threshold
    ) {
      anomalies.push({ time, value, kind: 'jump', label: '疑似跳变' })
    }

    if (anomalies.length > 24) break
  }

  return anomalies
}

function createSignal(
  table: FlightTable,
  rawName: string,
  values: number[],
  time: number[],
  derivedFrom?: string,
): SignalColumn {
  const semantic = parseColumn(rawName)
  const finite = values.filter(Number.isFinite)
  const typicalValue = finite.length ? median(finite) : 0
  const latestValue = finite.length ? finite[finite.length - 1] : 0
  const baseSignal = {
    id: `${table.id}:${rawName}`,
    tableId: table.id,
    tableName: table.name,
    rawName,
    semantic,
    time,
    values,
    typicalValue,
    latestValue,
    anomalies: [],
    derivedFrom,
  }

  return {
    ...baseSignal,
    anomalies: detectAnomalies(baseSignal),
  }
}

function relative(values: number[]) {
  const first = values.find(Number.isFinite) ?? 0
  return values.map((value) => value - first)
}

function diff(values: number[]) {
  return values.map((value, index) => (index === 0 ? 0 : value - values[index - 1]))
}

function reciprocalRate(deltaValues: number[]) {
  return deltaValues.map((delta) => (delta > 0 ? 1 / delta : 0))
}

export function appendDerivedSignals(table: FlightTable) {
  const derived: SignalColumn[] = []

  for (const signal of table.columns) {
    const { quantity } = signal.semantic
    const baseId = signal.id

    if (quantity === 'time' || quantity === 'counter') {
      const deltaRawName = `${signal.rawName}_delta`
      const deltaValues = diff(signal.values)
      const deltaSignal = createSignal(table, deltaRawName, deltaValues, signal.time, baseId)
      derived.push(deltaSignal)

      if (quantity === 'time') {
        derived.push(createSignal(table, `${signal.rawName}_rate_Hz`, reciprocalRate(deltaValues), signal.time, baseId))
      }
    }

    if (quantity === 'height' || quantity === 'position') {
      derived.push(createSignal(table, `${signal.rawName}_relative`, relative(signal.values), signal.time, baseId))
    }
  }

  table.columns = [...table.columns, ...derived]
  return table
}

export function indexSignals(tables: FlightTable[]) {
  return tables.reduce<Record<string, SignalColumn>>((index, table) => {
    for (const column of table.columns) {
      index[column.id] = column
    }
    return index
  }, {})
}
