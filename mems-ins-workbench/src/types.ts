import type { SemanticSignal } from './semantics/types'

export interface AnomalyPoint {
  time: number
  value: number
  kind: 'jump' | 'outlier' | 'time-step' | 'counter'
  label: string
}

export interface SignalColumn {
  id: string
  tableId: string
  tableName: string
  rawName: string
  semantic: SemanticSignal
  time: number[]
  values: number[]
  typicalValue: number
  latestValue: number
  anomalies: AnomalyPoint[]
  derivedFrom?: string
}

export interface FlightTable {
  id: string
  name: string
  description: string
  rowCount: number
  timeStart: number
  timeEnd: number
  estimatedHz: number
  parseStatus: 'ok' | 'partial' | 'failed'
  columns: SignalColumn[]
}

export interface FlightDataset {
  sortieName: string
  status: 'empty' | 'loading' | 'parsed' | 'partial'
  timeSpan: [number, number]
  sampleSummary: string
  tables: FlightTable[]
  signals: Record<string, SignalColumn>
  generatedAt: string
  parserWarnings: string[]
}

export type CurveTransform = 'raw' | 'relative' | 'smooth' | 'normalize'

export interface ActiveCurve {
  signalId: string
  transform: CurveTransform
  hidden: boolean
  color?: string
  yAxis?: 'left' | 'right'
}

export type PresetId =
  | 'free'
  | 'trajectory-attitude'
  | 'height-error'
  | 'timing'
  | 'ins-stability'
  | 'divergence'
  | 'gnss-quality'
  | 'visual-matching'
  | 'filter-health'
  | 'vibration-spectrum'

export interface StatsRow {
  name: string
  unit: string
  color: string
  mean: number
  std: number
  min: number
  max: number
  peakToPeak: number
  slope: number
}

export interface StatsSelection {
  range: [number, number]
  rows: StatsRow[]
}
