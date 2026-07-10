import { useMemo } from 'react'
import { buildTimeSeriesOption } from '../charts/timeSeries'
import { ChartPanel } from '../components/ChartPanel'
import { MetricCard } from '../components/MetricCard'
import type { FlightDataset, SignalColumn } from '../types'
import { findSignal, findSignals, makeVirtualSignal, nearestValue } from './selectors'

interface HeightErrorProps {
  dataset: FlightDataset
}

function positionErrors(dataset: FlightDataset) {
  const refN = findSignal(dataset, { source: 'GNSS', quantity: 'position', axis: 'North', derived: false })
  const refE = findSignal(dataset, { source: 'GNSS', quantity: 'position', axis: 'East', derived: false })
  const testN = findSignal(dataset, { source: 'Visual-INS', quantity: 'position', axis: 'North', derived: false })
  const testE = findSignal(dataset, { source: 'Visual-INS', quantity: 'position', axis: 'East', derived: false })
  if (!refN || !refE || !testN || !testE) return []
  const northErr = refN.time.map((time, index) => nearestValue(testN, time) - refN.values[index])
  const eastErr = refE.time.map((time, index) => nearestValue(testE, time) - refE.values[index])
  const horizontal = northErr.map((value, index) => Math.hypot(value, eastErr[index]))
  return [
    makeVirtualSignal(refN, 'visual_gnss_north_error', '北向误差', northErr, '#2563c9', 'm'),
    makeVirtualSignal(refE, 'visual_gnss_east_error', '东向误差', eastErr, '#8b5cf6', 'm'),
    makeVirtualSignal(refN, 'visual_gnss_horizontal_error', '水平合误差', horizontal, '#d33a35', 'm'),
  ]
}

function velocityErrors(dataset: FlightDataset) {
  const axes = [
    ['North', 'N', '#2563c9'],
    ['East', 'E', '#8b5cf6'],
    ['Up', 'U', '#d33a35'],
  ] as const
  const rows: SignalColumn[] = []

  axes.forEach(([axis, label, color]) => {
    const ref = findSignal(dataset, { source: 'GNSS', quantity: 'velocity', axis, derived: false })
    const test = findSignal(dataset, { source: 'Visual-INS', quantity: 'velocity', axis, derived: false })
    if (!ref || !test) return
    const values = ref.time.map((time, index) => nearestValue(test, time) - ref.values[index])
    rows.push(makeVirtualSignal(ref, `visual_gnss_vel_${label}_error`, `${label} 速度误差`, values, color, 'm/s'))
  })

  return rows
}

export function HeightErrorPreset({ dataset }: HeightErrorProps) {
  const heightSignals = useMemo(() => {
    return [
      ...findSignals(dataset, { quantity: 'height', derived: true }).filter((signal) => signal.rawName.includes('relative')),
    ].slice(0, 8)
  }, [dataset])

  const errors = useMemo(() => positionErrors(dataset), [dataset])
  const velocity = useMemo(() => velocityErrors(dataset), [dataset])
  const horizontal = errors[2]?.values ?? []
  const mean = horizontal.length ? horizontal.reduce((sum, value) => sum + value, 0) / horizontal.length : 0
  const max = horizontal.length ? Math.max(...horizontal) : 0
  const vertical = heightSignals[0]?.values.map((value, index) => Math.abs(value - (heightSignals[1]?.values[index] ?? value))) ?? []
  const verticalMean = vertical.length ? vertical.reduce((sum, value) => sum + value, 0) / vertical.length : 0

  return (
    <div className="preset-layout">
      <div className="metric-grid metric-grid--three">
        <MetricCard label="巡航段水平误差均值" value={mean.toFixed(2)} unit="m" tone="orange" />
        <MetricCard label="巡航段水平误差最大值" value={max.toFixed(2)} unit="m" tone="red" />
        <MetricCard label="垂直误差均值" value={verticalMean.toFixed(2)} unit="m" tone="blue" />
      </div>
      <div className="stacked-panels">
        <ChartPanel title="多源高度" unit="m · 减初值叠加" option={buildTimeSeriesOption(heightSignals.map((signal) => ({ signal })))} height={220} />
        <ChartPanel title="水平位置误差" unit="m · GNSS 更新时刻采样" option={buildTimeSeriesOption(errors.map((signal) => ({ signal })))} height={220} />
        <ChartPanel title="速度误差" unit="m/s · 分量模式" option={buildTimeSeriesOption(velocity.map((signal) => ({ signal })))} height={220} />
      </div>
    </div>
  )
}
