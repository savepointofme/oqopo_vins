import type { EChartsOption, SeriesOption } from 'echarts'
import { useMemo, useState } from 'react'
import { buildTrajectoryOption, type TrajectorySource } from '../charts/trajectory'
import { downsamplePairs } from '../charts/timeSeries'
import { unwrapDegrees } from '../charts/linkedCursor'
import { ChartPanel } from '../components/ChartPanel'
import { MetricCard } from '../components/MetricCard'
import type { FlightDataset, SignalColumn } from '../types'
import { findSignal, nearestValue } from './selectors'

interface TrajectoryAttitudeProps {
  dataset: FlightDataset
}

const sourceLabels = {
  GNSS: '卫导',
  'Visual-INS': '视觉惯导',
  'Strapdown-INS': '捷联惯导',
  'Pure-INS': '纯惯导',
} as const

function sourcePair(dataset: FlightDataset, source: keyof typeof sourceLabels) {
  const north = findSignal(dataset, { source, quantity: 'position', axis: 'North', derived: false })
  const east = findSignal(dataset, { source, quantity: 'position', axis: 'East', derived: false })
  if (!north || !east) return null
  return { north, east }
}

function calcHorizontalError(referenceN: SignalColumn, referenceE: SignalColumn, testN: SignalColumn, testE: SignalColumn) {
  let max = 0
  let end = 0
  for (let index = 0; index < referenceN.time.length; index += 1) {
    const time = referenceN.time[index]
    const dn = nearestValue(testN, time) - referenceN.values[index]
    const de = nearestValue(testE, time) - referenceE.values[index]
    const error = Math.hypot(dn, de)
    if (error > max) max = error
    if (index === referenceN.time.length - 1) end = error
  }
  return { max, end }
}

function attitudeStackOption(dataset: FlightDataset): EChartsOption {
  const rows: Array<{ quantity: 'roll' | 'pitch' | 'yaw'; title: string }> = [
    { quantity: 'roll', title: 'Roll 横滚' },
    { quantity: 'pitch', title: 'Pitch 俯仰' },
    { quantity: 'yaw', title: 'Yaw 航向' },
  ]
  const sources: Array<keyof typeof sourceLabels> = ['Visual-INS', 'Strapdown-INS', 'Pure-INS']
  const series: SeriesOption[] = []

  rows.forEach((row, rowIndex) => {
    sources.forEach((source) => {
      const signal = findSignal(dataset, { source, quantity: row.quantity, derived: false })
      if (!signal) return
      const values = row.quantity === 'yaw' ? unwrapDegrees(signal.values) : signal.values
      series.push({
        type: 'line',
        name: `${sourceLabels[source]} ${row.title}`,
        xAxisIndex: rowIndex,
        yAxisIndex: rowIndex,
        data: downsamplePairs(signal.time, values, 1800),
        showSymbol: false,
        lineStyle: {
          width: 1.3,
          type: signal.semantic.lineStyle === 'dashdot' ? 'dashed' : signal.semantic.lineStyle,
        },
        itemStyle: { color: signal.semantic.color },
      })
    })
  })

  return {
    animation: false,
    color: ['#2563c9', '#d33a35', '#d97706'],
    axisPointer: { link: [{ xAxisIndex: [0, 1, 2] }] },
    tooltip: {
      trigger: 'axis',
      axisPointer: { type: 'cross', label: { backgroundColor: '#1d2733' } },
      valueFormatter: (value) => (typeof value === 'number' ? `${value.toFixed(2)}°` : String(value)),
    },
    grid: [
      { left: 50, right: 18, top: 24, height: '24%' },
      { left: 50, right: 18, top: '38%', height: '24%' },
      { left: 50, right: 18, top: '70%', height: '24%' },
    ],
    xAxis: rows.map((_, index) => ({
      type: 'value',
      gridIndex: index,
      axisLabel: { show: index === 2, color: '#64748b', fontFamily: 'var(--font-mono)' },
      splitLine: { lineStyle: { color: '#edf0f4' } },
    })),
    yAxis: rows.map((row, index) => ({
      type: 'value',
      gridIndex: index,
      name: row.title,
      scale: true,
      axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
      splitLine: { lineStyle: { color: '#edf0f4' } },
    })),
    dataZoom: [
      { type: 'inside', xAxisIndex: [0, 1, 2], filterMode: 'none' },
      { type: 'slider', xAxisIndex: [0, 1, 2], height: 18, bottom: 8, filterMode: 'none' },
    ],
    series,
  }
}

export function TrajectoryAttitudePreset({ dataset }: TrajectoryAttitudeProps) {
  const [visible, setVisible] = useState<Record<string, boolean>>({
    GNSS: true,
    'Visual-INS': true,
    'Strapdown-INS': true,
    'Pure-INS': true,
  })

  const sources = useMemo(() => {
    return (Object.keys(sourceLabels) as Array<keyof typeof sourceLabels>)
      .map((source) => {
        const pair = sourcePair(dataset, source)
        if (!pair) return null
        const signal = pair.north
        return {
          name: sourceLabels[source],
          color: signal.semantic.color,
          lineStyle: signal.semantic.lineStyle,
          north: pair.north,
          east: pair.east,
          visible: visible[source] ?? true,
        } satisfies TrajectorySource
      })
      .filter(Boolean) as TrajectorySource[]
  }, [dataset, visible])

  const metrics = useMemo(() => {
    const reference = sourcePair(dataset, 'GNSS')
    const visual = sourcePair(dataset, 'Visual-INS')
    if (!reference || !visual) return { max: 0, end: 0 }
    return calcHorizontalError(reference.north, reference.east, visual.north, visual.east)
  }, [dataset])

  return (
    <div className="preset-layout preset-layout--trajectory">
      <div className="metric-grid metric-grid--three">
        <MetricCard label="最大水平偏差" value={metrics.max.toFixed(2)} unit="m" tone="red" />
        <MetricCard label="末端位置误差" value={metrics.end.toFixed(2)} unit="m" tone="orange" />
        <MetricCard label="轨迹时长" value={(dataset.timeSpan[1] - dataset.timeSpan[0]).toFixed(0)} unit="s" tone="green" />
      </div>
      <div className="trajectory-grid">
        <ChartPanel title="二维水平轨迹" unit="起点对齐 · 等比例坐标" option={buildTrajectoryOption(sources)} height={508}>
          <div className="legend-chips">
            {sources.map((source) => (
              <button
                type="button"
                key={source.name}
                className={`legend-chip ${source.visible ? '' : 'legend-chip--off'}`}
                onClick={() =>
                  setVisible((prev) => ({
                    ...prev,
                    [Object.entries(sourceLabels).find(([, label]) => label === source.name)?.[0] ?? source.name]: !source.visible,
                  }))
                }
              >
                <span style={{ backgroundColor: source.color }} />
                {source.name}
              </button>
            ))}
          </div>
        </ChartPanel>
        <ChartPanel title="姿态联动" unit="deg · yaw 已连续化" option={attitudeStackOption(dataset)} height={508} />
      </div>
    </div>
  )
}
