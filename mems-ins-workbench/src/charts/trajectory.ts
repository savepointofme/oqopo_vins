import type { EChartsOption, SeriesOption } from 'echarts'
import type { SignalColumn } from '../types'

export interface TrajectorySource {
  name: string
  color: string
  lineStyle: 'solid' | 'dashed' | 'dashdot' | 'dotted'
  north: SignalColumn
  east: SignalColumn
  visible: boolean
}

function downsampleXY(x: number[], y: number[], maxPoints = 2800) {
  const length = Math.min(x.length, y.length)
  if (length <= maxPoints) return Array.from({ length }, (_, index) => [x[index], y[index]])
  const step = Math.ceil(length / maxPoints)
  const pairs: number[][] = []
  for (let index = 0; index < length; index += step) {
    pairs.push([x[index], y[index]])
  }
  return pairs
}

export function buildTrajectoryOption(sources: TrajectorySource[]): EChartsOption {
  const series: SeriesOption[] = sources
    .filter((source) => source.visible)
    .map((source) => {
      const startN = source.north.values[0] ?? 0
      const startE = source.east.values[0] ?? 0
      const x = source.east.values.map((value) => value - startE)
      const y = source.north.values.map((value) => value - startN)
      const data = downsampleXY(x, y, 2800)
      return {
        type: 'line',
        name: source.name,
        data,
        showSymbol: false,
        symbolSize: 4,
        lineStyle: {
          color: source.color,
          width: 1.8,
          type: source.lineStyle === 'dashdot' ? 'dashed' : source.lineStyle,
        },
        itemStyle: { color: source.color },
        markPoint: {
          symbolSize: 36,
          label: { formatter: '{b}', color: '#1d2733', fontSize: 10 },
          data: [
            { name: '起点', coord: data[0], itemStyle: { color: '#1f7a4d' } },
            { name: '终点', coord: data[data.length - 1], itemStyle: { color: source.color } },
          ],
        },
      }
    })

  return {
    animation: false,
    grid: { left: 58, right: 24, top: 18, bottom: 46 },
    tooltip: {
      trigger: 'axis',
      axisPointer: { type: 'cross', label: { backgroundColor: '#1d2733' } },
      valueFormatter: (value) => (typeof value === 'number' ? value.toFixed(2) : String(value)),
    },
    xAxis: {
      type: 'value',
      name: 'East m',
      scale: true,
      axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
      splitLine: { lineStyle: { color: '#edf0f4' } },
    },
    yAxis: {
      type: 'value',
      name: 'North m',
      scale: true,
      axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
      splitLine: { lineStyle: { color: '#edf0f4' } },
    },
    series,
  }
}
