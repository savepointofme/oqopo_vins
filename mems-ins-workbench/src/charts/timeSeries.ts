import type { EChartsOption, SeriesOption } from 'echarts'
import { toMarkPoints } from './anomalyMarkers'
import type { ActiveCurve, SignalColumn, StatsRow, StatsSelection } from '../types'

export interface SeriesInput {
  signal: SignalColumn
  curve?: Partial<ActiveCurve>
  name?: string
  values?: number[]
  unit?: string
  color?: string
  yAxisIndex?: number
  showMarkers?: boolean
}

export function downsamplePairs(time: number[], values: number[], maxPoints = 2400) {
  const length = Math.min(time.length, values.length)
  if (length <= maxPoints) {
    return Array.from({ length }, (_, index) => [time[index], values[index]])
  }

  const step = Math.ceil(length / maxPoints)
  const pairs: number[][] = []

  for (let index = 0; index < length; index += step) {
    const end = Math.min(length, index + step)
    let minValue = Number.POSITIVE_INFINITY
    let maxValue = Number.NEGATIVE_INFINITY
    let minTime = time[index]
    let maxTime = time[index]

    for (let cursor = index; cursor < end; cursor += 1) {
      const value = values[cursor]
      if (value < minValue) {
        minValue = value
        minTime = time[cursor]
      }
      if (value > maxValue) {
        maxValue = value
        maxTime = time[cursor]
      }
    }

    pairs.push([minTime, minValue])
    if (maxTime !== minTime) pairs.push([maxTime, maxValue])
  }

  pairs.sort((a, b) => a[0] - b[0])
  return pairs
}

export function transformValues(values: number[], transform: ActiveCurve['transform'] = 'raw') {
  if (transform === 'relative') {
    const first = values.find(Number.isFinite) ?? 0
    return values.map((value) => value - first)
  }

  if (transform === 'normalize') {
    const finite = values.filter(Number.isFinite)
    const min = Math.min(...finite)
    const max = Math.max(...finite)
    const span = max - min || 1
    return values.map((value) => (value - min) / span)
  }

  if (transform === 'smooth') {
    const window = 9
    return values.map((_, index) => {
      const start = Math.max(0, index - window)
      const end = Math.min(values.length, index + window + 1)
      let sum = 0
      let count = 0
      for (let cursor = start; cursor < end; cursor += 1) {
        if (Number.isFinite(values[cursor])) {
          sum += values[cursor]
          count += 1
        }
      }
      return count ? sum / count : values[index]
    })
  }

  return values
}

export function buildTimeSeriesOption(inputs: SeriesInput[], title = ''): EChartsOption {
  const visible = inputs.filter((input) => !input.curve?.hidden)
  const primaryUnit = visible[0]?.unit ?? visible[0]?.signal.semantic.unit ?? ''
  const secondaryUnit = visible.find((input) => (input.unit ?? input.signal.semantic.unit) !== primaryUnit)?.unit ?? ''

  const series: SeriesOption[] = visible.map((input) => {
    const unit = input.unit ?? input.signal.semantic.unit
    const yAxisIndex = input.yAxisIndex ?? (unit && primaryUnit && unit !== primaryUnit ? 1 : 0)
    const values = input.values ?? transformValues(input.signal.values, input.curve?.transform ?? 'raw')
    return {
      type: 'line',
      name: input.name ?? input.signal.semantic.displayName,
      yAxisIndex,
      data: downsamplePairs(input.signal.time, values),
      showSymbol: false,
      smooth: false,
      lineStyle: {
        width: 1.45,
        type: input.signal.semantic.lineStyle === 'dashdot' ? 'dashed' : input.signal.semantic.lineStyle,
      },
      itemStyle: { color: input.color ?? input.curve?.color ?? input.signal.semantic.color },
      markPoint:
        input.showMarkers === false
          ? undefined
          : {
              symbolSize: 22,
              label: { show: false },
              data: toMarkPoints({ ...input.signal, values }),
            },
    }
  })

  return {
    animation: false,
    color: visible.map((input) => input.color ?? input.curve?.color ?? input.signal.semantic.color),
    title: title ? { text: title, left: 8, top: 4, textStyle: { fontSize: 12, fontWeight: 600 } } : undefined,
    grid: { left: 54, right: secondaryUnit ? 58 : 24, top: title ? 38 : 20, bottom: 42 },
    tooltip: {
      trigger: 'axis',
      axisPointer: { type: 'cross', label: { backgroundColor: '#1d2733' } },
      valueFormatter: (value) => (typeof value === 'number' ? value.toFixed(3) : String(value)),
    },
    brush: {
      toolbox: ['lineX', 'clear'],
      xAxisIndex: 0,
      brushMode: 'single',
      throttleType: 'debounce',
      throttleDelay: 80,
      transformable: false,
      brushStyle: { color: 'rgba(37, 99, 201, 0.12)', borderColor: '#2563c9' },
    },
    dataZoom: [
      { type: 'inside', xAxisIndex: [0], filterMode: 'none' },
      { type: 'slider', xAxisIndex: [0], height: 18, bottom: 12, filterMode: 'none' },
    ],
    xAxis: {
      type: 'value',
      name: 's',
      nameTextStyle: { color: '#828c99' },
      axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
      splitLine: { lineStyle: { color: '#edf0f4' } },
    },
    yAxis: [
      {
        type: 'value',
        name: primaryUnit,
        scale: true,
        axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
        splitLine: { lineStyle: { color: '#edf0f4' } },
      },
      {
        type: 'value',
        name: secondaryUnit,
        scale: true,
        position: 'right',
        axisLabel: { color: '#64748b', fontFamily: 'var(--font-mono)' },
        splitLine: { show: false },
      },
    ],
    series,
  }
}

export function calculateStats(inputs: SeriesInput[], range: [number, number]): StatsSelection {
  const [start, end] = range[0] <= range[1] ? range : [range[1], range[0]]
  const rows: StatsRow[] = []

  for (const input of inputs.filter((item) => !item.curve?.hidden)) {
    const values = input.values ?? transformValues(input.signal.values, input.curve?.transform ?? 'raw')
    const selected: { time: number; value: number }[] = []
    for (let index = 0; index < input.signal.time.length; index += 1) {
      const time = input.signal.time[index]
      if (time >= start && time <= end && Number.isFinite(values[index])) {
        selected.push({ time, value: values[index] })
      }
    }
    if (!selected.length) continue
    const numeric = selected.map((point) => point.value)
    const mean = numeric.reduce((sum, value) => sum + value, 0) / numeric.length
    const variance = numeric.reduce((sum, value) => sum + (value - mean) ** 2, 0) / Math.max(1, numeric.length - 1)
    const min = Math.min(...numeric)
    const max = Math.max(...numeric)
    const first = selected[0]
    const last = selected[selected.length - 1]
    rows.push({
      name: input.name ?? input.signal.semantic.displayName,
      unit: input.unit ?? input.signal.semantic.unit,
      color: input.color ?? input.curve?.color ?? input.signal.semantic.color,
      mean,
      std: Math.sqrt(variance),
      min,
      max,
      peakToPeak: max - min,
      slope: last.time !== first.time ? (last.value - first.value) / (last.time - first.time) : 0,
    })
  }

  return { range: [start, end], rows }
}
