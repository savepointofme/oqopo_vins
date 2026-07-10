import { buildTimeSeriesOption } from '../charts/timeSeries'
import { ChartPanel } from '../components/ChartPanel'
import { MetricCard } from '../components/MetricCard'
import type { FlightDataset } from '../types'
import { findSignals } from './selectors'

interface TimingPresetProps {
  dataset: FlightDataset
}

export function TimingPreset({ dataset }: TimingPresetProps) {
  const rateSignals = findSignals(dataset, { quantity: 'frequency', derived: true }).slice(0, 6)
  const deltaSignals = findSignals(dataset, { derived: true }).filter((signal) => signal.rawName.endsWith('_delta')).slice(0, 6)
  const anomalyCount = [...rateSignals, ...deltaSignals].reduce((sum, signal) => sum + signal.anomalies.length, 0)
  const avgHz = rateSignals[0]?.values.filter((value) => value > 0).reduce((sum, value) => sum + value, 0) ?? 0
  const denom = rateSignals[0]?.values.filter((value) => value > 0).length || 1

  return (
    <div className="preset-layout">
      <div className="metric-grid metric-grid--three">
        <MetricCard label="平均更新频率" value={(avgHz / denom).toFixed(2)} unit="Hz" tone="green" />
        <MetricCard label="异常计数" value={anomalyCount} unit="events" tone={anomalyCount ? 'red' : 'green'} />
        <MetricCard label="最大间隔" value={Math.max(...deltaSignals.flatMap((signal) => signal.values)).toFixed(3)} unit="s" tone="orange" />
      </div>
      <div className="stacked-panels">
        <ChartPanel title="各表更新频率" unit="Hz" option={buildTimeSeriesOption(rateSignals.map((signal) => ({ signal })))} height={260} />
        <ChartPanel title="时间步长异常事件轴" unit="delta s / count" option={buildTimeSeriesOption(deltaSignals.map((signal) => ({ signal })))} height={260} />
      </div>
    </div>
  )
}
