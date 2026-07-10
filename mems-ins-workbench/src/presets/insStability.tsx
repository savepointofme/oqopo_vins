import { buildTimeSeriesOption } from '../charts/timeSeries'
import { summarizeAnomalies } from '../charts/anomalyMarkers'
import { ChartPanel } from '../components/ChartPanel'
import { MetricCard } from '../components/MetricCard'
import type { FlightDataset, SignalColumn } from '../types'
import { findSignals } from './selectors'

interface InsStabilityPresetProps {
  dataset: FlightDataset
}

function staticStd(signal: SignalColumn) {
  const values = signal.values.filter((_, index) => signal.time[index] <= 25)
  const mean = values.reduce((sum, value) => sum + value, 0) / Math.max(1, values.length)
  const variance = values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / Math.max(1, values.length - 1)
  return { mean, std: Math.sqrt(variance) }
}

export function InsStabilityPreset({ dataset }: InsStabilityPresetProps) {
  const gyro = findSignals(dataset, { quantity: 'gyro', source: 'IMU', derived: false })
  const accel = findSignals(dataset, { quantity: 'accel', source: 'IMU', derived: false })
  const temp = findSignals(dataset, { quantity: 'temperature', source: 'IMU', derived: false })
  const counters = findSignals(dataset, { quantity: 'counter', derived: true }).filter((signal) => signal.rawName.includes('counter'))
  const gyroStats = gyro[0] ? staticStd(gyro[0]) : { mean: 0, std: 0 }
  const accelStats = accel[2] ? staticStd(accel[2]) : { mean: 0, std: 0 }
  const anomalies = summarizeAnomalies(counters)

  return (
    <div className="preset-layout">
      <div className="metric-grid metric-grid--four">
        <MetricCard label="陀螺零偏" value={gyroStats.mean.toFixed(4)} unit="rad/s" tone="blue" />
        <MetricCard label="加速度零偏" value={(accelStats.mean - 9.81).toFixed(4)} unit="m/s²" tone="blue" />
        <MetricCard label="噪声 std" value={gyroStats.std.toFixed(4)} unit="rad/s" tone="green" />
        <MetricCard label="丢帧总数" value={anomalies.total} unit="events" tone={anomalies.total ? 'red' : 'green'} />
      </div>
      <div className="stability-grid">
        <ChartPanel title="陀螺三轴" unit="rad/s · 起飞前 25s 静止段" option={buildTimeSeriesOption(gyro.map((signal) => ({ signal })))} height={250} />
        <ChartPanel title="加速度三轴" unit="m/s²" option={buildTimeSeriesOption(accel.map((signal) => ({ signal })))} height={250} />
        <ChartPanel title="传感器温度" unit="°C" option={buildTimeSeriesOption(temp.map((signal) => ({ signal })))} height={250} />
        <ChartPanel title="计数器差分 / 丢帧竖线" unit="count" option={buildTimeSeriesOption(counters.map((signal) => ({ signal })))} height={250} />
      </div>
    </div>
  )
}
