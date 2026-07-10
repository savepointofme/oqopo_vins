import { ChartPanel } from '../components/ChartPanel'
import { MetricCard } from '../components/MetricCard'
import type { FlightDataset, PresetId } from '../types'
import type { EChartsOption } from 'echarts'

const names: Record<PresetId, string> = {
  free: '自由分析',
  'trajectory-attitude': '轨迹与姿态',
  'height-error': '高度与误差',
  timing: '收发延迟 / 时序',
  'ins-stability': '惯导稳定性',
  divergence: '多源一致性 / 发散',
  'gnss-quality': '卫导质量',
  'visual-matching': '下视视觉质量',
  'filter-health': '滤波器健康度',
  'vibration-spectrum': '振动频谱',
}

const emptyOption: EChartsOption = {
  animation: false,
  grid: { left: 52, right: 24, top: 24, bottom: 38 },
  xAxis: { type: 'value', splitLine: { lineStyle: { color: '#edf0f4' } } },
  yAxis: { type: 'value', splitLine: { lineStyle: { color: '#edf0f4' } } },
  series: [
    {
      type: 'line',
      data: [
        [0, 0],
        [1, 0.2],
        [2, 0.16],
        [3, 0.3],
        [4, 0.24],
      ],
      showSymbol: false,
      lineStyle: { color: '#94a3b8', type: 'dashed' },
    },
  ],
}

interface PresetPlaceholderProps {
  id: PresetId
  dataset: FlightDataset
}

export function PresetPlaceholder({ id, dataset }: PresetPlaceholderProps) {
  return (
    <div className="preset-layout">
      <div className="metric-grid metric-grid--three">
        <MetricCard label="可用信号" value={Object.keys(dataset.signals).length} unit="cols" tone="blue" />
        <MetricCard label="解析告警" value={dataset.parserWarnings.length} unit="cols" tone={dataset.parserWarnings.length ? 'orange' : 'green'} />
        <MetricCard label="预制布局" value="V1" unit="reserved" tone="gray" />
      </div>
      <div className="placeholder-grid">
        <ChartPanel title={`${names[id]} · 主趋势`} unit="预留语义选择器" option={emptyOption} height={260} />
        <ChartPanel title={`${names[id]} · 事件 / 质量轴`} unit="预留事件轨" option={emptyOption} height={260} />
      </div>
    </div>
  )
}
