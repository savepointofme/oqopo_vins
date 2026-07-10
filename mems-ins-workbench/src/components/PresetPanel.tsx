import {
  Activity,
  BarChart3,
  Clock3,
  Eye,
  GitCompare,
  HeartPulse,
  Route,
  Satellite,
  Waves,
} from 'lucide-react'
import type { PresetId } from '../types'

const presets: Array<{ id: PresetId; title: string; subtitle: string; icon: typeof Route }> = [
  { id: 'trajectory-attitude', title: '轨迹与姿态', subtitle: 'Trajectory & Attitude', icon: Route },
  { id: 'height-error', title: '高度与误差', subtitle: 'Height & Error', icon: BarChart3 },
  { id: 'timing', title: '收发延迟 / 时序', subtitle: 'Timing', icon: Clock3 },
  { id: 'ins-stability', title: '惯导稳定性', subtitle: 'INS Stability', icon: Activity },
  { id: 'divergence', title: '多源一致性', subtitle: 'Divergence', icon: GitCompare },
  { id: 'gnss-quality', title: '卫导质量', subtitle: 'GNSS Quality', icon: Satellite },
  { id: 'visual-matching', title: '下视视觉质量', subtitle: 'Visual Matching', icon: Eye },
  { id: 'filter-health', title: '滤波器健康度', subtitle: 'Filter Health', icon: HeartPulse },
  { id: 'vibration-spectrum', title: '振动频谱', subtitle: 'Vibration Spectrum', icon: Waves },
]

interface PresetPanelProps {
  activePreset: PresetId
  onSelect: (preset: PresetId) => void
}

export function PresetPanel({ activePreset, onSelect }: PresetPanelProps) {
  return (
    <aside className="left-panel">
      <div className="panel-heading">
        <span>一键诊断</span>
        <button type="button" className="mini-button" onClick={() => onSelect('free')}>
          自由分析
        </button>
      </div>
      <nav className="preset-list">
        {presets.map((preset) => {
          const Icon = preset.icon
          return (
            <button
              type="button"
              key={preset.id}
              className={`preset-item ${activePreset === preset.id ? 'preset-item--active' : ''}`}
              onClick={() => onSelect(preset.id)}
            >
              <Icon size={17} />
              <span>
                <strong>{preset.title}</strong>
                <small>{preset.subtitle}</small>
              </span>
            </button>
          )
        })}
      </nav>
    </aside>
  )
}
