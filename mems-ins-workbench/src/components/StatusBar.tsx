import type { CsvLoadProgress } from '../data/csvLoader'
import type { FlightDataset } from '../types'

interface StatusBarProps {
  dataset: FlightDataset | null
  progress: CsvLoadProgress
  draggingSignalName?: string
}

export function StatusBar({ dataset, progress, draggingSignalName }: StatusBarProps) {
  return (
    <footer className="status-bar">
      <span>{dataset ? '工作台就绪' : progress.message}</span>
      <span>加载 {progress.phase} · {Math.round(progress.percent)}%</span>
      <span>{draggingSignalName ? `拖拽曲线中：${draggingSignalName}` : '拖拽状态：空闲'}</span>
      <span>CSV loader / worker / streaming parser 接口已预留</span>
    </footer>
  )
}
