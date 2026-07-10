import { Database, FolderOpen, RefreshCcw, XCircle } from 'lucide-react'
import type { CsvLoadProgress } from '../data/csvLoader'
import type { FlightDataset } from '../types'

interface TopBarProps {
  dataset: FlightDataset | null
  progress: CsvLoadProgress
  onReloadSynthetic: () => void
  onClear: () => void
}

export function TopBar({ dataset, progress, onReloadSynthetic, onClear }: TopBarProps) {
  const status = dataset?.status === 'parsed' ? '语义解析完成' : dataset?.status === 'partial' ? '部分解析' : progress.message
  const span = dataset ? `${dataset.timeSpan[0].toFixed(0)}-${dataset.timeSpan[1].toFixed(0)} s` : '--'

  return (
    <header className="topbar">
      <div className="topbar__brand">
        <Database size={18} />
        <div>
          <strong>MEMS 惯导飞行数据工作台</strong>
          <span>MEMS INS Flight Data Workbench</span>
        </div>
      </div>
      <div className="topbar__meta">
        <span>{dataset?.sortieName ?? '未加载架次'}</span>
        <span className={`status-badge status-badge--${dataset?.status ?? progress.phase}`}>{status}</span>
        <span>{span}</span>
        <span>{dataset?.sampleSummary ?? '等待数据'}</span>
        <button type="button" className="icon-button" onClick={onReloadSynthetic} title="重新生成 synthetic sortie">
          <RefreshCcw size={15} />
        </button>
        <button type="button" className="icon-button" title="预留真实 CSV 打开入口">
          <FolderOpen size={15} />
        </button>
        <button type="button" className="icon-button" onClick={onClear} title="清空架次">
          <XCircle size={15} />
        </button>
      </div>
    </header>
  )
}
