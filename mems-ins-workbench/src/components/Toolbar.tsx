import { Columns3, PanelLeftClose, PanelLeftOpen, PanelRightClose, PanelRightOpen } from 'lucide-react'
import type { ActiveCurve, FlightDataset } from '../types'

interface ToolbarProps {
  dataset: FlightDataset | null
  curves: ActiveCurve[]
  leftCollapsed: boolean
  rightCollapsed: boolean
  onToggleLeft: () => void
  onToggleRight: () => void
  onSetFreeMode: () => void
}

export function Toolbar({ dataset, curves, leftCollapsed, rightCollapsed, onToggleLeft, onToggleRight, onSetFreeMode }: ToolbarProps) {
  return (
    <div className="workspace-toolbar">
      <button type="button" className="tool-button" onClick={onToggleLeft} title={leftCollapsed ? '展开左栏' : '收起左栏'}>
        {leftCollapsed ? <PanelLeftOpen size={15} /> : <PanelLeftClose size={15} />}
      </button>
      <button type="button" className="tool-button" onClick={onSetFreeMode}>
        <Columns3 size={15} />
        自由画布
      </button>
      <div className="toolbar-divider" />
      <span className="mono">曲线 {curves.filter((curve) => !curve.hidden).length}/{curves.length}</span>
      <span className="mono">{dataset?.parserWarnings.length ? `未知列 ${dataset.parserWarnings.length}` : '语义规则 OK'}</span>
      <button type="button" className="tool-button tool-button--right" onClick={onToggleRight} title={rightCollapsed ? '展开右栏' : '收起右栏'}>
        {rightCollapsed ? <PanelRightOpen size={15} /> : <PanelRightClose size={15} />}
      </button>
    </div>
  )
}
