import { Download, EyeOff, Palette, Scissors, Sigma, Trash2, Waves } from 'lucide-react'

export type CurveAction = 'relative' | 'smooth' | 'normalize' | 'hide' | 'remove' | 'color' | 'export'

interface ContextMenuProps {
  x: number
  y: number
  open: boolean
  onClose: () => void
  onAction: (action: CurveAction) => void
}

const actions: Array<{ id: CurveAction; label: string; icon: typeof Scissors }> = [
  { id: 'relative', label: '减初值', icon: Scissors },
  { id: 'smooth', label: '平滑', icon: Waves },
  { id: 'normalize', label: '归一化', icon: Sigma },
  { id: 'hide', label: '隐藏', icon: EyeOff },
  { id: 'remove', label: '删除', icon: Trash2 },
  { id: 'color', label: '改颜色', icon: Palette },
  { id: 'export', label: '导出该曲线', icon: Download },
]

export function ContextMenu({ x, y, open, onClose, onAction }: ContextMenuProps) {
  if (!open) return null
  return (
    <div className="context-menu-backdrop" onClick={onClose}>
      <menu className="context-menu" style={{ left: x, top: y }} onClick={(event) => event.stopPropagation()}>
        {actions.map((action) => {
          const Icon = action.icon
          return (
            <button
              type="button"
              key={action.id}
              onClick={() => {
                onAction(action.id)
                onClose()
              }}
            >
              <Icon size={14} />
              {action.label}
            </button>
          )
        })}
      </menu>
    </div>
  )
}
