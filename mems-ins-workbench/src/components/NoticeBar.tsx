interface NoticeBarProps {
  warnings: string[]
  progressPercent: number
  phase: string
}

export function NoticeBar({ warnings, progressPercent, phase }: NoticeBarProps) {
  const warningText = warnings.length ? `未识别列已放入“其它”：${warnings.length} 项` : '列名语义解析规则来自集中词条字典；未知列保留在“其它”分组'

  return (
    <div className="notice-bar">
      <span>时间轴按任务起始相对秒对齐；误差仅在参考源更新时刻采样</span>
      <span>{warningText}</span>
      <span className="notice-bar__progress">{phase} {Math.round(progressPercent)}%</span>
    </div>
  )
}
