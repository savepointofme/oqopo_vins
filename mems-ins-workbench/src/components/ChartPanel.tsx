import * as echarts from 'echarts'
import type { EChartsOption } from 'echarts'
import { Download, Maximize2, MousePointer2, RotateCcw } from 'lucide-react'
import { useEffect, useRef } from 'react'

interface ChartPanelProps {
  title: string
  unit?: string
  option: EChartsOption
  height?: number | string
  className?: string
  onBrushRange?: (range: [number, number]) => void
  onContext?: (x: number, y: number) => void
  children?: React.ReactNode
}

export function ChartPanel({ title, unit, option, height = 320, className, onBrushRange, onContext, children }: ChartPanelProps) {
  const nodeRef = useRef<HTMLDivElement | null>(null)
  const chartRef = useRef<echarts.ECharts | null>(null)

  useEffect(() => {
    if (!nodeRef.current) return
    chartRef.current = echarts.init(nodeRef.current, undefined, { renderer: 'canvas' })

    const chart = chartRef.current
    const handleResize = () => chart.resize()
    window.addEventListener('resize', handleResize)

    if (onBrushRange) {
      chart.on('brushEnd', (params: unknown) => {
        const event = params as { areas?: Array<{ coordRange?: [number, number] }> }
        const range = event.areas?.[0]?.coordRange
        if (range) onBrushRange(range)
      })
    }

    if (onContext) {
      chart.getZr().on('contextmenu', (event) => {
        event.event.preventDefault()
        const rawEvent = event.event as MouseEvent
        onContext(rawEvent.clientX, rawEvent.clientY)
      })
    }

    return () => {
      window.removeEventListener('resize', handleResize)
      chart.dispose()
      chartRef.current = null
    }
  }, [onBrushRange, onContext])

  useEffect(() => {
    chartRef.current?.setOption(option, true)
  }, [option])

  function resetZoom() {
    chartRef.current?.dispatchAction({ type: 'dataZoom', start: 0, end: 100 })
  }

  function armBrush() {
    chartRef.current?.dispatchAction({
      type: 'takeGlobalCursor',
      key: 'brush',
      brushOption: { brushType: 'lineX', brushMode: 'single' },
    })
  }

  function exportPng() {
    const url = chartRef.current?.getDataURL({ type: 'png', pixelRatio: 2, backgroundColor: '#ffffff' })
    if (!url) return
    const link = document.createElement('a')
    link.href = url
    link.download = `${title}.png`
    link.click()
  }

  return (
    <section className={`chart-panel ${className ?? ''}`}>
      <header className="chart-panel__header">
        <div>
          <strong>{title}</strong>
          {unit ? <span className="mono">{unit}</span> : null}
        </div>
        <div className="chart-panel__tools">
          {children}
          <button type="button" className="icon-button icon-button--light" onClick={armBrush} title="框选统计">
            <MousePointer2 size={14} />
          </button>
          <button type="button" className="icon-button icon-button--light" onClick={resetZoom} title="复位缩放">
            <RotateCcw size={14} />
          </button>
          <button type="button" className="icon-button icon-button--light" onClick={exportPng} title="导出图片">
            <Download size={14} />
          </button>
          <button type="button" className="icon-button icon-button--light" onClick={() => chartRef.current?.resize()} title="刷新画布">
            <Maximize2 size={14} />
          </button>
        </div>
      </header>
      <div ref={nodeRef} className="chart-panel__canvas" style={{ height }} />
    </section>
  )
}
