import { useMemo, useState } from 'react'
import './App.css'
import { calculateStats, buildTimeSeriesOption, type SeriesInput } from './charts/timeSeries'
import { ChartPanel } from './components/ChartPanel'
import { ColumnBrowser } from './components/ColumnBrowser'
import { ContextMenu, type CurveAction } from './components/ContextMenu'
import { MetricCard } from './components/MetricCard'
import { NoticeBar } from './components/NoticeBar'
import { PresetPanel } from './components/PresetPanel'
import { StatsPopover } from './components/StatsPopover'
import { StatusBar } from './components/StatusBar'
import { Toolbar } from './components/Toolbar'
import { TopBar } from './components/TopBar'
import type { CsvLoadProgress } from './data/csvLoader'
import { generateSyntheticFlightData } from './data/syntheticFlightData'
import { HeightErrorPreset } from './presets/heightError'
import { InsStabilityPreset } from './presets/insStability'
import { PresetPlaceholder } from './presets/PresetPlaceholder'
import { TimingPreset } from './presets/timing'
import { TrajectoryAttitudePreset } from './presets/trajectoryAttitude'
import type { ActiveCurve, FlightDataset, PresetId, StatsSelection } from './types'

const initialProgress: CsvLoadProgress = {
  phase: 'done',
  loadedRows: 0,
  percent: 100,
  message: 'synthetic sortie 已加载',
}

const initialDataset = generateSyntheticFlightData()

function firstSignals(dataset: FlightDataset) {
  const candidates = Object.values(dataset.signals).filter(
    (signal) =>
      signal.semantic.quantity === 'height' &&
      signal.semantic.isDerived &&
      signal.rawName.includes('relative') &&
      ['GNSS', 'Visual-INS', 'Radar', 'Barometer', 'Derived'].includes(signal.semantic.source),
  )
  return candidates.slice(0, 4).map((signal) => ({
    signalId: signal.id,
    transform: 'raw' as const,
    hidden: false,
  }))
}

function App() {
  const [dataset, setDataset] = useState<FlightDataset | null>(() => initialDataset)
  const [progress, setProgress] = useState<CsvLoadProgress>(initialProgress)
  const [activePreset, setActivePreset] = useState<PresetId>('trajectory-attitude')
  const [curves, setCurves] = useState<ActiveCurve[]>(() => firstSignals(initialDataset))
  const [stats, setStats] = useState<StatsSelection | null>(null)
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number; open: boolean }>({ x: 0, y: 0, open: false })
  const [draggingSignalId, setDraggingSignalId] = useState<string | undefined>()
  const [leftCollapsed, setLeftCollapsed] = useState(false)
  const [rightCollapsed, setRightCollapsed] = useState(false)
  const [dragActive, setDragActive] = useState(false)

  const selectedSignalIds = curves.map((curve) => curve.signalId)
  const draggingSignalName = draggingSignalId && dataset?.signals[draggingSignalId]?.semantic.displayName

  const freeInputs = useMemo<SeriesInput[]>(() => {
    if (!dataset) return []
    return curves
      .map((curve) => {
        const signal = dataset.signals[curve.signalId]
        if (!signal) return null
        return { signal, curve } satisfies SeriesInput
      })
      .filter(Boolean) as SeriesInput[]
  }, [curves, dataset])

  function reloadSynthetic() {
    setProgress({ phase: 'loading', loadedRows: 0, percent: 25, message: '生成 synthetic flight data' })
    const nextDataset = generateSyntheticFlightData()
    setDataset(nextDataset)
    setCurves(firstSignals(nextDataset))
    setStats(null)
    setActivePreset('trajectory-attitude')
    setProgress({ phase: 'done', loadedRows: 0, percent: 100, message: 'synthetic sortie 已加载' })
  }

  function clearDataset() {
    setDataset(null)
    setCurves([])
    setStats(null)
    setActivePreset('free')
    setProgress({ phase: 'idle', loadedRows: 0, percent: 0, message: '空状态：未加载架次' })
  }

  function addSignal(signalId: string) {
    if (!dataset?.signals[signalId]) return
    setCurves((prev) => {
      if (prev.some((curve) => curve.signalId === signalId)) return prev
      return [...prev, { signalId, transform: 'raw', hidden: false }]
    })
    setActivePreset('free')
  }

  function handleDrop(event: React.DragEvent<HTMLDivElement>) {
    event.preventDefault()
    const signalId = event.dataTransfer.getData('application/x-signal-id')
    if (signalId) addSignal(signalId)
    setDraggingSignalId(undefined)
    setDragActive(false)
  }

  function updateLastCurve(action: CurveAction) {
    setCurves((prev) => {
      if (!prev.length) return prev
      const last = prev[prev.length - 1]
      if (action === 'remove') return prev.slice(0, -1)
      if (action === 'hide') return prev.map((curve, index) => (index === prev.length - 1 ? { ...curve, hidden: true } : curve))
      if (action === 'relative') return prev.map((curve, index) => (index === prev.length - 1 ? { ...curve, transform: 'relative' } : curve))
      if (action === 'smooth') return prev.map((curve, index) => (index === prev.length - 1 ? { ...curve, transform: 'smooth' } : curve))
      if (action === 'normalize') return prev.map((curve, index) => (index === prev.length - 1 ? { ...curve, transform: 'normalize' } : curve))
      if (action === 'color') return prev.map((curve, index) => (index === prev.length - 1 ? { ...curve, color: '#8b5cf6' } : curve))
      if (action === 'export' && dataset?.signals[last.signalId]) {
        const signal = dataset.signals[last.signalId]
        const csv = ['time,value', ...signal.time.map((time, index) => `${time},${signal.values[index]}`)].join('\n')
        const url = URL.createObjectURL(new Blob([csv], { type: 'text/csv;charset=utf-8' }))
        const link = document.createElement('a')
        link.href = url
        link.download = `${signal.rawName}.csv`
        link.click()
        URL.revokeObjectURL(url)
      }
      return prev
    })
  }

  function renderCenter() {
    if (!dataset) {
      return (
        <div className="empty-canvas">
          <MetricCard label="空状态" value="0" unit="tables" tone="gray" />
          <button type="button" className="primary-button" onClick={reloadSynthetic}>
            加载 synthetic sortie
          </button>
        </div>
      )
    }

    if (activePreset === 'trajectory-attitude') return <TrajectoryAttitudePreset dataset={dataset} />
    if (activePreset === 'height-error') return <HeightErrorPreset dataset={dataset} />
    if (activePreset === 'timing') return <TimingPreset dataset={dataset} />
    if (activePreset === 'ins-stability') return <InsStabilityPreset dataset={dataset} />
    if (activePreset !== 'free') return <PresetPlaceholder id={activePreset} dataset={dataset} />

    return (
      <div className="free-canvas">
        <div className="metric-grid metric-grid--three">
          <MetricCard label="已加载曲线" value={curves.length} unit="signals" tone="blue" />
          <MetricCard label="异常标记" value={freeInputs.reduce((sum, input) => sum + input.signal.anomalies.length, 0)} unit="events" tone="orange" />
          <MetricCard label="Y 轴分配" value={new Set(freeInputs.map((input) => input.signal.semantic.unit)).size} unit="units" tone="green" />
        </div>
        <ChartPanel
          title="自由曲线画布"
          unit="共享时间轴 · 左/右 Y 轴自动分配"
          option={buildTimeSeriesOption(freeInputs)}
          height={520}
          onBrushRange={(range) => setStats(calculateStats(freeInputs, range))}
          onContext={(x, y) => setContextMenu({ x, y, open: true })}
        >
          <div className="legend-chips">
            {curves.map((curve) => {
              const signal = dataset.signals[curve.signalId]
              if (!signal) return null
              return (
                <button
                  type="button"
                  key={curve.signalId}
                  className={`legend-chip ${curve.hidden ? 'legend-chip--off' : ''}`}
                  onClick={() =>
                    setCurves((prev) =>
                      prev.map((item) => (item.signalId === curve.signalId ? { ...item, hidden: !item.hidden } : item)),
                    )
                  }
                >
                  <span style={{ backgroundColor: curve.color ?? signal.semantic.color }} />
                  {signal.semantic.displayName}
                </button>
              )
            })}
          </div>
        </ChartPanel>
        <StatsPopover stats={stats} onClose={() => setStats(null)} />
      </div>
    )
  }

  return (
    <div className="app-shell">
      <TopBar dataset={dataset} progress={progress} onReloadSynthetic={reloadSynthetic} onClear={clearDataset} />
      <NoticeBar warnings={dataset?.parserWarnings ?? []} progressPercent={progress.percent} phase={progress.phase} />
      <Toolbar
        dataset={dataset}
        curves={curves}
        leftCollapsed={leftCollapsed}
        rightCollapsed={rightCollapsed}
        onToggleLeft={() => setLeftCollapsed((value) => !value)}
        onToggleRight={() => setRightCollapsed((value) => !value)}
        onSetFreeMode={() => setActivePreset('free')}
      />
      <main
        className={`workspace ${leftCollapsed ? 'workspace--left-collapsed' : ''} ${rightCollapsed ? 'workspace--right-collapsed' : ''}`}
      >
        {!leftCollapsed ? <PresetPanel activePreset={activePreset} onSelect={setActivePreset} /> : null}
        <section
          className={`center-workbench ${dragActive ? 'center-workbench--drag' : ''}`}
          onDrop={handleDrop}
          onDragOver={(event) => {
            event.preventDefault()
            setDragActive(true)
          }}
          onDragLeave={() => setDragActive(false)}
        >
          {renderCenter()}
        </section>
        {!rightCollapsed ? (
          <ColumnBrowser
            dataset={dataset}
            selectedSignalIds={selectedSignalIds}
            onAddSignal={addSignal}
            onDragStart={(signalId) => {
              setDraggingSignalId(signalId)
              setDragActive(true)
            }}
          />
        ) : null}
      </main>
      <StatusBar dataset={dataset} progress={progress} draggingSignalName={draggingSignalName} />
      <ContextMenu
        open={contextMenu.open}
        x={contextMenu.x}
        y={contextMenu.y}
        onClose={() => setContextMenu((prev) => ({ ...prev, open: false }))}
        onAction={updateLastCurve}
      />
    </div>
  )
}

export default App
