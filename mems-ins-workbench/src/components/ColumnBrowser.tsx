import { ChevronDown, ChevronRight, Search } from 'lucide-react'
import { useMemo, useState } from 'react'
import type { FlightDataset, SignalColumn } from '../types'
import { ColumnRow } from './ColumnRow'

interface ColumnBrowserProps {
  dataset: FlightDataset | null
  selectedSignalIds: string[]
  onAddSignal: (signalId: string) => void
  onDragStart: (signalId: string) => void
}

type GroupMode = 'source' | 'quantity'

function groupSignals(signals: SignalColumn[], mode: GroupMode) {
  return signals.reduce<Record<string, SignalColumn[]>>((groups, signal) => {
    const key = signal.semantic.isDerived
      ? '派生'
      : mode === 'source'
        ? signal.semantic.source
        : signal.semantic.group || signal.semantic.quantity
    groups[key] ??= []
    groups[key].push(signal)
    return groups
  }, {})
}

export function ColumnBrowser({ dataset, selectedSignalIds, onAddSignal, onDragStart }: ColumnBrowserProps) {
  const [query, setQuery] = useState('')
  const [groupMode, setGroupMode] = useState<GroupMode>('source')
  const [expandedTables, setExpandedTables] = useState<Record<string, boolean>>({})

  const normalizedQuery = query.trim().toLowerCase()
  const filteredTables = useMemo(() => {
    if (!dataset) return []
    return dataset.tables.map((table) => ({
      ...table,
      columns: table.columns.filter((signal) => {
        if (!normalizedQuery) return true
        return (
          signal.semantic.displayName.toLowerCase().includes(normalizedQuery) ||
          signal.rawName.toLowerCase().includes(normalizedQuery) ||
          signal.semantic.source.toLowerCase().includes(normalizedQuery) ||
          signal.semantic.group.toLowerCase().includes(normalizedQuery)
        )
      }),
    }))
  }, [dataset, normalizedQuery])

  return (
    <aside className="right-panel">
      <div className="panel-heading">
        <span>数据表 / 列浏览器</span>
        <span className="mono">{dataset ? Object.keys(dataset.signals).length : 0} cols</span>
      </div>
      <div className="column-browser__controls">
        <label className="search-box">
          <Search size={14} />
          <input value={query} onChange={(event) => setQuery(event.target.value)} placeholder="搜索语义名 / 原始列名" />
        </label>
        <div className="segmented-control">
          <button type="button" className={groupMode === 'source' ? 'active' : ''} onClick={() => setGroupMode('source')}>
            来源
          </button>
          <button type="button" className={groupMode === 'quantity' ? 'active' : ''} onClick={() => setGroupMode('quantity')}>
            物理量
          </button>
        </div>
      </div>

      <div className="table-tree">
        {!dataset ? <div className="empty-state">未加载架次</div> : null}
        {filteredTables.map((table) => {
          const expanded = expandedTables[table.id] ?? true
          const groups = groupSignals(table.columns, groupMode)
          return (
            <section className="table-node" key={table.id}>
              <button
                type="button"
                className="table-node__header"
                onClick={() => setExpandedTables((prev) => ({ ...prev, [table.id]: !expanded }))}
              >
                {expanded ? <ChevronDown size={15} /> : <ChevronRight size={15} />}
                <span>
                  <strong>{table.name}</strong>
                  <small>
                    {table.rowCount.toLocaleString()} 行 · {table.timeStart.toFixed(0)}-{table.timeEnd.toFixed(0)} s ·{' '}
                    {table.estimatedHz.toFixed(1)} Hz
                  </small>
                </span>
              </button>
              {expanded
                ? Object.entries(groups).map(([group, signals]) => (
                    <div className="signal-group" key={group}>
                      <div className="signal-group__title">
                        <span>{group}</span>
                        <span>{signals.length}</span>
                      </div>
                      {signals.map((signal) => (
                        <ColumnRow
                          key={signal.id}
                          signal={signal}
                          selected={selectedSignalIds.includes(signal.id)}
                          onAdd={onAddSignal}
                          onDragStart={onDragStart}
                        />
                      ))}
                    </div>
                  ))
                : null}
            </section>
          )
        })}
      </div>
    </aside>
  )
}
