import { axisRules, quantityRules, semanticColors, sourceRules } from './dictionary'
import type { AxisKind, Quantity, SemanticSignal, SourceKind } from './types'

const DERIVED_SUFFIX_LABELS: Record<string, string> = {
  delta: '差分',
  rate_hz: '频率',
  relative: '相对',
  smoothed: '平滑',
  normalized: '归一化',
}

function normalizeName(rawName: string) {
  return rawName.toLowerCase().replace(/[.\-\s/]+/g, '_')
}

function includesAny(normalized: string, tokens: string[]) {
  return tokens.some((token) => normalized.includes(token.toLowerCase()))
}

function detectDerivedSuffix(normalized: string) {
  if (normalized.endsWith('_delta') || normalized.includes('差分')) return 'delta'
  if (normalized.endsWith('_rate_hz') || normalized.includes('频率')) return 'rate_hz'
  if (normalized.endsWith('_relative') || normalized.includes('相对')) return 'relative'
  return ''
}

function detectUnit(rawName: string, quantity: Quantity) {
  const normalized = normalizeName(rawName)
  if (normalized.includes('rad_s')) return 'rad/s'
  if (normalized.includes('mps2') || normalized.includes('m_s2')) return 'm/s²'
  if (normalized.includes('mps') || normalized.includes('m_s')) return 'm/s'
  if (normalized.includes('deg') || normalized.includes('degree')) return 'deg'
  if (normalized.includes('_c') || normalized.includes('temp')) return '°C'
  if (normalized.includes('hz')) return 'Hz'
  if (normalized.includes('_m') || normalized.includes('meter')) return 'm'
  return quantityRules.find((rule) => rule.quantity === quantity)?.unit ?? ''
}

export function parseColumn(rawName: string): SemanticSignal {
  const normalized = normalizeName(rawName)
  const derivedSuffix = detectDerivedSuffix(normalized)

  const sourceRule =
    sourceRules.find((rule) => includesAny(normalized, rule.tokens)) ??
    (derivedSuffix
      ? {
          source: 'Derived' as SourceKind,
          label: '派生',
          color: '#8b5cf6',
          lineStyle: 'solid' as const,
          tokens: [],
        }
      : undefined)

  const quantityRule = quantityRules.find((rule) => includesAny(normalized, rule.tokens))
  const axisRule = axisRules.find((rule) => includesAny(normalized, rule.tokens))

  const quantity = quantityRule?.quantity ?? 'unknown'
  const source = sourceRule?.source ?? 'Unknown'
  const axis = axisRule?.axis ?? ('None' as AxisKind)
  const unit = derivedSuffix === 'rate_hz' ? 'Hz' : detectUnit(rawName, quantity)
  const color = sourceRule?.color ?? semanticColors.unknown
  const lineStyle = sourceRule?.lineStyle ?? 'solid'
  const group = derivedSuffix ? '派生' : (quantityRule?.group ?? '其它')

  const sourceLabel = sourceRule?.label ?? '未知来源'
  const quantityLabel = quantityRule?.label ?? rawName
  const axisLabel = axisRule && axisRule.axis !== 'None' ? axisRule.label : ''
  const suffixLabel = derivedSuffix ? DERIVED_SUFFIX_LABELS[derivedSuffix] : ''

  const displayName =
    quantity === 'unknown'
      ? rawName
      : [sourceLabel, axisLabel, quantityLabel, suffixLabel].filter(Boolean).join(' ')

  const baseConfidence = quantity === 'unknown' ? 0.2 : 0.74
  const confidence = Math.min(0.98, baseConfidence + (sourceRule ? 0.18 : 0) + (axisRule ? 0.04 : 0))

  return {
    displayName,
    rawName,
    quantity,
    source,
    unit,
    axis,
    group,
    color,
    lineStyle,
    isDerived: Boolean(derivedSuffix),
    confidence,
  }
}
