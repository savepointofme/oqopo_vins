export type Quantity =
  | 'time'
  | 'height'
  | 'roll'
  | 'pitch'
  | 'yaw'
  | 'velocity'
  | 'position'
  | 'gyro'
  | 'accel'
  | 'temperature'
  | 'counter'
  | 'confidence'
  | 'satellite'
  | 'status'
  | 'frequency'
  | 'error'
  | 'vibration'
  | 'unknown'

export type SourceKind =
  | 'GNSS'
  | 'Visual-INS'
  | 'Strapdown-INS'
  | 'Pure-INS'
  | 'Downward-Vision'
  | 'Barometer'
  | 'Radar'
  | 'Filter'
  | 'IMU'
  | 'Derived'
  | 'Unknown'

export type AxisKind = 'X' | 'Y' | 'Z' | 'North' | 'East' | 'Up' | 'Horizontal' | 'None'

export type LineStyle = 'solid' | 'dashed' | 'dashdot' | 'dotted'

export interface SemanticSignal {
  displayName: string
  rawName: string
  quantity: Quantity
  source: SourceKind
  unit: string
  axis: AxisKind
  group: string
  color: string
  lineStyle: LineStyle
  isDerived: boolean
  confidence: number
}
