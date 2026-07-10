import type { AxisKind, LineStyle, Quantity, SourceKind } from './types'

export interface QuantityRule {
  quantity: Quantity
  label: string
  unit: string
  group: string
  tokens: string[]
}

export interface SourceRule {
  source: SourceKind
  label: string
  color: string
  lineStyle: LineStyle
  tokens: string[]
}

export interface AxisRule {
  axis: AxisKind
  label: string
  tokens: string[]
}

export const sourceRules: SourceRule[] = [
  {
    source: 'GNSS',
    label: '卫导',
    color: '#1f7a4d',
    lineStyle: 'solid',
    tokens: ['gnss', 'gps', 'sat', '卫导', '卫星', 'reference', 'ref'],
  },
  {
    source: 'Visual-INS',
    label: '视觉惯导',
    color: '#2563c9',
    lineStyle: 'solid',
    tokens: ['visual_ins', 'vio', 'vision_ins', '视觉惯导', '融合视觉'],
  },
  {
    source: 'Strapdown-INS',
    label: '捷联惯导',
    color: '#d33a35',
    lineStyle: 'dashed',
    tokens: ['strapdown', 'sINS'.toLowerCase(), '捷联', 'strap'],
  },
  {
    source: 'Pure-INS',
    label: '纯惯导',
    color: '#d97706',
    lineStyle: 'dashdot',
    tokens: ['pure_ins', 'pure', '纯惯导'],
  },
  {
    source: 'Downward-Vision',
    label: '下视视觉',
    color: '#0891b2',
    lineStyle: 'solid',
    tokens: ['downvision', 'downward', 'down_view', 'downward_vision', '下视'],
  },
  {
    source: 'Barometer',
    label: '气压计',
    color: '#7c3aed',
    lineStyle: 'solid',
    tokens: ['baro', 'barometer', 'pressure', '气压'],
  },
  {
    source: 'Radar',
    label: '雷达',
    color: '#0f766e',
    lineStyle: 'solid',
    tokens: ['radar', 'rangefinder', '雷达'],
  },
  {
    source: 'Filter',
    label: '滤波器',
    color: '#475569',
    lineStyle: 'solid',
    tokens: ['filter', 'kf', 'ekf', '卡尔曼', '滤波'],
  },
  {
    source: 'IMU',
    label: 'IMU',
    color: '#334155',
    lineStyle: 'solid',
    tokens: ['imu', 'gyro', 'accel', '陀螺', '加速度'],
  },
]

export const quantityRules: QuantityRule[] = [
  { quantity: 'frequency', label: '更新频率', unit: 'Hz', group: '时序', tokens: ['rate_hz', 'frequency', 'freq', '频率'] },
  { quantity: 'time', label: '时间', unit: 's', group: '时间', tokens: ['time', 'timestamp', 't_s', '时间'] },
  { quantity: 'counter', label: '计数器', unit: 'count', group: '时序', tokens: ['counter', 'packet_id', 'frame_id', 'count', 'seq', '计数', '帧号'] },
  { quantity: 'height', label: '高度', unit: 'm', group: '高度', tokens: ['height', 'alt', 'altitude', 'up_', '_up', 'up_m', '高度', '高'] },
  { quantity: 'roll', label: '横滚', unit: 'deg', group: '姿态', tokens: ['roll', '横滚'] },
  { quantity: 'pitch', label: '俯仰', unit: 'deg', group: '姿态', tokens: ['pitch', '俯仰'] },
  { quantity: 'yaw', label: '航向', unit: 'deg', group: '姿态', tokens: ['yaw', 'heading', '航向'] },
  { quantity: 'velocity', label: '速度', unit: 'm/s', group: '速度', tokens: ['vel', 'velocity', 'speed', '速度', 'v_'] },
  { quantity: 'position', label: '位置', unit: 'm', group: '位置', tokens: ['north', 'east', 'pos', 'position', 'lat', 'lon', 'latitude', 'longitude', '位置', '经度', '纬度'] },
  { quantity: 'gyro', label: '陀螺', unit: 'rad/s', group: 'IMU', tokens: ['gyro', 'gyr', '陀螺'] },
  { quantity: 'accel', label: '加速度', unit: 'm/s²', group: 'IMU', tokens: ['accel', 'acc', '加速度'] },
  { quantity: 'temperature', label: '温度', unit: '°C', group: 'IMU', tokens: ['temp', 'temperature', '温度'] },
  { quantity: 'confidence', label: '置信度', unit: '', group: '视觉质量', tokens: ['confidence', 'quality', 'score', 'conf', '置信'] },
  { quantity: 'satellite', label: '卫星数', unit: 'count', group: 'GNSS 质量', tokens: ['satellite', 'sat_count', 'num_sat', 'sv', '卫星数'] },
  { quantity: 'status', label: '状态位', unit: '', group: '状态', tokens: ['status', 'mode', 'flag', '状态'] },
  { quantity: 'vibration', label: '振动', unit: 'g', group: 'IMU', tokens: ['vibration', 'vibe', 'rms_g', '振动'] },
]

export const axisRules: AxisRule[] = [
  { axis: 'North', label: '北向', tokens: ['north', '_n_', 'vel_n', 'pos_n', '纬度', '北'] },
  { axis: 'East', label: '东向', tokens: ['east', '_e_', 'vel_e', 'pos_e', '经度', '东'] },
  { axis: 'Up', label: '天向', tokens: ['up', '_u_', 'vel_u', 'height', 'alt', '高度', '天'] },
  { axis: 'X', label: 'X 轴', tokens: ['_x', 'x_', 'gyro_x', 'accel_x'] },
  { axis: 'Y', label: 'Y 轴', tokens: ['_y', 'y_', 'gyro_y', 'accel_y'] },
  { axis: 'Z', label: 'Z 轴', tokens: ['_z', 'z_', 'gyro_z', 'accel_z'] },
  { axis: 'Horizontal', label: '水平', tokens: ['horizontal', 'horiz', 'hor', '水平'] },
]

export const semanticColors = {
  warning: '#d33a35',
  invalid: '#94a3b8',
  selected: '#8b5cf6',
  unknown: '#64748b',
}
