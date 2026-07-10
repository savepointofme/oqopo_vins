import { parseColumn } from '../semantics/parser'
import type { FlightDataset, FlightTable, SignalColumn } from '../types'
import { appendDerivedSignals, detectAnomalies, indexSignals } from './derivedSignals'

type NumericColumns = Record<string, number[]>

let seed = 20260627

function random() {
  seed = (1664525 * seed + 1013904223) % 4294967296
  return seed / 4294967296
}

function noise(scale = 1) {
  return (random() + random() + random() + random() - 2) * scale
}

function wrapDeg(deg: number) {
  let value = deg
  while (value > 180) value -= 360
  while (value < -180) value += 360
  return value
}

function truthAt(t: number) {
  const north = 0.78 * t + 72 * Math.sin(t / 46) + 8 * Math.sin(t / 7)
  const east = 42 * Math.sin(t / 61) + 0.22 * t + 16 * Math.cos(t / 31)
  const climb = t < 32 ? 0 : t < 92 ? (t - 32) * 1.82 : 109
  const descend = t > 318 ? Math.max(22, 109 - (t - 318) * 0.72) : climb
  const height = descend + 2.4 * Math.sin(t / 23) + 0.8 * Math.sin(t / 7.3)
  const dn = 0.78 + (72 / 46) * Math.cos(t / 46) + (8 / 7) * Math.cos(t / 7)
  const de = (42 / 61) * Math.cos(t / 61) + 0.22 - (16 / 31) * Math.sin(t / 31)
  const du = t < 32 ? 0 : t < 92 ? 1.82 : t > 318 ? -0.72 : (2.4 / 23) * Math.cos(t / 23)
  const yaw = wrapDeg((Math.atan2(de, dn) * 180) / Math.PI)
  const roll = 7.5 * Math.sin(t / 13) + 2.4 * Math.sin(t / 5.5)
  const pitch = (t > 32 && t < 92 ? 5.2 : t > 318 ? -3.8 : 1.2) + 1.1 * Math.sin(t / 17)

  return { north, east, height, velN: dn, velE: de, velU: du, yaw, roll, pitch }
}

function makeTime(duration: number, hz: number, anomalyAt?: number) {
  const values: number[] = []
  const dt = 1 / hz
  let t = 0

  while (t <= duration + 1e-6) {
    values.push(Number(t.toFixed(4)))
    if (anomalyAt && Math.abs(t - anomalyAt) < dt / 2) {
      t += dt * 7
    } else {
      t += dt
    }
  }

  return values
}

function quantize(value: number, step: number) {
  return Math.round(value / step) * step
}

function createTable(id: string, name: string, description: string, time: number[], columns: NumericColumns): FlightTable {
  const placeholder: FlightTable = {
    id,
    name,
    description,
    rowCount: time.length,
    timeStart: time[0] ?? 0,
    timeEnd: time[time.length - 1] ?? 0,
    estimatedHz: time.length > 1 ? (time.length - 1) / ((time[time.length - 1] ?? 1) - (time[0] ?? 0)) : 0,
    parseStatus: 'ok',
    columns: [],
  }

  placeholder.columns = Object.entries(columns).map(([rawName, values]) => {
    const semantic = parseColumn(rawName)
    const finite = values.filter(Number.isFinite)
    const typicalValue = finite.length ? finite[Math.floor(finite.length / 2)] : 0
    const latestValue = finite.length ? finite[finite.length - 1] : 0
    const signal: SignalColumn = {
      id: `${id}:${rawName}`,
      tableId: id,
      tableName: name,
      rawName,
      semantic,
      time,
      values,
      typicalValue,
      latestValue,
      anomalies: [],
    }
    signal.anomalies = detectAnomalies(signal)
    return signal
  })

  return appendDerivedSignals(placeholder)
}

function buildNavigationTable(duration: number) {
  const time = makeTime(duration, 10)
  const columns: NumericColumns = {
    time_s: time,
    north_visual_ins_m: [],
    east_visual_ins_m: [],
    height_visual_ins_m: [],
    vel_n_visual_ins_mps: [],
    vel_e_visual_ins_mps: [],
    vel_u_visual_ins_mps: [],
    roll_visual_ins_deg: [],
    pitch_visual_ins_deg: [],
    yaw_visual_ins_deg: [],
    north_strapdown_ins_m: [],
    east_strapdown_ins_m: [],
    height_strapdown_ins_m: [],
    roll_strapdown_ins_deg: [],
    pitch_strapdown_ins_deg: [],
    yaw_strapdown_ins_deg: [],
    north_pure_ins_m: [],
    east_pure_ins_m: [],
    height_pure_ins_m: [],
    roll_pure_ins_deg: [],
    pitch_pure_ins_deg: [],
    yaw_pure_ins_deg: [],
    nav_height_filter_m: [],
    filter_state_status: [],
  }

  for (const t of time) {
    const truth = truthAt(t)
    const visualDrift = 0.013 * t + 0.00007 * t ** 2
    const strapDrift = 0.048 * t + 0.00018 * t ** 2
    const pureDrift = 0.12 * t + 0.00042 * t ** 2
    const visualJump = t > 245 && t < 253 ? 4.2 : 0

    columns.north_visual_ins_m.push(truth.north + visualDrift + visualJump + noise(0.12))
    columns.east_visual_ins_m.push(truth.east - 0.6 * visualDrift + noise(0.12))
    columns.height_visual_ins_m.push(truth.height + 0.018 * t + noise(0.18))
    columns.vel_n_visual_ins_mps.push(truth.velN + noise(0.018))
    columns.vel_e_visual_ins_mps.push(truth.velE + noise(0.018))
    columns.vel_u_visual_ins_mps.push(truth.velU + noise(0.014))
    columns.roll_visual_ins_deg.push(truth.roll + noise(0.08))
    columns.pitch_visual_ins_deg.push(truth.pitch + noise(0.07))
    columns.yaw_visual_ins_deg.push(wrapDeg(truth.yaw + 0.012 * t + noise(0.16)))

    columns.north_strapdown_ins_m.push(truth.north + strapDrift + noise(0.18))
    columns.east_strapdown_ins_m.push(truth.east - 0.38 * strapDrift + noise(0.18))
    columns.height_strapdown_ins_m.push(truth.height + 0.04 * t + noise(0.24))
    columns.roll_strapdown_ins_deg.push(truth.roll + 0.012 * t + noise(0.11))
    columns.pitch_strapdown_ins_deg.push(truth.pitch - 0.008 * t + noise(0.1))
    columns.yaw_strapdown_ins_deg.push(wrapDeg(truth.yaw + 0.045 * t + noise(0.2)))

    columns.north_pure_ins_m.push(truth.north + pureDrift + noise(0.25))
    columns.east_pure_ins_m.push(truth.east - 0.55 * pureDrift + noise(0.25))
    columns.height_pure_ins_m.push(truth.height + 0.085 * t + noise(0.32))
    columns.roll_pure_ins_deg.push(truth.roll + 0.025 * t + noise(0.17))
    columns.pitch_pure_ins_deg.push(truth.pitch - 0.017 * t + noise(0.17))
    columns.yaw_pure_ins_deg.push(wrapDeg(truth.yaw + 0.095 * t + noise(0.3)))

    columns.nav_height_filter_m.push(truth.height + 0.015 * t + noise(0.12))
    columns.filter_state_status.push(t < 32 ? 1 : t < 92 ? 2 : t < 318 ? 3 : 4)
  }

  return createTable('nav_solution', '组合导航总表', '视觉惯导、捷联、纯惯导与滤波状态', time, columns)
}

function buildGnssTable(duration: number) {
  const time = makeTime(duration, 5, 176)
  const columns: NumericColumns = {
    time_s: time,
    north_gnss_m: [],
    east_gnss_m: [],
    up_gnss_m: [],
    height_gnss_m: [],
    vel_n_gnss_mps: [],
    vel_e_gnss_mps: [],
    vel_u_gnss_mps: [],
    satellite_count_gnss: [],
    pdop_gnss: [],
    gnss_status: [],
    gnss_packet_id: [],
    raw_unknown_voltage: [],
  }

  let packet = 4000
  for (const t of time) {
    const truth = truthAt(t)
    const outage = t > 172 && t < 178
    const precision = outage ? 2.8 : 0.46
    packet += outage && t > 175 ? 3 : 1
    columns.north_gnss_m.push(quantize(truth.north + noise(precision), 0.05))
    columns.east_gnss_m.push(quantize(truth.east + noise(precision), 0.05))
    columns.up_gnss_m.push(quantize(truth.height + noise(0.62), 0.05))
    columns.height_gnss_m.push(quantize(truth.height + noise(0.72), 0.05))
    columns.vel_n_gnss_mps.push(truth.velN + noise(0.06))
    columns.vel_e_gnss_mps.push(truth.velE + noise(0.06))
    columns.vel_u_gnss_mps.push(truth.velU + noise(0.08))
    columns.satellite_count_gnss.push(Math.round(18 - (outage ? 7 : 0) + noise(1.4)))
    columns.pdop_gnss.push(Math.max(0.8, 1.35 + (outage ? 1.8 : 0) + noise(0.12)))
    columns.gnss_status.push(outage ? 2 : 4)
    columns.gnss_packet_id.push(packet)
    columns.raw_unknown_voltage.push(23.4 + 0.15 * Math.sin(t / 42) + noise(0.03))
  }

  return createTable('gnss_receiver', '卫导接收表', '低频卫导位置、速度与质量', time, columns)
}

function buildImuTable(duration: number) {
  const time = makeTime(duration, 50, 214)
  const columns: NumericColumns = {
    time_s: time,
    gyro_x_rad_s: [],
    gyro_y_rad_s: [],
    gyro_z_rad_s: [],
    accel_x_mps2: [],
    accel_y_mps2: [],
    accel_z_mps2: [],
    imu_temp_c: [],
    vibration_rms_g: [],
    imu_counter: [],
  }

  let counter = 90000
  for (const t of time) {
    const truth = truthAt(t)
    const engineVibe = t > 25 && t < 330 ? 1 : 0.25
    counter += t > 213.8 && t < 214.4 ? 4 : 1

    columns.gyro_x_rad_s.push(0.004 + (truth.roll / 180) * 0.035 + noise(0.003) + engineVibe * 0.0014 * Math.sin(t * 31))
    columns.gyro_y_rad_s.push(-0.003 + (truth.pitch / 180) * 0.031 + noise(0.003) + engineVibe * 0.0011 * Math.sin(t * 28))
    columns.gyro_z_rad_s.push(0.006 + (truth.yaw / 180) * 0.02 + noise(0.004) + engineVibe * 0.0018 * Math.sin(t * 34))
    columns.accel_x_mps2.push(0.04 * truth.velN + noise(0.025))
    columns.accel_y_mps2.push(0.04 * truth.velE + noise(0.025))
    columns.accel_z_mps2.push(9.81 + 0.06 * truth.velU + noise(0.035))
    columns.imu_temp_c.push(28 + Math.min(17, t * 0.035) + noise(0.08))
    columns.vibration_rms_g.push(0.018 + engineVibe * (0.035 + 0.012 * Math.sin(t / 4)) + noise(0.003))
    columns.imu_counter.push(counter)
  }

  return createTable('imu_raw', '原始 IMU 表', '陀螺、加速度、温度、振动与计数器', time, columns)
}

function buildDownwardVisionTable(duration: number) {
  const time = makeTime(duration, 2)
  const columns: NumericColumns = {
    time_s: time,
    downvision_confidence: [],
    downvision_match_count: [],
    radar_height_m: [],
    baro_height_m: [],
    visual_frame_id: [],
  }

  let frame = 2000
  for (const t of time) {
    const truth = truthAt(t)
    const lowTexture = t > 238 && t < 267
    const altitudePenalty = truth.height > 92 ? 0.14 : 0
    frame += lowTexture && t > 251 && t < 252 ? 3 : 1
    columns.downvision_confidence.push(Math.max(0.04, Math.min(0.98, 0.86 - altitudePenalty - (lowTexture ? 0.48 : 0) + noise(0.04))))
    columns.downvision_match_count.push(Math.max(0, Math.round(120 - (lowTexture ? 78 : 0) - altitudePenalty * 80 + noise(8))))
    columns.radar_height_m.push(Math.max(0, truth.height + noise(0.09)))
    columns.baro_height_m.push(truth.height + 1.8 * Math.sin(t / 80) + 0.018 * t + noise(0.35))
    columns.visual_frame_id.push(frame)
  }

  return createTable('downward_vision', '下视视觉与高度表', '视觉匹配质量、雷达高和气压高', time, columns)
}

export function generateSyntheticFlightData(): FlightDataset {
  seed = 20260627
  const duration = 380
  const tables = [
    buildNavigationTable(duration),
    buildGnssTable(duration),
    buildImuTable(duration),
    buildDownwardVisionTable(duration),
  ]
  const signals = indexSignals(tables)
  const totalRows = tables.reduce((sum, table) => sum + table.rowCount, 0)
  const warnings = Object.values(signals)
    .filter((signal) => signal.semantic.quantity === 'unknown')
    .map((signal) => `未知列已保留：${signal.rawName}`)

  return {
    sortieName: 'SYN-INS-0427 / synthetic sortie',
    status: warnings.length ? 'partial' : 'parsed',
    timeSpan: [0, duration],
    sampleSummary: `${tables.length} 张表 · ${totalRows.toLocaleString()} 行 · 2-50 Hz`,
    tables,
    signals,
    generatedAt: new Date().toISOString(),
    parserWarnings: warnings,
  }
}
