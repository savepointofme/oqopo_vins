import type { FlightDataset } from '../types'

export interface CsvLoadProgress {
  phase: 'idle' | 'loading' | 'parsing' | 'semantic' | 'done' | 'failed'
  fileName?: string
  loadedRows: number
  totalRows?: number
  percent: number
  message: string
}

export type CsvProgressHandler = (progress: CsvLoadProgress) => void

export async function loadCsvSortie(_files: FileList, onProgress?: CsvProgressHandler): Promise<FlightDataset> {
  onProgress?.({
    phase: 'failed',
    loadedRows: 0,
    percent: 0,
    message: '真实 CSV 接口已预留，V1 默认使用 synthetic sortie。',
  })

  throw new Error('CSV loader interface is reserved for V1 integration.')
}
