# MEMS INS Flight Data Workbench

中文名：MEMS 惯导飞行数据交互式分析工作台。

这是一个 Windows 桌面端工程分析软件的 V1 Web 原型，使用 React + TypeScript + Vite + ECharts 实现。当前默认加载 synthetic flight data，用于演示“列名语义解析 + 右栏拖拽作图 + 一键预制诊断”的核心闭环。

## 安装与运行

```powershell
npm install
npm run dev
```

生产构建：

```powershell
npm run build
```

代码检查：

```powershell
npm run lint
```

## 已实现功能

- 三栏式工作台：左侧一键诊断、中间主画布、右侧数据表/列浏览器。
- 默认 synthetic sortie：多表、多采样率、多来源导航结果、GNSS 低频台阶、VIO 慢漂、捷联/纯惯导漂移、IMU、雷达高、气压高、视觉置信度、少量异常与未知列。
- 语义解析：将原始列名解析为 displayName、quantity、source、unit、axis、group、color、lineStyle。
- 派生列：时间/计数器差分、时间频率 Hz、高度/位置减初值 relative。
- 右栏列浏览器：搜索、按来源/物理量分组、表级展开收起、语义名、原始列名 tooltip、单位、来源、最新值、sparkline。
- 主画布：拖拽或双击列添加曲线、多曲线叠加、左/右 Y 轴表现、游标 tooltip、缩放、复位、框选统计、异常点标记、右键变换菜单。
- 预制诊断：轨迹与姿态完整页；高度与误差高保真布局；时序和惯导稳定性基础页；其它诊断保留专业占位布局。
- UI 状态：默认工作状态、空状态、加载/解析进度、拖拽状态、部分解析告警、左右面板收起。

## 代码位置

- 语义词条规则：`src/semantics/dictionary.ts`
- 语义解析器：`src/semantics/parser.ts`
- synthetic 数据：`src/data/syntheticFlightData.ts`
- 派生列与异常检测：`src/data/derivedSignals.ts`
- CSV loader 预留接口：`src/data/csvLoader.ts`
- 右栏列浏览器：`src/components/ColumnBrowser.tsx`
- 主图表面板：`src/components/ChartPanel.tsx`
- 时间序列图表工具：`src/charts/timeSeries.ts`
- 轨迹图表工具：`src/charts/trajectory.ts`
- 预制诊断：`src/presets/`
- 视觉主题：`src/styles/tokens.css` 和 `src/App.css`

## 新增数据来源或物理量

1. 在 `src/semantics/dictionary.ts` 添加 `sourceRules` 或 `quantityRules`。
2. 为来源指定稳定语义色和线型。
3. 为物理量指定默认单位、分组和列名 token。
4. 预制诊断如果需要自动选中新信号，在 `src/presets/selectors.ts` 的查询条件中按 quantity/source/axis 组合读取。

## 后续真实 CSV 接入方式

`src/data/csvLoader.ts` 已预留 `loadCsvSortie(files, onProgress)` 接口。建议后续实现：

- 使用 Web Worker 解析大 CSV，避免阻塞 UI。
- 支持 streaming parser 和分块进度回调。
- 加载后复用 `parseColumn` 生成语义信息。
- 对每张表生成 `rowCount / time span / estimatedHz` metadata。
- 复用 `appendDerivedSignals` 生成派生列。
- 对图表输入执行固定预算 downsampling。

## 已知限制

- V1 仍是本地 Web 原型，Electron/Tauri 目录尚未接入。
- ECharts 被打到单 chunk，生产构建会提示体积超过 500 kB；后续可按预制诊断动态拆分。
- 右键菜单当前作用于最后加入的自由曲线，后续应按鼠标命中的具体 series 操作。
- CSV loader 只有接口和进度状态，真实多文件导入尚未实现。
- 大数据策略已有下采样与接口预留，但还不是完整工业级 streaming/worker 管线。
