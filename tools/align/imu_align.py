"""
Use IMU velocity (Wx,Wy,Wz — common signal between exp and ref)
to find temporal alignment via cross-correlation (numpy FFT, no scipy needed).
"""
import pandas as pd
import numpy as np

BASE = r"C:\Users\baloney\Desktop\pianshang"

for fly_name in ["fly2", "fly3"]:
    print(f"\n{'='*70}")
    print(f"  {fly_name}: IMU-based temporal alignment")
    print(f"{'='*70}")

    exp = pd.read_csv(f"{BASE}\\fly2\\VINS_0_{fly_name}_gps_aligned.csv",
                      usecols=['UTC', 'WxInsE', 'WyInsE', 'WzInsE'])
    ref = pd.read_csv(f"{BASE}\\{fly_name}\\reference.csv",
                      usecols=['UTC', 'WxInsE', 'WyInsE', 'WzInsE'])

    # Downsample: every 20th row (200Hz -> 10Hz) for manageable correlation
    STEP = 20
    exp_sub = exp.iloc[::STEP].reset_index(drop=True)
    ref_sub = ref.iloc[::STEP].reset_index(drop=True)

    # Use gyro norm as 1D signal
    exp_w = exp_sub[['WxInsE','WyInsE','WzInsE']].to_numpy(dtype=np.float64)
    ref_w = ref_sub[['WxInsE','WyInsE','WzInsE']].to_numpy(dtype=np.float64)

    # Remove NaN rows
    exp_valid = ~np.any(np.isnan(exp_w), axis=1)
    ref_valid = ~np.any(np.isnan(ref_w), axis=1)
    exp_w = exp_w[exp_valid]
    ref_w = ref_w[ref_valid]

    # Also use accel for robustness
    exp_a = exp_sub[['WxInsE','WyInsE','WzInsE']].to_numpy(dtype=np.float64)
    ref_a = ref_sub[['WxInsE','WyInsE','WzInsE']].to_numpy(dtype=np.float64)
    exp_a = exp_a[exp_valid]
    ref_a = ref_a[ref_valid]
    # Actually, use gyro data as main signal (more distinctive)

    # Use gyro magnitude as 1D signal
    exp_sig = np.sqrt(np.sum(exp_w**2, axis=1))
    ref_sig = np.sqrt(np.sum(ref_w**2, axis=1))

    print(f"  Exp signal: {len(exp_sig)} pts, Ref signal: {len(ref_sig)} pts")
    print(f"  Exp range: {exp_sig.min():.4f} to {exp_sig.max():.4f}")
    print(f"  Ref range: {ref_sig.min():.4f} to {ref_sig.max():.4f}")

    # Cross-correlation via FFT
    # corr[k] = sum(exp[i] * ref[i+k]), k is lag
    # Use FFT: corr = IFFT(FFT(exp) * conj(FFT(ref_reversed)))
    n = len(exp_sig) + len(ref_sig) - 1
    # Zero-pad to power of 2 for FFT efficiency
    n_fft = 1
    while n_fft < n:
        n_fft *= 2

    # Normalize signals (remove mean for better correlation)
    exp_sig = exp_sig - exp_sig.mean()
    ref_sig = ref_sig - ref_sig.mean()

    exp_fft = np.fft.rfft(exp_sig, n=n_fft)
    ref_fft = np.fft.rfft(ref_sig[::-1], n=n_fft)  # reversed for correlation
    corr = np.fft.irfft(exp_fft * ref_fft, n=n_fft)[:n]

    # lags: negative means ref leads exp, positive means ref lags exp
    lags = np.arange(len(corr)) - (len(ref_sig) - 1)

    peak_idx = np.argmax(corr)
    best_lag_samples = lags[peak_idx]
    best_lag_sec = best_lag_samples * 0.005 * STEP  # 0.005s per sample, subsampled by STEP

    print(f"\n  Cross-correlation peak:")
    print(f"    Lag: {best_lag_samples} subsamples = {best_lag_sec:.4f} seconds")
    print(f"    Correlation: {corr[peak_idx]:.2f}")

    # Check top 5 peaks for sanity
    top5 = np.argsort(corr)[-5:][::-1]
    print(f"\n  Top 5 correlation peaks:")
    for rank, idx in enumerate(top5):
        lag = lags[idx]
        lag_sec = lag * 0.005 * STEP
        print(f"    #{rank+1}: lag={lag} samples = {lag_sec:.4f}s, corr={corr[idx]:.4f}")

    # Original-sample lag
    lag_orig = best_lag_samples * STEP
    print(f"\n  Original-sample lag: {lag_orig} (each sample = 0.005s)")
    print(f"  exp[i] corresponds to ref[i + {lag_orig}]")

    # Verify: for a random segment, compare IMU signals at this lag
    seg_start = len(exp) // 3  # 1/3 into the data
    seg_len = 200
    if 0 <= seg_start + lag_orig < len(ref) - seg_len:
        exp_seg_wx = pd.to_numeric(exp['WxInsE'].iloc[seg_start:seg_start+seg_len], errors='coerce').to_numpy()
        ref_seg_wx = pd.to_numeric(ref['WxInsE'].iloc[seg_start+lag_orig:seg_start+lag_orig+seg_len], errors='coerce').to_numpy()
        valid = ~np.isnan(exp_seg_wx) & ~np.isnan(ref_seg_wx)
        if valid.sum() > 10:
            corr_val = np.corrcoef(exp_seg_wx[valid], ref_seg_wx[valid])[0,1]
            print(f"\n  Verification (WxInsE segment at 1/3 mark): corr={corr_val:.4f}")

    # Now use this lag to pull GPS from reference
    GPS_COLS = ["UTCGps","GpsSts","LatGps","HgtGps","LonGps","VnGps","VuGps","VeGps",
                "RollGps","YawGps","PitchGps","PosxGps","PosyGps","PoszGps"]

    ref_full = pd.read_csv(f"{BASE}\\{fly_name}\\reference.csv")
    exp_full = pd.read_csv(f"{BASE}\\fly2\\VINS_0_{fly_name}_gps_aligned.csv")
    total = len(exp_full)

    print(f"\n  === Extracting GPS by IMU-time-alignment ===")
    aligned_gps = {}
    for col in GPS_COLS:
        ref_vals = ref_full[col].to_numpy()
        indices = np.arange(total) + lag_orig
        valid = (indices >= 0) & (indices < len(ref_vals))
        result = np.full(total, np.nan, dtype=object)
        result[valid] = ref_vals[indices[valid]]
        aligned_gps[col] = result
        n_valid = valid.sum()
        n_clipped = total - n_valid
        if n_clipped > 0:
            print(f"    {col}: {n_valid} mapped, {n_clipped} clipped (out of ref range)")

    # Compare against OLD alignment
    old = pd.read_csv(f"{BASE}\\{fly_name}\\old_align_{fly_name}.csv")

    print(f"\n  === IMU-aligned GPS vs OLD alignment ===")
    for col in ['LatGps','HgtGps','LonGps','VnGps','VuGps','VeGps',
                'RollGps','YawGps','PitchGps']:
        if col not in old.columns:
            continue
        av = pd.to_numeric(pd.Series(aligned_gps[col]), errors='coerce').to_numpy()
        ov = pd.to_numeric(old[col], errors='coerce').to_numpy()
        valid = ~np.isnan(av) & ~np.isnan(ov)
        diff = np.abs(av[valid] - ov[valid])
        match = diff < 1e-6
        n_match = match.sum()
        n_diff = (~match).sum()
        print(f"    {col}: match={n_match}/{valid.sum()}, diff={n_diff} "
              f"({n_diff/valid.sum()*100:.2f}%)")

print("\nDone.")
