import json
from pathlib import Path
import numpy as np

root = Path(__file__).parent / '655-adc-zero/evidence/scope-noise-data'
a = np.load(root / 'waveform_raw.npz')
fs = float(a['sample_rate_hz'])
result = {'sample_rate_hz': fs, 'channels': {}}
for channel in [1, 2]:
    x = a[f'ch{channel}_v']
    n = len(x)
    # Average 0.1 ms blocks only for low-frequency analysis; raw data retained.
    size = round(fs * 0.0001)
    slow = x[:n // size * size].reshape(-1, size).mean(axis=1)
    t = (np.arange(len(slow)) + 0.5) * size / fs
    y = slow - slow.mean()
    freq = np.fft.rfftfreq(n, 1 / fs)
    amp = np.abs(np.fft.rfft((x-x.mean()) * np.hanning(n))) * 4 / n
    peaks = np.flatnonzero((amp[1:-1] > amp[:-2]) & (amp[1:-1] >= amp[2:])) + 1
    def top(lo, hi):
        p = peaks[(freq[peaks] >= lo) & (freq[peaks] <= hi)]
        p = p[np.argsort(amp[p])[-6:][::-1]]
        return [{'hz': float(freq[i]), 'peak_amplitude_v': float(amp[i])} for i in p]
    fits = []
    for hz in np.arange(40, 60.001, 0.05):
        design = np.column_stack([np.sin(2*np.pi*hz*t), np.cos(2*np.pi*hz*t), np.ones(len(t))])
        coeff = np.linalg.lstsq(design, slow, rcond=None)[0]
        error = np.mean((slow-design@coeff)**2)
        fits.append((error, hz, 2*np.hypot(coeff[0],coeff[1])))
    best = min(fits)
    blocks = x[:n // size * size].reshape(-1, size)
    envelope = blocks.std(axis=1)
    ef = np.fft.rfftfreq(len(envelope), size/fs)
    ea = np.abs(np.fft.rfft((envelope-envelope.mean())*np.hanning(len(envelope))))*4/len(envelope)
    ep = np.flatnonzero((ea[1:-1]>ea[:-2]) & (ea[1:-1]>=ea[2:]))+1
    ep = ep[(ef[ep]>=20)&(ef[ep]<=1000)]
    ep = ep[np.argsort(ea[ep])[-5:][::-1]]
    result['channels'][str(channel)] = {
        'n': n, 'mean_v': float(x.mean()), 'std_v': float(x.std()),
        'min_v': float(x.min()), 'max_v': float(x.max()),
        'block_mean_std_v': float(y.std()), 'fft_resolution_hz': fs/n,
        'low_frequency_peaks': top(10, 1000), 'high_frequency_peaks': top(1000, fs/2),
        'noise_envelope_peaks': [{'hz':float(ef[i]),'amplitude_v':float(ea[i])} for i in ep],
        'best_40_60hz_sine_fit': {'hz': float(best[1]), 'peak_to_peak_v': float(best[2]),
           'block_variance_explained': float(1-best[0]/np.var(slow))}}
result['raw_channel_correlation'] = float(np.corrcoef(a['ch1_v'], a['ch2_v'])[0,1])
(root / 'analysis.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
print(json.dumps(result, indent=2))
