from pathlib import Path
import json
import numpy as np

root=Path(__file__).parent
dtype=np.dtype([('cycles','<u4'),('seq','<u4')]+[(k,'<u2') for k in ['u','v','cnt','flags','ccr1','ccr2','ccr3','reserved']])
results={}
for f in sorted(root.glob('valid-*.bin')):
    a=np.fromfile(f,dtype=dtype)
    assert len(a)==2048
    dt=np.diff(a['cycles']).astype(np.uint32)/168.0
    seq=np.diff(a['seq'])
    stats={}
    # In moving runs remove only the slow trend to describe high frequency residuals.
    residuals=[]
    for ch in ['u','v']:
        x=a[ch].astype(float)
        trend=np.convolve(np.pad(x,(20,20),mode='edge'),np.ones(41)/41,'valid')
        e=x-trend
        residuals.append(e)
        stats[ch]={'mean':float(x.mean()),'min':int(x.min()),'max':int(x.max()),'std':float(x.std()),
                   'residual_rms_counts':float(np.std(e)),
                   'residual_abs_percentiles':np.percentile(abs(e),[50,90,95,99,99.9,100]).tolist(),
                   'residual_over50_pct':float(np.mean(abs(e)>50)*100),
                   'residual_over100_pct':float(np.mean(abs(e)>100)*100),
                   'adjacent_jump_max':float(np.max(abs(np.diff(x))))}
    stats['uv_raw_correlation']=float(np.corrcoef(a['u'],a['v'])[0,1])
    stats['uv_residual_correlation']=float(np.corrcoef(*residuals)[0,1])
    stats['interval_us']=[float(dt.min()),float(np.median(dt)),float(dt.max())]
    stats['nonunit_seq']=int(np.count_nonzero(seq!=1))
    stats['bad_interval']=int(np.count_nonzero((dt<45)|(dt>55)))
    stats['cnt_range']=[int(a['cnt'].min()),int(a['cnt'].max())]
    stats['flags']=np.unique(a['flags']).tolist()
    stats['ccr_ranges']={ch:[int(a[ch].min()),int(a[ch].max())] for ch in ['ccr1','ccr2','ccr3']}
    results[f.stem]=stats
    print(f.stem, json.dumps(stats,ensure_ascii=False))
(root/'analysis.json').write_text(json.dumps(results,indent=2),encoding='utf8')
