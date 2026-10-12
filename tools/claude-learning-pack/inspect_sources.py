from pathlib import Path
import json, subprocess
from pypdf import PdfReader

ROOT = Path(r'C:\百度网盘\硬石F407-PRO资料 -压缩包版')
files = [Path(p) for p in subprocess.check_output(['rg','--files',str(ROOT)], timeout=30).decode('utf-8').splitlines()]
names = ['硬石电机控制专题指导手册(完整版)_20200602.pdf','硬石电机控制专题指导手册(完整版)_20260310.pdf','附件8：STM32F407编程参考手册（英文版）.pdf']
for name in names:
    path = next(p for p in files if p.name == name)
    r = PdfReader(path)
    print(json.dumps({'path':str(path),'pages':len(r.pages),'MB':round(path.stat().st_size/1e6,2)},ensure_ascii=False))
    for i in range(min(25,len(r.pages))):
        s = r.pages[i].extract_text() or ''
        lines=[ln for ln in s.splitlines() if (ln.strip().startswith('第') and '章' in ln) or any(t in ln for t in ['Advanced-control timers','General-purpose timers','Analog-to-digital converter','DMA controller','Reset and clock control'])]
        if lines: print('PAGE',i+1,'\n'.join(lines))
print('PPT', json.dumps([{'path':str(p),'MB':round(p.stat().st_size/1e6,2)} for p in files if p.suffix == '.pptx' and any(t in p.name for t in ['专题8','专题7','专题2','第17章','第18章'])],ensure_ascii=False))
md=Path(r'C:\Users\33192\Documents\my-cloud\电机\PaddleOCR-硬石YS-F4Pro_20241025\standard_md\chapters')
print('MD', '\n'.join(p.name for p in md.iterdir() if any(t in p.name for t in ['ADC','定时器','编码器','无刷','PID','SPWM'])))
