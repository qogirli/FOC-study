from pathlib import Path
import hashlib,json,re,subprocess
from pypdf import PdfReader

HERE=Path(__file__).resolve().parent
data=json.loads((HERE/'build/manifest.json').read_text(encoding='utf-8'))
out=Path(data['output'])
expected={r['file'] for r in data['files']}
assert {p.name for p in out.iterdir()}==expected
page_counts={}
def pages(p):
    key=str(p)
    if key not in page_counts:page_counts[key]=len(PdfReader(p).pages)
    return page_counts[key]
for r in data['files']:
    p=out/r['file']
    assert hashlib.sha256(p.read_bytes()).hexdigest()==r['sha256']
    assert p.stat().st_size<29_000_000
    if p.suffix=='.pdf':assert 0<pages(p)<=80
    else:
        s=p.read_text(encoding='utf-8')
        assert '\x00' not in s,p.name
        # Check every remapped Markdown image reference resolves within the pack.
        for name,page in re.findall(r'配图：([^\n]+?\.pdf) 第(\d+)页',s):
            assert (out/name).exists(),name
            assert int(page)<=pages(out/name)
        assert s.count('===== BEGIN FILE:')==s.count('===== END FILE:')

source_entries=[]
for p in out.glob('*.txt'):
    s=p.read_text(encoding='utf-8')
    source_entries+=re.findall(r'===== BEGIN FILE: (.*?) =====',s)
required=['Core/Src/main.c','Core/Src/adc.c','Core/Src/tim.c','Core/Src/gpio.c','Core/Src/stm32f4xx_it.c','Core/Inc/main.h']
for name in required:assert name in source_entries,name
for name in ['hall_speed_pos_fdbk.c','encoder_speed_pos_fdbk.c','pid_regulator.c','mc_math.c','sto_pll_speed_pos_fdbk.c']:
    assert any(p.endswith('/'+name) for p in source_entries),name
assert any('r3_2' in p.lower() and p.endswith('.c') for p in source_entries)

render=HERE/'build/preview';render.mkdir(exist_ok=True)
pdftoppm=Path(r'C:\Users\33192\.cache\codex-runtimes\codex-primary-runtime\dependencies\native\poppler\Library\bin\pdftoppm.exe')
samples=[('10_现成Markdown_外设_配图01.pdf',1),('11_现成Markdown_电机_配图01.pdf',1)]
for prefix in ['14_专题原图_FOC基础','15_PPT图文_专题8','42_驱动板2024']:
    p=next(p for p in out.glob('*.pdf') if p.name.startswith(prefix))
    samples.append((p.name,min(12,len(PdfReader(p).pages))))
for i,(name,page) in enumerate(samples):
    subprocess.run([str(pdftoppm),'-f',str(page),'-l',str(page),'-scale-to','1500','-singlefile','-png',str(out/name),str(render/f'sample{i+1}')],check=True,timeout=45,capture_output=True)
result={'files':len(expected),'pdfs':sum(r['file'].endswith('.pdf') for r in data['files']),'texts':sum(r['file'].endswith('.txt') for r in data['files']),'source_entries':len(source_entries),'total_MB':round(sum(r['bytes'] for r in data['files'])/1e6,2),'largest_MB':round(max(r['bytes'] for r in data['files'])/1e6,2),'checks':'hashes, file sizes, page counts, image links, source boundaries and required modules passed','visual_samples':samples}
(HERE/'build/validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
