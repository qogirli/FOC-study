"""Add byte-identical source trees and organize the existing reading package."""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
OUT=Path(r'C:\Users\33192\Desktop\FOC-Claude学习资料包-20261012').resolve()
ROOT=Path(r'C:\百度网盘\硬石F407-PRO资料 -压缩包版')
MD=Path(r'C:\Users\33192\Documents\my-cloud\电机\PaddleOCR-硬石YS-F4Pro_20241025\standard_md')
copied=[];moved=[];projects=[]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def inside(p):
    assert p.resolve().is_relative_to(OUT),p
    return p
def copy(src,dst):
    inside(dst);dst.parent.mkdir(parents=True,exist_ok=True)
    if dst.exists():assert sha(src)==sha(dst),f'Refuse overwrite: {dst}'
    else:shutil.copy2(src,dst)
    assert sha(src)==sha(dst),src
    copied.append({'source':str(src),'destination':str(dst),'sha256':sha(dst)})
def files(p):
    return [Path(x) for x in subprocess.check_output(['rg','--files','--hidden','--no-ignore',str(p)],timeout=30).decode('utf-8').splitlines()]
def missing_project_files(p):
    out=[]
    for n in ET.parse(p).iter('FilePath'):
        value=(n.text or '').strip()
        if value and not (p.parent/value).resolve().is_file():out.append(value)
    return sorted(set(out))
def project(src,dst):
    omitted=[];count=0
    exclude={'.o','.d','.crf','.axf','.hex','.bin','.elf','.map','.lnp','.dep','.iex','.lst','.log','.uvoptx','.bak','.tmp','.exe','.zip','.7z'}
    for p in files(src):
        rel=p.relative_to(src)
        if any(x.lower() in {'.git','.vscode','__pycache__'} for x in rel.parts) or p.suffix.lower() in exclude or '.uvguix.' in p.name or p.name.endswith('.build_log.htm'):
            omitted.append(rel.as_posix());continue
        copy(p,dst/rel);count+=1
    for p in dst.rglob('*.uvprojx'):
        original=src/p.relative_to(dst)
        before=missing_project_files(original);after=missing_project_files(p)
        newly_missing=sorted(set(after)-set(before))
        assert not newly_missing,(p,newly_missing)
        projects.append({'project':str(p),'preexisting_missing':before,'new_missing':newly_missing})
    print(json.dumps({'copied_project':str(dst),'files':count,'excluded_artifacts':len(omitted)},ensure_ascii=False),flush=True)

assert OUT.is_dir()
for name in ['260921-current-sampling','260901-openloop-svpwm','260818-hall-six-step']:
    parent='01_当前工程' if name.startswith('260921') else '02_历史学习工程'
    project(REPO/name,OUT/parent/name)
base=ROOT/'YS-F4Pro参考例程'/'16.FOC矢量控制例程'/'16.FOC矢量控制例程'/'16.FOC矢量控制例程'/'硬石YS-F4Pro控制例程_矢量控制FOC'
for n in [620,622,655,654,657,652]:
    src=next(p for p in base.iterdir() if p.name.startswith(f'YSF4_HAL_MOTOR-{n}.') and p.is_dir())
    project(src,OUT/'03_硬石参考例程'/src.name)

nums={10,12,14,15,17,19,20,21,22,23,40,43,47,48,50}
mdroot=OUT/'04_教程资料'/'现成Markdown'
for p in sorted((MD/'chapters').glob('*.md')):
    if int(p.name[:2]) not in nums:continue
    dest=mdroot/p.relative_to(MD);copy(p,dest)
    for value in re.findall(r'!\[[^\]]*\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
        image=(p.parent/value).resolve()
        assert image.is_relative_to(MD.resolve())
        copy(image,mdroot/image.relative_to(MD))
        assert (dest.parent/value).resolve().is_file()
copy(MD/'qa/manual_review_summary.md',mdroot/'qa/manual_review_summary.md')

# Move only verified files created by the previous export; all targets stay in OUT.
old=json.loads((HERE/'build/manifest.json').read_text(encoding='utf-8'))
for item in old['files']:
    src=OUT/item['file'];name=src.name
    if name.startswith('00_'):dest=OUT/'00_学习交接'/name
    elif name.startswith('01_'):dest=OUT/'07_可选TXT阅读版'/'旧版目录_重分类前.txt'
    elif name.startswith(('20_','21_','22_','23_','30_','31_')):dest=OUT/'07_可选TXT阅读版'/'源码合订'/name
    elif name.startswith(('10_','11_','12_')):dest=OUT/'07_可选TXT阅读版'/'Markdown合订与图册'/name
    elif name.startswith(('13_','14_','15_','48_','50_','51_')):dest=OUT/'04_教程资料'/'专题补充与图文'/name
    elif name.startswith('40_'):dest=OUT/'06_参数与测试记录'/name
    else:dest=OUT/'05_硬件资料'/name
    inside(src);inside(dest)
    if src.exists():
        assert sha(src)==item['sha256'],f'Previously generated file changed: {src}'
        assert not dest.exists(),dest
        dest.parent.mkdir(parents=True,exist_ok=True)
        src.rename(dest)
    assert sha(dest)==item['sha256'],dest
    moved.append({'before':str(src),'after':str(dest),'sha256':sha(dest)})

for folder in ['motor-reference','sensorless-652','adc-raw-20260928','tutorial-retest-20260928']:
    src=REPO/'diagnostics'/folder/'README.md'
    copy(src,OUT/'06_参数与测试记录'/folder/'README.md')
for p in files(base/'开发板电机参数'):
    if p.name.lower()=='42pmsm.xml':copy(p,OUT/'06_参数与测试记录'/'42PMSM.xml')

totals={}
for p in OUT.rglob('*'):
    if p.is_file():
        top=p.relative_to(OUT).parts[0]
        row=totals.setdefault(top,{'files':0,'bytes':0})
        row['files']+=1;row['bytes']+=p.stat().st_size
receipt={'copied_files':len(copied),'moved_reading_files':len(moved),'project_checks':projects,'folders':totals,'copies':copied,'moves':moved}
(HERE/'build/folder-manifest.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'copied_files':len(copied),'projects_checked':len(projects),'new_missing_dependencies':0,'folders':totals},ensure_ascii=False),flush=True)
