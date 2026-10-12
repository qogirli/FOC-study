from pathlib import Path
import hashlib,json

HERE=Path(__file__).resolve().parent
OUT=Path(r'C:\Users\33192\Desktop\FOC-Claude学习资料包-20261012').resolve()
ARCHIVE=Path(r'C:\Users\33192\Desktop\FOC-Claude旧版阅读包-20261012').resolve()
src=(OUT/'07_可选TXT阅读版').resolve()
assert src.parent==OUT
assert ARCHIVE.parent==OUT.parent and ARCHIVE!=OUT and ARCHIVE.name=='FOC-Claude旧版阅读包-20261012'
if src.exists():
    assert not ARCHIVE.exists(),'Refuse to overwrite archive'
    src.rename(ARCHIVE)
assert ARCHIVE.is_dir()

receipt=json.loads((HERE/'build/folder-manifest.json').read_text(encoding='utf-8'))
for entry in receipt['copies']:
    source=Path(entry['source']);dest=Path(entry['destination'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==entry['sha256'],source
    assert hashlib.sha256(dest.read_bytes()).hexdigest()==entry['sha256'],dest
for entry in receipt['moves']:
    old=Path(entry['after'])
    dest=ARCHIVE/old.relative_to(OUT/'07_可选TXT阅读版') if old.is_relative_to(OUT/'07_可选TXT阅读版') else old
    assert hashlib.sha256(dest.read_bytes()).hexdigest()==entry['sha256'],dest
counts={}
for p in OUT.rglob('*'):
    if p.is_file():counts[p.suffix.lower()]=counts.get(p.suffix.lower(),0)+1
summary={'root':str(OUT),'archive':str(ARCHIVE),'verified_copies':len(receipt['copies']),'project_dependency_checks':len(receipt['project_checks']),'preexisting_missing_dependencies':{x['project']:x['preexisting_missing'] for x in receipt['project_checks'] if x['preexisting_missing']},'files_by_extension':counts}
(HERE/'build/folder-validation.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))
