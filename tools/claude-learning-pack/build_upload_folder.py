"""Create a curated upload folder; never merge sources or modify originals."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
SOURCE = Path('C:/Users/33192/Desktop/FOC-Claude学习资料包-20261012')
OUT = Path('C:/Users/33192/Desktop/FOC-Claude-全部上传')
assert not OUT.exists(), 'Refuse to overwrite an existing delivery'
records = []

def files(folder):
    return [Path(x) for x in subprocess.check_output(
        ['rg', '--files', str(folder)], timeout=30).decode('utf-8').splitlines()]

def copy(src, rel):
    dst = OUT / rel
    assert dst.resolve().is_relative_to(OUT.resolve())
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.exists():
        shutil.copy2(src, dst)
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    assert digest == hashlib.sha256(dst.read_bytes()).hexdigest()
    records.append({'source': str(src), 'file': dst.relative_to(OUT).as_posix(), 'sha256': digest})

# The editable application, not thousands of repeated HAL/CMSIS files.
current = REPO / '260921-current-sampling'
for p in files(current / 'Core'):
    if p.suffix in {'.c', '.h'}:
        copy(p, Path('01_当前工程/260921-current-sampling') / p.relative_to(current))
for p in current.glob('*.ioc'):
    copy(p, Path('01_当前工程/260921-current-sampling') / p.name)
copy(HERE / 'handoff.txt', Path('00_学习交接.txt'))

reference = SOURCE / '03_硬石参考例程'
common = {'main', 'mc_tasks', 'mc_config', 'mc_parameters', 'drive_parameters',
          'pmsm_motor_parameters', 'power_stage_parameters', 'parameters_conversion'}
modules = {
    620: {'main'}, 622: {'main'},
    655: common | {'mc_math', 'pid_regulator', 'hall_speed_pos_fdbk',
                  'r3_2_f4xx_pwm_curr_fdbk', 'r3_2_f4xx_pwm_curr_fdbk_class',
                  'pwm_curr_fdbk', 'speed_torq_ctrl', 'revup_ctrl'},
    654: {'mc_tasks', 'drive_parameters', 'encoder_speed_pos_fdbk', 'enc_align_ctrl'},
    657: {'mc_tasks', 'drive_parameters', 'trajectory_ctrl'},
    652: {'mc_tasks', 'drive_parameters', 'sto_pll_speed_pos_fdbk', 'revup_ctrl'},
}
for number, stems in modules.items():
    root = next(p for p in reference.iterdir() if p.name.startswith(f'YSF4_HAL_MOTOR-{number}.'))
    chosen = [p for p in files(root) if p.suffix in {'.c', '.h'} and p.stem in stems]
    assert chosen, number
    for p in chosen:
        copy(p, Path('02_教程参考源码') / p.relative_to(reference))

# Reuse original Markdown and keep its images and relative paths intact.
md = SOURCE / '04_教程资料/现成Markdown'
for p in sorted((md / 'chapters').glob('*.md')):
    if int(p.name[:2]) not in {21, 23, 43, 48}:
        continue
    copy(p, Path('03_教程Markdown') / p.relative_to(md))
    for link in re.findall(r'!\[[^\]]*\]\(([^)]+)\)', p.read_text(encoding='utf-8')):
        img = (p.parent / link).resolve()
        assert img.is_relative_to(md.resolve())
        copy(img, Path('03_教程Markdown') / img.relative_to(md.resolve()))

# One FOC slide deck plus targeted references, without duplicate book extracts.
docs = SOURCE / '04_教程资料/专题补充与图文'
for prefix in ['15_PPT图文_专题8 ', '50_PI设计补充_', '51_无感FOC_AN1078补充_']:
    chosen = list(docs.glob(prefix + '*.pdf'))
    assert len(chosen) == 1, prefix
    copy(chosen[0], Path('04_FOC图文教程') / chosen[0].name)
hardware = SOURCE / '05_硬件资料'
for prefix in ['41_', '42_', '43_', '44_', '47_']:
    chosen = list(hardware.glob(prefix + '*.pdf'))
    assert len(chosen) == 1, prefix
    copy(chosen[0], Path('05_硬件与电机') / chosen[0].name)
for name in ['motor-reference', 'adc-raw-20260928', 'sensorless-652']:
    copy(SOURCE / '06_参数与测试记录' / name / 'README.md',
         Path('06_实测记录') / name / 'README.md')

unique = {r['file']: r for r in records}
index = '''FOC 学习上传包（2026-10-12）

本目录内的文件全部属于本次上传范围，没有需要用户再次挑选的备用项。
这是供网页AI学习和讨论源码的精选包，不是完整可编译工程；没有打包全部HAL/CMSIS和编译产物。
源码保持独立.c/.h文件及原始相对目录，不要把参考例程覆盖到当前工程。

给接手AI：先读00_学习交接.txt，再读01_当前工程中的Core/Src/main.c和Core/Inc/main.h。
实际驱动仍是电压开环；电流PI仅观察，下一步接逆Park和电流闭环。请核对源码再提出小范围修改。
上传全部资料不意味着每轮通读全部资料；按问题定位文件，引用相对路径。不要声称未读取的文件已经读过。

资料导航：
01_当前工程：本次从本地工程复制的应用源码与CubeMX配置。源码快照不会自动同步。
02_教程参考源码：620/622为SVPWM；655为当前霍尔FOC主参考；654编码器；657位置环；652无感。
  这些是精选算法与配置文件，不是完整参考工程。MCSDK版本保留在原目录中，不要混用不同版本。
03_教程Markdown：原有高级定时器、ADC、编码器、PID章节和配图；没有重新OCR。公式有疑点核对配图。
04_FOC图文教程：硬石专题8，以及PI设计、无感补充。AN1078仅作原理补充，不是STM32移植代码。
05_硬件与电机：接线、参数、驱动板与两版主板图；2021/2025版必须匹配实物，不要混引脚。
06_实测记录：电机核对、ADC噪声和无感测试记录；旧例程通过不等于当前工程已闭环。

后续路线：霍尔电流闭环 → 速度环 → 编码器 → 位置环 → 无感。
缺失依赖时明确指出，不能把本包说成可直接编译的全工程。采样噪声和霍尔偏移仍有待验收。

文件目录（保持下列路径区分同名文件）：
'''
(OUT / '00_开始阅读.txt').write_text(index + '\n'.join(sorted(unique)) + '\n', encoding='utf-8')
all_files = [p for p in OUT.rglob('*') if p.is_file()]
for p in all_files:
    assert p.stat().st_size < 30 * 1024 * 1024, p
for p in (OUT / '03_教程Markdown/chapters').glob('*.md'):
    for link in re.findall(r'!\[[^\]]*\]\(([^)]+)\)', p.read_text(encoding='utf-8')):
        assert (p.parent / link).resolve().is_file(), (p, link)
summary = {'folder': str(OUT), 'files': len(all_files),
           'MiB': round(sum(p.stat().st_size for p in all_files)/1024**2, 2),
           'verified_source_copies': len(unique), 'upload_tested': False,
           'copies': list(unique.values())}
(HERE / 'build/upload-folder-validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k != 'copies'}, ensure_ascii=False))
