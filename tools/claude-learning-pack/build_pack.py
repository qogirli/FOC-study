"""Read-only source collection; writes only a new output folder and build receipts."""
from pathlib import Path
import hashlib, io, json, re, shutil, subprocess, sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'build/deps'))
from pypdf import PdfReader, PdfWriter
import pymupdf as fitz
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ROOT = Path(r'C:\百度网盘\硬石F407-PRO资料 -压缩包版')
OUT = Path(r'C:\Users\33192\Desktop\FOC-Claude学习资料包-20261012')
WORK = HERE / 'build'
MD = Path(r'C:\Users\33192\Documents\my-cloud\电机\PaddleOCR-硬石YS-F4Pro_20241025\standard_md')
FILES = [Path(p) for p in subprocess.check_output(['rg','--files',str(ROOT)], timeout=30).decode('utf-8').splitlines()]
manifest, source_records, pdf_checks = [], [], []
MAX_BYTES = 29_000_000

def digest(data): return hashlib.sha256(data).hexdigest()
def filehash(p): return digest(p.read_bytes())
def find(name, hint=''):
    found=sorted((p for p in FILES if p.name == name and hint in str(p)),key=lambda p:(len(str(p)),str(p)))
    if not found: raise FileNotFoundError((name,hint))
    return found[0]
def record_source(p):
    r={'path':str(p),'sha256':filehash(p)}
    source_records.append(r)
    return r
def write_text(name, content, sources=(), note=''):
    p=OUT / (name+'.txt')
    if p.exists():
        assert p.read_text(encoding='utf-8')==content.replace('\r\n','\n'), 'Refuse to replace different existing file: '+str(p)
    else:p.write_text(content,encoding='utf-8',newline='\n')
    assert p.read_text(encoding='utf-8') == content.replace('\r\n','\n')
    manifest.append({'file':p.name,'bytes':p.stat().st_size,'sha256':filehash(p),'sources':list(sources),'note':note})
    return p
def decoded(p):
    raw=p.read_bytes()
    for enc in ('utf-8-sig','gb18030','cp1252'):
        try: return raw.decode(enc), enc
        except UnicodeDecodeError: pass
    raise ValueError('Cannot decode source without data loss: '+str(p))
def pack_code(label, root, files, note):
    chunks=[]; sources=[]; part=1
    def flush():
        nonlocal chunks,sources,part
        if not chunks:return
        text=f'# {label} / 第{part}册\n\n{note}\n来源根目录：{root}\n编码统一为UTF-8，文件边界和源码行序保留。只读学习快照，不是可编译工程。\n\n'+''.join(chunks)
        write_text(f'{label}_{part:02}',text,sources,note)
        chunks=[];sources=[];part+=1
    for p in sorted(files):
        s,enc=decoded(p)
        record_source(p)
        rel=p.relative_to(root).as_posix()
        block=f'\n===== BEGIN FILE: {rel} =====\n原始编码：{enc}；SHA256：{filehash(p)}\n\n{s}\n===== END FILE: {rel} =====\n'
        if chunks and sum(len(x.encode('utf-8')) for x in chunks)+len(block.encode('utf-8'))>900_000: flush()
        chunks.append(block);sources.append(str(p))
    flush()
def rgfiles(root):
    return [Path(p) for p in subprocess.check_output(['rg','--files',str(root)],timeout=30).decode('utf-8').splitlines()]

def pdf_parts(src,label,start=1,end=None):
    record_source(src)
    reader=fitz.open(src);end=end or len(reader)
    assert 1<=start<=end<=len(reader)
    def piece(a,b):
        dest=OUT/f'{label}_P{a:04}-{b:04}.pdf'
        if dest.exists():raw=dest.read_bytes()
        else:
            w=fitz.open();w.insert_pdf(reader,from_page=a-1,to_page=b-1,links=False)
            raw=w.tobytes(garbage=3,deflate=True);w.close()
        if len(raw)>=MAX_BYTES:
            if a==b:raise ValueError('Single page too large: '+str(src))
            mid=(a+b)//2;piece(a,mid);piece(mid+1,b);return
        if not dest.exists():dest.write_bytes(raw)
        check=fitz.open(dest)
        assert len(check)==b-a+1
        for j,page in enumerate(check):
            old=reader[a-1+j]
            assert page.rect==old.rect
            assert page.get_text()==old.get_text(), (str(src),a+j)
        check.close()
        manifest.append({'file':dest.name,'bytes':len(raw),'sha256':filehash(dest),'sources':[str(src)],'pages':[a,b],'note':'原版图文页，页码为源PDF物理页码，未OCR或重新排版'})
        pdf_checks.append({'source':str(src),'output':str(dest),'start':a,'end':b})
    for a in range(start,end+1,80):piece(a,min(a+79,end))
    reader.close()

def extract_pdf_text(src,label,ranges):
    reader=PdfReader(src)
    for title,a,b in ranges:
        cached=OUT/(label+'_'+title+'.txt')
        if cached.exists():
            text=cached.read_text(encoding='utf-8')
            assert f'来源：{src}' in text
            assert [int(v) for v in re.findall(r'^## 原PDF第(\d+)页$',text,re.M)]==list(range(a,b+1))
            write_text(label+'_'+title,text,[str(src)],f'原PDF物理页{a}-{b}，文本层提取')
            continue
        text=f'# {title}\n\n来源：{src}\n直接提取原PDF文本层，未使用OCR。公式、矩阵和版面顺序可能丢失，须对照同页原图，不能当作已校对Markdown。\n'
        for i in range(a-1,b): text+=f'\n\n## 原PDF第{i+1}页\n\n'+(reader.pages[i].extract_text() or '[本页没有可提取文字]')
        write_text(label+'_'+title,text,[str(src)],f'原PDF物理页{a}-{b}，文本层提取')

def md_chapters():
    # Only reuse existing OCR results. This script never invokes an OCR engine.
    groups=[('10_现成Markdown_外设',[10,12,14,15,17,19,20,21,22,23]),('11_现成Markdown_电机',[40,43,47,48,50])]
    for label,nums in groups:
        selected=[p for p in sorted((MD/'chapters').glob('*.md')) if int(p.name[:2]) in nums]
        images=[];image_idx={};texts=[]
        for p in selected:
            s=p.read_text(encoding='utf-8');record_source(p)
            def image_link(m):
                path=(p.parent/m.group(1)).resolve()
                if not path.exists():raise FileNotFoundError(path)
                if path not in image_idx:image_idx[path]=len(images)+1;images.append(path)
                n=image_idx[path];volume=(n-1)//60+1;page=(n-1)%60+1
                return f'[配图：{label}_配图{volume:02}.pdf 第{page}页；原图 {path.name}]'
            s=re.sub(r'!\[[^\]]*\]\(([^)]+)\)',image_link,s)
            texts.append(f'\n\n===== 原Markdown：{p.name} =====\n{s}')
        write_text(label,'# 现成Markdown章节合订\n正文复用现有standard_md，不重新OCR。仅将本地图片链接换成随包配图册的页码。保留原始页码、公式及表格。OCR原有误识别可能仍存在，特别是引脚表。\n'+''.join(texts),[str(p) for p in selected])
        for start in range(0,len(images),60):
            dest=OUT/f'{label}_配图{start//60+1:02}.pdf'
            cn=canvas.Canvas(str(dest),pagesize=(840,650))
            for n,img in enumerate(images[start:start+60],start+1):
                record_source(img)
                cn.setFont('Helvetica',9);cn.drawString(18,632,f'Image {n} | {img.parent.name}/{img.name}')
                ir=ImageReader(str(img));w,h=ir.getSize();ratio=min(800/w,590/h)
                cn.drawImage(ir,(840-w*ratio)/2,24+(590-h*ratio)/2,w*ratio,h*ratio,mask='auto')
                cn.showPage()
            cn.save()
            assert len(PdfReader(dest).pages)==len(images[start:start+60])
            manifest.append({'file':dest.name,'bytes':dest.stat().st_size,'sha256':filehash(dest),'sources':[str(p) for p in images[start:start+60]],'note':'现成Markdown关联原图，一图一页；非新OCR'})
    pack_code('12_Markdown转换质量说明',MD/'qa',[MD/'qa/manual_review_summary.md'], '已有OCR复核记录，不代表全部内容均已人工验收。')

def sources():
    for dirname,label in [('260921-current-sampling','20_当前工程'),('260901-openloop-svpwm','21_历史开环参考'),('260818-hall-six-step','22_历史霍尔六步参考')]:
        root=REPO/dirname
        selected=[p for p in rgfiles(root) if p.suffix.lower() in ('.c','.h','.s','.ioc','.uvprojx','.md') and 'Drivers' not in p.parts]
        pack_code(label,root,selected,'当前工程以20开头文件为准。21/22仅用于回顾，不覆盖当前项目。导出包含未提交的工作区内容。')
    root=REPO/'260921-current-sampling'
    halnames={'stm32f4xx_hal'+s+e for s in ('','_adc','_adc_ex','_tim','_tim_ex','_gpio','_rcc','_rcc_ex','_cortex','_dma','_dma_ex','_uart','_pwr','_pwr_ex','_flash','_flash_ex','_def') for e in ('.c','.h')}
    pack_code('23_当前HAL外设源码',root,[p for p in rgfiles(root/'Drivers') if p.name in halnames], '当前工程附带HAL版本的相关外设源码，不用于替代教程MCSDK配套HAL版本。')
    base=ROOT/'YS-F4Pro参考例程'/'16.FOC矢量控制例程'/'16.FOC矢量控制例程'/'16.FOC矢量控制例程'/'硬石YS-F4Pro控制例程_矢量控制FOC'
    libraries={}
    for no,title in [(620,'SVPWM基础'),(622,'SVPWM原理验证'),(655,'霍尔FOC_5.2_主参考'),(654,'编码器FOC_5.2'),(652,'无感FOC_5.2'),(657,'编码器位置环_5.4.3')]:
        root=next(p for p in base.iterdir() if p.is_dir() and p.name.startswith(f'YSF4_HAL_MOTOR-{no}.'))
        app=[]
        for p in rgfiles(root):
            if p.suffix.lower() not in ('.c','.h','.s','.ioc','.uvprojx','.xml'):continue
            rel=p.relative_to(root).as_posix()
            if '/MCLib/' in rel and any(t in rel for t in ('/Any/','/F4xx/')):
                version='5.4.3' if '5.4.3' in rel else '5.2'
                key=(version,rel.split('/MCLib/',1)[1],filehash(p))
                libraries.setdefault(key,p)
            elif any(t in rel for t in ('Drivers/','CMSIS/','MCSDK','Middlewares/')):continue
            elif any(t.lower() in rel.lower() for t in ('font','picture','image','bitmap')):continue
            else:app.append(p)
        pack_code(f'30_例程{no}_{title}',root,app,'硬石原始参考例程。应用层/配置/启动文件合订，通用HAL和显示字库未收入。MCSDK库另见31开头文件；不同库版本不能混搭。')
    for version in ('5.2','5.4.3'):
        pp=[p for key,p in libraries.items() if key[0]==version]
        pack_code('31_MCSDK_'+version+'_Any_F4xx',ROOT,pp,'保留Any及F4xx电机控制库源码，按版本、相对路径及内容哈希去重。包含采样、PI、变换、霍尔、编码器、观测器、启动及保护等模块。')
    motors=[p for p in rgfiles(base/'开发板电机参数') if p.name.lower()=='42pmsm.xml']
    notes=[REPO/'diagnostics/motor-reference/README.md',REPO/'diagnostics/sensorless-652/README.md',REPO/'diagnostics/adc-raw-20260928/README.md',REPO/'diagnostics/tutorial-retest-20260928/README.md']
    pack_code('40_电机参数与既往实测记录',Path('C:/'),motors+notes,'既往记录有自己的日期和固件版本，不能冒充当前工程实测结果。')

def documents():
    book=find('硬石电机控制专题指导手册(完整版)_20200602.pdf','无刷驱动板资料')
    ranges=[('FOC基础与电流采样',337,425),('FOC算法与代码分析一',426,500),('FOC算法与代码分析二',501,574),('FOC5.2源码解释一',736,810),('FOC5.2源码解释二',811,885),('FOC5.2源码解释三',886,950),('编码器位置环及FOC5算法',951,1008)]
    extract_pdf_text(book,'13_专题手册文本补充',ranges)
    # Only the FOC-specific chapters missing from the existing Markdown need original figures.
    for title,a,b in ranges:pdf_parts(book,'14_专题原图_'+title,a,b)
    choices=[
      ('YS_F4Pro开发板—FOC控制PMSM接线方法.pdf','16.FOC','41_配套PMSM接线与参数'),
      ('YS_BLDC&PMSM（2024版本）.pdf','无刷驱动板资料','42_驱动板2024原理图'),
      ('2021年版_YS-F4Pro开发板主板原理图.pdf','尺寸图','43_主板2021_需匹配实物'),
      ('2025年版_YS-F4Pro开发板主板原理图.pdf','尺寸图','44_主板2025_需匹配实物'),
      ('YS_MOTOR_3HalfBridge.pdf','10.永磁','45_新驱动板_后续参考勿混用'),
      ('YS_42JSF840AS_DB15toMCU.pdf','10.永磁','46_编码器接线_新板参考'),
      ('42JSF840AS-1000-8 (1).pdf','10.永磁','47_42PMSM厂家参数'),
      ('YS-F4Pro FOC v5.2.0移植说明手册v1.0.pdf','16.FOC','48_FOC5.2移植说明'),
      ('永磁同步电机PI设计.pdf','','50_PI设计补充'),
      ('附件7： AN1078_PMSM 电机的无传感器磁场定向控制.pdf','无刷驱动板资料','51_无感FOC_AN1078补充'),
      ('MCP6024T-I.pdf','无刷驱动板资料','52_电流放大器MCP6024'),
      ('IR2110STRPBF.pdf','无刷驱动板资料','53_栅极驱动IR2110')]
    for name,hint,label in choices:pdf_parts(find(name,hint),label)
    rm=find('附件8：STM32F407编程参考手册（英文版）.pdf')
    for label,a,b in [('ADC',389,434),('TIM1_TIM8',511,580),('TIM2_TIM5',581,639),('DMA',303,343)]:
        pdf_parts(rm,'54_RM0090_'+label,a,b)
    for p in sorted((WORK/'slides').glob('*.pdf')):pdf_parts(p,'15_PPT图文_'+p.stem)

def build():
    if '--finish' in sys.argv:
        state=json.loads((WORK/'partial.json').read_text(encoding='utf-8'))
        manifest.extend(state['files']);source_records.extend(state['source_records'])
    else:
        if OUT.exists():raise FileExistsError('Refuse to overwrite existing package: '+str(OUT))
        OUT.mkdir();WORK.mkdir(exist_ok=True)
        write_text('00_先读_学习交接', (HERE/'handoff.txt').read_text(encoding='utf-8'),[str(HERE/'handoff.txt')])
        md_chapters();print('Markdown done',flush=True)
        sources();print('Source packs done',flush=True)
        if '--texts' in sys.argv:
            (WORK/'partial.json').write_text(json.dumps({'files':manifest,'source_records':source_records},ensure_ascii=False,indent=2),encoding='utf-8')
            return
    documents();print('Documents done',flush=True)
    # All original source hashes are checked again after collection.
    for r in source_records:
        assert filehash(Path(r['path']))==r['sha256'], 'Source changed while collecting: '+r['path']
    for r in manifest:
        assert 0<r['bytes']<MAX_BYTES,(r['file'],r['bytes'])
    index='''# 上传说明与资料索引

这是2026-10-12的本地资料快照。打开此文件夹，全选其中的文件，上传到Claude项目的Files/知识库；如果网页限制单次选择数量，可分批添加到同一项目。不是上传到普通对话里后就自动成为项目知识。

先阅读00_先读_学习交接，再看20_当前工程。当前源码、历史学习工程、硬石参考例程分别有清楚前缀。

现成Markdown内容保持标题、公式和表格，以UTF-8的.txt扩展名封装，方便项目上传。图片链接已换成同包配图册页码。不是重新OCR，也没有把正文重新转成PDF。10/11是已有Markdown，13是此前未找到Markdown的FOC专题补充，14是它的原版图文页，15是视频PPT图文。

学习顺序：当前采样/Park/观察PI → 655霍尔电流环和速度环 → 654编码器 → 657位置环 → 652无感观测器及启动。31是相应版本的MCSDK库源码，查底层函数时必读。不要因为文件全上传就一次通读全部文件。

代码只读合订包保留文件边界、相对路径、原始编码与SHA256；没有加入行号干扰复制。源码没有被改写。通用HAL只有当前工程所需外设的阅读版，未打包所有ARM DSP、字体、LCD图片及工具链。因此这些文本包不是可直接编译的工程备份。

未收入：工具安装程序、视频大文件、重复工程/旧库、编译产物、Git对象、连接器凭据及无关个人笔记。现成Markdown对应开发板手册，不等于完整FOC专题手册。图册保留原图，OCR正文中已有的错误未在本次逐项重校。

硬件版本：2021/2025主板与2024/新驱动板分开标记，不宣称都适用于当前实物。电机参数差异见40。2026版电机手册与2020主参考未混合。

更新方式：本地代码改好后重新导出20组当前工程并替换项目里的旧版本；本目录不会自动同步。不要同时保留多个同名“当前”快照。

上传可用性：已核对官方文档，项目单文件30MB，本包按更保守的29MB检查，图文PDF每册不超过80页。项目总体上下文容量仍受账号/检索功能约束；本次没有登录Claude，不声称已经实际上传通过。
官方说明：https://support.claude.com/en/articles/8241126-upload-files-to-claude

文件清单（源PDF页码是物理页码）：
'''
    for r in manifest:index+=f"\n- {r['file']} | {r['bytes']/1e6:.2f} MB | {r.get('note','')}\n"
    write_text('01_上传说明与目录',index)
    (WORK/'manifest.json').write_text(json.dumps({'output':str(OUT),'files':manifest,'source_records':source_records,'pdf_checks':pdf_checks},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'folder':str(OUT),'files':len(manifest),'MB':sum(r['bytes'] for r in manifest)/1e6,'max_MB':max(r['bytes'] for r in manifest)/1e6,'sources_checked':len(source_records)},ensure_ascii=False),flush=True)

if __name__=='__main__':build()
