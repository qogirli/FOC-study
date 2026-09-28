from pathlib import Path
import fitz

base=Path('C:/百度网盘/硬石F407-PRO资料 -压缩包版')
files=[base/'YS-F4Pro各类文档/1. 开发手册/3. 硬石电机接线方法(YS-F1Pro&YS-F4Pro板)_20190605.pdf',
base/'YS-F4Pro参考例程/16.FOC矢量控制例程/16.FOC矢量控制例程/16.FOC矢量控制例程/YS_F4Pro开发板—FOC控制PMSM接线方法.pdf']
out=Path(__file__).parent/'motor-reference'
out.mkdir(exist_ok=True)
for i,f in enumerate(files):
    doc=fitz.open(f)
    print('FILE',f,'PAGES',len(doc))
    for n,page in enumerate(doc):
        txt=page.get_text()
        if i == 1 or any(k in txt for k in ['JSF','PMSM','永磁']):
            print('PAGE',n+1,txt)
        if len(doc)<8 or any(k in txt for k in ['JSF','PMSM','永磁']):
            page.get_pixmap(matrix=fitz.Matrix(1.3,1.3)).save(out/f'{i}-page{n+1}.png')
f=base/'YS-F4Pro各类文档/硬石YS-F4Pro【2021版】开发板开发手册_20241025.pdf'
doc=fitz.open(f)
for n,page in enumerate(doc):
    txt=page.get_text()
    if any(k in txt.upper() for k in ['JSF','63AS','AS-1000']):
        print('MANUAL PAGE',n+1,txt)
        page.get_pixmap(matrix=fitz.Matrix(1.3,1.3)).save(out/f'manual-page{n+1}.png')
