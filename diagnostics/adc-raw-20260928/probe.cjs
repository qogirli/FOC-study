const fs = require('fs'), cp = require('child_process'), path = require('path');
const dir=__dirname, root=path.resolve(dir,'../..');
const buildStem=root+'/260921-current-sampling/MDK-ARM/260818-start-six-foc/260818-start-six-foc';
const stem=dir+'/diagnostic';
function run(cmds,tag) {
  const r=cp.spawnSync('C:/Keil_v5/ARM/Segger/JLink.exe',
    ['-device','STM32F407IG','-if','SWD','-speed','1000','-autoconnect','1'],
    {input:cmds.join('\n')+'\nexit\n',encoding:'utf8',timeout:45000,windowsHide:true,maxBuffer:2000000});
  fs.writeFileSync(dir+'/'+tag+'.log',r.stdout||'');
  console.log(r.stdout);
  if(r.error || r.status!==0 || /Failed to|Cannot |Error while/i.test(r.stdout)) throw r.error||Error('JLink failed');
  return r.stdout;
}
function addr(n) {
  const m=fs.readFileSync(stem+'.map','utf8').match(new RegExp('\\b'+n+'\\s+(0x[0-9a-f]+)','i'));
  if(!m) throw Error('No symbol: '+n);
  return m[1];
}
const action=process.argv[2];
if(action==='build') {
  const r=cp.spawnSync('C:/Keil_v5/UV4/UV4.exe',['-b',root+'/260921-current-sampling/MDK-ARM/260818-start-six-foc.uvprojx','-o',dir+'/build.log'],{timeout:120000,windowsHide:true});
  console.log(fs.readFileSync(dir+'/build.log','utf8'));
  if(r.error || r.status!==0) throw r.error||Error('Build failed');
  if(!/\bdiag_data\s+0x/.test(fs.readFileSync(buildStem+'.map','utf8'))) throw Error('Diagnostic patch not applied');
  for(const ext of ['.hex','.map']) fs.copyFileSync(buildStem+ext,stem+ext);
} else if(action==='flash') {
  if(fs.statSync(dir+'/flash-before.bin').size!==1048576) throw Error('Missing backup');
  run(['w4 0x4001040C, 0','w4 0x40021C18, 0x02000000','h','w4 0x40021C18, 0x02000000',
    'loadfile "'+stem+'.hex"','r','g','sleep 500','mem32 0x40021C14 1'], 'flash');
} else if(action==='capture') {
  const tag=process.argv[3], mode=Number(process.argv[4]||1);
  const compare=Number(process.argv[5]||4198), sample=Number(process.argv[6]||0), channel=Number(process.argv[7]||6);
  const hex=n=>'0x'+n.toString(16);
  const outputs=process.argv[8]!=='nooutputs';
  if(!/^[a-z0-9-]+$/.test(tag) || ![1,2].includes(mode)) throw Error('Bad args');
  if(![4198,3600,2800].includes(compare)||![0,4,7].includes(sample)||![6,17].includes(channel)||(mode===2 && channel===17)) throw Error('Unsafe config');
  if(!outputs && mode!==1) throw Error('Disable-output test is stop-only');
  const log=run(['w4 0x4001040C, 0','w4 0x40021C18, 0x02000000','r','g','sleep 500',
       'w4 0x40010440, '+hex(compare),
       'w4 0x4001200C, '+hex(7<<21),'w4 0x40012010, '+hex(sample<<18),
       'w4 0x40012110, '+hex(sample<<24),
       'w4 0x40012304, 0x00810000','w4 0x40012038, '+hex(channel<<15),'sleep 100',
       ...(outputs?[]:['w4 0x40010420, 0x1000']),
       'w4 '+addr('diag_request')+', '+mode,'sleep '+(mode===2?2800:400),
       'mem32 '+addr('diag_done')+' 1','mem32 '+addr('diag_count')+' 1',
       'mem32 '+addr('adc_u_count')+' 1','mem32 '+addr('adc_v_count')+' 1',
       'mem32 0x40021C14 1','mem32 0x40010400 20',
       'savebin "'+dir+'/'+tag+'.bin", '+addr('diag_data')+', 0xC000'],tag);
  const memory=(name,value)=>new RegExp(addr(name).slice(2)+' = '+value,'i').test(log);
  if(!memory('diag_done','00000001') || !memory('diag_count','00000800')) throw Error('Incomplete capture; do not analyze this buffer');
} else if(action==='restore') {
  if(fs.statSync(dir+'/flash-before.bin').size!==1048576) throw Error('Invalid backup');
  run(['w4 0x4001040C, 0','w4 0x40021C18, 0x02000000','h','w4 0x40021C18, 0x02000000',
    'loadbin "'+dir+'/flash-before.bin", 0x08000000','r','g','sleep 1000',
    'mem32 0x40021C14 1','savebin "'+dir+'/flash-restored.bin", 0x08000000, 0x100000'],'restore');
  if(!fs.readFileSync(dir+'/flash-before.bin').equals(fs.readFileSync(dir+'/flash-restored.bin'))) throw Error('Restoration verification failed');
  console.log('FULL FLASH BYTE COMPARISON: PASS');
} else if(action==='commands') {
  run(process.argv.slice(4),process.argv[3]);
} else throw Error('Unknown action');
