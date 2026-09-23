const fs = require('fs'), cp = require('child_process'), path = require('path');
const root = __dirname + '/655-adc-zero';
const out = root + '/evidence';
const stem = 'YSF4_FOC-014. FOC_v5.2.0_42PMSM_HALLSensor';
const binary = root + '/MDK-ARM/' + stem + '/YSF4_FOC-014. FOC_v5.2';
function run(commands, label) {
  const r = cp.spawnSync('C:/Keil_v5/ARM/Segger/JLink.exe',
    ['-device','STM32F407IG','-if','SWD','-speed','1000','-autoconnect','1'],
    {input: commands.join('\n')+'\nexit\n', encoding:'utf8',timeout:30000,windowsHide:true,maxBuffer:2000000});
  fs.writeFileSync(out+'/'+label+'.log',r.stdout||'');
  if (r.error || r.status !== 0) throw r.error || Error(r.stderr || 'JLink process failed');
  console.log(r.stdout);
  if (/Failed to|Cannot |Error while/i.test(r.stdout)) throw Error('J-Link reported a command failure');
}
const action=process.argv[2];
if(action==='backup') {
  if(fs.existsSync(out+'/original-flash.bin'))throw Error('Backup already exists');
  run(['mem32 0x40021C14 1','mem32 0x40010420 1',
       'savebin "'+out+'/original-flash.bin", 0x08000000, 0x100000'],'backup');
  if(fs.statSync(out+'/original-flash.bin').size!==1048576)throw Error('Incomplete backup');
} else if(action==='flash') {
  if(fs.statSync(out+'/original-flash.bin').size!==1048576)throw Error('Missing backup');
  run(['h','loadfile "'+binary+'.hex"','r','g','sleep 1000',
       'mem32 0x40021C00 6','mem32 0x40010400 18'],'flash-diagnostic');
} else if(action==='capture') {
  const map=fs.readFileSync(binary+'.map','utf8');
  function addr(n) {
    const m=map.match(new RegExp('\\b'+n+'\\s+(0x[0-9a-f]+)','i'));
    if(!m)throw Error('No symbol '+n);
    return m[1];
  }
  const code=Number(process.argv[3]||0);
  if(![0,4,7].includes(code))throw Error('Unsupported sampling time');
  const tag=process.argv[4]||('sample-'+code);
  const channel=process.argv[5]==='17'?'0x11':'0x6';
  run(['w4 '+addr('diag_channel_u')+', '+channel,'w4 '+addr('diag_sample_code')+', '+code,'w4 '+addr('diag_request')+', 1','sleep 700',
      'mem32 '+addr('diag_done')+' 1','mem32 '+addr('diag_error')+' 1','mem32 '+addr('diag_count')+' 1',
      'mem32 0x40021C14 1','mem32 0x40012000 20','mem32 0x40012100 20',
      'savebin "'+out+'/'+tag+'-u.bin", '+addr('diag_u')+', 0x2000',
      'savebin "'+out+'/'+tag+'-v.bin", '+addr('diag_v')+', 0x2000'],tag);
  const stats={};
  for(const phase of ['u','v']){
    const b=fs.readFileSync(out+'/'+tag+'-'+phase+'.bin');
    if(b.length!==8192)throw Error('Invalid sample file size');
    const a=Array.from({length:b.length/2},(_,i)=>b.readUInt16LE(i*2));
    const mean=a.reduce((s,x)=>s+x,0)/a.length;
    stats[phase]={n:a.length,min:Math.min(...a),max:Math.max(...a),mean,sd:Math.sqrt(a.reduce((s,x)=>s+(x-mean)**2,0)/a.length)};
  }
  fs.writeFileSync(out+'/'+tag+'-stats.json',JSON.stringify(stats,null,2));
  console.log(JSON.stringify(stats,null,2));
} else if(action==='restore'){
  run(['h','loadbin "'+out+'/original-flash.bin", 0x08000000','r','g','sleep 1000',
       'mem32 0x40021C14 1','mem32 0x40010420 1'],'restore-original');
} else throw Error('Choose backup, flash, capture, restore');
