const fs=require('fs'),cp=require('child_process'),path=require('path');
const out=__dirname,root=path.resolve(out,'../..');
const original='C:/百度网盘/硬石F407-PRO资料 -压缩包版/YS-F4Pro参考例程/16.FOC矢量控制例程/16.FOC矢量控制例程/16.FOC矢量控制例程/硬石YS-F4Pro控制例程_矢量控制FOC/YSF4_HAL_MOTOR-655. FOC_v5.2.0_PMSM_HALL_精简-FOC视频参考程序/YSF4_HAL_MOTOR-655. FOC_v5.2.0_PMSM_HALL_精简/YSF4_FOC-014. FOC_v5.2.0_42PMSM_HALLSensor';
const project=root+'/diagnostics/655-adc-zero';
const name='YSF4_FOC-014. FOC_v5.2.0_42PMSM_HALLSensor';
const stem=project+'/MDK-ARM/'+name+'/YSF4_FOC-014. FOC_v5.2';
function run(cmd,tag){
 const x=cp.spawnSync('C:/Keil_v5/ARM/Segger/JLink.exe',['-device','STM32F407IG','-if','SWD','-speed','1000','-autoconnect','1'],{input:cmd.join('\n')+'\nexit\n',encoding:'utf8',timeout:45000,maxBuffer:2000000});
 fs.writeFileSync(out+'/'+tag+'.log',x.stdout||'');
 if(x.error||x.status!==0||/Failed to|Cannot |Error while/.test(x.stdout))throw x.error||Error(x.stdout);
 console.log(x.stdout.slice(-2600));return x.stdout;
}
function addr(n){let m=fs.readFileSync(stem+'.map','utf8').match(new RegExp('\\b'+n+'\\s+(0x[0-9a-f]+)','i'));if(!m)throw Error(n);return m[1];}
const action=process.argv[2];
if(action==='inspect'){
 for(const args of [['-n','-C','3','InjectedSamplingTime|ClockPrescaler|MC_StartMotor1|MX_MotorControl_Init',original+'/Src/main.c'],['-n','-g','*.h','SAMPLING_TIME|SAMPLING_CYCLE|PWM_FREQUENCY',original+'/Inc']]){
  const x=cp.spawnSync('rg',args,{encoding:'utf8',timeout:10000});console.log(x.stdout);fs.appendFileSync(out+'/original-config.txt',x.stdout);
 }
}else if(action==='build'){
 const x=cp.spawnSync('C:/Keil_v5/UV4/UV4.exe',['-b',project+'/MDK-ARM/'+name+'.uvprojx','-o',out+'/build.log'],{timeout:120000});
 console.log(fs.readFileSync(out+'/build.log','utf8'));if(x.error||x.status>1)throw x.error||Error('Build failed');
}else if(action==='backup'){
 if(fs.existsSync(out+'/before.bin'))throw Error('Backup exists');
 run(['w4 0x4001040C, 0','w4 0x40021C18, 0x02000000','h','w4 0x40021C18, 0x02000000','savebin "'+out+'/before.bin", 0x08000000, 0x100000'],'backup');
 if(fs.statSync(out+'/before.bin').size!==1048576)throw Error('Invalid backup');
}else if(action==='flash'){
 if(fs.statSync(out+'/before.bin').size!==1048576)throw Error('Invalid backup');
 run(['h','w4 0x40021C18, 0x02000000','loadfile "'+stem+'.hex"','r','g','sleep 1000','mem32 0x40021C14 1'],'flash');
}else if(action==='art'){
 const mode=process.argv[3];if(!['read','off','on'].includes(mode))throw Error('Bad ART mode');
 run([...(mode==='read'?[]:['w4 0x40023C00, '+(mode==='off'?'0x605':'0x705')]),'mem32 0x40023C00 1','mem32 0x40021C14 1'],'art-'+mode);
}else if(action==='capture'){
 const code=Number(process.argv[3]),tag=process.argv[4],channel=Number(process.argv[5]||6);
 if(![0,4,7].includes(code)||![6,17].includes(channel)||!/^[a-z0-9-]+$/.test(tag))throw Error('Invalid params');
 const log=run(['w4 '+addr('diag_channel_u')+', 0x'+channel.toString(16),'w4 '+addr('diag_sample_code')+', 0x'+code.toString(16),
 'w4 '+addr('diag_request')+', 1','sleep 700','mem32 '+addr('diag_done')+' 1','mem32 '+addr('diag_error')+' 1','mem32 '+addr('diag_count')+' 1',
 'mem32 0x40021C14 1','mem32 0x40010420 1','mem32 0x4001200C 2','mem32 0x4001210C 2',
 'savebin "'+out+'/'+tag+'-u.bin", '+addr('diag_u')+', 0x2000','savebin "'+out+'/'+tag+'-v.bin", '+addr('diag_v')+', 0x2000'],tag);
 for(const [n,v]of [['diag_done','00000001'],['diag_error','00000000'],['diag_count','00001000']])if(!new RegExp(addr(n).slice(2)+' = '+v,'i').test(log))throw Error('Incomplete capture '+n);
 const stats={};for(const ch of ['u','v']){const b=fs.readFileSync(out+'/'+tag+'-'+ch+'.bin');if(b.length!==8192)throw Error('Size');let a=Array.from({length:4096},(_,i)=>b.readUInt16LE(i*2)),mean=a.reduce((s,v)=>s+v,0)/4096;stats[ch]={mean,min:Math.min(...a),max:Math.max(...a),std:Math.sqrt(a.reduce((s,v)=>s+(v-mean)**2,0)/4096)};}fs.writeFileSync(out+'/'+tag+'.json',JSON.stringify(stats,null,2));console.log(stats);
}else if(action==='restore'){
 run(['h','w4 0x40021C18, 0x02000000','loadbin "'+out+'/before.bin", 0x08000000','savebin "'+out+'/restored.bin", 0x08000000, 0x100000','r','g','sleep 1000','mem32 0x40021C14 1'],'restore');
 if(!fs.readFileSync(out+'/before.bin').equals(fs.readFileSync(out+'/restored.bin')))throw Error('Restore mismatch');console.log('FLASH MATCH PASS');
}else throw Error('Unknown action');
