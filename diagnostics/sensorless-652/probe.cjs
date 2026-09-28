const fs=require('fs'),cp=require('child_process');
const out=__dirname+'/evidence',project=__dirname+'/project';
const name='YSF4_FOC-013. FOC_v5.2.0_42PMSM_Sensorless';
const stem=project+'/MDK-ARM/'+name+'/YSF4_FOC-013. FOC_v5.2';
function run(commands,tag){
 const r=cp.spawnSync('C:/Keil_v5/ARM/Segger/JLink.exe',['-device','STM32F407IG','-if','SWD','-speed','1000','-autoconnect','1'],{input:commands.join('\n')+'\nexit\n',encoding:'utf8',timeout:60000,maxBuffer:4000000});
 fs.writeFileSync(out+'/'+tag+'.log',r.stdout||'');
 console.log((r.stdout||'').slice(-4000));
 if(r.error||r.status!==0||/Failed to|Cannot |Error while/.test(r.stdout))throw r.error||Error('JLink failed');
 return r.stdout;
}
function addr(n){const m=fs.readFileSync(stem+'.map','utf8').match(new RegExp('\\b'+n+'\\s+(0x[0-9a-f]+)','i'));if(!m)throw Error(n);return m[1];}
const off=['w4 0x4001040C, 0','w4 0x40021C18, 0x02000000','h','w4 0x40021C18, 0x02000000'];
const vars=['diag_ready','diag_reason','diag_active','diag_done','diag_started','diag_count','diag_hall_edges','diag_hall_invalid'];
const action=process.argv[2];
if(action==='build'){
 const r=cp.spawnSync('C:/Keil_v5/UV4/UV4.exe',['-b',project+'/MDK-ARM/'+name+'.uvprojx','-o',out+'/build.log'],{timeout:180000});
 const log=fs.readFileSync(out+'/build.log','utf8');console.log(log);
 if(r.error||!log.includes('0 Error(s)'))throw r.error||Error('Build failed');
}else if(action==='backup'){
 if(fs.existsSync(out+'/before.bin'))throw Error('Backup already exists');
 run([...off,'savebin "'+out+'/before.bin", 0x08000000, 0x100000'],'backup');
 if(fs.statSync(out+'/before.bin').size!==1048576)throw Error('Invalid backup');
}else if(action==='flash'){
 if(fs.statSync(out+'/before.bin').size!==1048576)throw Error('No backup');
 run([...off,'loadfile "'+stem+'.hex"','r','g','sleep 1500',...vars.map(n=>'mem32 '+addr(n)+' 1'),
 'mem32 '+addr('diag_preflight')+' 15','mem32 0x40021C14 1'],'flash');
}else if(action==='inspect'){
 run([...vars.map(n=>'mem32 '+addr(n)+' 1'),'mem32 '+addr('diag_preflight')+' 15',
 'mem32 '+addr('STM')+' 2','mem32 '+addr('RealBusVoltageSensorParamsM1')+' 16',
 'mem32 '+addr('TempSensorParamsM1')+' 16','mem32 0x40021C14 1'],'inspect');
}else if(action==='start'){
 if(fs.existsSync(out+'/start.log'))throw Error('One attempt only');
 run(['w4 '+addr('diag_request')+', 0x652A18','sleep 5800',...vars.map(n=>'mem32 '+addr(n)+' 1'),
 'mem32 0x40021C14 1','savebin "'+out+'/rows.bin", '+addr('diag_rows')+', 0x5780'], 'start');
}else if(action==='restore'){
 run([...off,'loadbin "'+out+'/before.bin", 0x08000000','savebin "'+out+'/restored.bin", 0x08000000, 0x100000','r','g','sleep 1500','mem32 0x40021C14 1'],'restore');
 if(!fs.readFileSync(out+'/before.bin').equals(fs.readFileSync(out+'/restored.bin')))throw Error('Flash mismatch');
 console.log('FLASH MATCH PASS');
}else throw Error('Unknown action');
