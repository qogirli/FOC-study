const fs=require('fs'),path=require('path');
const source='C:/百度网盘/硬石F407-PRO资料 -压缩包版/YS-F4Pro参考例程/16.FOC矢量控制例程/16.FOC矢量控制例程/16.FOC矢量控制例程/硬石YS-F4Pro控制例程_矢量控制FOC/YSF4_HAL_MOTOR-652. FOC_v5.2.0_42PMSM_Sensorless/YSF4_HAL_MOTOR-652. FOC_v5.2.0_42PMSM_Sensorless';
const dest=__dirname+'/project';
if(fs.existsSync(dest)&&fs.readdirSync(dest).length)throw Error('Refusing to overwrite nonempty test project');
const copy=require('child_process').spawnSync('robocopy',[source,dest,'/E','/R:0','/W:0','/NFL','/NDL'],{encoding:'utf8',timeout:60000});
if(copy.error||copy.status>7)throw copy.error||Error(copy.stdout);
// Mechanical encoding normalization only, allowing precise UTF-8 patches in the copy.
for(const rel of ['Src/main.c','Src/stm32f4xx_mc_it.c','Src/ui_task.c','Inc/drive_parameters.h']){
 const b=fs.readFileSync(dest+'/'+rel);let s;
 try{s=new TextDecoder('utf-8',{fatal:true}).decode(b);}catch{s=new TextDecoder('gb18030').decode(b);}
 fs.writeFileSync(dest+'/'+rel,s);
}
fs.mkdirSync(__dirname+'/evidence',{recursive:true});
// Reapply the exact bounded-test overlay tracked alongside this script.
for(const rel of ['Src/main.c','Src/stm32f4xx_mc_it.c','Inc/pmsm_motor_parameters.h','Inc/diag_test.h']) {
 fs.copyFileSync(__dirname+'/overlay/'+rel,dest+'/'+rel);
}
console.log(dest);
