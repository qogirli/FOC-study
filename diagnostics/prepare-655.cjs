const fs = require('fs');
const path = require('path');
const source = "C:/百度网盘/硬石F407-PRO资料 -压缩包版/YS-F4Pro参考例程/16.FOC矢量控制例程/16.FOC矢量控制例程/16.FOC矢量控制例程/硬石YS-F4Pro控制例程_矢量控制FOC/YSF4_HAL_MOTOR-655. FOC_v5.2.0_PMSM_HALL_精简-FOC视频参考程序/YSF4_HAL_MOTOR-655. FOC_v5.2.0_PMSM_HALL_精简/YSF4_FOC-014. FOC_v5.2.0_42PMSM_HALLSensor";
const target = "C:/Users/33192/Desktop/FOC-study/diagnostics/655-adc-zero";
if (fs.existsSync(target)) throw new Error('Refusing to overwrite existing diagnostic directory');
fs.cpSync(source, target, {
  recursive: true,
  filter: s => !['main.c', 'stm32f4xx_mc_it.c'].includes(path.basename(s)) && !s.endsWith('.uvoptx')
});
fs.mkdirSync(target + '/evidence', {recursive: true});
fs.cpSync('C:/Users/33192/Desktop/FOC-study/260921-current-sampling', target + '/baseline-project', {recursive: true});
console.log('Tutorial copy and full baseline backup created');

