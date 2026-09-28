const fs=require('fs'),crypto=require('crypto');
const out=__dirname+'/evidence',b=fs.readFileSync(out+'/rows.bin');
const rows=[];
for(let o=0;o+32<=b.length;o+=32){
 if(!b.readUInt32LE(o))break;
 const r={ms:b.readUInt32LE(o),state:b.readUInt32LE(o+4),faults:b.readUInt32LE(o+8),hall_edges:b.readUInt32LE(o+12)};
 ['rpm','iq','id','iu','iv','bus_v','temp_c','reliable'].forEach((k,i)=>r[k]=b.readInt16LE(o+16+i*2));
 rows.push(r);
}
const run=rows.filter(r=>r.state===6),steady=run.filter(r=>r.ms>=3000&&r.ms<5000);
const stats=a=>({min:Math.min(...a),max:Math.max(...a),mean:a.reduce((s,x)=>s+x,0)/a.length});
const first=steady[0],last=steady.at(-1);
const summary={
 rows:rows.length, transitions:rows.filter((r,i)=>i===0||r.state!==rows[i-1].state),
 run_start_ms:run[0].ms,run_last_sample_ms:run.at(-1).ms,
 run_reliable:run.every(r=>r.reliable===1),any_fault:rows.some(r=>r.faults!==0),
 steady_observer_rpm:stats(steady.map(r=>r.rpm)),
 steady_hall_rpm:(last.hall_edges-first.hall_edges)*60000/((last.ms-first.ms)*6*4),
 steady_iq_raw:stats(steady.map(r=>r.iq)),steady_id_raw:stats(steady.map(r=>r.id)),
 bus_volts:stats(rows.map(r=>r.bus_v)),temperature_c:stats(rows.map(r=>r.temp_c)),
 flash_before_sha256:crypto.createHash('sha256').update(fs.readFileSync(out+'/before.bin')).digest('hex'),
 flash_restored_sha256:crypto.createHash('sha256').update(fs.readFileSync(out+'/restored.bin')).digest('hex'),
};
fs.writeFileSync(out+'/summary.json',JSON.stringify(summary,null,2));
const keys=Object.keys(rows[0]);fs.writeFileSync(out+'/timeline.csv',keys.join(',')+'\n'+rows.map(r=>keys.map(k=>r[k]).join(',')).join('\n')+'\n');
console.log(summary);
