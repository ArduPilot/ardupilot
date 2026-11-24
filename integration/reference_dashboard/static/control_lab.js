/* Control Lab has one shared state and no hardware write client. */
'use strict';
class SampleRing {
  constructor(capacity=6000) { this.capacity=capacity; this.clear(); }
  clear(){this.rows=new Array(this.capacity);this.start=0;this.size=0;}
  push(row){if(this.size<this.capacity)this.rows[(this.start+this.size++)%this.capacity]=row;else{this.rows[this.start]=row;this.start=(this.start+1)%this.capacity;}}
  values(){return Array.from({length:this.size},(_,i)=>this.rows[(this.start+i)%this.capacity]);}
}
const labState={axis:'roll',tab:'manual',schema:{},active:{},staged:{},revision:null,status:{},
  buffer:new SampleRing(),cursor:0,profiles:[],paused:false,frozen:[],replay:null,sessionA:null,sessionB:null};
const $=id=>document.getElementById(id);
const esc=s=>String(s??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=(n,d=4)=>typeof n==='number'&&Number.isFinite(n)?Number(n.toFixed(d)).toString():'—';
const exact=n=>typeof n==='number'&&Number.isFinite(n)?String(n):'—';
const wrap=n=>(n+180)%360<0?((n+180)%360+360)-180:(n+180)%360-180;
const fresh=(row,key,max=0.5)=>Number.isFinite(row?.[key])&&row[key]>0&&row.t-row[key]>=0&&row.t-row[key]<=max;
function announce(message,error=false){$('notice').textContent=message;$('notice').classList.toggle('error',error);$('notice').hidden=false;}
async function api(path,body){const r=await fetch('/api/control-lab/'+path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const v=await r.json();if(!r.ok)throw Error(v.error||`Request failed (${r.status})`);return v;}
function run(fn){return async(...args)=>{try{await fn(...args);}catch(e){announce(e.message,true);}};}
function table(headers,rows){return `<table><thead><tr>${headers.map(x=>`<th>${esc(x)}</th>`).join('')}</tr></thead><tbody>${rows.map(row=>'<tr>'+row.map(x=>`<td>${esc(x)}</td>`).join('')+'</tr>').join('')}</tbody></table>`;}
function download(name,data,type='application/json'){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([data],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
function displayed(){return labState.paused?labState.frozen:labState.buffer.values();}
function activeVerified(){return !labState.replay&&labState.status.readback_verified;}
function setTab(tab){labState.tab=tab;document.querySelectorAll('[data-tab]').forEach(b=>b.setAttribute('aria-selected',String(b.dataset.tab===tab)));document.querySelectorAll('[role=tabpanel]').forEach(p=>p.hidden=p.id!==`view-${tab}`);location.hash=tab;if(tab==='profiles')run(loadProfiles)();if(tab==='response')run(loadSessions)();draw();}
function setAxis(axis){labState.axis=axis;document.querySelectorAll('[data-axis]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.axis===axis)));$('axis-name').textContent=axis[0].toUpperCase()+axis.slice(1);renderEditor();charts.forEach(c=>c.reset());draw();}
function renderEditor(){const schema=Object.values(labState.schema).filter(p=>p.axes.includes(labState.axis)).sort((a,b)=>a.order-b.order);$('parameter-editor').innerHTML=['outer','inner'].map(loop=>`<div class="loop-label">${loop==='outer'?'OUTER / POSITION → VELOCITY':'INNER / VELOCITY → EFFORT'}${['roll','pitch','yaw'].includes(labState.axis)?(loop==='outer'?' · SQRT CONTROLLER':' · RATE PID'):(loop==='inner'?' · ACCELERATION PID → MASS FACTOR → EFFORT':'')}</div>`+schema.filter(p=>p.loop===loop).map(p=>{
  const active=activeVerified()?labState.active[p.name]:undefined;
  const value=Object.hasOwn(labState.staged,p.name)?labState.staged[p.name]:active;
  return `<div class="parameter-row"><div><code>${esc(p.name)}</code><small>${esc(p.unit)}${p.axes.length>1?' · shared '+esc(p.axes.join('/')):''}</small></div><span class="active-value">${exact(active)}</span><div class="input-wrap"><input aria-label="Staged ${esc(p.name)}" data-param="${esc(p.name)}" type="number" step="any" value="${value===undefined?'':esc(value)}" placeholder="—" class="${Object.hasOwn(labState.staged,p.name)?'changed':''}" title="${esc(p.range_source)}"><button class="reset-one" data-reset="${esc(p.name)}" aria-label="Reset ${esc(p.name)}">↶</button></div></div>`;
}).join('')).join('');$('staged-count').textContent=`${Object.keys(labState.staged).length} staged`;}
function renderCascade(){const rows=displayed(),r=rows.at(-1),i=['roll','pitch','yaw','depth'].indexOf(labState.axis),pid=r?.pid;
  if(i<0){$('cascade-state').textContent='UNAVAILABLE';$('cascade').innerHTML='<p class="footnote">Surge and sway use shared XY position/velocity gains. Per-axis PID terms are not published by the position controller; the angular cascade is unavailable for these axes.</p>';return;}
  const valid=i>=0&&pid&&(!labState.replay?Date.now()/1000-r.t<2.5:true);
  $('cascade-state').textContent=labState.replay?'OFFLINE REPLAY':!valid?'NO FRESH DATA':r.loop_enabled?.[i]===false?'LOOP DISABLED':'CONTROLLER VALUES';
  const u=i===3?'m/s':'deg/s',angleUnit=i===3?'m':'deg';
  const stages=[['Desired '+(i===3?'depth':'angle'),valid?r.targets?.[i]:null,angleUnit],
    ['Controller error',valid?pid[i*6]:null,angleUnit],['Outer sqrt_controller',valid?pid[i*6+1]:null,u],
    ['Measured '+(i===3?'vertical velocity':'body angular rate'),valid?pid[i*6+2]:null,u],
    ['Rate error',valid?pid[i*6+1]-pid[i*6+2]:null,u],
    ['Inner PID · P / I / D',valid?`${num(pid[i*6+3])} / ${num(pid[i*6+4])} / ${num(pid[i*6+5])}`:'—',i===3?'m/s² (before mass factor / trim / tilt boost)':'effort'],
    ['FF term',valid?r.ff?.[i]:null,i===3?'m/s² (depth thrust trim separate)':'effort'],
    ['Requested effort',valid?r.effort?.[i]:null,'normalized'],
    ['Mixer saturation flags',fresh(r,'mixer_t')?r.limits?.map((v,j)=>(['RP','Yaw','Depth','XY'][j])+': '+(v?'LIMIT':'clear')).join(' · '):'—','published limits']];
  $('cascade').innerHTML=stages.map(([name,v,unit])=>`<div class="cascade-stage"><span>${esc(name)}<small>${esc(unit)}</small></span><strong>${typeof v==='string'?esc(v):num(v,3)}</strong></div>`).join('');}
async function refreshStatus(){try{const status=await api('status');const was=activeVerified();labState.status=status;
  $('mode').textContent=labState.replay?'OFFLINE REPLAY':status.mode;
  $('connection').textContent=status.controller_connected?'Controller telemetry received':'Disconnected';
  $('operating').textContent=labState.replay?'Offline replay':(status.operating_mode||(status.bridge_command_mode?'bridge: '+status.bridge_command_mode:'—'));
  $('parameter-status').textContent=activeVerified()?'Readback verified (<10 s)':'Unverified · read to refresh';
  $('pid-health').textContent=`${num(streamHz(),1)} Hz / ${num(status.pid_age,2)} s`;
  $('recording-status').textContent=status.recording?`${status.recording.samples} samples`:'Idle';
  $('record').textContent=status.recording?'■ Stop recording':'● Record session';
  $('record').disabled=!!labState.replay||(!status.recording&&!status.pid_stream_fresh);
  $('safety-reason').textContent=status.write_reason;
  $('autotune-reason').textContent=status.autotune.reason;
  $('sim-reason').textContent=status.simulation.reason;
  if(was!==activeVerified())renderEditor();renderHealth();renderCascade();
}catch(e){labState.status={};$('mode').textContent=labState.replay?'OFFLINE REPLAY':'DISCONNECTED';$('connection').textContent='Backend unavailable';$('parameter-status').textContent='Unverified';renderEditor();}}
function streamHz(){const r=displayed().slice(-100);return r.length>1?(r.length-1)/(r.at(-1).t-r[0].t):null;}
function renderHealth(){const s=labState.status,r=displayed().slice(-100),dt=r.slice(1).map((x,i)=>x.t-r[i].t),mean=dt.length?dt.reduce((a,b)=>a+b,0)/dt.length:null;
  const jitter=mean===null?null:Math.sqrt(dt.reduce((a,b)=>a+(b-mean)**2,0)/dt.length);
  const mixer=r.filter(x=>fresh(x,'mixer_t')&&x.limits);
  const saturation=mixer.length?100*mixer.filter(x=>x.limits.some(Boolean)).length/mixer.length:null;
  const values=[['IMU update rate',num(s.imu_hz,1)+' Hz'],['IMU message age',num(s.imu_age,3)+' s'],['PID stream rate',num(streamHz(),1)+' Hz'],['Stream receipt dt',num(mean===null?null:mean*1000,2)+' ms'],['Receipt interval jitter (σ)',num(jitter===null?null:jitter*1000,2)+' ms'],['Actual controller dt (latest)',num(r.at(-1)?.controller_dt==null?null:r.at(-1).controller_dt*1000,2)+' ms'],['Controller execution jitter','Not measured'],['Motor command rate',num(s.motor_hz,1)+' Hz'],['Saturation (last 100 received samples)',num(saturation,1)+' %'],['Integrator clamping','Not published'],['Achievable effort / normalization','Not published'],['Gain synchronization',activeVerified()?'Verified readback':'Unverified'],['Simulation connection','Unavailable']];
  $('health-values').innerHTML=values.map(([k,v])=>`<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join('');}
let polling=false;
async function pollTelemetry(){if(polling||labState.replay)return;polling=true;try{const r=await api(`telemetry?after=${labState.cursor}`);if(!labState.replay){if(r.cursor<labState.cursor){labState.buffer.clear();announce('Bridge restarted. Telemetry buffer reset; read gains again.');labState.active={};labState.status.readback_verified=false;renderEditor();}labState.cursor=r.cursor;r.samples.forEach(x=>labState.buffer.push(x));if(r.dropped)announce(`${r.dropped} samples lost before collection; recordings retain explicit drop counts.`,true);}}catch(e){/* Status poll reports the disconnected state. */}finally{polling=false;}}
const palette=['#38bdf8','#fbbf24','#a78bfa','#34d399','#fb7185','#fb923c','#94a3b8','#818cf8'];
function signals(kind){const i=['roll','pitch','yaw','depth'].indexOf(labState.axis),p=i*6;
  if(kind==='motor')return Array.from({length:8},(_,j)=>({label:`M${j+1}`,value:r=>fresh(r,'motor_t')?r[$('motor-signal').value]?.[j]:null}));
  if(kind==='limits')return ['Roll / pitch','Yaw','Depth','XY'].map((label,j)=>({label,value:r=>fresh(r,'mixer_t')&&r.limits?Number(r.limits[j]):null}));
  if(i<0)return [];
  if(kind==='angle')return [{label:'Target',value:r=>i===3&&r.mode==='throttle'?null:r.targets?.[i]},{label:'Measured',value:r=>r.angles?.[i]},{label:'Controller error',value:r=>r.pid?.[p]}];
  if(kind==='rate')return [{label:'Target rate',value:r=>r.pid?.[p+1]},{label:'Measured rate',value:r=>r.pid?.[p+2]},{label:'Rate error',value:r=>r.pid?r.pid[p+1]-r.pid[p+2]:null}];
  if(kind==='pid')return [{label:'P',value:r=>r.pid?.[p+3]},{label:'I',value:r=>r.pid?.[p+4]},{label:'D',value:r=>r.pid?.[p+5]},{label:'FF',value:r=>r.ff?.[i]},...(i===3?[]:[{label:'Requested effort',value:r=>r.effort?.[i]}])];
}
class ScopeChart{
  constructor(kind,title,unit){this.kind=kind;this.title=title;this.unit=unit;this.enabled={};this.zoom=null;this.pan=0;this.hover=null;
    const card=document.createElement('article');card.className='lab-card chart-card';card.innerHTML=`<div class="card-heading"><h2>${esc(title)}</h2><div><span class="badge"></span> <button>PNG ↓</button></div></div><canvas aria-label="${esc(title)}" role="img"></canvas><div class="chart-tooltip" hidden></div><div class="chart-legend"></div>`;
    $('charts').append(card);this.card=card;this.canvas=card.querySelector('canvas');this.tooltip=card.querySelector('.chart-tooltip');this.legend=card.querySelector('.chart-legend');card.querySelector('button').onclick=()=>{const a=document.createElement('a');a.href=this.canvas.toDataURL('image/png');a.download=`atal-${labState.axis}-${kind}.png`;a.click();};
    this.canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.1,Math.min(600,(this.zoom||Number($('time-window').value)||60)*(e.deltaY>0?1.25:.8)));this.draw();},{passive:false});
    this.canvas.onpointerdown=e=>{this.drag={x:e.clientX,pan:this.pan};this.canvas.setPointerCapture(e.pointerId);};
    this.canvas.onpointerup=()=>{this.drag=null;};
    this.canvas.onpointermove=e=>{const rect=this.canvas.getBoundingClientRect();if(this.drag){this.pan=Math.max(0,this.drag.pan+(e.clientX-this.drag.x)/(rect.width-62)*(this.span||30));}this.hover=e.clientX-rect.left;this.draw();};
    this.canvas.onpointerleave=()=>{this.hover=null;this.tooltip.hidden=true;this.draw();};
  }
  reset(){this.zoom=null;this.pan=0;this.enabled={};this.signature=null;}
  draw(){if(labState.tab!=='response')return;const sig=signals(this.kind),signature=labState.axis+sig.map(s=>s.label).join();if(signature!==this.signature){this.signature=signature;this.legend.innerHTML='';sig.forEach((s,j)=>{const label=document.createElement('label');label.style.color=palette[j];const check=document.createElement('input');check.type='checkbox';check.checked=this.enabled[s.label]!==false;check.onchange=()=>{this.enabled[s.label]=check.checked;this.draw();};label.append(check,document.createTextNode(s.label));this.legend.append(label);});}
    const selected=sig.map((s,j)=>({...s,color:palette[j]})).filter(s=>this.enabled[s.label]!==false),rows=displayed(),rect=this.canvas.getBoundingClientRect(),w=Math.max(250,rect.width),h=250,dpr=devicePixelRatio||1;
    if(this.canvas.width!==Math.round(w*dpr)||this.canvas.height!==h*dpr){this.canvas.width=Math.round(w*dpr);this.canvas.height=h*dpr;}const c=this.canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.fillStyle='#0c1423';c.fillRect(0,0,w,h);
    const axis=labState.axis,unit=this.kind==='motor'?($('motor-signal').value==='pwm'?'PWM µs':'normalized thrust'):this.kind==='limits'?'limit flag (0/1)':this.kind==='pid'?(axis==='depth'?'m/s² (PID; thrust shown in cascade)':'effort'):axis==='depth'?(this.kind==='angle'?'m':'m/s'):(this.kind==='angle'?'deg':'deg/s');this.card.querySelector('.badge').textContent=unit;
    const last=rows.at(-1)?.t||0;this.span=this.zoom||Number($('time-window').value)||(rows.length?last-rows[0].t:60)||1;
    const end=last-this.pan,start=end-this.span,visible=rows.filter(r=>r.t>=start&&r.t<=end);let low=Infinity,high=-Infinity;
    for(const r of visible)for(const s of selected){const v=s.value(r);if(Number.isFinite(v)){low=Math.min(low,v);high=Math.max(high,v);}}
    const left=58,right=w-15,top=19,bottom=h-32;const has=Number.isFinite(low);if(!has){low=-1;high=1;}let margin=Math.max((high-low)*.12,.01);low-=margin;high+=margin;
    const x=t=>left+(t-start)/this.span*(right-left),y=v=>bottom-(v-low)/(high-low)*(bottom-top);
    c.font='9px monospace';c.lineWidth=1;for(let k=0;k<5;k++){const yy=top+k/4*(bottom-top);c.strokeStyle='#243148';c.beginPath();c.moveTo(left,yy);c.lineTo(right,yy);c.stroke();c.fillStyle='#6b82a0';c.fillText(num(high-k/4*(high-low),2),4,yy+3);const xx=left+k/4*(right-left);c.beginPath();c.moveTo(xx,top);c.lineTo(xx,bottom);c.stroke();c.fillText(`${num((start+k/4*this.span)-last,1)}s`,xx-12,h-12);}
    c.save();c.beginPath();c.rect(left,top,right-left,bottom-top);c.clip();for(const s of selected){c.strokeStyle=s.color;c.lineWidth=1.4;c.beginPath();let pen=false,prev=null,prevVal=null;for(const r of visible){const v=s.value(r);if(!Number.isFinite(v)){pen=false;prev=null;continue;}if(!pen||r.t-prev>.25||(this.kind==='angle'&&axis!=='depth'&&Math.abs(v-prevVal)>180))c.moveTo(x(r.t),y(v));else c.lineTo(x(r.t),y(v));pen=true;prev=r.t;prevVal=v;}c.stroke();}c.restore();
    if(!has){c.fillStyle='#6b82a0';c.textAlign='center';c.fillText(iUnavailable(axis)?'Velocity-loop PID telemetry is not published':'Waiting for actual controller telemetry',(left+right)/2,125);c.textAlign='left';}
    const stale=!labState.replay&&rows.length&&Date.now()/1000-last>=2.5;if(stale){c.fillStyle='#fbbf24';c.fillText('STALE · historical data',left+10,top+14);}
    if(this.hover!==null&&visible.length){const t=start+(this.hover-left)/(right-left)*this.span;let nearest=visible.reduce((a,b)=>Math.abs(a.t-t)<Math.abs(b.t-t)?a:b);c.strokeStyle='#64748b';c.beginPath();c.moveTo(x(nearest.t),top);c.lineTo(x(nearest.t),bottom);c.stroke();this.tooltip.textContent=`${new Date(nearest.t*1000).toISOString()}\n`+selected.map(s=>`${s.label}: ${num(s.value(nearest))} ${unit}`).join('\n');this.tooltip.hidden=false;}else this.tooltip.hidden=true;
  }
}
function iUnavailable(axis){return ['surge','sway'].includes(axis);}
const charts=[];
function draw(){charts.forEach(c=>c.draw());renderCascade();$('buffer-status').textContent=`${labState.buffer.size} / 6000 samples${labState.paused?' · display paused':''}${labState.replay?' · replay':''}`;}
async function loadProfiles(){labState.profiles=(await api('profiles')).profiles;$('profile-list').innerHTML=labState.profiles.length?labState.profiles.map(p=>`<div class="profile-item"><div><strong>${esc(p.name)}</strong><small>${esc(p.created_at)} · ${Object.keys(p.params).length} parameters · ${esc(p.validation_status)}</small></div><button data-stage-profile="${esc(p.id)}">Load into stage</button><button data-export-profile="${esc(p.id)}">Export JSON</button></div>`).join(''):'<p class="footnote">No saved candidates. Read controller gains or stage parameters to create one.</p>';
  for(const id of ['compare-a','compare-b']){const prev=$(id).value;$(id).innerHTML=labState.profiles.map(p=>`<option value="${esc(p.id)}">${esc(p.name)} · ${esc(p.id.slice(0,6))}</option>`).join('');if(labState.profiles.some(p=>p.id===prev))$(id).value=prev;}
  if(labState.profiles.length>1&&!$('compare-b').dataset.initialized){$('compare-b').selectedIndex=1;$('compare-b').dataset.initialized='1';}}
async function loadSessions(){const sessions=(await api('sessions')).sessions;$('session-list').innerHTML=sessions.length?sessions.map(s=>`<div class="session-item"><div><strong>${esc(s.name)}</strong><small>${esc(s.mode)} · ${esc(s.status)} · ${s.samples} samples · ${s.dropped} dropped · ${esc(s.created_at)}</small></div><a href="/api/control-lab/sessions/${esc(s.id)}/export" class="file-button">Export JSON</a></div>`).join(''):'<p class="footnote">No recorded sessions. Recording requires a fresh controller PID stream.</p>';}
function compareProfiles(){const a=labState.profiles.find(p=>p.id===$('compare-a').value),b=labState.profiles.find(p=>p.id===$('compare-b').value);if(!a||!b)throw Error('Save two candidates before comparing profiles.');const keys=[...new Set([...Object.keys(a.params),...Object.keys(b.params)])].sort();$('profile-comparison').innerHTML=table(['Parameter',a.name,b.name,'B − A'],keys.map(k=>[k,exact(a.params[k]),exact(b.params[k]),k in a.params&&k in b.params?exact(b.params[k]-a.params[k]):'Unavailable']));}
async function readSession(file){if(!file)throw Error('Choose an exported session.');if(file.size>128*1024*1024)throw Error('Session file exceeds 128 MB. Export a shorter interval.');const s=JSON.parse(await file.text());if(!s.metadata||!Array.isArray(s.samples)||s.samples.some(r=>!Number.isFinite(r.t)||!Array.isArray(r.pid)||r.pid.length!==24||r.pid.some(v=>!Number.isFinite(v))||(r.angles!==null&&!Array.isArray(r.angles))||(r.targets!==null&&!Array.isArray(r.targets))||(r.effort!==null&&!Array.isArray(r.effort))||(r.pwm!==null&&!Array.isArray(r.pwm))))throw Error('Invalid session export schema.');for(let i=1;i<s.samples.length;i++)if(s.samples[i].t<=s.samples[i-1].t)throw Error('Session timestamps must increase.');return s;}
function sessionMetrics(s){const rows=s.samples,axis=['roll','pitch','yaw','depth'].indexOf(labState.axis);const values=axis>=0?rows.map(r=>r.pid[axis*6]):[];const flags=rows.filter(r=>fresh(r,'mixer_t')&&Array.isArray(r.limits));return {name:s.metadata.name,mode:s.metadata.mode,sourceVerified:s.metadata.mode_verified?'verified':'unverified',samples:rows.length,rms:values.length?Math.sqrt(values.reduce((a,b)=>a+b*b,0)/values.length):null,peak:values.length?values.reduce((a,b)=>Math.max(a,Math.abs(b)),0):null,saturation:flags.length?100*flags.filter(r=>r.limits.some(Boolean)).length/flags.length:null};}
function compareSessions(){const a=labState.sessionA,b=labState.sessionB;if(!a||!b){$('session-comparison').textContent='Choose both session files to compare.';return;}const x=sessionMetrics(a),y=sessionMetrics(b);$('session-comparison').innerHTML=table(['Recorded metric',x.name,y.name],[['Source',`${x.mode} (${x.sourceVerified})`,`${y.mode} (${y.sourceVerified})`],['Samples',x.samples,y.samples],[`${labState.axis} controller error RMS`,num(x.rms),num(y.rms)],['Peak absolute controller error',num(x.peak),num(y.peak)],['Published saturation samples (%)',num(x.saturation),num(y.saturation)],['Overshoot / settling time','Needs validated step interval','Needs validated step interval']])+table(['Gain parameter','Session A initial','Session B initial'],[...new Set([...Object.keys(a.metadata.initial_parameters||{}),...Object.keys(b.metadata.initial_parameters||{})])].sort().map(k=>[k,num(a.metadata.initial_parameters[k]),num(b.metadata.initial_parameters[k])]));}
async function initialize(){charts.push(new ScopeChart('angle','Attitude / depth response'),new ScopeChart('rate','Body angular rate / vertical velocity'),new ScopeChart('pid','PID contributions & requested effort'),new ScopeChart('motor','Motor commands'),new ScopeChart('limits','Mixer saturation timeline'));
  document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>setTab(b.dataset.tab));document.querySelectorAll('[data-axis]').forEach(b=>b.onclick=()=>{setAxis(b.dataset.axis);compareSessions();});
  $('parameter-editor').oninput=e=>{const k=e.target.dataset.param;if(!k)return;const raw=e.target.value;if(raw===''){delete labState.staged[k];}else{const v=Number(raw);labState.staged[k]=v;if(activeVerified()&&v===labState.active[k])delete labState.staged[k];}e.target.classList.toggle('changed',Object.hasOwn(labState.staged,k));$('staged-count').textContent=`${Object.keys(labState.staged).length} staged`;};
  $('parameter-editor').onclick=e=>{const k=e.target.closest('[data-reset]')?.dataset.reset;if(k){delete labState.staged[k];renderEditor();}};
  $('reset-stage').onclick=()=>{labState.staged={};renderEditor();};
  $('read-gains').onclick=run(async()=>{if(labState.replay)throw Error('Exit offline replay before reading the active controller.');$('read-gains').disabled=true;try{const r=await api('parameters/read',{});labState.active=r.params;labState.revision=r.revision;labState.schema=r.schema;await refreshStatus();renderEditor();announce(`Read ${Object.keys(r.params).length} actual controller parameters. ${Object.values(r.errors).join(' ')}`);}finally{$('read-gains').disabled=false;}});
  $('review').onclick=run(async()=>{if(!Object.keys(labState.staged).length)throw Error('Stage at least one parameter first.');const r=await api('validate',{params:labState.staged,revision:labState.revision});$('diff-table').innerHTML=table(['Parameter','Verified active','Proposed'],r.diff.map(d=>[d.parameter,exact(d.before),exact(d.after)]));$('validation-message').textContent=r.warning+(!r.readback_verified?' Active values are unverified.':'');$('diff-dialog').showModal();});
  $('close-diff').onclick=()=>$('diff-dialog').close();
  $('pause').onclick=()=>{labState.paused=!labState.paused;if(labState.paused)labState.frozen=labState.buffer.values();else labState.frozen=[];$('pause').textContent=labState.paused?'▶ Resume display':'Ⅱ Pause display';draw();};
  $('reset-zoom').onclick=()=>{charts.forEach(c=>c.reset());draw();};$('motor-signal').onchange=draw;$('time-window').onchange=()=>{charts.forEach(c=>c.reset());draw();};
  $('csv').onclick=()=>{const rows=labState.buffer.values();if(!rows.length)return announce('No actual telemetry to export.',true);const axes=['roll','pitch','yaw','depth'];const headers=['receipt_time_s','sequence',...axes.flatMap((a,i)=>['error_'+(i===3?'m':'deg'),'target_rate_'+(i===3?'m_s':'deg_s'),'measured_rate_'+(i===3?'m_s':'deg_s'),'p','i','d'].map(k=>a+'_'+k)),...axes.map(a=>a+'_target'),...axes.map(a=>a+'_measured'),...axes.map(a=>a+'_ff'),...axes.map(a=>a+'_requested_effort'),...Array.from({length:8},(_,i)=>'motor_'+(i+1)+'_pwm_us'),...Array.from({length:8},(_,i)=>'motor_'+(i+1)+'_normalized_thrust'),'controller_dt_s','controller_ros_time_s','imu_receipt_s','motor_receipt_s','mixer_receipt_s','rp_limited','yaw_limited','depth_limited','xy_limited'];const fields=(a,n)=>a||Array(n).fill('');download('atal-telemetry.csv',headers.join(',')+'\n'+rows.map(r=>[r.t,r.seq,...r.pid,...fields(r.targets,4),...fields(r.angles,4),...fields(r.ff,4),...fields(r.effort,4),...fields(r.pwm,8),...fields(r.thrust,8),r.controller_dt??'',r.controller_t??'',r.imu_t,r.motor_t,r.mixer_t,...fields(r.limits,4)].join(',')).join('\n'),'text/csv');};
  $('record').onclick=run(async()=>{if(labState.replay)throw Error('Live session recording is unavailable in replay.');if(labState.status.recording)await api('sessions/stop',{});else{const name=prompt('Tuning session name');if(name===null)return;await api('sessions',{name});}await refreshStatus();await loadSessions();});
  $('reload-sessions').onclick=run(loadSessions);
  $('save-profile').onclick=run(async()=>{const params={...(activeVerified()?labState.active:{}),...labState.staged};await api('profiles',{name:$('profile-name').value,notes:$('profile-notes').value,params});await loadProfiles();announce('Candidate saved. Nothing was applied or approved.');});
  $('profile-list').onclick=run(async e=>{const b=e.target.closest('button');if(!b)return;const id=b.dataset.stageProfile||b.dataset.exportProfile,p=labState.profiles.find(x=>x.id===id);if(!p)return;if(b.dataset.exportProfile)download(`atal-profile-${p.id}.json`,JSON.stringify(p,null,2));else{labState.staged={...p.params};renderEditor();announce(`Loaded “${p.name}” into staged values. No controller changes.`);setTab('manual');}});
  $('compare-profiles').onclick=run(compareProfiles);
  for(const id of ['session-a','session-b'])$(id).onchange=run(async e=>{labState[id==='session-a'?'sessionA':'sessionB']=await readSession(e.target.files[0]);compareSessions();});
  $('replay-file').onchange=run(async e=>{const s=await readSession(e.target.files[0]);labState.replay=s.metadata;labState.buffer.clear();s.samples.slice(-6000).forEach(r=>labState.buffer.push(r));labState.paused=false;labState.frozen=[];$('pause').textContent='Ⅱ Pause display';$('end-replay').hidden=false;charts.forEach(c=>c.reset());await refreshStatus();renderEditor();draw();announce(`Offline replay: ${s.metadata.name}. Showing the last ${labState.buffer.size} original samples.`);});
  $('end-replay').onclick=()=>{labState.replay=null;labState.buffer.clear();labState.cursor=0;labState.paused=false;$('end-replay').hidden=true;labState.frozen=[];$('pause').textContent='Ⅱ Pause display';refreshStatus();announce('Returned to live telemetry. Waiting for controller samples.');};
  $('analyze-coupling').onclick=run(async()=>{const rows=displayed(),last=rows.at(-1)?.t;if(!last)throw Error('No samples available.');const start=Number($('coupling-start').value),end=Number($('coupling-end').value);if(!Number.isFinite(start)||!Number.isFinite(end)||start<=end||end<0)throw Error('Start must be earlier than end, with nonnegative offsets.');const selected=rows.filter(r=>r.t>=last-start&&r.t<=last-end);const result=await api('coupling',{axis:labState.axis,samples:selected});const names=['roll','pitch','yaw'];$('coupling-result').innerHTML=table(['Commanded axis','Measured roll RMS','Measured pitch RMS','Measured yaw RMS'],names.map(a=>[a,...(a===result.axis?result.rms.map(v=>num(v,4)+' deg/s'):['—','—','—'])]))+table(['Measured axis','RMS ratio','Peak (deg/s)','∫|rate| dt (deg)'],names.map((a,j)=>[a,num(result.ratios[j]),num(result.peaks[j]),num(result.integrated_absolute[j])]))+`<p class="footnote">${result.sample_count} samples · ${num(result.duration,3)} s · zero-command tolerance ${result.tolerance} deg/s · ${esc(new Date(result.start*1000).toISOString())}</p>`;});
  try{labState.schema=(await api('schema')).schema;renderEditor();}catch(e){announce(e.message,true);}
  setTab(['manual','response','autotune','simulation','profiles','diagnostics'].includes(location.hash.slice(1))?location.hash.slice(1):'manual');
  await refreshStatus();await pollTelemetry();setInterval(refreshStatus,1000);setInterval(pollTelemetry,100);
  let lastDraw=0;setInterval(()=>{const now=performance.now();if(now-lastDraw>=Number($('refresh-rate').value)){draw();lastDraw=now;}},50);
  window.addEventListener('resize',draw);
}
if(typeof document!=='undefined')document.addEventListener('DOMContentLoaded',initialize);
if(typeof module!=='undefined')module.exports={SampleRing,wrap,fresh,sessionMetrics};
