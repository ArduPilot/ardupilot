/* Feedback-driven target cards, timed movement and explicit settling breaks. */
(() => {
  const $=id=>document.getElementById('tm-'+id), copy=v=>JSON.parse(JSON.stringify(v));
  const rpyd=['roll','pitch','yaw','depth'];
  const colors={roll:'#f87171',pitch:'#38bdf8',yaw:'#fbbf24',depth:'#34d399',forward:'#818cf8',lateral:'#c084fc'};
  const target=()=>({type:'target',label:'RPYD target',roll:0,pitch:0,yaw:0,depth:.3,roll_tol:2,pitch_tol:2,yaw_tol:2,depth_tol:.05,stable_s:.5,max_time_s:15,timeout_action:'abort'});
  const blank=()=>({version:2,name:'Untitled mission',repeat:1,max_time_s:120,blocks:[target()]});
  const budget=b=>b.type==='target'?b.max_time_s:b.duration_s;
  const canonical=v=>JSON.stringify(v, (key,value)=>value&&typeof value==='object'&&!Array.isArray(value)?Object.fromEntries(Object.keys(value).sort().map(k=>[k,value[k]])):value);
  let draft=blank(), selected=0, savedId='', library=[], undo=[], redo=[], live={}, busy=false, polling=false, dirty=false, drag=null, parked=null;
  const snap=()=>({mission:copy(draft),id:savedId});
  function restore(s){draft=s.mission;savedId=s.id;}
  function feedback(text,bad=false){$('feedback').textContent=text;$('feedback').style.color=bad?'var(--rd)':'var(--tx)';}
  const ranges={roll:[-45,45],pitch:[-45,45],yaw:[-180,180],depth:[0,30],roll_tol:[.1,15],pitch_tol:[.1,15],yaw_tol:[.1,15],depth_tol:[.005,1],stable_s:[.1,10],max_time_s:[.1,120],duration_s:[.1,120],forward:[-.5,.5],lateral:[-.5,.5]};
  function validate(m){
    if(!m||m.version!==2||Object.keys(m).sort().join()!==['version','name','repeat','max_time_s','blocks'].sort().join())throw Error('Use a version 2 target mission; legacy throttle missions must be recreated.');
    if(typeof m.name!=='string'||!m.name.trim()||m.name.length>80||!Number.isInteger(m.repeat)||m.repeat<1||m.repeat>20||!Number.isFinite(m.max_time_s)||m.max_time_s<.1||m.max_time_s>900||!Array.isArray(m.blocks)||!m.blocks.length||m.blocks.length>100)throw Error('Invalid mission name, repeats, overall deadline or blocks');
    m.blocks.forEach(b=>{
      const fields=b?.type==='target'?['roll','pitch','yaw','depth','roll_tol','pitch_tol','yaw_tol','depth_tol','stable_s','max_time_s','timeout_action']:b?.type==='move'?['duration_s','forward','lateral']:b?.type==='break'?['duration_s']:null;
      if(!fields||Object.keys(b).sort().join()!==['type','label',...fields].sort().join()||typeof b.label!=='string'||!b.label.trim()||b.label.length>60)throw Error('Invalid block fields');
      fields.filter(k=>k!=='timeout_action').forEach(k=>{if(!Number.isFinite(b[k])||b[k]<ranges[k][0]||b[k]>ranges[k][1])throw Error(k+' outside supported range');});
      if(b.type==='target'&&(!['abort','continue'].includes(b.timeout_action)||b.stable_s>=b.max_time_s))throw Error('Target deadline must exceed stable dwell; choose a valid timeout policy');
    });return m;
  }
  async function api(path,body,method){const r=await fetch('/api/timed-missions/'+path,body===undefined&&!method?{}:{method:method||'POST',headers:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});const d=await r.json();if(!r.ok||!d.ok)throw Error(d.error||'Mission request failed');return d;}
  function button(text,fn,title){const b=document.createElement('button');b.type='button';b.textContent=text;b.title=title||text;b.onclick=()=>{try{fn();}catch(e){feedback(e.message,true);}};return b;}
  function edit(fn){if(live.active)return;const next=copy(draft);fn(next);validate(next);undo.push(snap());if(undo.length>50)undo.shift();redo=[];draft=next;render();}
  function move(from,to){if(to<0||to>=draft.blocks.length)return;edit(m=>m.blocks.splice(to,0,m.blocks.splice(from,1)[0]));selected=to;render();}
  function render(){
    dirty=false;selected=Math.max(0,Math.min(selected,draft.blocks.length-1));
    ['name','repeat','max_time_s'].forEach(k=>$(k).value=draft[k]);
    $('blocks').replaceChildren(...draft.blocks.map((b,i)=>{
      const row=document.createElement('div');row.className='tm-block'+(i===selected?' selected':'');row.dataset.index=i;row.draggable=!live.active;
      row.ondragstart=e=>{drag=i;e.dataTransfer.setData('text/plain',String(i));};row.ondragover=e=>e.preventDefault();row.ondrop=e=>{e.preventDefault();if(drag!==null&&!live.active)move(drag,i);drag=null;};
      const choose=button(`${i+1}. ${b.label}`,()=>{selected=i;render();});choose.className='tm-select';
      const summary=document.createElement('span');summary.className='tm-muted';summary.textContent=b.type==='target'?`RPY ${b.roll}° / ${b.pitch}° / ${b.yaw}° · depth ${b.depth}m · max ${b.max_time_s}s`:b.type==='break'?`Settle ${b.duration_s}s · hold last RPYD`:`Move ${b.duration_s}s · forward ${b.forward}, lateral ${b.lateral}`;choose.append(document.createElement('br'),summary);row.append(choose);
      [['↑',()=>move(i,i-1),'Move up'],['↓',()=>move(i,i+1),'Move down'],['⧉',()=>edit(m=>m.blocks.splice(i+1,0,copy(m.blocks[i]))),'Duplicate'],['×',()=>edit(m=>{if(m.blocks.length===1)throw Error('Keep at least one block');m.blocks.splice(i,1);}),'Delete']].forEach(([text,fn,title])=>{const b=button(text,fn,title);b.disabled=!!live.active;row.append(b);});return row;
    }));
    $('label').value=draft.blocks[selected].label;renderFields();timeline();controls();
    try{if(!live.active)localStorage.setItem('auv-rpyd-mission-draft-v2',JSON.stringify(draft));}catch(e){}
  }
  function renderFields(){
    const b=draft.blocks[selected];$('fields').replaceChildren();
    const labels={roll:'Roll target (degrees)',pitch:'Pitch target (degrees)',yaw:'Yaw target (degrees)',depth:'Depth target (m)',roll_tol:'Roll tolerance (degrees)',pitch_tol:'Pitch tolerance (degrees)',yaw_tol:'Yaw tolerance (degrees)',depth_tol:'Depth tolerance (m)',stable_s:'Continuous in-tolerance dwell (s)',max_time_s:'Maximum target wait (s)',duration_s:b.type==='break'?'Settling break duration (s)':'Movement duration (s)',forward:'Forward input (normalized)',lateral:'Lateral input (normalized)'};
    Object.keys(b).filter(k=>!['type','label'].includes(k)).forEach(k=>{
      const label=document.createElement('label');label.textContent=labels[k]||'On target timeout';
      let field;if(k==='timeout_action'){label.style.gridColumn='1 / -1';field=document.createElement('select');field.append(new Option('Abort and disarm (default)','abort'),new Option('Continue without reaching target','continue'));}
      else{field=document.createElement('input');field.type='number';field.min=ranges[k][0];field.max=ranges[k][1];field.step=k.includes('tol')||k==='depth'?'.01':'.1';}
      field.id='tm-block-'+k;field.value=b[k];field.oninput=()=>{dirty=true;controls();};label.append(field);$('fields').append(label);
    });
  }
  function timeline(){
    const total=draft.blocks.reduce((s,b)=>s+budget(b),0);
    $('duration').textContent=`Maximum block budgets ${total.toFixed(1)}s × ${draft.repeat}; overall hard deadline ${draft.max_time_s}s (includes pauses). Targets may finish early.`;
    $('timeline').replaceChildren();let offset=0;const ruler=document.createElement('div');ruler.className='tm-lane';
    draft.blocks.forEach(b=>{const tick=document.createElement('div');tick.className='tm-segment';tick.style.width=100*budget(b)/total+'%';tick.textContent=offset.toFixed(1)+'s';tick.style.color='var(--tx3)';ruler.append(tick);offset+=budget(b);});$('timeline').append(ruler);
    const held={roll:'current',pitch:'current',yaw:'current',depth:'current'};
    const values=draft.blocks.map(b=>{if(b.type==='target')rpyd.forEach(k=>held[k]=b[k]);return {...held,forward:b.type==='move'?b.forward:0,lateral:b.type==='move'?b.lateral:0};});
    [...rpyd,'forward','lateral'].forEach(axis=>{
      const label=document.createElement('div');label.className='tm-muted';label.textContent=axis+(axis==='depth'?' (m)':rpyd.includes(axis)?' (degrees)':' input');
      const lane=document.createElement('div');lane.className='tm-lane';lane.style.position='relative';
      draft.blocks.forEach((b,i)=>{const seg=button(`${b.type==='target'?'Target':b.type==='break'?'Break':'Move'} · ${values[i][axis]}`,()=>{selected=i;render();},`${b.label}: ${budget(b)}s ${b.type==='target'?'maximum':'duration'}`);seg.className='tm-segment';seg.style.width=100*budget(b)/total+'%';seg.style.background=values[i][axis]!==0?colors[axis]:'var(--bg3)';seg.style.color=values[i][axis]!==0?'#07111f':'var(--tx3)';lane.append(seg);});const head=document.createElement('div');head.className='tm-playhead';lane.append(head);$('timeline').append(label,lane);
    });
  }
  function controls(){
    const active=!!live.active;['name','repeat','max_time_s','editor','library','new','load','delete','save','save-copy','import','undo','redo'].forEach(k=>$(k).disabled=active||busy);
    $('palette').querySelectorAll('button').forEach(b=>b.disabled=active||busy);
    $('start').disabled=active||busy;
    $('pause').disabled=live.status!=='RUNNING'||busy;$('resume').disabled=live.status!=='PAUSED'||busy;$('abort').disabled=false;
    $('status').textContent=live.status||'OFFLINE';$('reason').textContent=[live.reason,live.feedback_status,live.precheck_reason].filter(Boolean).join(' · ')||'Waiting for vehicle feedback';
    $('progress').value=live.total_s?Math.min(100,100*live.elapsed_s/live.total_s):0;
    $('live').textContent=live.mission?`${live.mission.name} · cycle ${live.cycle}/${live.mission.repeat} · block ${(live.block_index||0)+1} · ${Number(live.block_remaining_s||0).toFixed(1)}s step budget left`:'No active mission';
    $('stable').textContent=live.mission?`Overall: ${Number(live.elapsed_s||0).toFixed(1)} / ${live.total_s}s · ${Math.max(0,live.total_s-live.elapsed_s).toFixed(1)}s until hard deadline · stable dwell ${Number(live.stable_elapsed_s||0).toFixed(1)}s`:'';
    $('rpyd').replaceChildren(...rpyd.map(k=>{const row=document.createElement('tr');const value=live.mission?live.command?.[k]:undefined, measured=live.measured?.[k];[k,Number.isFinite(value)?value.toFixed(2):'—',Number.isFinite(measured)?measured.toFixed(2):'—',Number.isFinite(live.errors?.[k])?live.errors[k].toFixed(2):'—'].forEach(v=>{const c=document.createElement('td');c.textContent=v;row.append(c);});return row;}));
    $('events').replaceChildren(...(live.events||[]).slice(-5).map(e=>{const li=document.createElement('li');li.textContent=`Cycle ${e.cycle}, block ${e.block_index+1}: ${e.result}`;return li;}));
    const same=active&&canonical(draft)===canonical(live.mission);$('blocks').querySelectorAll('.tm-block').forEach(row=>row.classList.toggle('current',same&&Number(row.dataset.index)===live.block_index));
    const total=draft.blocks.reduce((s,b)=>s+budget(b),0), before=draft.blocks.slice(0,live.block_index||0).reduce((s,b)=>s+budget(b),0);
    $('timeline').querySelectorAll('.tm-playhead').forEach(p=>{p.style.display=same?'block':'none';p.style.left=100*(before+Math.min(live.step_elapsed_s||0,budget(draft.blocks[live.block_index||0]||draft.blocks[0])))/total+'%';});
  }
  async function refreshLibrary(){library=(await api('library')).missions;const previous=$('library').value;$('library').replaceChildren(new Option('Choose a saved mission',''));library.forEach(e=>$('library').append(new Option((e.unsupported?'[Legacy] ':'')+e.mission.name,e.id)));$('library').value=savedId||previous;}
  async function save(copyAsNew=false){applyEditor();const d=await api('library',{mission:validate(draft),id:copyAsNew?'':savedId});savedId=d.id;await refreshLibrary();feedback('Mission saved on the NUC');}
  async function poll(){if(polling)return;polling=true;try{live=await api('state');if(live.active&&canonical(draft)!==canonical(live.mission)){if(!parked)parked=snap();draft=copy(live.mission);savedId='';selected=live.block_index||0;render();}else if(!live.active&&parked){restore(parked);parked=null;render();}controls();}catch(e){live={status:'OFFLINE',precheck_reason:e.message};controls();}finally{polling=false;}}
  async function action(fn){busy=true;controls();try{await fn();}catch(e){feedback(e.message,true);}finally{busy=false;await poll();controls();}}
  const presets=[['RPYD Target',target],['Forward',()=>({type:'move',label:'Forward',duration_s:2,forward:.15,lateral:0})],['Lateral',()=>({type:'move',label:'Lateral',duration_s:2,forward:0,lateral:.15})],['Custom move',()=>({type:'move',label:'Movement',duration_s:2,forward:0,lateral:0})],['Break / settle',()=>({type:'break',label:'Break / settle',duration_s:2})]];
  presets.forEach(([label,make])=>{const b=button('+ '+label,()=>{edit(m=>m.blocks.push(make()));selected=draft.blocks.length-1;render();});b.className='btn btn-k';$('palette').append(b);});
  ['name','repeat','max_time_s'].forEach(k=>$(k).onchange=()=>{try{edit(m=>m[k]=k==='name'?$(k).value:Number($(k).value));}catch(e){feedback(e.message,true);render();}});
  $('label').oninput=()=>{dirty=true;controls();};
  function applyEditor(){if(!dirty)return;const b=copy(draft.blocks[selected]);b.label=$('label').value;Object.keys(b).filter(k=>!['type','label'].includes(k)).forEach(k=>{const text=$('block-'+k).value;if(!text.trim())throw Error('Every block field is required');b[k]=k==='timeout_action'?text:Number(text);});edit(m=>m.blocks[selected]=b);}
  $('apply').onclick=()=>{try{applyEditor();}catch(e){feedback(e.message,true);}};
  $('undo').onclick=()=>{if(undo.length&&!live.active){redo.push(snap());restore(undo.pop());render();}};$('redo').onclick=()=>{if(redo.length&&!live.active){undo.push(snap());restore(redo.pop());render();}};
  $('new').onclick=()=>{if(live.active)return;undo.push(snap());redo=[];draft=blank();savedId='';render();};$('save').onclick=()=>action(()=>save());$('save-copy').onclick=()=>action(()=>save(true));
  $('load').onclick=()=>{try{const e=library.find(e=>e.id===$('library').value);if(!e||live.active)return;if(e.unsupported)throw Error(e.error);undo.push(snap());redo=[];draft=copy(validate(e.mission));savedId=e.id;render();}catch(e){feedback(e.message,true);}};
  $('delete').onclick=()=>action(async()=>{const id=$('library').value;if(!id)throw Error('Choose a saved mission');if(!confirm('Delete this saved mission?'))return;await api('library/'+id,undefined,'DELETE');if(savedId===id)savedId='';await refreshLibrary();});
  $('export').onclick=()=>{try{validate(draft);const url=URL.createObjectURL(new Blob([JSON.stringify(draft,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='rpyd-mission.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}catch(e){feedback(e.message,true);}};
  $('import').onclick=()=>$('file').click();$('file').onchange=async e=>{try{const f=e.target.files[0];if(!f)return;if(f.size>200000)throw Error('Mission file too large');const next=validate(JSON.parse(await f.text()));if(live.active)throw Error('Abort before importing');undo.push(snap());redo=[];draft=next;savedId='';render();feedback('Imported as a draft; save to add to the library');}catch(err){feedback(err.message,true);}finally{e.target.value='';}};
  $('start').onclick=()=>action(async()=>{applyEditor();await api('start',{mission:validate(draft)});feedback('Controller-target mission started');});$('pause').onclick=()=>action(()=>api('pause',{}));$('resume').onclick=()=>action(()=>api('resume',{}));$('abort').onclick=async()=>{try{await api('abort',{});feedback('Abort and disarm requested');}catch(e){feedback(e.message,true);}await poll();};
  try{const stored=localStorage.getItem('auv-rpyd-mission-draft-v2');if(stored)draft=validate(JSON.parse(stored));}catch(e){}
  render();refreshLibrary().catch(e=>feedback(e.message,true));poll();setInterval(poll,250);
})();
