const $ = id => document.getElementById(id);
const labels = ['底座旋转','肩关节','肘关节','腕俯仰','腕旋转','夹爪'];
let current = null;
let dirty = false;
const dirtyAxes = new Set();
let firstState = true;
let pendingActions = 0;
let polling = false;
let actionError = '';
let serviceAvailable = false;
let previousEnabled = false;
let lastEventSignature = '';
let lastTargetSignature = '';
let calibrationSignature = '';
let ports = [];
let scanning = false;
let portEdited = false;
let initialScan = true;

function axisRows(calibration) {
  $('axis-list').replaceChildren();
  $('calibration-list').replaceChildren();
  calibration.forEach((cal, i) => {
    const row = document.createElement('div');
    row.className = 'axis-row';
    row.innerHTML = `<div class="axis-id"><span class="axis-num">${String(cal.id).padStart(2,'0')}</span><div class="axis-name"><strong>${labels[i]}</strong><small>标定 ${cal.min}–${cal.max}<br>下发 ${Math.min(cal.safe_min,cal.safe_max)}–${Math.max(cal.safe_min,cal.safe_max)}</small></div></div><div class="axis-target"><input id="slider-${i}" type="range" step="0.001" aria-label="${labels[i]} 目标"><input id="number-${i}" type="number" step="0.001" aria-label="${labels[i]} 归一化目标"><div class="target-detail"><span id="target-raw-${i}">目标 —</span><div class="jog-controls"><button data-jog="${i}" data-delta="-5" aria-label="${labels[i]} 减少5刻度">−5</button><button data-jog="${i}" data-delta="5" aria-label="${labels[i]} 增加5刻度">+5</button></div></div></div><div class="axis-feedback"><strong id="raw-${i}">—</strong><small id="feedback-${i}">等待反馈</small><small id="difference-${i}">未下发</small></div>`;
    $('axis-list').append(row);
    const slider=$(`slider-${i}`), number=$(`number-${i}`);
    slider.min=number.min=cal.allowed_min;
    slider.max=number.max=cal.allowed_max;
    slider.value=(cal.allowed_min+cal.allowed_max)/2;
    number.value=Number(slider.value).toFixed(3);
    slider.addEventListener('input',()=>{dirty=true;dirtyAxes.add(i);number.value=Number(slider.value).toFixed(3);updateTargetState();});
    number.addEventListener('input',()=>{
      dirty=true;dirtyAxes.add(i);
      const n=Number(number.value);
      if(Number.isFinite(n) && n>=cal.allowed_min && n<=cal.allowed_max) slider.value=n;
      updateTargetState();
    });
    const cr=document.createElement('div');
    cr.className='cal-row';
    cr.innerHTML=`<div class="cal-row-name">ID ${cal.id} · ${labels[i]}<small>${cal.name}</small></div><span class="cal-range">${cal.min} — ${cal.max}${cal.reverse?' · 反向':''}</span><button data-joint="${cal.name}" data-endpoint="0">采集 0</button><button data-joint="${cal.name}" data-endpoint="1">采集 1</button>`;
    $('calibration-list').append(cr);
  });
  document.querySelectorAll('[data-joint]').forEach(b=>b.addEventListener('click',()=>action('capture',{joint:b.dataset.joint,endpoint:Number(b.dataset.endpoint)})));
  document.querySelectorAll('[data-jog]').forEach(b=>b.addEventListener('click',()=>{
    const i=Number(b.dataset.jog);
    action('jog',{joint:current.calibration[i].name,delta:Number(b.dataset.delta)});
  }));
}

function ratioToRaw(cal, ratio) {
  return Math.round(cal.min+(cal.reverse?1-ratio:ratio)*(cal.max-cal.min));
}

function setTargets(state, rawTargets=null) {
  if(state.joints.length!==6) return;
  state.joints.forEach((joint,i)=>{
    const cal=state.calibration[i];
    const raw=rawTargets?.[joint.id] ?? joint.raw;
    const ratio=(raw-cal.min)/(cal.max-cal.min);
    const value=cal.reverse?1-ratio:ratio;
    $(`slider-${i}`).value=value;
    // Number shows the true pose, including endpoint holds outside the inner target range.
    $(`number-${i}`).value=value.toFixed(3);
  });
  dirty=false;dirtyAxes.clear();
  updateTargetState();
}

function updateTargetState() {
  const phases={idle:'尚未启用控制',holding:'已接管 · 保持当前姿态',moving:'正在执行目标',reached:'目标已到位',stopped:'下一步：点击「开始控制」，无需归位',paused:'已暂停 · 保持扭矩，可继续操作',failed:'执行失败 · 请检查运行记录'};
  $('target-state').textContent=dirty?`待发送：${[...dirtyAxes].map(i=>labels[i]).join('、')}；其余关节保持原位`:phases[current?.motion_state]||'尚未启用控制';
  $('target-state').classList.toggle('pending',dirty);
  if(!current) return;
  $('send-btn').disabled=!serviceAvailable||!current.connected||!current.control_enabled||current.busy||pendingActions>0||dirtyAxes.size===0;
  current.calibration.forEach((cal,i)=>{
    const input=$(`number-${i}`);
    const valid=input.value.trim()!=='' && Number.isFinite(Number(input.value));
    input.closest('.axis-row').classList.toggle('edited',dirtyAxes.has(i));
    $(`target-raw-${i}`).textContent=dirtyAxes.has(i)?(valid?`新目标 ${ratioToRaw(cal,Number(input.value))}`:'目标无效'):(current.joints[i]?`保持 ${current.joints[i].raw}`:'等待反馈');
  });
}

function render(state) {
  current=state;
  const signature=JSON.stringify(state.calibration);
  if(signature!==calibrationSignature) {
    axisRows(state.calibration);
    calibrationSignature=signature;
    dirty=false;dirtyAxes.clear();
    lastTargetSignature='';
  }
  if(firstState) {
    if(state.default_port && !portEdited) $('device-port').value=state.default_port;
    firstState=false;
    refreshPorts();
  }
  const connected=state.connected, enabled=state.control_enabled;
  document.querySelector('.connection-panel').classList.toggle('connected',connected);
  $('connection-summary').hidden=!connected;
  $('connection-summary-port').textContent=state.port+' · '+(state.backend==='serial'?'原生串口':'LeRobot');
  const busy=state.busy||pendingActions>0;
  const usable=serviceAvailable&&connected&&!busy;
  $('connection-badge').classList.toggle('connected',connected&&serviceAvailable);
  $('connection-badge').lastChild.textContent=!serviceAvailable?' 服务失联':connected?' 已连接':' 未连接';
  $('hero-state').textContent=!serviceAvailable?'服务失联 · 当前状态未确认':!connected?'等待连接机械臂':enabled?'控制已启用 · 从当前姿态操作':'设备在线 · 只读，尚未启用控制';
  $('status-text').textContent=actionError||state.error||state.status;
  $('status-text').parentElement.classList.toggle('error',Boolean(actionError||state.error));
  $('motion-label').textContent=busy?(state.stopping?'正在停止':'任务执行中'):enabled?'控制已启用':state.torque_on?'扭矩未确认关闭':'只读待机';
  $('connect-btn').disabled=!serviceAvailable||connected||busy;
  ['device-port','port-select','backend','baudrate'].forEach(id=>$(id).disabled=connected||busy);
  $('refresh-ports-btn').disabled=scanning;
  $('step-connect').classList.toggle('complete',connected);
  $('step-enable').classList.toggle('complete',connected&&enabled);
  $('step-connect').classList.toggle('current',!connected);
  $('step-enable').classList.toggle('current',connected&&!enabled);
  $('step-control').classList.toggle('current',connected&&enabled);
  const blockers=state.joints.length===6?state.joints.flatMap((j,i)=>{
    const c=state.calibration[i];
    if(j.status) return [`${labels[i]}有状态报警`];
    return j.raw<c.min||j.raw>c.max?[`${labels[i]}当前 ${j.raw}，需要在 ${c.min}–${c.max}`]:[];
  }):['尚未取得六关节反馈'];
  const ready=connected&&!state.torque_on&&blockers.length===0;
  $('control-hint').textContent=busy?'正在执行任务，可随时停止':!serviceAvailable?'服务失联，当前状态未确认':!connected?'第 1 步：连接设备':enabled?'第 3 步：调整要移动的关节，然后发送；其余关节保持原位':state.torque_on?'先点「释放扭矩 / 急停」，再开始控制':blockers.length?`暂不能接管：${blockers.join('；')}`:'当前位置可以接管。下一步：点击「开始控制」，无需归位或手动换姿态。';
  $('control-hint').classList.toggle('ready',ready&&!enabled);
  $('disconnect-btn').disabled=!usable;
  $('enable-btn').disabled=!usable||enabled||!ready;
  $('enable-btn').textContent=enabled?'控制已开启':'开始控制（保持当前姿态）';
  $('home-btn').disabled=!usable||state.home_required;
  $('reset-target-btn').disabled=!usable||state.joints.length!==6;
  $('send-btn').disabled=!usable||!enabled||dirtyAxes.size===0;
  // Keep stop available on service loss so the user can retry a stop request.
  $('stop-btn').disabled=!connected||state.stopping;
  const capturesReady=state.calibration.every(c=>{const ends=state.captures[c.name];return ends&&ends[0]!=null&&ends[1]!=null&&Math.abs(ends[1]-ends[0])>=20;});
  $('save-cal-btn').disabled=!usable||state.torque_on||!capturesReady;
  $('teach-btn').disabled=!usable||state.torque_on;
  document.querySelectorAll('[data-demo]').forEach(b=>b.disabled=!usable||state.home_required);
  document.querySelectorAll('[data-jog]').forEach(b=>{
    const i=Number(b.dataset.jog),j=state.joints[i],c=state.calibration[i],delta=Number(b.dataset.delta);
    const low=Math.min(c.safe_min,c.safe_max),high=Math.max(c.safe_min,c.safe_max),raw=j?.raw,next=raw+delta;
    const inward=raw>=c.min&&raw<low&&delta>0&&next<=high || raw<=c.max&&raw>high&&delta<0&&next>=low;
    const allowed=j&&(next>=low&&next<=high || inward);
    b.disabled=!usable||!enabled||!allowed;
    b.title=allowed?'立即移动此关节 5 刻度，其余保持原位':'该方向会接近端点；请向区间内部点动';
  });
  document.querySelectorAll('[data-joint]').forEach(b=>{
    b.disabled=!usable||state.torque_on;
    const val=state.captures[b.dataset.joint]?.[Number(b.dataset.endpoint)];
    b.textContent=val==null?`采集 ${b.dataset.endpoint}`:`${b.dataset.endpoint} = ${val}`;
    b.classList.toggle('captured',val!=null);
  });
  for(let i=0;i<6;i++) {
    const j=state.joints[i];
    $(`raw-${i}`).textContent=j?j.raw:'—';
    $(`feedback-${i}`).textContent=j?`${j.voltage.toFixed(1)} V · ${j.temperature}°C`:'等待反馈';
    const target=state.targets?.[j?.id];
    $(`difference-${i}`).textContent=j&&target!=null&&enabled?`已下发 ${target} · 差 ${j.raw-target}`:j?'当前位置 · 尚未下发':'等待连接';
    $(`difference-${i}`).classList.toggle('fault',Boolean(j?.status));
    $(`slider-${i}`).disabled=$(`number-${i}`).disabled=!usable||!enabled;
  }
  $('online-count').textContent=connected?`${state.joints.length} / 6`:'— / 6';
  $('control-state').textContent=!serviceAvailable?'未确认':enabled?'已接管':'未启用';
  $('torque-state').textContent=!serviceAvailable?'未确认':state.torque_on?'已开启':'已关闭';
  $('home-state').textContent=state.homed?'已到达':state.home_required?'未配置 · 不影响普通控制':'未归位 · 可选';
  $('voltage-range').textContent=state.joints.length?`${Math.min(...state.joints.map(j=>j.voltage)).toFixed(1)}–${Math.max(...state.joints.map(j=>j.voltage)).toFixed(1)} V`:'—';
  $('fault-count').textContent=state.joints.length?String(state.joints.filter(j=>j.status).length):'—';
  const targetSignature=JSON.stringify(state.targets);
  if(enabled&&!previousEnabled) {
    setTargets(state,state.targets);
    lastTargetSignature=targetSignature;
  } else if(!dirty && !state.busy && connected && state.joints.length===6 && (!enabled || targetSignature!==lastTargetSignature)) {
    setTargets(state,enabled?state.targets:null);
    lastTargetSignature=targetSignature;
  }
  previousEnabled=enabled;
  updateTargetState();
  renderCartesian(state);
  const eventSignature=JSON.stringify(state.events.slice(-7));
  if(eventSignature!==lastEventSignature) {
    lastEventSignature=eventSignature;
    $('event-list').replaceChildren();
    state.events.slice(-7).reverse().forEach(event=>{
      const li=document.createElement('li'),small=document.createElement('small');
      small.textContent=event.time;li.append(small,document.createTextNode(event.message));$('event-list').append(li);
    });
    if(!state.events.length) {const li=document.createElement('li');li.textContent='等待连接机械臂';$('event-list').append(li);}
  }
}

async function poll() {
  if(polling) return;
  polling=true;
  try {
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(3000)});
    if(!response.ok) throw new Error(`HTTP ${response.status}`);
    serviceAvailable=true;
    render(await response.json());
  } catch(error) {
    serviceAvailable=false;
    if(current) render(current);
    $('status-text').textContent=`界面服务不可用：${error.message}；状态可能已过期`;
    $('status-text').parentElement.classList.add('error');
  } finally {polling=false;}
}

async function action(name,payload={}) {
  if(pendingActions && !['stop','pause'].includes(name)) return;
  pendingActions++;
  actionError='';
  if(['stop','pause'].includes(name)){if(!payload.owner&&streamOwner)payload={...payload,owner:streamOwner};cancelDirection(false);}
  if(name==='stop'){keyboardArmed=false;updateKeyboard();}
  if(['home','demo'].includes(name)) $('optional-status').textContent='';
  if(current) render(current);
  try {
    const response=await fetch('/api/action',{method:'POST',signal:AbortSignal.timeout(5000),headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify({action:name,payload})});
    const body=await response.json();
    if(!response.ok) throw new Error(body.error||`HTTP ${response.status}`);
    if(['move','jog','enable_control','home','demo','stop'].includes(name)) {dirty=false;dirtyAxes.clear();}
    await poll();
  } catch(error) {
    if(['home','demo'].includes(name)) {
      let message=error.message;
      current?.calibration.forEach((c,i)=>{message=message.replaceAll(c.name,labels[i]);});
      $('optional-status').textContent=`本次预设动作未开始：${message}。普通控制不受该入口限制；请使用「开始控制」。`;
    } else actionError=error.message;
  }
  finally {pendingActions--;if(current) render(current);}
}

async function refreshPorts() {
  if(scanning) return;
  scanning=true;
  $('refresh-ports-btn').disabled=true;
  try {
    const response=await fetch('/api/ports',{cache:'no-store',signal:AbortSignal.timeout(3000)});
    const body=await response.json();
    if(!response.ok) throw new Error(body.error||`HTTP ${response.status}`);
    ports=body.ports;
    const select=$('port-select');select.replaceChildren();
    select.append(new Option(ports.length?'选择设备或手动输入':'未检测到串口 · 可手动输入',''));
    ports.forEach(p=>select.append(new Option(`${p.device} · ${p.description}`,p.device)));
    if(initialScan && !portEdited && !current?.connected && ports.length===1 && !ports.some(p=>p.device===$('device-port').value)) $('device-port').value=ports[0].device;
    select.value=ports.some(p=>p.device===$('device-port').value)?$('device-port').value:'';
    initialScan=false;
  } catch(error) {
    $('port-select').replaceChildren(new Option('扫描失败 · 可手动输入',''));
    $('port-select').title=error.message;
  } finally {scanning=false;$('refresh-ports-btn').disabled=false;}
}

$('port-select').addEventListener('change',()=>{if($('port-select').value){portEdited=true;$('device-port').value=$('port-select').value;}});
$('device-port').addEventListener('input',()=>{portEdited=true;$('port-select').value=ports.some(p=>p.device===$('device-port').value)?$('device-port').value:'';});
$('refresh-ports-btn').addEventListener('click',refreshPorts);
$('connect-btn').addEventListener('click',()=>action('connect',{port:$('device-port').value,backend:$('backend').value,baudrate:Number($('baudrate').value)}));
$('disconnect-btn').addEventListener('click',()=>action('disconnect'));
$('enable-btn').addEventListener('click',()=>action('enable_control'));
$('home-btn').addEventListener('click',()=>action('home'));
$('pause-btn').addEventListener('click',()=>action('pause'));
$('stop-btn').addEventListener('click',()=>action('stop'));
$('reset-target-btn').addEventListener('click',()=>{if(current)setTargets(current);});
$('send-btn').addEventListener('click',()=>{
  const values=Array.from({length:6},(_,i)=>Number($(`number-${i}`).value));
  const invalid=values.findIndex((v,i)=>{
    if(!dirtyAxes.has(i)) return false;
    const c=current.calibration[i],raw=ratioToRaw(c,v);
    return $(`number-${i}`).value.trim()===''||!Number.isFinite(v)||v<0||v>1||raw<Math.min(c.safe_min,c.safe_max)||raw>Math.max(c.safe_min,c.safe_max);
  });
  if(invalid!==-1) {
    const c=current.calibration[invalid];
    actionError=`${labels[invalid]}目标超出下发范围 ${Math.min(c.safe_min,c.safe_max)}–${Math.max(c.safe_min,c.safe_max)}；标定 ${c.min}–${c.max}`;
    render(current);return;
  }
  action('move',{values,joints:[...dirtyAxes].map(i=>current.calibration[i].name)});
});
document.querySelectorAll('[data-demo]').forEach(b=>b.addEventListener('click',()=>action('demo',{profile:b.dataset.demo})));
$('save-cal-btn').addEventListener('click',()=>action('save_calibration'));
$('teach-btn').addEventListener('click',()=>action('teach_home'));
window.addEventListener('pagehide',()=>{
  if(current?.torque_on) fetch('/api/action',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify({action:'stop',payload:{}})}).catch(()=>{});
});
document.querySelectorAll('[data-view]').forEach(button=>button.addEventListener('click',()=>{
  cancelDirection(true);keyboardArmed=false;updateKeyboard();
  activeView=button.dataset.view;
  ['control','cartesian','calibration'].forEach(view=>$(view+'-view').hidden=view!==activeView);
  $('page-name').textContent=$('page-title').textContent={control:'关节控制',cartesian:'末端控制',calibration:'校准与示教'}[activeView];
  document.querySelectorAll('[data-view]').forEach(tab=>{const active=tab===button;tab.classList.toggle('active',active);if(active)tab.setAttribute('aria-current','page');else tab.removeAttribute('aria-current');});
}));
let activeView='control';
let cartMode='continuous';
let keyboardArmed=false;
const heldKeys=new Set();
let pointerDirection=null;
let streamOwner=null;
let streamSequence=0;
let directionGeneration=0;
let intentInFlight=false;
let previewSignature='';
let targetEdited=false;
let observedPreview=null;
const keyDirections={w:[1,0,0],s:[-1,0,0],a:[0,1,0],d:[0,-1,0],r:[0,0,1],f:[0,0,-1]};
function desiredDirection(){
  const value=pointerDirection?[...pointerDirection]:[0,0,0];
  heldKeys.forEach(k=>keyDirections[k]?.forEach((v,i)=>value[i]+=v));
  const norm=Math.hypot(...value);
  return norm?value.map(v=>v/norm):value;
}
function cartReady(){return serviceAvailable&&current?.control_enabled&&!current.stopping;}
function updateKeyboard(){
  $('keyboard-btn').textContent=keyboardArmed?'关闭键盘控制':'开启键盘控制';
  $('keyboard-btn').setAttribute('aria-pressed',String(keyboardArmed));
  $('keyboard-hint').classList.toggle('armed',keyboardArmed);
}
function cancelDirection(pause=true){
  const owner=streamOwner;
  const hadInput=Boolean(owner||pointerDirection||heldKeys.size);
  directionGeneration++;streamOwner=null;heldKeys.clear();pointerDirection=null;
  document.querySelectorAll('[data-direction]').forEach(b=>b.classList.remove('pressed'));
  if(pause&&hadInput) action('pause',{owner});
}
async function motionRequest(name,payload){
  const response=await fetch('/api/action',{method:'POST',signal:AbortSignal.timeout(1000),headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify({action:name,payload})});
  const body=await response.json();
  if(!response.ok)throw new Error(body.error||'控制请求失败');
}
async function beginDirection(){
  if(!cartReady()||current.busy||streamOwner||pendingActions)return;
  const direction=desiredDirection();
  if(!Math.hypot(...direction))return;
  const generation=++directionGeneration;
  const owner=crypto.randomUUID();streamOwner=owner;
  try{
    await motionRequest('cartesian_start',{owner,sequence:++streamSequence,direction,speed:Number($('tcp-speed').value)});
    if(generation!==directionGeneration){action('pause',{owner});return;}
    await renewDirection();
  }catch(error){
    if(generation!==directionGeneration)return;
    cancelDirection(true);actionError=error.message;if(current)render(current);
  }
}
async function renewDirection(){
  if(!streamOwner||intentInFlight)return;
  const direction=desiredDirection();
  if(!Math.hypot(...direction)){cancelDirection(true);return;}
  const generation=directionGeneration;intentInFlight=true;
  try{
    await motionRequest('cartesian_intent',{owner:streamOwner,sequence:++streamSequence,direction,speed:Number($('tcp-speed').value)});
  }catch(error){
    if(generation===directionGeneration){cancelDirection(true);actionError=error.message;if(current)render(current);}
  }finally{intentInFlight=false;}
}
function renderPath(preview){
  const signature=JSON.stringify(preview);
  if(signature===previewSignature)return;
  previewSignature=signature;observedPreview=preview;
  $('path-preview').hidden=!preview;
  if(!preview)return;
  for(const [id,axis] of [['path-xy',1],['path-xz',2]]){
    const points=preview.path_mm;
    const lowX=Math.min(...points.map(p=>p[0]),preview.target_mm[0]),highX=Math.max(...points.map(p=>p[0]),preview.target_mm[0]);
    const lowY=Math.min(...points.map(p=>p[axis]),preview.target_mm[axis]),highY=Math.max(...points.map(p=>p[axis]),preview.target_mm[axis]);
    const scale=Math.min(240/Math.max(highX-lowX,10),95/Math.max(highY-lowY,10));
    const project=p=>[150+(p[0]-(highX+lowX)/2)*scale,70-(p[axis]-(highY+lowY)/2)*scale];
    const svg=$(id);svg.replaceChildren();
    const node=(tag,attrs,text)=>{const e=document.createElementNS('http://www.w3.org/2000/svg',tag);Object.entries(attrs).forEach(([k,v])=>e.setAttribute(k,String(v)));if(text)e.textContent=text;svg.append(e);};
    node('path',{d:'M 25 15 V 125 H 280',stroke:'#b9c0c9',fill:'none'});
    node('polyline',{points:points.map(p=>project(p).join(',')).join(' '),stroke:preview.reachable?'#e76b31':'#b34b3d','stroke-width':2,fill:'none'});
    const start=project(points[0]),target=project(preview.target_mm);
    node('rect',{x:start[0]-3,y:start[1]-3,width:6,height:6,fill:'#34414e'});
    node('rect',{x:target[0]-4,y:target[1]-4,width:8,height:8,fill:'none',stroke:'#e76b31'});
    node('text',{x:25,y:145,fill:'#7d8792','font-size':10},`${lowX.toFixed(1)} → ${highX.toFixed(1)} mm · 起点 ■ / 目标 □`);
  }
  $('path-summary').textContent=`${preview.message} · 距离 ${preview.distance_mm.toFixed(1)} mm · 预计 ${preview.duration_s.toFixed(1)} s · 朝向变化约 ${preview.orientation_change_deg.toFixed(1)}°`;
  $('path-summary').classList.toggle('fault',!preview.reachable);
  $('path-joints').textContent='预计关节变化：'+preview.joint_change_deg.map((v,i)=>`${labels[i]} ${v.toFixed(1)}°`).join(' / ');
}
function previewMatches(){
  return observedPreview?.reachable&&Date.now()<observedPreview.expires_at*1000&&['x','y','z'].every((axis,i)=>Math.abs(Number($('target-'+axis).value)-observedPreview.target_mm[i])<.001)&&Number($('tcp-speed').value)===observedPreview.speed;
}
function renderCartesian(state){
  const c=state.cartesian;
  if(!c)return;
  const ready=cartReady(),idle=ready&&!state.busy&&!pendingActions;
  const streaming=ready&&Boolean(streamOwner)&&(state.busy||c.stream_active);
  if(!serviceAvailable||!state.control_enabled){cancelDirection(false);keyboardArmed=false;updateKeyboard();}
  ['x','y','z'].forEach((axis,i)=>$('tcp-'+axis).textContent=c.actual?c.actual.xyz_mm[i].toFixed(2):'—');
  $('tcp-target').textContent=c.requested_mm||c.command_mm?'目标 '+(c.requested_mm||c.command_mm).map(v=>v.toFixed(2)).join(' / ')+' mm':'目标 —';
  $('tcp-error').textContent=c.tracking_error_mm==null?'跟随误差 —':`末端差 ${c.tracking_error_mm.toFixed(2)} mm · 关节最大差 ${(c.joint_error_deg||0).toFixed(1)}°`;
  $('tcp-hint').textContent=state.control_enabled?(state.busy?'正在处理请求，可随时暂停并保持扭矩。':'已接管：按方向按钮，或开启键盘控制；无需归位。'):$('control-hint').textContent;
  $('tcp-enable-btn').disabled=$('enable-btn').disabled;
  $('tcp-enable-btn').textContent=$('enable-btn').textContent;
  $('pause-btn').disabled=!ready;
  document.querySelectorAll('[data-direction]').forEach(b=>b.disabled=!(idle||streaming));
  $('keyboard-btn').disabled=!ready||(!streaming&&state.busy)||cartMode!=='continuous';
  $('tcp-notice').textContent=c.notice;
  $('tcp-copy').disabled=!serviceAvailable||!c.actual||state.busy;
  $('tcp-preview').disabled=!serviceAvailable||!state.connected||state.busy||Boolean(streamOwner);
  renderPath(c.preview);
  $('tcp-execute').disabled=!idle||!previewMatches();
  const grip=state.joints[5];$('tcp-grip-feedback').textContent=grip?grip.raw:'—';
  ['minus','plus'].forEach((dir,i)=>{
    const source=document.querySelector(`[data-jog="5"][data-delta="${i?5:-5}"]`);
    $('tcp-grip-'+dir).disabled=source?.disabled??true;
  });
  if(!targetEdited&&c.actual){['x','y','z'].forEach((axis,i)=>$('target-'+axis).value=c.actual.xyz_mm[i].toFixed(2));targetEdited=true;}
}
$('tcp-enable-btn').addEventListener('click',()=>action('enable_control'));
$('keyboard-btn').addEventListener('click',()=>{cancelDirection(true);keyboardArmed=!keyboardArmed;updateKeyboard();});
for(const [id,mode] of [['continuous-mode','continuous'],['step-mode','step']]){
  $(id).addEventListener('click',()=>{cancelDirection(true);keyboardArmed=false;updateKeyboard();cartMode=mode;['continuous','step'].forEach(m=>{const b=$(m+'-mode');b.classList.toggle('active',m===mode);b.setAttribute('aria-pressed',String(m===mode));});$('step-distance-wrap').hidden=mode!=='step';if(current)render(current);});
}
document.querySelectorAll('[data-speed]').forEach(b=>b.addEventListener('click',()=>{$('tcp-speed').value=b.dataset.speed;document.querySelectorAll('[data-speed]').forEach(p=>p.classList.toggle('active',p===b));if(current)render(current);}));
$('tcp-speed').addEventListener('input',()=>{document.querySelectorAll('[data-speed]').forEach(p=>p.classList.toggle('active',Number(p.dataset.speed)===Number($('tcp-speed').value)));if(current)render(current);});
document.querySelectorAll('[data-direction]').forEach(button=>{
  button.addEventListener('pointerdown',event=>{
    if(cartMode!=='continuous'||!cartReady()||(!streamOwner&&current.busy))return;
    event.preventDefault();button.setPointerCapture(event.pointerId);
    pointerDirection=button.dataset.direction.split(',').map(Number);button.classList.add('pressed');
    if(!streamOwner)beginDirection();else renewDirection();
  });
  for(const name of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(name,()=>{if(pointerDirection)cancelDirection(true);});
  button.addEventListener('click',()=>{if(cartMode==='step')action('cartesian_step',{direction:button.dataset.direction.split(',').map(Number),distance:Number($('tcp-distance').value),speed:Number($('tcp-speed').value)});});
});
window.addEventListener('keydown',event=>{
  const editing=event.target.closest('input,select,textarea,[contenteditable="true"]');
  if(event.code==='Escape'){cancelDirection(true);keyboardArmed=false;updateKeyboard();if(current?.control_enabled)action('pause');return;}
  if(event.code==='Space'&&!editing&&current?.connected){event.preventDefault();cancelDirection(false);keyboardArmed=false;updateKeyboard();action('stop');return;}
  const key=event.key.toLowerCase();
  if(!keyboardArmed||activeView!=='cartesian'||editing||!keyDirections[key]||!cartReady())return;
  event.preventDefault();if(event.repeat||heldKeys.has(key))return;
  heldKeys.add(key);if(!streamOwner)beginDirection();else renewDirection();
});
window.addEventListener('keyup',event=>{
  const key=event.key.toLowerCase();if(!heldKeys.has(key))return;
  heldKeys.delete(key);if(Math.hypot(...desiredDirection()))renewDirection();else cancelDirection(true);
});
function loseKeyboard(){cancelDirection(true);keyboardArmed=false;updateKeyboard();}
window.addEventListener('blur',loseKeyboard);
document.addEventListener('visibilitychange',()=>{if(document.hidden)loseKeyboard();});
document.addEventListener('focusin',event=>{if(event.target.closest('input,select,textarea,[contenteditable="true"]'))loseKeyboard();});
['x','y','z'].forEach(axis=>$('target-'+axis).addEventListener('input',()=>{targetEdited=true;$('tcp-execute').disabled=true;}));
$('tcp-copy').addEventListener('click',()=>{if(current?.cartesian.actual){['x','y','z'].forEach((axis,i)=>$('target-'+axis).value=current.cartesian.actual.xyz_mm[i].toFixed(2));targetEdited=true;$('tcp-execute').disabled=true;}});
$('tcp-preview').addEventListener('click',()=>{
  const xyz=['x','y','z'].map(axis=>Number($('target-'+axis).value));
  if(['x','y','z'].some(axis=>!$('target-'+axis).value.trim())){actionError='请输入完整 XYZ';render(current);return;}
  action('cartesian_preview',{xyz,speed:Number($('tcp-speed').value)});
});
$('tcp-execute').addEventListener('click',()=>{if(previewMatches())action('cartesian_execute',{preview_id:observedPreview.id});});
$('tcp-grip-minus').addEventListener('click',()=>action('jog',{joint:'gripper',delta:-5}));
$('tcp-grip-plus').addEventListener('click',()=>action('jog',{joint:'gripper',delta:5}));
setInterval(renewDirection,80);
poll();setInterval(poll,250);
setInterval(()=>{if(current&&!current.connected)refreshPorts();},5000);
