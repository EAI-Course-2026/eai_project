const $ = id => document.getElementById(id);
const labels = ['底座旋转','肩关节','肘关节','腕俯仰','腕旋转','夹爪'];
let current = null;
let dirty = false;
let lastEventSignature = '';
let firstState = true;
let actionPending = false;

function axisRows(calibration) {
  $('axis-list').replaceChildren();
  $('calibration-list').replaceChildren();
  calibration.forEach((cal, i) => {
    const row = document.createElement('div');
    row.className = 'axis-row';
    row.innerHTML = `<div class="axis-id"><span class="axis-num">${String(i+1).padStart(2,'0')}</span><div class="axis-name"><strong>${labels[i]}</strong><small>${cal.name} · ${cal.allowed_min.toFixed(2)}–${cal.allowed_max.toFixed(2)}</small></div></div><div class="axis-target"><input id="slider-${i}" type="range" min="0" max="1" step="0.001" value="0.5" aria-label="${labels[i]} 目标"><input id="number-${i}" type="number" min="0" max="1" step="0.001" value="0.500" aria-label="${labels[i]} 归一化目标"></div><div class="axis-feedback"><strong id="raw-${i}">—</strong><small id="feedback-${i}">等待反馈</small></div>`;
    $('axis-list').append(row);
    const slider = $(`slider-${i}`), number = $(`number-${i}`);
    slider.min=cal.allowed_min; slider.max=cal.allowed_max;
    number.min=cal.allowed_min; number.max=cal.allowed_max;
    slider.value=(cal.allowed_min+cal.allowed_max)/2; number.value=Number(slider.value).toFixed(3);
    slider.addEventListener('input', () => { dirty=true; number.value=Number(slider.value).toFixed(3); });
    number.addEventListener('input', () => { dirty=true; const n=Number(number.value); if(Number.isFinite(n) && n>=0 && n<=1) slider.value=n; });
    const cr = document.createElement('div');
    cr.className='cal-row';
    cr.innerHTML=`<div class="cal-row-name">ID ${i+1} · ${labels[i]}<small>${cal.name} · ${cal.allowed_min.toFixed(2)}–${cal.allowed_max.toFixed(2)}</small></div><span class="cal-range">${cal.min} — ${cal.max}${cal.reverse ? ' · 反向' : ''}</span><button data-joint="${cal.name}" data-endpoint="0">采集 0</button><button data-joint="${cal.name}" data-endpoint="1">采集 1</button>`;
    $('calibration-list').append(cr);
  });
  document.querySelectorAll('[data-joint]').forEach(button => button.addEventListener('click', () => action('capture', {joint:button.dataset.joint, endpoint:Number(button.dataset.endpoint)})));
}

function setTargetsFromFeedback(state) {
  if(!state.joints || state.joints.length!==6) return;
  state.joints.forEach((joint, i) => {
    const bounds=state.calibration[i];
    const ratio = Math.max(bounds.allowed_min, Math.min(bounds.allowed_max, Number(joint.ratio)));
    $(`slider-${i}`).value=ratio;
    $(`number-${i}`).value=ratio.toFixed(3);
  });
  dirty=false;
}

function render(state) {
  current=state;
  if(firstState) {
    axisRows(state.calibration);
    if(state.default_port) $('device-port').value=state.default_port;
    firstState=false;
  }
  const connected=state.connected, busy=state.busy, torque=state.torque_on;
  const badge=$('connection-badge');
  badge.classList.toggle('connected', connected);
  badge.lastChild.textContent=connected?' 已连接':' 未连接';
  $('hero-state').textContent=connected ? (state.homed?'归位完成 · 可以控制': state.home_required?'设备在线 · 请先示教起点':'设备在线 · 等待归位') : '等待连接机械臂';
  $('status-text').textContent=state.error || state.status;
  $('status-text').parentElement.classList.toggle('error', Boolean(state.error));
  $('motion-label').textContent=busy ? (state.stopping?'STOPPING':'IN MOTION') : torque?'TORQUE ON':'READ ONLY';
  $('connect-btn').disabled=connected||busy;
  $('disconnect-btn').disabled=!connected||busy;
  $('home-btn').disabled=!connected||busy||state.home_required;
  $('reset-target-btn').disabled=!connected||busy||!state.joints.length;
  $('send-btn').disabled=!connected||busy||!state.homed;
  $('stop-btn').disabled=!connected||state.stopping;
  const capturesReady=state.calibration.every(c=>{const ends=state.captures[c.name];return ends&&ends[0]!=null&&ends[1]!=null&&Math.abs(ends[1]-ends[0])>=20});
  $('save-cal-btn').disabled=!connected||busy||torque||!capturesReady;
  $('teach-btn').disabled=!connected||busy||torque;
  document.querySelectorAll('[data-demo]').forEach(b=>b.disabled=!connected||busy||state.home_required);
  document.querySelectorAll('[data-joint]').forEach(b=>{
    b.disabled=!connected||busy||torque;
    const val=state.captures[b.dataset.joint]?.[Number(b.dataset.endpoint)];
    b.textContent=val==null?`采集 ${b.dataset.endpoint}`:`${b.dataset.endpoint} = ${val}`;
    b.classList.toggle('captured', val!=null);
  });
  for(let i=0;i<6;i++) {
    const j=state.joints[i];
    $(`raw-${i}`).textContent=j?j.raw:'—';
    $(`feedback-${i}`).textContent=j?`${j.voltage.toFixed(1)} V · ${j.temperature}°C`:'等待反馈';
    $(`slider-${i}`).disabled=!connected||busy||!state.homed;
    $(`number-${i}`).disabled=!connected||busy||!state.homed;
  }
  $('online-count').textContent=connected?`${state.joints.length} / 6`:'— / 6';
  $('torque-state').textContent=torque?'已开启':'已关闭';
  $('home-state').textContent=state.homed?'已确认':state.home_required?'待示教':'未进入';
  $('voltage-range').textContent=state.joints.length?`${Math.min(...state.joints.map(j=>j.voltage)).toFixed(1)}–${Math.max(...state.joints.map(j=>j.voltage)).toFixed(1)} V`:'—';
  $('fault-count').textContent=state.joints.length?String(state.joints.filter(j=>j.status).length):'—';
  if(!dirty && !busy && connected && state.joints.length) setTargetsFromFeedback(state);
  const eventSignature=JSON.stringify(state.events.slice(-7));
  if(eventSignature!==lastEventSignature) {
    lastEventSignature=eventSignature;
    const list=$('event-list'); list.replaceChildren();
    state.events.slice(-7).reverse().forEach(event=>{
      const li=document.createElement('li'), small=document.createElement('small');
      small.textContent=event.time; li.append(small,document.createTextNode(event.message)); list.append(li);
    });
    if(!state.events.length) { const li=document.createElement('li');li.textContent='等待连接机械臂';list.append(li); }
  }
}

async function poll() {
  try {
    const response=await fetch('/api/state',{cache:'no-store'});
    if(!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch(error) {
    $('status-text').textContent=`界面服务不可用：${error.message}`;
    $('status-text').parentElement.classList.add('error');
    ['connect-btn','disconnect-btn','home-btn','send-btn','stop-btn'].forEach(id=>$(id).disabled=true);
  }
}

async function action(name, payload={}) {
  if(actionPending && name!=='stop') return;
  actionPending=true;
  try {
    const response=await fetch('/api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify({action:name,payload})});
    const body=await response.json();
    if(!response.ok) throw new Error(body.error||`HTTP ${response.status}`);
    await poll();
  } catch(error) {
    $('status-text').textContent=error.message;
    $('status-text').parentElement.classList.add('error');
  } finally { actionPending=false; }
}

$('connect-btn').addEventListener('click',()=>action('connect',{port:$('device-port').value,backend:$('backend').value,baudrate:Number($('baudrate').value)}));
$('disconnect-btn').addEventListener('click',()=>action('disconnect'));
$('home-btn').addEventListener('click',()=>action('home'));
$('stop-btn').addEventListener('click',()=>action('stop'));
$('reset-target-btn').addEventListener('click',()=>{if(current)setTargetsFromFeedback(current)});
$('send-btn').addEventListener('click',()=>{
  const values=Array.from({length:6},(_,i)=>Number($(`number-${i}`).value));
  if(values.some((v,i)=>$(`number-${i}`).value.trim()===''||!Number.isFinite(v)||v<current.calibration[i].allowed_min-0.0006||v>current.calibration[i].allowed_max+0.0006)) {
    $('status-text').textContent='请输入六个处于当前演示安全范围的 0–1 目标值';
    $('status-text').parentElement.classList.add('error');return;
  }
  dirty=false;action('move',{values});
});
document.querySelectorAll('[data-demo]').forEach(button=>button.addEventListener('click',()=>action('demo',{profile:button.dataset.demo})));
$('save-cal-btn').addEventListener('click',()=>action('save_calibration'));
$('teach-btn').addEventListener('click',()=>action('teach_home'));
window.addEventListener('pagehide',()=>{
  if(current?.torque_on) fetch('/api/action',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify({action:'stop',payload:{}})}).catch(()=>{});
});
poll();setInterval(poll,1200);
