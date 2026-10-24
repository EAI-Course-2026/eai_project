'use strict';
(() => {
  const el = id => document.getElementById('agent-' + id);
  let state = {}, recording = null, preparingMic = false, lastAudio = '', initialized = false, signature = '', localError = '', pending = false, epoch = 0, pollTimer = null, polling = false;
  const phases = {idle:'准备好了', recognizing:'正在识别语音…', thinking:'正在思考…', tool:'正在调用工具…', synthesizing:'正在合成语音…', stopping:'正在停止…'};
  async function post(path, body) {
    const response = await fetch('/api/agent/' + path, {method:'POST', headers:{'Content-Type':'application/json','X-Arm-Desk':'1'}, body:JSON.stringify(body)});
    if (!response.ok) throw new Error((await response.json()).error || 'request_failed');
  }
  function controls() {
    const busy = pending || state.busy || preparingMic;
    el('text').disabled = !state.enabled || busy || Boolean(recording);
    el('send').disabled = !state.enabled || busy || Boolean(recording);
    el('mic').disabled = !state.enabled || !state.asr_enabled || busy;
    el('speak').disabled = !state.tts_enabled;
    el('status').textContent = localError || state.error_code || (!state.enabled ? '点击右上角「API 接入」开始。' : recording ? '正在录音，点击「结束并发送」完成。' : state.notice || phases[state.status] || '准备好了');
  }
  function stopPlayback() {
    el('audio').pause(); el('audio').removeAttribute('src'); el('audio').load(); el('audio').hidden = true;
  }
  async function send(body, token = epoch) {
    stopPlayback(); pending = true; localError = ''; controls();
    try { if (token === epoch) {await post('turn', {...body, generation:state.generation, runtime_id:state.runtime_id, speak:Boolean(state.tts_enabled && el('speak').checked)});if(token===epoch)state.busy=true;} }
    catch (error) { localError = '发送失败：' + error.message; }
    finally { pending = false; controls(); }
  }
  el('form').addEventListener('submit', event => {
    event.preventDefault(); const text = el('text').value.trim();
    if (!text || state.busy || pending) return;
    el('text').value = ''; send({text});
  });
  el('text').addEventListener('keydown', event => {
    if (event.key === 'Enter' && !event.shiftKey && !event.isComposing) { event.preventDefault(); el('form').requestSubmit(); }
  });
  async function finishRecording(sendAudio) {
    if (!recording) return;
    const r = recording; recording = null; clearTimeout(r.timer);
    if(sendAudio){pending=true;controls();}
    r.processor.disconnect(); r.source.disconnect(); r.mute.disconnect(); r.stream.getTracks().forEach(t => t.stop());
    await r.context.close(); el('mic').textContent = '开始说话'; controls();
    if (!sendAudio || r.epoch !== epoch) {pending=false;controls();return;}
    pending = true; controls();
    try {
      const length = r.chunks.reduce((n,c) => n + c.length, 0);
      if (!length) throw new Error('没有录到声音');
      const offline = new OfflineAudioContext(1, Math.ceil(length * 16000 / r.rate), 16000);
      const buffer = offline.createBuffer(1, length, r.rate); let offset = 0;
      for (const chunk of r.chunks) { buffer.getChannelData(0).set(chunk,offset); offset += chunk.length; }
      const source = offline.createBufferSource(); source.buffer = buffer; source.connect(offline.destination); source.start();
      const samples = (await offline.startRendering()).getChannelData(0);
      const bytes = new Uint8Array(44 + samples.length * 2), view = new DataView(bytes.buffer);
      const word = (at,s) => [...s].forEach((c,i) => view.setUint8(at+i,c.charCodeAt(0)));
      word(0,'RIFF'); view.setUint32(4,bytes.length-8,true); word(8,'WAVE'); word(12,'fmt ');
      view.setUint32(16,16,true); view.setUint16(20,1,true); view.setUint16(22,1,true); view.setUint32(24,16000,true);
      view.setUint32(28,32000,true); view.setUint16(32,2,true); view.setUint16(34,16,true); word(36,'data'); view.setUint32(40,samples.length*2,true);
      samples.forEach((s,i) => view.setInt16(44+i*2, Math.round(Math.max(-1,Math.min(1,s)) * (s<0?32768:32767)),true));
      let binary = ''; for (let i=0;i<bytes.length;i+=8192) binary += String.fromCharCode(...bytes.subarray(i,i+8192));
      await send({audio:btoa(binary)}, r.epoch);
    } catch(error) { localError = '录音处理失败：' + error.message; }
    finally { pending = false; controls(); }
  }
  el('mic').addEventListener('click', async () => {
    if (recording) { await finishRecording(true); return; }
    if (pending || state.busy || preparingMic) return;
    const token = epoch; preparingMic = true; stopPlayback(); controls();
    let stream, context;
    try {
      stream = await navigator.mediaDevices.getUserMedia({audio:{echoCancellation:true,noiseSuppression:true},video:false});
      if (token !== epoch) { stream.getTracks().forEach(t=>t.stop()); return; }
      context = new AudioContext(); await context.resume();
      if(token!==epoch){stream.getTracks().forEach(t=>t.stop());await context.close();return;}
      const source = context.createMediaStreamSource(stream), processor = context.createScriptProcessor(4096,1,1), mute = context.createGain();
      mute.gain.value = 0;
      const r = {stream,context,source,processor,mute,rate:context.sampleRate,chunks:[],epoch:token};
      processor.onaudioprocess = event => { if(recording===r) r.chunks.push(new Float32Array(event.inputBuffer.getChannelData(0))); };
      source.connect(processor); processor.connect(mute); mute.connect(context.destination); recording = r;
      r.timer = setTimeout(()=>finishRecording(true),29000); el('mic').textContent = '结束并发送'; localError = '';
    } catch(error) { stream?.getTracks().forEach(t=>t.stop()); if(context) await context.close(); localError = '无法开启麦克风：' + error.message; }
    finally { preparingMic = false; controls(); }
  });
  function localStop() { epoch++; stopPlayback(); finishRecording(false); }
  el('stop').addEventListener('click', async () => { localStop(); try { await post('stop',{}); } catch(error) { localError = '停止请求失败：' + error.message; } controls(); });
  document.getElementById('stop-btn').addEventListener('click',localStop);
  window.addEventListener('pagehide',()=>{const active=state.busy||pending||recording||preparingMic;localStop();if(active)fetch('/api/agent/stop',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:'{}'}).catch(()=>{});});
  document.addEventListener('visibilitychange',()=>{if(document.hidden) finishRecording(false);});
  document.querySelectorAll('[data-view]').forEach(button=>button.addEventListener('click',()=>{if(button.dataset.view!=='agent'){epoch++;finishRecording(false);stopPlayback();}}));
  document.querySelectorAll('[data-agent-prompt]').forEach(button=>button.addEventListener('click',()=>{el('text').value=button.dataset.agentPrompt;if(!state.enabled)document.dispatchEvent(new Event('agent:settings-open'));else el('text').focus();}));
  for(const mode of ['text','voice']) el('mode-'+mode).addEventListener('click',()=>{for(const item of ['text','voice']){el('mode-'+item).classList.toggle('selected',mode===item);el('mode-'+item).setAttribute('aria-pressed',String(mode===item));}if(mode==='voice'){if(!state.asr_enabled)document.dispatchEvent(new Event('agent:settings-open'));else el('mic').focus();}else{finishRecording(false);el('text').focus();}});
  document.addEventListener('agent:settings-saved',()=>{localStop();state={};signature='';initialized=false;lastAudio='';localError='';clearTimeout(pollTimer);poll();});
  async function poll() {
    if(polling)return;polling=true;
    try {
      const response = await fetch('/api/agent/state'); if(!response.ok) throw new Error();
      if(localError==='Agent 服务连接中断，请检查上位机终端。') localError='';
      const previousGeneration=state.generation, previousRuntime=state.runtime_id;
      state = await response.json();
      if(previousRuntime && previousRuntime!==state.runtime_id) {localStop();initialized=false;lastAudio='';}
      if(previousGeneration!==undefined && previousGeneration!==state.generation) {stopPlayback();if(state.notice)localStop();}
      el('model-label').textContent=state.api_settings?.chat?.model||'选择模型';
      el('welcome').hidden=Boolean(state.messages?.length);
      el('suggestions').hidden=Boolean(state.messages?.length);
      controls();
      const next = JSON.stringify([state.messages,state.tools]);
      if(next !== signature) {
        signature = next; el('messages').replaceChildren();
        for(const message of state.messages) { const row=document.createElement('div'); row.className='agent-message '+message.role; const label=document.createElement('strong'); label.textContent=message.role==='user'?'你':'Agent'; const text=document.createElement('p');text.textContent=message.text||'…';row.append(label,text);el('messages').append(row); }
        el('messages').scrollTop=el('messages').scrollHeight;
        el('tools').textContent=(state.tools||[]).map(t=>t.tool+' · '+t.status).join(' / ');
      }
      const audio = [...state.messages].reverse().find(m=>m.audio_url==='/api/agent/audio/'+state.runtime_id+'/'+state.generation)?.audio_url;
      if(!initialized) {lastAudio=audio||'';initialized=true;if(audio){el('audio').src=audio;el('audio').hidden=false;}}
      else if(audio && audio!==lastAudio && el('speak').checked) { lastAudio=audio; el('audio').src=audio;el('audio').hidden=false;el('audio').play().catch(()=>{localError='回答已生成，请点击播放器播放。';controls();}); }
    } catch { localError='Agent 服务连接中断，请检查上位机终端。';controls(); }
    finally { polling=false;pollTimer=setTimeout(poll,400); }
  }
  if(location.hash==='#agent'||!location.hash) document.getElementById('agent-tab').click();
  poll();
})();
