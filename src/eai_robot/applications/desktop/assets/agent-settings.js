'use strict';
(() => {
  const byId = id => document.getElementById(id);
  const dialog = byId('agent-settings'), area = byId('api-stage-fields'), status = byId('api-settings-status');
  const providerNames = {'bailian-beijing':'千问 / 百炼 · 北京','bailian-singapore':'百炼 · 新加坡','deepseek':'DeepSeek','kimi':'Kimi','minimax':'MiniMax','openai':'OpenAI','custom':'自定义兼容服务'};
  let publicSettings = {}, catalog = null, opening = false;
  const friendly = code => ({credential_required_for_new_endpoint:'更换服务或地址后，请填写对应的 API Key。',missing_credential:'请填写 API Key。',invalid_api_settings:'请检查服务地址、模型 ID 和请求参数。',busy:'Agent 正在处理请求，请先停止本轮。',session_limit:'当前会话数量已达上限，请稍后重试。'})[code]||code;
  const field = (stage,name) => byId('api-'+stage+'-'+name);
  function input(stage,name,label,type='text',wide=false) {
    return `<label class="${wide?'wide':''}">${label}<input id="api-${stage}-${name}" type="${type}" autocomplete="${type==='password'?'new-password':'off'}" ${type==='password'?'data-secret="true"':''}></label>`;
  }
  function card(stage,title,note,body) {
    return `<section class="api-stage-card"><label class="api-stage-heading"><input type="checkbox" id="api-${stage}-enabled">${title}<small>${note}</small></label><fieldset id="api-${stage}-fields" class="api-fields">${body}</fieldset></section>`;
  }
  function renderFields() {
    const providers = Object.entries(providerNames).map(([value,label])=>`<option value="${value}">${label}</option>`).join('');
    area.innerHTML = card('chat','聊天与工具','OpenAI 标准协议',
      `<label>服务提供方<select id="api-chat-provider">${providers}</select></label><label>API 协议<select id="api-chat-adapter"><option value="chat_completions">Chat Completions</option><option value="responses">Responses</option></select></label>`+
      input('chat','base_url','Base URL','url',true)+input('chat','model','模型 ID')+input('chat','api_key','API Key','password')+
      `<details><summary>请求参数</summary><div class="api-advanced">${input('chat','max_tokens','最大输出 Token','number')}${input('chat','timeout_s','请求超时（秒）','number')}<label>Token 参数<select id="api-chat-token_limit_parameter"><option value="max_tokens">max_tokens</option><option value="max_completion_tokens">max_completion_tokens</option></select></label><label>model_options（JSON）<textarea id="api-chat-model_options" rows="2"></textarea></label><label class="wide">extra_body（JSON）<textarea id="api-chat-extra_body" rows="2"></textarea></label></div></details>`);
    for(const [stage,title] of [['asr','语音识别 ASR'],['tts','语音合成 TTS']]) {
      area.insertAdjacentHTML('beforeend',card(stage,title,'DashScope 原生语音',
        input(stage,'websocket_url','WebSocket URL','url',true)+input(stage,'model','模型 ID')+input(stage,'api_key','API Key','password')+
        (stage==='tts'?input(stage,'voice','音色 ID'):'')+input(stage,'timeout_s','请求超时（秒）','number')));
    }
    for(const stage of ['chat','asr','tts']) {
      field(stage,'enabled').addEventListener('change',()=>{field(stage,'fields').disabled=!field(stage,'enabled').checked;});
    }
    field('chat','provider').addEventListener('change',()=>{
      const name=field('chat','provider').value, preset=catalog.presets[name]||{};
      field('chat','base_url').value=preset.base_url||'';field('chat','model').value=name==='bailian-beijing'?'qwen-plus':'';
      field('chat','token_limit_parameter').value=preset.token_limit_parameter||'max_completion_tokens';
      field('chat','extra_body').value=JSON.stringify(name==='bailian-beijing'?{enable_thinking:false}:{},null,2);
      field('chat','api_key').value='';field('chat','api_key').placeholder='填写这个服务的 Key';
    });
  }
  function populate() {
    for(const stage of ['chat','asr','tts']) {
      const saved=publicSettings[stage], defaults=stage==='chat'?{provider:'bailian-beijing',adapter:'chat_completions',base_url:catalog.presets['bailian-beijing'].base_url,model:'qwen-plus',max_tokens:1024,timeout_s:30,token_limit_parameter:'max_tokens',model_options:{},extra_body:{enable_thinking:false}}:{model:stage==='asr'?'qwen-audio-3.1-asr-flash-message':'qwen-audio-3.1-tts-flash',voice:'longanhuan_v3.1',websocket_url:'',timeout_s:30};
      const spec={...defaults,...saved};
      field(stage,'enabled').checked=Boolean(saved)||stage==='chat';field(stage,'fields').disabled=!field(stage,'enabled').checked;
      for(const [name,value] of Object.entries(spec)) {
        const target=field(stage,name);if(target && name!=='api_key')target.value=typeof value==='object'?JSON.stringify(value,null,2):value;
      }
      field(stage,'api_key').value='';field(stage,'api_key').placeholder=saved?.key_configured?'已设置 · 留空保留':'填写此服务的 Key';
      if(stage!=='chat')field(stage,'websocket_url').placeholder='wss://工作空间ID.cn-beijing.maas.aliyuncs.com/api-ws/v1/inference';
    }
  }
  async function open() {
    if(opening||dialog.open)return;opening=true;
    try {
      const response=await fetch('/api/agent/state');if(!response.ok)throw new Error('无法读取当前会话');
      publicSettings=(await response.json()).api_settings||{};
      if(!catalog){const result=await fetch('/api/agent/catalog');if(!result.ok)throw new Error('无法读取服务列表');catalog=await result.json();renderFields();}
      populate();status.textContent='';dialog.showModal();
    } catch(error) { status.textContent=error.message;dialog.showModal(); }
    finally {opening=false;}
  }
  for(const id of ['agent-settings-open','agent-settings-sidebar','agent-model']) byId(id).addEventListener('click',open);
  document.addEventListener('agent:settings-open',open);
  byId('api-close').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('close',()=>area.querySelectorAll('[data-secret]').forEach(input=>input.value=''));
  async function request(path,body) {
    const response=await fetch('/api/agent/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Arm-Desk':'1'},body:JSON.stringify(body)});
    const result=await response.json();if(!response.ok)throw new Error(friendly(result.error||'请求未完成'));return result;
  }
  byId('agent-settings-form').addEventListener('submit',async event=>{
    event.preventDefault();byId('api-save').disabled=true;status.textContent='正在应用…';
    try {
      const body={};
      for(const stage of ['chat','asr','tts']) {
        if(!field(stage,'enabled').checked)continue;
        const value=name=>field(stage,name).value.trim();
        if(stage==='chat')body.chat={provider:value('provider'),adapter:value('adapter'),execution:'managed_api',base_url:value('base_url'),model:value('model'),api_key:value('api_key'),streaming:true,max_tokens:Number(value('max_tokens')),timeout_s:Number(value('timeout_s')),token_limit_parameter:value('token_limit_parameter'),model_options:JSON.parse(value('model_options')||'{}'),extra_body:JSON.parse(value('extra_body')||'{}')};
        else body[stage]={provider:'dashscope',websocket_url:value('websocket_url'),model:value('model'),api_key:value('api_key'),timeout_s:Number(value('timeout_s')),...(stage==='tts'?{voice:value('voice')}:{})};
      }
      await request('config',body);dialog.close();document.dispatchEvent(new Event('agent:settings-saved'));
    } catch(error) { status.textContent='未保存：'+(error instanceof SyntaxError?'请求参数需要有效 JSON。':error.message); }
    finally {byId('api-save').disabled=false;area.querySelectorAll('[data-secret]').forEach(input=>input.value='');}
  });
  byId('api-clear').addEventListener('click',async()=>{
    try {await request('clear',{});dialog.close();document.dispatchEvent(new Event('agent:settings-saved'));}
    catch(error){status.textContent='未清除：'+error.message;}
  });
})();
