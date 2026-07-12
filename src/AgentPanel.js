/**
 * AgentPanel.js — 交互型智能体状态展示面板 (演示视频字幕标注)
 *
 * 复用 wheeltec-sim2real 的 RosBridge, 订阅自主作业 agent 发布的话题:
 *   订阅 /agent/status  (std_msgs/String, JSON {status, info})  → 顶部大字幕横幅
 *   订阅 /agent/log     (std_msgs/String, 逐步日志)            → 滚动日志 + 阶段字幕
 *   发布 /agent/instruction (std_msgs/String)                   → 指令输入/快捷按钮
 *
 * 设计目标: 演示视频里观众能直观看到 "感知-决策-执行-验证-重试" 全流程字幕。
 * 自包含: DOM 全部 JS 构建, 不改 index.html; 仅 main.js 加 import + 实例化。
 * 风格与 style.css 深色主题一致; 版本自适应 (ROS1/ROS2 消息类型字符串)。
 */
import ROSLIB from 'roslib';
import { TaskDecomposer } from './TaskDecomposer.js';
import { SpeechRecognizer } from './SpeechRecognizer.js';

const STR_TYPE = {
  ros1: 'std_msgs/String',
  ros2: 'std_msgs/msg/String',
};

// 演示用快捷指令 (一键下发, 录视频用)
const QUICK_CMDS = [
  '把右侧的扳手放到料箱第三格',
  '给我一把螺丝刀',
  '帮我把滚柱放到料箱的第二个格子中',
  '抓取那个螺母',
];

// 阶段 → 字幕文案 (从 log 前缀解析)
function phaseFromLog(line) {
  if (!line) return null;
  if (line.startsWith('>>> 指令')) return { icon: '🗣️', text: '接收自然语言指令', detail: line.replace('>>> 指令:', '').trim() };
  if (line.startsWith('[LLM] 启动')) return { icon: '🧠', text: '启动大模型决策', detail: 'LLM + MCP 工具调用模式' };
  if (line.match(/^\[LLM\] 第\d+轮对话/)) return { icon: '🧠', text: '大模型思考中', detail: line.replace('[LLM]', '').trim() };
  if (line.startsWith('[tool]')) {
    const m = line.match(/^\[tool\]\s*(\w+)/);
    const tool = m?.[1] || '';
    const map = {
      perceive: ['🔍', '感知环境 · 识别工具'],
      get_scene_info: ['📦', '读取场景与料箱信息'],
      get_robot_state: ['📡', '读取机器人状态'],
      move_base: ['🚗', '移动底盘到作业站位'],
      plan_arm_motion: ['🦾', '规划并执行机械臂运动'],
      grasp: ['🤏', '闭合夹爪 · 夹取工具'],
      release: ['📤', '张开夹爪 · 放置物品'],
      retract: ['↩️', '收回机械臂到安全姿态'],
      verify: ['✅', '验证任务结果'],
    };
    const [icon, text] = map[tool] || ['⚙️', '调用 MCP 工具'];
    return { icon, text, detail: line.replace('[tool]', '').trim() };
  }
  if (line.startsWith('[MCP] 执行段')) return { icon: '🦾', text: '执行运动轨迹', detail: line.replace('[MCP]', '').trim() };
  if (line.startsWith('[MCP] 调用工具')) return { icon: '⚙️', text: 'MCP 工具执行中', detail: line.replace('[MCP]', '').trim() };
  if (line.match(/^\[MCP\]\s+\w+\s+→\s+ok/)) return { icon: '✅', text: '工具执行成功', detail: line.replace('[MCP]', '').trim() };
  if (line.match(/^\[MCP\]\s+\w+\s+→\s+fail/)) return { icon: '⚠️', text: '工具执行失败 · 正在恢复', detail: line.replace('[MCP]', '').trim() };
  if (line.startsWith('[perceive]')) return { icon: '🔍', text: '感知环境 · 识别工具', detail: line.replace('[perceive]', '').trim() };
  if (line.startsWith('[nlu]')) return { icon: '🧠', text: '理解自然语言指令', detail: line.replace('[nlu]', '').trim() };
  if (line.startsWith('[plan]')) return { icon: '📋', text: '分解作业任务序列', detail: line.replace('[plan]', '').trim() };
  if (line.startsWith('[exec')) {
    const m = line.match(/\[exec (\d+)\]\s*(\S+)\(.*?\)\s*→\s*ok=(\w+)/);
    if (m) {
      const step = parseInt(m[1]) + 1;
      const skill = m[2];
      const ok = m[3] === 'True';
      return { icon: '⚙️', text: `执行步骤 ${step}`, detail: `${skill} ${ok ? '✓' : '✗'}` };
    }
    return { icon: '⚙️', text: '执行任务序列', detail: line.replace(/^\[exec \d+\]\s*/, '').trim() };
  }
  if (line.startsWith('[replan]')) return { icon: '🔁', text: '失败感知 · 自主重试', detail: line.replace('[replan]', '').trim() };
  if (line.startsWith('<<< 结果')) {
    const ok = line.includes('done');
    return { icon: ok ? '✅' : '❌', text: ok ? '任务完成' : '任务失败', detail: line.replace('<<< 结果:', '').trim() };
  }
  return null;
}

export class AgentPanel {
  /**
   * @param {Object} ros  RosBridge 实例 (需有 .ros, .version, .connected)
   */
  constructor(ros) {
    this.ros = ros;
    this._statusSub = null;
    this._logSub = null;
    this._toolAnnoSub = null;
    this._instrPub = null;
    this._lastConn = false;
    this._lastVer = null;
    this._logs = [];          // 滚动日志缓冲
    this._objects = [];       // 检测物体 (从 perceive log 解析)
    this._busy = false;

    this._buildDOM();
    // 轻量轮询: 连接/版本变化时重建订阅 (自包含, 不侵入 UIController)
    this._poll = setInterval(() => this._syncSubscriptions(), 1000);
    this._syncSubscriptions();
  }

  // ─────────────── DOM 构建 ───────────────
  _buildDOM() {
    // 1. 顶部大字幕横幅 (演示视频字幕)
    this.banner = document.createElement('div');
    this.banner.id = 'agent-banner';
    this.banner.innerHTML = `
      <span class="ab-icon">🤖</span>
      <span class="ab-text">等待指令…</span>
      <span class="ab-detail"></span>`;
    document.body.appendChild(this.banner);

    // 2. 控制台面板 (指令输入 + 快捷按钮 + 日志 + 物体列表)
    this.console = document.createElement('div');
    this.console.id = 'agent-console';
    this.console.innerHTML = `
  <div class="ac-head">
    <span class="ac-title">🤖 交互型智能体</span>
    <span class="ac-status" id="ac-status">未连接</span>
    <button class="mini-btn" id="ac-collapse">折叠</button>
  </div>
      <div class="ac-body">
        <div class="ac-section">
          <div class="ac-row">
            <input type="text" id="ac-instr" placeholder="输入自然语言指令, 如 把左侧的扳手放到料箱第三格" />
            <button id="ac-mic" class="ac-mic-btn" title="按住说话 (语音识别)">🎤</button>
            <button id="ac-send">发送</button>
            <button id="ac-stop" disabled style="background:#422;color:#F88;border:1px solid #644">停止</button>
          </div>
          <div class="ac-quick" id="ac-quick"></div>
        </div>
        <div class="ac-section">
          <div class="ac-subtitle">检测物体</div>
          <div class="ac-objects" id="ac-objects">—</div>
        </div>
        <div class="ac-section">
          <div class="ac-subtitle">任务日志</div>
          <div class="ac-log" id="ac-log"></div>
        </div>
      </div>`;
    document.body.appendChild(this.console);

    // 快捷按钮
    const quick = this.console.querySelector('#ac-quick');
    QUICK_CMDS.forEach(cmd => {
      const b = document.createElement('button');
      b.className = 'ac-quick-btn';
      b.textContent = cmd;
      b.addEventListener('click', () => this.sendInstruction(cmd));
      quick.appendChild(b);
    });
    // 工控按钮: 重置 + 模拟失败
    const ctrlRow = document.createElement('div');
    ctrlRow.className = 'ac-ctrl-row';
    const btnReset = document.createElement('button');
    btnReset.className = 'ac-quick-btn ctrl';
    btnReset.textContent = '重置场景';
    btnReset.onclick = () => { if (this._mockAgent) this._mockAgent.resetScene(); };
    ctrlRow.appendChild(btnReset);

    const btnSpawn = document.createElement('button');
    btnSpawn.className = 'ac-quick-btn ctrl';
    btnSpawn.textContent = '生成测试工具';
    btnSpawn.onclick = () => { if (this._mockAgent) this._mockAgent.spawnTestTools(); };
    ctrlRow.appendChild(btnSpawn);

    quick.appendChild(ctrlRow);

    // LLM 大模型配置区
    const llmRow = document.createElement('div');
    llmRow.className = 'ac-llm-config';
    const llmLbl = document.createElement('span');
    llmLbl.className = 'ac-llm-label';
    llmLbl.textContent = '大模型 + MCP 工具调用';
    llmRow.appendChild(llmLbl);
    // API Base
    const inpBase = document.createElement('input');
    inpBase.type = 'text';
    inpBase.className = 'ac-llm-input';
    inpBase.placeholder = 'API Base (如 https://api.openai.com/v1)';
    inpBase.value = localStorage.getItem('llm_api_base') || 'https://api.openai.com/v1';
    llmRow.appendChild(inpBase);
    // API Key
    const inpKey = document.createElement('input');
    inpKey.type = 'password';
    inpKey.className = 'ac-llm-input';
    inpKey.placeholder = 'API Key';
    inpKey.value = localStorage.getItem('llm_api_key') || '';
    llmRow.appendChild(inpKey);
    // Model
    const inpModel = document.createElement('input');
    inpModel.type = 'text';
    inpModel.className = 'ac-llm-input';
    inpModel.placeholder = 'Model (如 gpt-4o)';
    inpModel.value = localStorage.getItem('llm_model') || 'gpt-4o';
    llmRow.appendChild(inpModel);
    // 保存按钮
    const btnSaveLLM = document.createElement('button');
    btnSaveLLM.className = 'ac-llm-btn save';
    btnSaveLLM.textContent = '保存并启用 LLM 模式';
    btnSaveLLM.onclick = () => {
      const base = inpBase.value.trim();
      const key = inpKey.value.trim();
      const model = inpModel.value.trim();
      localStorage.setItem('llm_api_base', base);
      localStorage.setItem('llm_api_key', key);
      localStorage.setItem('llm_model', model);
      if (this._llmAgent) {
        this._llmAgent.apiBase = base;
        this._llmAgent.apiKey = key;
        this._llmAgent.model = model;
      }
      const mode = key ? 'LLM 大模型模式' : '未配置 (需API Key)';
      this._pushLog(`[llm] 配置已保存 → ${mode}, model=${model}`);
      this._refreshLLMStatus();
    };
    llmRow.appendChild(btnSaveLLM);

    // 验证连接按钮
    const btnTestLLM = document.createElement('button');
    btnTestLLM.className = 'ac-llm-btn test';
    btnTestLLM.textContent = '验证大模型连接';
    btnTestLLM.onclick = async () => {
      const base = inpBase.value.trim();
      const key = inpKey.value.trim();
      const model = inpModel.value.trim();
      if (!key) {
        this._pushLog('[llm] 未填写 API Key, 无法验证');
        return;
      }
      btnTestLLM.disabled = true;
      btnTestLLM.textContent = '验证中...';
      btnTestLLM.classList.remove('ok', 'fail');
      this._pushLog(`[llm] 正在连接 ${base} (model=${model})...`);
      try {
        const resp = await fetch(`${base}/chat/completions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${key}`,
          },
          body: JSON.stringify({
            model: model,
            messages: [
              { role: 'system', content: '你是工业机器人助手, 请简短回复。' },
              { role: 'user', content: '请回复: 连接成功' },
            ],
            max_tokens: 50,
            temperature: 0,
          }),
        });
        if (resp.ok) {
          const data = await resp.json();
          const reply = data.choices?.[0]?.message?.content || '(空回复)';
          this._pushLog(`[llm] ✓ 连接成功! 模型回复: ${reply}`);
          btnTestLLM.textContent = '✓ 已连接';
          btnTestLLM.classList.add('ok');
          // 自动保存配置
          localStorage.setItem('llm_api_base', base);
          localStorage.setItem('llm_api_key', key);
          localStorage.setItem('llm_model', model);
          if (this._llmAgent) {
            this._llmAgent.apiBase = base;
            this._llmAgent.apiKey = key;
            this._llmAgent.model = model;
          }
          this._refreshLLMStatus(true);
        } else {
          const errText = await resp.text();
          const errShort = errText.substring(0, 150);
          this._pushLog(`[llm] ✗ 连接失败: HTTP ${resp.status} — ${errShort}`);
          btnTestLLM.textContent = '✗ 连接失败';
          btnTestLLM.classList.add('fail');
        }
      } catch (e) {
        this._pushLog(`[llm] ✗ 网络错误: ${e.message}`);
        btnTestLLM.textContent = '✗ 网络错误';
        btnTestLLM.classList.add('fail');
      } finally {
        btnTestLLM.disabled = false;
        setTimeout(() => {
          btnTestLLM.textContent = '验证大模型连接';
          btnTestLLM.classList.remove('ok', 'fail');
        }, 5000);
      }
    };
    llmRow.appendChild(btnTestLLM);

    // 模式快捷切换 — 云端 API / 本地外置机大模型 (OpenAI 兼容, 推理在外置 RTX)
    const modeRow = document.createElement('div');
    modeRow.className = 'ac-llm-mode';
    const inpHost = document.createElement('input');
    inpHost.type = 'text';
    inpHost.className = 'ac-llm-input ac-host';
    inpHost.placeholder = '外置机 IP (本地模式)';
    inpHost.value = localStorage.getItem('llm_local_host') || '192.168.0.100';
    modeRow.appendChild(inpHost);
    const btnCloud = document.createElement('button');
    btnCloud.className = 'ac-llm-btn mode';
    btnCloud.textContent = '☁ API';
    btnCloud.onclick = () => {
      btnCloud.classList.add('active');
      btnLocal.classList.remove('active');
      inpBase.value = 'https://api.openai.com/v1';
      if (!inpModel.value || inpModel.value.startsWith('qwen')) inpModel.value = 'gpt-4o';
      this._applyLLMConfig(inpBase.value, inpKey.value, inpModel.value);
      this._pushLog(`[llm] 切换 → 云端 API 模式: base=${inpBase.value} model=${inpModel.value}`);
    };
    modeRow.appendChild(btnCloud);
    const btnLocal = document.createElement('button');
    btnLocal.className = 'ac-llm-btn mode';
    btnLocal.textContent = '🖥 本地';
    btnLocal.title = 'Ollama :11434/v1 (默认) / vLLM :8001/v1';
    btnLocal.onclick = () => {
      btnLocal.classList.add('active');
      btnCloud.classList.remove('active');
      const host = inpHost.value.trim() || '192.168.0.100';
      inpBase.value = `http://${host}:11434/v1`;
      inpModel.value = 'qwen2.5:3b';
      if (!inpKey.value) inpKey.value = 'ollama';
      localStorage.setItem('llm_local_host', host);
      this._applyLLMConfig(inpBase.value, inpKey.value, inpModel.value);
      this._pushLog(`[llm] 切换 → 本地外置机大模型: base=${inpBase.value} model=${inpModel.value} (vLLM 改 :8001/v1)`);
    };
    modeRow.appendChild(btnLocal);
    // 初始 active 状态: 根据当前 apiBase 判断
    {
      const savedBase = (localStorage.getItem('llm_api_base') || '').toLowerCase();
      if (savedBase.includes('127.0.0.1') || savedBase.includes('localhost') ||
          savedBase.match(/\d+\.\d+\.\d+\.\d+/) || savedBase.includes(':11434') || savedBase.includes(':8001')) {
        btnLocal.classList.add('active');
      } else {
        btnCloud.classList.add('active');
      }
    }
    llmRow.appendChild(modeRow);
    quick.appendChild(llmRow);

    // 事件
    this.console.querySelector('#ac-send').addEventListener('click', () => this._onSend());
    this.console.querySelector('#ac-stop').addEventListener('click', () => this._onStop());
    this.console.querySelector('#ac-instr').addEventListener('keydown', (e) => {
      if (e.key === 'Enter') this._onSend();
    });
    this.console.querySelector('#ac-collapse').addEventListener('click', () => {
      this.console.classList.toggle('collapsed');
    });

    // 语音识别 (PTT 按住说话): 按下开始录音, 松开停止并上传到外置机 ASR
    this._initSpeechRecognizer(inpHost);
  }

  /** 初始化语音识别模块 + PTT 事件绑定 */
  _initSpeechRecognizer(inpHost) {
    const micBtn = this.console.querySelector('#ac-mic');
    if (!micBtn) return;

    // ASR server URL: http://<外置机IP>:8766, 复用 LLM 模式区的外置机 IP 输入
    const host = (inpHost && inpHost.value) || localStorage.getItem('llm_local_host') || '192.168.0.100';
    this._speechRecognizer = new SpeechRecognizer({
      serverUrl: `http://${host}:8766`,
      onText: (text, meta) => this._onAsrText(text, meta),
      onState: (state) => this._onAsrState(state),
      onError: (msg) => this._onAsrError(msg),
    });

    // 浏览器不支持时禁用按钮
    if (!this._speechRecognizer.isSupported()) {
      micBtn.disabled = true;
      micBtn.title = '浏览器不支持语音录制';
      micBtn.style.opacity = '0.4';
      return;
    }

    // 外置机 IP 变化时同步更新 ASR 地址
    if (inpHost) {
      inpHost.addEventListener('change', () => {
        const h = inpHost.value.trim() || '192.168.0.100';
        this._speechRecognizer.setServerUrl(`http://${h}:8766`);
      });
    }

    // PTT: 按下开始录音, 松开停止并上传 (鼠标 + 触摸)
    const startRec = (e) => { e.preventDefault(); this._speechRecognizer.start(); };
    const stopRec = (e) => { e.preventDefault(); this._speechRecognizer.stop(); };
    micBtn.addEventListener('mousedown', startRec);
    micBtn.addEventListener('mouseup', stopRec);
    micBtn.addEventListener('mouseleave', stopRec);
    micBtn.addEventListener('touchstart', startRec, { passive: false });
    micBtn.addEventListener('touchend', stopRec, { passive: false });
  }

  /** ASR 识别成功: 填入输入框并自动发送 */
  _onAsrText(text, meta) {
    const inp = this.console.querySelector('#ac-instr');
    if (inp) inp.value = text;
    const emoStr = meta && meta.emotion ? ` 情绪=${meta.emotion}` : '';
    const langStr = meta && meta.language ? ` 语言=${meta.language}` : '';
    this._pushLog(`[asr] 语音识别: "${text}"${langStr}${emoStr} (${meta ? Math.round(meta.latency_ms) : '?'}ms)`);
    this._setBanner('🎤', '语音指令', text);
    this.sendInstruction(text);
  }

  /** ASR 状态变化: 更新麦克风按钮视觉 */
  _onAsrState(state) {
    const micBtn = this.console.querySelector('#ac-mic');
    if (!micBtn) return;
    micBtn.classList.remove('recording', 'processing');
    if (state === 'recording') {
      micBtn.classList.add('recording');
      micBtn.textContent = '◉';
      micBtn.title = '录音中…松开发送';
    } else if (state === 'processing') {
      micBtn.classList.add('processing');
      micBtn.textContent = '⏳';
      micBtn.title = '识别中…';
    } else {
      micBtn.textContent = '🎤';
      micBtn.title = '按住说话 (语音识别)';
    }
  }

  /** ASR 错误 */
  _onAsrError(msg) {
    const micBtn = this.console.querySelector('#ac-mic');
    if (micBtn) {
      micBtn.textContent = '🎤';
      micBtn.classList.remove('recording', 'processing');
    }
    this._pushLog(`[asr] ⚠ ${msg}`);
    this._setBanner('⚠️', '语音识别失败', msg);
    this.banner.classList.add('warn');
    setTimeout(() => this.banner.classList.remove('warn'), 4000);
  }

  /** 停止运行中的 LLM 对话 */
  _onStop() {
    if (this._llmAgent) this._llmAgent.stop();
    this._busy = false;
    this._pushLog('[local] 用户已停止对话');
    this._setBanner('⏹️', '对话已停止', '用户手动终止');
    this.banner.classList.remove('warn');
    this._updateRunButtons(false);
  }

  /** 同步发送/停止按钮可用状态 */
  _updateRunButtons(running) {
    const send = this.console.querySelector('#ac-send');
    const stop = this.console.querySelector('#ac-stop');
    if (send) send.disabled = running;
    if (stop) stop.disabled = !running;
  }

  _onSend() {
    const inp = this.console.querySelector('#ac-instr');
    const text = inp.value.trim();
    if (text) this.sendInstruction(text);
    inp.value = '';
  }

  // ─────────────── 订阅/发布 ───────────────
  _syncSubscriptions() {
    const conn = this.ros.connected;
    const ver = this.ros.version;
    if (conn === this._lastConn && ver === this._lastVer) return;
    this._lastConn = conn;
    this._lastVer = ver;
    this._setStatusUI(conn);
    this._stopSubs();
    if (conn && this.ros.ros) this._startSubs();
  }

  _setStatusUI(conn) {
    const el = this.console.querySelector('#ac-status');
    if (conn) { el.textContent = '已连接 · ROS2'; el.className = 'ac-status ok'; }
    else { el.textContent = '未连接'; el.className = 'ac-status'; }
  }

  _startSubs() {
    const t = STR_TYPE[this.ros.version || 'ros2'];
    this._statusSub = new ROSLIB.Topic({
      ros: this.ros.ros, name: '/agent/status', messageType: t, throttle_rate: 50,
    });
    this._statusSub.subscribe((msg) => this._onStatus(msg.data));
    this._logSub = new ROSLIB.Topic({
      ros: this.ros.ros, name: '/agent/log', messageType: t, throttle_rate: 30,
    });
    this._logSub.subscribe((msg) => this._onLog(msg.data));
    this._instrPub = new ROSLIB.Topic({
      ros: this.ros.ros, name: '/agent/instruction', messageType: t, queue_size: 10,
    });
    // LocateAnything 自然语言 prompt 下发 (perceive(query) → Jetson locate_anything_client)
    this._locateQueryPub = new ROSLIB.Topic({
      ros: this.ros.ros, name: '/locate/query', messageType: t, queue_size: 10,
    });
    this._toolAnnoSub = new ROSLIB.Topic({
      ros: this.ros.ros, name: '/industrial_tools/annotations', messageType: t, throttle_rate: 200,
    });
    this._toolAnnoSub.subscribe((msg) => this._onToolAnnotations(msg.data));
  }

  _stopSubs() {
    [this._statusSub, this._logSub, this._toolAnnoSub].forEach(s => {
      if (s) { try { s.unsubscribe(); } catch (e) {} }
    });
    this._statusSub = null; this._logSub = null; this._toolAnnoSub = null;
    this._locateQueryPub = null;
  }

  /** 下发 LocateAnything 自然语言 prompt 到 Jetson (/locate/query) */
  publishLocateQuery(query) {
    if (!this._locateQueryPub) {
      this._pushLog('[locate] 未连接 ROS, 无法下发 prompt (前端将用虚拟场景感知)');
      return;
    }
    try {
      this._locateQueryPub.publish({ data: query || '' });
      this._pushLog(`[locate] 已下发 prompt: ${query}`);
    } catch (e) {
      this._pushLog(`[locate] 下发失败: ${e.message}`);
    }
  }

  /** ROS 版本切换后重建 (消息类型字符串变了) */
  rebuild() {
    this._lastVer = null;     // 强制 _syncSubscriptions 重建
    this._syncSubscriptions();
  }

  /** 设置仿真智能体 (浏览器内跑全流程) */
  setMockAgent(agent) {
    this._mockAgent = agent;
    this._refreshLLMStatus();
    // 快捷按钮直接可用 (不依赖 ROS 连接)
  }

  /** 根据 localStorage 自动检查并刷新 LLM 配置状态显示 (解决刷新后丢失状态) */
  _refreshLLMStatus(connected = false) {
    const el = this.console.querySelector('#ac-status');
    if (!el) return;
    const savedKey = localStorage.getItem('llm_api_key') || '';
    if (connected) {
      el.textContent = 'LLM · 已连接 ✓';
      el.className = 'ac-status ok';
    } else if (savedKey) {
      const model = localStorage.getItem('llm_model') || 'gpt-4o';
      el.textContent = `LLM · ${model}`;
      el.className = 'ac-status ok';
    } else {
      el.textContent = '待配置 API Key';
      el.className = 'ac-status';
    }
  }

  /** 设置 LLM 智能体 */
  setLLMAgent(agent) {
    this._llmAgent = agent;
  }

  /** 设置指令解析模块 (InstructionParser) */
  setInstructionParser(parser) {
    this._instructionParser = parser;
  }

  /** 设置任务分解模块 (TaskDecomposer) */
  setTaskDecomposer(decomposer) {
    this._taskDecomposer = decomposer;
  }

  /** 保存 LLM 配置并同步到 agent + parser (供模式快捷按钮复用) */
  _applyLLMConfig(base, key, model) {
    localStorage.setItem('llm_api_base', base);
    localStorage.setItem('llm_api_key', key);
    localStorage.setItem('llm_model', model);
    if (this._llmAgent) {
      this._llmAgent.apiBase = base;
      this._llmAgent.apiKey = key;
      this._llmAgent.model = model;
    }
    if (this._instructionParser) this._instructionParser.setConfig({ apiBase: base, apiKey: key, model });
    this._refreshLLMStatus();
  }

  /** 发送自然语言指令 */
  async sendInstruction(text) {
    // 新任务 → 清空日志面板, 确保本次对话完整显示
    this._clearLog();
    // 指令解析模块 (LLM 云端/本地外置机, 或正则回退)
    // 提取 动作/目标工具/位置/料箱格 + 视觉定位 query, 并提前下发到 LocateAnything
    let plan = null;
    if (this._instructionParser) {
      try {
        plan = await this._instructionParser.parse(text);
        if (plan && plan.ok) {
          this._pushLog(`[NLU] 指令解析 (${plan.source}): 动作=${plan.action} 目标=${plan.target_tool || '按query定位'} 位置=${plan.side || '任意'} 格子=${plan.slot || '无'} query="${plan.query}"`);
          if (plan.query) this.publishLocateQuery(plan.query);
          // 任务序列分解: 生成显式步骤列表并展示
          this._showTaskSteps(plan);
        } else {
          this._pushLog(`[NLU] 解析失败: ${plan?.reason || '未知'}, 交由智能体自行理解`);
          this._clearTaskSteps();
        }
      } catch (e) {
        this._pushLog(`[NLU] 解析异常: ${e.message}`);
        this._clearTaskSteps();
      }
    }
    if (this._mockAgent) {
      this._pushLog(`[local] 已发送指令: ${text}`);
      this._setBanner('🗣️', '接收自然语言指令', text);
      this._busy = true;
      this.banner.classList.add('active');
      this._updateRunButtons(true);
      this._mockAgent.run(text, plan);
      return;
    }
    if (!this._instrPub || !this.ros.connected) {
      this._pushLog('[local] 未连接 ROS, 无法发送指令');
      return;
    }
    this._instrPub.publish(new ROSLIB.Message({ data: text }));
    this._pushLog(`[local] 已发送指令: ${text}`);
    this._setBanner('🗣️', '接收自然语言指令', text);
    this._busy = true;
    this.banner.classList.add('active');
    this._updateRunButtons(true);
  }

  // ─────────────── 消息处理 ───────────────
  _onStatus(data) {
    let status = 'running', info = '';
    try { const j = JSON.parse(data); status = j.status || 'running'; info = j.info || ''; }
    catch (e) { info = data; }
    const map = {
      running: ['🤖', '智能体运行中', info],
      done:    ['✅', '任务完成', info],
      failed:  ['❌', '任务失败', info],
    };
    const m = map[status] || ['🤖', status, info];
    this._setBanner(m[0], m[1], m[2]);
    this.banner.classList.toggle('success', status === 'done');
    this.banner.classList.toggle('fail', status === 'failed');
    if (status === 'done' || status === 'failed') {
      this._busy = false;
      this._updateRunButtons(false);
    }
  }

  _onLog(line) {
    this._pushLog(line);
    // 任务步骤进度跟踪
    this._updateTaskStepsFromLog(line);
    // 解析阶段 → 更新字幕
    const ph = phaseFromLog(line);
    if (ph) this._setBanner(ph.icon, ph.text, ph.detail);
    // 解析检测物体
    if (line.startsWith('[perceive]') && line.includes('objs:')) {
      this._parseObjects(line);
    }
    // 失败重试高亮
    if (line.startsWith('[replan]')) this.banner.classList.add('warn');
    else if (!line.startsWith('[replan]')) this.banner.classList.remove('warn');
  }

  _parseObjects(line) {
    // 形如 [perceive] N objs: wrench@[0.25, 0.05, 0.02], nut@[...]
    const after = line.split('objs:')[1] || '';
    const objs = [];
    const re = /(\w+)@\[([^\]]*)\]/g;
    let m;
    while ((m = re.exec(after)) !== null) {
      objs.push({ class: m[1], xyz: m[2].trim() });
    }
    if (objs.length) this._objects = objs;
    this._renderObjects();
  }

  _onToolAnnotations(data) {
    try {
      const payload = JSON.parse(data);
      const objects = Array.isArray(payload.objects) ? payload.objects : [];
      // 注入真实检测结果到 MockAgent (同步 3D 场景 + this.tools, sim2real 闭环)
      // 只接受有 position_world_m 的检测结果 (TF 变换成功的世界坐标)
      // TF 失败时的 camera-frame 坐标不能作为世界坐标, 否则物体位置完全错误
      if (this._mockAgent) {
        const validObjs = objects.filter(o => Array.isArray(o.position_world_m) && o.position_world_m.length === 3);
        if (validObjs.length < objects.length && validObjs.length === 0) {
          this._pushLog(`[perception] ⚠ ${objects.length - validObjs.length} 个物体无世界坐标 (TF 变换失败), 跳过`);
        }
        this._mockAgent.setRealAnnotations(
          validObjs.map(o => ({
            class: o.class || 'unknown',
            class_cn: o.class_cn,
            confidence: o.confidence,
            position_world_m: o.position_world_m,
            bbox_xywh: o.bbox_xywh,
          }))
        );
      }
      this._objects = objects.map(o => ({
        class: o.class_cn || o.class || 'unknown',
        conf: o.confidence,
        xyz: Array.isArray(o.position_world_m)
          ? o.position_world_m.map(v => Number(v).toFixed(3)).join(', ')
          : (Array.isArray(o.position_table_m)
            ? o.position_table_m.map(v => Number(v).toFixed(3)).join(', ') + ' [cam]'
            : 'N/A'),
        bbox: Array.isArray(o.bbox_xywh) ? o.bbox_xywh.join(',') : '',
        real: true,
      }));
      this._renderObjects();
    } catch (e) {
      this._pushLog(`[perception] annotations parse error: ${e.message}`);
    }
  }

  // ─────────────── 渲染 ───────────────

  /** 轻量 Markdown 渲染: 将 **bold** 转为 <b>, 安全转义 HTML */
  _renderMd(str) {
    if (!str) return '';
    return str
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/\*\*(.+?)\*\*/g, '<b>$1</b>')
      .replace(/`([^`]+)`/g, '<code>$1</code>');
  }

  _setBanner(icon, text, detail) {
    this.banner.querySelector('.ab-icon').textContent = icon;
    this.banner.querySelector('.ab-text').textContent = text;
    this.banner.querySelector('.ab-detail').innerHTML = this._renderMd(detail) || '';
    this.banner.classList.add('active');
    clearTimeout(this._bannerTimer);
    // 运行中常驻; 完成/失败常驻; 阶段切换 1.5s 后淡出 (仅当非 busy)
    if (!this._busy) {
      this._bannerTimer = setTimeout(() => this.banner.classList.remove('active'), 1500);
    }
  }

  _clearLog() {
    this._logs = [];
    const box = this.console.querySelector('#ac-log');
    if (box) box.innerHTML = '';
  }

  _pushLog(line) {
    this._logs.push(line);
    const box = this.console.querySelector('#ac-log');
    const div = document.createElement('div');
    div.className = 'ac-log-line';
    // 颜色编码
    if (line.includes('→ ok=True') || line.includes('✅') || line.includes('完成')) div.classList.add('ok');
    else if (line.includes('ok=False') || line.includes('失败') || line.includes('[replan]')) div.classList.add('warn');
    else if (line.startsWith('[exec')) div.classList.add('exec');
    else if (line.startsWith('[perceive]')) div.classList.add('sense');
    else if (line.startsWith('[plan]') || line.startsWith('[nlu]')) div.classList.add('plan');
    // LLM 文本响应 (排除系统消息, 用独特样式高亮模型输出)
    else if (line.startsWith('[LLM]') && !/\[LLM\]\s*(第\d+轮|启动|模型返回空|模型调用异常|工具参数|⚠|同一目标|连续|guard|图执行|异常|提示计算)/.test(line)) {
      div.classList.add('llm-resp');
    }
    // 渲染 Markdown (**bold** → <b>, 安全转义)
    div.innerHTML = this._renderMd(line);
    box.appendChild(div);
    box.scrollTop = box.scrollHeight;
  }

  /** 从 perceive 工具结果更新物体显示 (LLM/MockAgent 模式) */
  updateObjects(perceiveResult) {
    if (!perceiveResult || !Array.isArray(perceiveResult.objects)) return;
    this._objects = perceiveResult.objects.map(o => ({
      class: o.class || 'unknown',
      conf: o.confidence,
      xyz: o.position ? [o.position.x, o.position.y, o.position.z].map(v => Number(v).toFixed(3)).join(', ') : '',
      reachable: o.reachable,
      real: !!o.real,
    }));
    this._renderObjects();
  }

  _renderObjects() {
    const el = this.console.querySelector('#ac-objects');
    if (!this._objects.length) { el.innerHTML = '—'; return; }
    el.innerHTML = this._objects.map(o => {
      const conf = o.conf != null ? ` conf=${Number(o.conf).toFixed(2)}` : '';
      const bbox = o.bbox ? ` bbox=[${o.bbox}]` : '';
      const reach = o.reachable === false ? ' <span style="color:#ffaa55">[远]</span>' : '';
      const tag = o.real ? ' <span style="color:#4af">[YOLO]</span>' : '';
      return `<span class="ac-obj"><b>${o.class}</b> @[${o.xyz}]${conf}${bbox}${reach}${tag}</span>`;
    }).join('');
  }

  // ─────────────── 任务序列分解 (显式步骤列表) ───────────────

  /** 生成并展示任务步骤列表 */
  _showTaskSteps(plan) {
    if (!this._taskDecomposer) return;
    this._taskSteps = this._taskDecomposer.decompose(plan);
    for (const s of this._taskSteps) s.retryCount = 0;
    this._renderTaskSteps();
  }

  /** 清空任务步骤 */
  _clearTaskSteps() {
    this._taskSteps = null;
    const el = document.querySelector('#ac-steps');
    if (el) el.innerHTML = '—';
  }

  /** 渲染任务步骤列表 */
  _renderTaskSteps() {
    const el = document.querySelector('#ac-steps');
    if (!el) return;
    if (!this._taskSteps || !this._taskSteps.length) { el.innerHTML = '—'; return; }
    const icons = { pending: '○', running: '◉', done: '✓', failed: '✗', skipped: '◌', retry: '↻' };
    el.innerHTML = this._taskSteps.map(s => {
      const opt = s.optional ? '<span class="ac-step-opt">可选</span>' : '';
      const cond = s.conditional ? '<span class="ac-step-opt">条件</span>' : '';
      const retry = s.retryCount > 0 ? `<span class="ac-step-opt">重试${s.retryCount}</span>` : '';
      return `<div class="ac-step ac-step-${s.status}">
        <span class="ac-step-icon">${icons[s.status] || '○'}</span>
        <div class="ac-step-body">
          <div class="ac-step-title">${s.id}. ${s.title} ${opt}${cond}${retry}</div>
          <div class="ac-step-desc">${s.description}</div>
        </div>
      </div>`;
    }).join('');
  }

  /** 从日志行更新任务步骤状态 */
  _updateTaskStepsFromLog(line) {
    if (!this._taskSteps || !this._taskSteps.length) return;

    // [replan] → 失败重试步骤触发
    if (line.startsWith('[replan]')) {
      const replanStep = this._taskSteps.find(s => s.tool === 'replan' && s.status !== 'done');
      if (replanStep) {
        this._markStepRunning(replanStep);
        for (const s of this._taskSteps) {
          if (s.tool === 'grasp' || s.tool === 'release' || s.tool === 'plan_arm_motion') {
            if (s.status === 'done') s.status = 'pending';
          }
        }
        this._renderTaskSteps();
      }
      return;
    }

    // <<< 结果: done/failed → 全部收尾
    if (line.startsWith('<<< 结果')) {
      const ok = line.includes('done');
      for (const s of this._taskSteps) {
        if (s.status === 'running' || s.status === 'retry') s.status = ok ? 'done' : 'failed';
        else if (s.status === 'pending' && !s.conditional) s.status = ok ? 'skipped' : 'pending';
      }
      this._renderTaskSteps();
      return;
    }

    // NLU 模式: [nlu] 步骤N:
    const nluStep = TaskDecomposer?.parseNLUStep?.(line);
    if (nluStep) {
      const tool = TaskDecomposer.NLU_STEP_TOOLS[nluStep];
      if (tool) {
        const prevRunning = this._taskSteps.find(s => s.status === 'running');
        if (prevRunning) prevRunning.status = 'done';
        const target = this._findNextPending(tool);
        if (target) target.status = 'running';
        this._renderTaskSteps();
      }
      return;
    }

    // LLM 模式: [MCP] 日志
    const mcp = TaskDecomposer?.parseMCPLog?.(line);
    if (!mcp) return;

    if (mcp.type === 'call') {
      // 1. 先找 failed (重试当前阶段失败的同名步骤)
      const failedStep = this._taskSteps.find(s => s.tool === mcp.tool && s.status === 'failed');
      if (failedStep) {
        // 重试: 标记前一个 running/retry 步骤为完成, 再显示 ↻ 重试中
        const prevRunning = this._taskSteps.find(s => s.status === 'running' || s.status === 'retry');
        if (prevRunning && prevRunning !== failedStep) prevRunning.status = 'done';
        failedStep.retryCount = (failedStep.retryCount || 0) + 1;
        failedStep.status = 'retry';
        this._renderTaskSteps();
        return;
      }
      // 2. 再找 pending (新步骤), 但只在 high water mark 之后
      let target = this._findNextPending(mcp.tool);
      if (target) {
        const prevRunning = this._taskSteps.find(s => s.status === 'running' || s.status === 'retry');
        if (prevRunning && prevRunning !== target) prevRunning.status = 'done';
        target.status = 'running';
        this._renderTaskSteps();
      }
    } else if (mcp.type === 'result') {
      const target = this._taskSteps.find(s => s.tool === mcp.tool && (s.status === 'running' || s.status === 'retry'));
      if (target) {
        target.status = mcp.ok ? 'done' : 'failed';
        this._renderTaskSteps();
      }
    }
  }

  /**
   * 在 high water mark (最后一个 done/skipped 步骤) 之后,
   * 找第一个 pending 且 tool 匹配的步骤。
   * 这样步骤2 (grasp阶段 move_base) 未执行时, 步骤7 (place阶段) 失败后
   * move_base 会正确匹配步骤6而非步骤2。
   */
  _findNextPending(tool) {
    let hwm = -1;
    for (let i = 0; i < this._taskSteps.length; i++) {
      const st = this._taskSteps[i].status;
      if (st === 'done' || st === 'skipped') hwm = i;
    }
    for (let i = hwm + 1; i < this._taskSteps.length; i++) {
      if (this._taskSteps[i].tool === tool && this._taskSteps[i].status === 'pending') {
        return this._taskSteps[i];
      }
    }
    return null;
  }

  /** 标记步骤为 running */
  _markStepRunning(step) {
    if (!step) return;
    step.status = 'running';
    this._renderTaskSteps();
  }
}
