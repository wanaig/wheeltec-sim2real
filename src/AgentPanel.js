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
import { LAYOUT_NAMES, LAYOUT_CN } from './SceneLayouts.js';

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
    ctrlRow.style.cssText = 'display:flex;gap:4px;margin-top:4px';
    const btnReset = document.createElement('button');
    btnReset.className = 'ac-quick-btn';
    btnReset.style.cssText = 'background:#333;color=#0F0;border-color:#080';
    btnReset.textContent = '重置场景';
    btnReset.onclick = () => { if (this._mockAgent) this._mockAgent.resetScene(); };
    ctrlRow.appendChild(btnReset);
    const btnFail = document.createElement('button');
    btnFail.className = 'ac-quick-btn';
    btnFail.style.cssText = 'background:#330;color:#FF0;border-color:#440';
    btnFail.textContent = '模拟放置失败';
    btnFail.onclick = () => {
      if (this._mockAgent) {
        this._mockAgent._injectFailure = true;
        this._pushLog('[local] 已设置: 下次放置将偏移 (模拟未入格)');
      }
    };
    ctrlRow.appendChild(btnFail);
    quick.appendChild(ctrlRow);

    // 场景布局切换 + 数据集生成
    const sceneRow = document.createElement('div');
    sceneRow.style.cssText = 'display:flex;flex-wrap:wrap;gap:4px;margin-top:4px;border-top:1px solid #333;padding-top:4px';
    const sceneLbl = document.createElement('span');
    sceneLbl.style.cssText = 'color:#888;font-size:11px;width:100%';
    sceneLbl.textContent = '场景布局 / 数据集';
    sceneRow.appendChild(sceneLbl);
    for (const name of LAYOUT_NAMES) {
      const b = document.createElement('button');
      b.className = 'ac-quick-btn';
      b.style.cssText = 'background:#224;font-size:11px;padding:2px 6px';
      b.textContent = LAYOUT_CN[name];
      b.onclick = () => { if (this._mockAgent) this._mockAgent.setLayout(name); };
      sceneRow.appendChild(b);
    }
    // 数据集生成按钮
    const btnGen = document.createElement('button');
    btnGen.className = 'ac-quick-btn';
    btnGen.style.cssText = 'background:#422;color=#FC0;border-color:#640;font-size:11px;padding:2px 6px;width:100%;margin-top:2px';
    btnGen.textContent = '生成数据集 (图像+YOLO标注)';
    btnGen.onclick = () => this._generateDataset();
    sceneRow.appendChild(btnGen);
    quick.appendChild(sceneRow);

    // LLM 大模型配置区
    const llmRow = document.createElement('div');
    llmRow.style.cssText = 'border-top:1px solid #333;padding-top:4px;margin-top:4px';
    const llmLbl = document.createElement('span');
    llmLbl.style.cssText = 'color:#888;font-size:11px;width:100%;display:block';
    llmLbl.textContent = '大模型 + MCP 工具调用';
    llmRow.appendChild(llmLbl);
    // API Base
    const inpBase = document.createElement('input');
    inpBase.type = 'text';
    inpBase.placeholder = 'API Base (如 https://api.openai.com/v1)';
    inpBase.style.cssText = 'width:100%;background:#111;color:#0F0;border:1px solid #333;font-size:10px;padding:2px 4px;margin-bottom:2px';
    inpBase.value = localStorage.getItem('llm_api_base') || 'https://api.openai.com/v1';
    llmRow.appendChild(inpBase);
    // API Key
    const inpKey = document.createElement('input');
    inpKey.type = 'password';
    inpKey.placeholder = 'API Key';
    inpKey.style.cssText = 'width:100%;background:#111;color:#0F0;border:1px solid #333;font-size:10px;padding:2px 4px;margin-bottom:2px';
    inpKey.value = localStorage.getItem('llm_api_key') || '';
    llmRow.appendChild(inpKey);
    // Model
    const inpModel = document.createElement('input');
    inpModel.type = 'text';
    inpModel.placeholder = 'Model (如 gpt-4o)';
    inpModel.style.cssText = 'width:100%;background:#111;color:#0F0;border:1px solid #333;font-size:10px;padding:2px 4px;margin-bottom:2px';
    inpModel.value = localStorage.getItem('llm_model') || 'gpt-4o';
    llmRow.appendChild(inpModel);
    // 保存按钮
    const btnSaveLLM = document.createElement('button');
    btnSaveLLM.className = 'ac-quick-btn';
    btnSaveLLM.style.cssText = 'background:#224;color:#8CF;border-color:#246;font-size:10px;padding:2px 6px;width:100%';
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
      const mode = key ? 'LLM 大模型模式' : '正则模式 (无API Key)';
      this._pushLog(`[llm] 配置已保存 → ${mode}, model=${model}`);
      const st = this.console.querySelector('#ac-status');
      if (st && this._mockAgent) {
        st.textContent = key ? 'LLM 模式 · 大模型+MCP' : 'Mock 模式 · 正则NLU';
      }
    };
    llmRow.appendChild(btnSaveLLM);

    // 验证连接按钮
    const btnTestLLM = document.createElement('button');
    btnTestLLM.className = 'ac-quick-btn';
    btnTestLLM.style.cssText = 'background:#242;color:#FC0;border-color:#462;font-size:10px;padding:2px 6px;width:100%;margin-top:2px';
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
          btnTestLLM.style.cssText = 'background:#242;color:#0F0;border-color:#462;font-size:10px;padding:2px 6px;width:100%;margin-top:2px';
          // 自动保存配置
          localStorage.setItem('llm_api_base', base);
          localStorage.setItem('llm_api_key', key);
          localStorage.setItem('llm_model', model);
          if (this._llmAgent) {
            this._llmAgent.apiBase = base;
            this._llmAgent.apiKey = key;
            this._llmAgent.model = model;
          }
          const st = this.console.querySelector('#ac-status');
          if (st) st.textContent = 'LLM 模式 · 已连接 ✓';
        } else {
          const errText = await resp.text();
          const errShort = errText.substring(0, 150);
          this._pushLog(`[llm] ✗ 连接失败: HTTP ${resp.status} — ${errShort}`);
          btnTestLLM.textContent = '✗ 连接失败';
          btnTestLLM.style.cssText = 'background:#422;color:#F44;border-color:#644;font-size:10px;padding:2px 6px;width:100%;margin-top:2px';
        }
      } catch (e) {
        this._pushLog(`[llm] ✗ 网络错误: ${e.message}`);
        btnTestLLM.textContent = '✗ 网络错误';
        btnTestLLM.style.cssText = 'background:#422;color:#F44;border-color:#644;font-size:10px;padding:2px 6px;width:100%;margin-top:2px';
      } finally {
        btnTestLLM.disabled = false;
        setTimeout(() => {
          btnTestLLM.textContent = '验证大模型连接';
          btnTestLLM.style.cssText = 'background:#242;color:#FC0;border-color:#462;font-size:10px;padding:2px 6px;width:100%;margin-top:2px';
        }, 5000);
      }
    };
    llmRow.appendChild(btnTestLLM);
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
  }

  _stopSubs() {
    [this._statusSub, this._logSub].forEach(s => {
      if (s) { try { s.unsubscribe(); } catch (e) {} }
    });
    this._statusSub = null; this._logSub = null;
  }

  /** ROS 版本切换后重建 (消息类型字符串变了) */
  rebuild() {
    this._lastVer = null;     // 强制 _syncSubscriptions 重建
    this._syncSubscriptions();
  }

  /** 设置 Mock 模式 (无需 ROS2, 浏览器内跑全流程) */
  setMockAgent(agent) {
    this._mockAgent = agent;
    const el = this.console.querySelector('#ac-status');
    el.textContent = 'Mock 模式 · 浏览器仿真';
    el.className = 'ac-status ok';
    // 快捷按钮直接可用 (不依赖 ROS 连接)
  }

  /** 设置数据集生成器 */
  setDatasetGenerator(gen) {
    this._datasetGen = gen;
  }

  /** 设置 LLM 智能体 */
  setLLMAgent(agent) {
    this._llmAgent = agent;
  }

  /** 生成数据集 */
  async _generateDataset() {
    if (!this._datasetGen) {
      this._pushLog('[dataset] 数据集生成器未初始化');
      return;
    }
    if (this._busy) {
      this._pushLog('[dataset] 智能体忙, 请稍后再试');
      return;
    }
    const btn = this.console.querySelector('.ac-quick-btn[style*="422"]');
    if (btn) { btn.disabled = true; btn.textContent = '生成中...'; }
    this._pushLog('[dataset] 开始生成数据集...');
    this._datasetGen.onProgress(msg => this._pushLog(`[dataset] ${msg}`));
    try {
      const samples = await this._datasetGen.generate({
        layouts: LAYOUT_NAMES,
        viewsPerLayout: 8,
        randomSeeds: 3,
        imgWidth: 640,
        imgHeight: 480,
      });
      this._pushLog(`[dataset] 完成! ${samples.length}张图像+标注已下载`);
    } catch (e) {
      this._pushLog(`[dataset] 错误: ${e.message}`);
    } finally {
      if (btn) { btn.disabled = false; btn.textContent = '生成数据集 (图像+YOLO标注)'; }
    }
  }

  /** 发送自然语言指令 */
  sendInstruction(text) {
    // 新任务 → 清空日志面板, 确保本次对话完整显示
    this._clearLog();
    if (this._mockAgent) {
      // Mock 模式: 直接调用浏览器内 agent
      this._pushLog(`[local] 已发送指令: ${text}`);
      this._setBanner('🗣️', '接收自然语言指令', text);
      this._busy = true;
      this.banner.classList.add('active');
      this._updateRunButtons(true);
      this._mockAgent.run(text);
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

  // ─────────────── 渲染 ───────────────
  _setBanner(icon, text, detail) {
    this.banner.querySelector('.ab-icon').textContent = icon;
    this.banner.querySelector('.ab-text').textContent = text;
    this.banner.querySelector('.ab-detail').textContent = detail || '';
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
    div.textContent = line;
    box.appendChild(div);
    box.scrollTop = box.scrollHeight;
  }

  _renderObjects() {
    const el = this.console.querySelector('#ac-objects');
    if (!this._objects.length) { el.innerHTML = '—'; return; }
    el.innerHTML = this._objects.map(o =>
      `<span class="ac-obj"><b>${o.class}</b> @[${o.xyz}]</span>`).join('');
  }
}
