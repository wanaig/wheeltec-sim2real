/**
 * InstructionParser.js — 自然语言指令解析模块
 *
 * 把工业场景的自然语言指令解析为结构化核心需求:
 *   { action, target_tool, side, slot, query }
 *
 *   action:       grasp(取/抓/拿) | place(放/放置)
 *   target_tool:  screwdriver|wrench|nut|roller|screw | null
 *   side:         left | right | null
 *   slot:         1-3 | null  (料箱格子)
 *   query:        目标物体的自然语言描述(含颜色/形状修饰), 供 LocateAnything 视觉定位
 *
 * 后端可切换 (OpenAI 兼容):
 *   - 云端 API:   apiBase = https://api.openai.com/v1 (或 DeepSeek/通义/智谱)
 *   - 本地大模型: apiBase = http://<外置机IP>:8001/v1 (vLLM) 或 :11434/v1 (Ollama)
 *                 本地推理跑在外置 RTX 上, 与 LocateAnything 同理
 *   - 无 LLM:     自动回退内置正则解析 (与 MockAgent.parseInstruction 一致)
 *
 * 用法:
 *   const parser = new InstructionParser({ apiBase, apiKey, model });
 *   const plan = await parser.parse('帮我把滚柱放到料箱的第三个格子中');
 *   // { action:'place', target_tool:'roller', side:null, slot:3, query:'滚柱', source:'llm' }
 */

const PARSER_SYSTEM_PROMPT = `你是工业机械臂小车的指令解析器。将用户的自然语言指令解析为结构化 JSON。

工业工具类别 (target_tool): screwdriver(螺丝刀), wrench(扳手), nut(螺母), roller(滚柱), screw(螺丝)。指令中的工具若无法映射到这5类, target_tool 设为 null。
动作 (action): grasp(取/拿/抓/给/捡/夹/取出/拿起), place(放/放置/丢/扔/放入/放到)。
位置 (side): left(左/左侧), right(右/右侧), 无法判断为 null。
料箱格 (slot): 1-3 的整数, 指令提到"第N格/第N个/料箱N"时提取, 无为 null。
query: 指令中描述目标物体的自然语言短语(保留颜色/形状/材质修饰, 去掉动作/位置/格子词), 用于视觉定位模型 LocateAnything。如"蓝色方形工件"/"扳手"/"滚柱"。

只输出 JSON, 不要 markdown 代码块, 不要解释:
{"action":"grasp|place","target_tool":"screwdriver|wrench|nut|roller|screw|null","side":"left|right|null","slot":1-3|null,"query":"目标描述"}

示例:
"取出左侧的扳手" → {"action":"grasp","target_tool":"wrench","side":"left","slot":null,"query":"扳手"}
"帮我把滚柱放到料箱的第三个格子中" → {"action":"place","target_tool":"roller","side":null,"slot":3,"query":"滚柱"}
"抓取蓝色方形工件" → {"action":"grasp","target_tool":null,"side":null,"slot":null,"query":"蓝色方形工件"}
"给我一把螺丝刀" → {"action":"grasp","target_tool":"screwdriver","side":null,"slot":null,"query":"螺丝刀"}
"把螺母放到第二格" → {"action":"place","target_tool":"nut","side":null,"slot":2,"query":"螺母"}`;

const TOOL_CN_MAP = {
  '螺丝刀': 'screwdriver', '扳手': 'wrench', '螺母': 'nut',
  '滚柱': 'roller', '螺丝': 'screw',
};
const TOOL_CN_KEYS = ['螺丝刀', '扳手', '螺母', '滚柱', '螺丝'];
const VALID_TOOLS = ['screwdriver', 'wrench', 'nut', 'roller', 'screw'];
const ACTION_WORDS = {
  grasp: ['取', '拿', '抓', '给', '捡', '夹', '取出', '拿起', '夹取'],
  place: ['放', '放置', '丢', '扔', '放入', '放到', '放置到'],
};

export class InstructionParser {
  constructor(config = {}) {
    this.apiBase = config.apiBase || localStorage.getItem('llm_api_base') || 'https://api.openai.com/v1';
    this.apiKey = config.apiKey || localStorage.getItem('llm_api_key') || '';
    this.model = config.model || localStorage.getItem('llm_model') || 'gpt-4o';
    this.timeout = config.timeout || 8000;
  }

  setConfig({ apiBase, apiKey, model } = {}) {
    if (apiBase !== undefined) this.apiBase = apiBase;
    if (apiKey !== undefined) this.apiKey = apiKey;
    if (model !== undefined) this.model = model;
  }

  /**
   * 解析自然语言指令
   * @param {string} instruction
   * @returns {Promise<{ok, action, target_tool, side, slot, query, raw, source}>}
   */
  async parse(instruction) {
    const raw = (instruction || '').trim();
    if (!raw) return { ok: false, reason: '空指令' };
    if (this.apiKey) {
      try {
        const r = await this._llmParse(raw);
        if (r && r.ok) return r;
      } catch (e) {
        // LLM 失败 → 回退正则
      }
    }
    return this._regexParse(raw);
  }

  async _llmParse(instruction) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeout);
    try {
      const resp = await fetch(`${this.apiBase}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.model,
          messages: [
            { role: 'system', content: PARSER_SYSTEM_PROMPT },
            { role: 'user', content: instruction },
          ],
          temperature: 0,
          max_tokens: 200,
        }),
        signal: controller.signal,
      });
      if (!resp.ok) {
        const txt = await resp.text();
        return { ok: false, reason: `HTTP ${resp.status}: ${txt.slice(0, 120)}` };
      }
      const data = await resp.json();
      const content = data.choices?.[0]?.message?.content || '';
      const obj = this._extractJson(content);
      if (!obj) return { ok: false, reason: 'LLM 未返回有效 JSON' };
      return {
        ok: true,
        action: this._normAction(obj.action),
        target_tool: this._normTool(obj.target_tool),
        side: this._normSide(obj.side),
        slot: this._normSlot(obj.slot),
        query: (obj.query || '').trim() || instruction,
        raw: instruction,
        source: 'llm',
      };
    } finally {
      clearTimeout(timer);
    }
  }

  _regexParse(instruction) {
    const t = instruction.replace(/\s/g, '');
    let action = null;
    for (const a of ACTION_WORDS.grasp) if (t.includes(a)) { action = 'grasp'; break; }
    if (!action) for (const a of ACTION_WORDS.place) if (t.includes(a)) { action = 'place'; break; }
    if (!action) return { ok: false, reason: '无法识别动作', raw: instruction };

    let target_tool = null;
    for (const cn of TOOL_CN_KEYS) {
      if (t.includes(cn)) { target_tool = TOOL_CN_MAP[cn]; break; }
    }

    let side = null;
    if (/左侧|左边|左面|左/.test(t)) side = 'left';
    else if (/右侧|右边|右面|右/.test(t)) side = 'right';

    let slot = null;
    const sm = t.match(/第\s*([一二三四1234])\s*[个]?(?:格|槽|料箱)/);
    if (sm) {
      slot = { '一': 1, '二': 2, '三': 3, '四': 4, '1': 1, '2': 2, '3': 3, '4': 4 }[sm[1]] || null;
    }

    // query: 工具中文名, 无标准工具则用去动作/格子词的剩余短语
    let query = '';
    const cnName = TOOL_CN_KEYS.find(k => t.includes(k));
    if (cnName) query = cnName;
    else {
      query = instruction
        .replace(/取|拿|抓|给|捡|夹|放|置|丢|扔|到|入|帮|我|把|个|第|[一二三四1234]格?|料箱|左侧|右侧|左边|右边/g, '')
        .trim();
      if (!query) query = instruction;
    }

    return {
      ok: true,
      action,
      target_tool,
      side,
      slot,
      query,
      raw: instruction,
      source: 'regex',
    };
  }

  _extractJson(text) {
    if (!text) return null;
    // 容错: 提取第一个 {...} 块 (模型可能带额外文字/markdown)
    const s = text.indexOf('{');
    const e = text.lastIndexOf('}');
    if (s < 0 || e < 0 || e < s) return null;
    try {
      return JSON.parse(text.slice(s, e + 1));
    } catch (err) {
      return null;
    }
  }

  _normAction(a) {
    return a === 'grasp' || a === 'place' ? a : null;
  }
  _normTool(t) {
    if (!t || t === 'null') return null;
    const low = String(t).toLowerCase();
    return VALID_TOOLS.includes(low) ? low : null;
  }
  _normSide(s) {
    if (!s || s === 'null') return null;
    const low = String(s).toLowerCase();
    return low === 'left' || low === 'right' ? low : null;
  }
  _normSlot(n) {
    if (n === null || n === undefined || n === 'null') return null;
    const i = Number(n);
    if (Number.isInteger(i) && i >= 1 && i <= 3) return i;
    return null;
  }
}
