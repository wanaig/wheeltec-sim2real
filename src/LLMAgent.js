/**
 * LLMAgent.js — 大模型智能体 (OpenAI 兼容 API + function calling)
 *
 * 工作流程:
 *   1. 用户输入自然语言指令
 *   2. LLMAgent 构造 system prompt (机器人能力 + 场景描述 + MCP 工具列表)
 *   3. 发送到大模型 API (OpenAI 兼容: /v1/chat/completions)
 *   4. 大模型返回 tool_calls → MCPToolExecutor 执行
 *   5. 执行结果回传大模型 → 继续决策
 *   6. 循环直到大模型返回 finish (无 tool_calls) 或超时
 *
 * 兼容: OpenAI / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等 OpenAI 格式 API。
 *
 * 用法:
 *   const agent = new LLMAgent({ apiBase, apiKey, model });
 *   agent.setExecutor(mcpExecutor);
 *   const result = await agent.run('把右侧的扳手放到料箱第三格');
 */
import { MCP_TOOLS } from './MCPTools.js';

// ─────────────── 系统提示词 ───────────────

const SYSTEM_PROMPT = `你是一个工业机械臂小车的自主作业智能体, 控制一台 WHEELTEC R550A 机械臂小车 (5-DOF臂+夹爪, 4WD底盘)。

## 场景描述
- 双工作台: 左台料箱(3格, slot1靠近过道), 右台工具, 中间过道(22cm)
- 工作台X范围[0.15,0.45], 左台Y[-0.49,-0.11], 右台Y[0.11,0.49], 过道Y[-0.11,0.11]
- 机械臂臂展0.40m, 基座[0.054, 0.001, 0.156]

## MCP 工具 (共9个)
perceive, get_scene_info, get_robot_state, move_base, plan_arm_motion, grasp, release, retract, verify

## 作业流程

### 1. 感知 (可同时调用)
- perceive: 返回每个工具的坐标/距离/可达性, 不可达时附带 suggested_chassis
- get_scene_info: 返回料箱格子坐标

### 2. 抓取
- 检查目标 reachable 字段
- 如果 reachable=false 或 plan_arm_motion 返回 ik_unreachable:
  ★ 直接 move_base 到 suggested_chassis 坐标, 不要自己猜!
- plan_arm_motion(x, y, z) 规划并执行到目标 (自动碰撞检测+经由点绕行, 一次调用完成)
- grasp 抓取

### 3. 收回
- retract (预设安全姿态, 总是成功)

### 4. 放置
- 检查到料箱格子的距离
- 不可达时 move_base (参考 get_scene_info 中 bin_slots 计算 suggested_chassis)
- plan_arm_motion(x, y, z) 规划并执行到格子坐标
- release 放置

### 5. 收回
- retract + verify(type=place, slot=N) (可同时调用)
- 回复完成

## 关键规则
- ★ plan_arm_motion 一次调用完成规划+执行, 无需单独 execute
- ★ plan_arm_motion 直接传目标坐标即可, 规划器自动经安全高度绕行, 无需先到上方
- ★ perceive/plan_arm_motion 返回 suggested_chassis 时, 直接 move_base 到该坐标
- ★ 抓取后必须 retract, 放置后必须 retract, move_base 前必须 retract
- ★ 底盘Y必须在过道内 |Y| ≤ 0.08
- 可以在同一轮调用多个无依赖的工具 (如 perceive + get_scene_info)

## 失败恢复
- ik_unreachable: 按返回的 suggested_chassis 移动底盘后重试
- grasp失败: 偏移抓取点
- 最多重试4次

## 输出要求
- 中文回复, 每步简要说明理由
- 工具调用给出明确坐标`;

// ─────────────── LLMAgent ───────────────

export class LLMAgent {
  constructor(config = {}) {
    this.apiBase = config.apiBase || 'https://api.openai.com/v1';
    this.apiKey = config.apiKey || '';
    this.model = config.model || 'gpt-4o';
    this.maxTurns = config.maxTurns || 30;  // 最大对话轮数
    this.executor = null;  // MCPToolExecutor
    this.onLog = null;
    this.onToolCall = null;
    this.messages = [];
  }

  setExecutor(executor) { this.executor = executor; }
  setLogCb(cb) { this.onLog = cb; }

  _log(msg) { if (this.onLog) this.onLog(msg); }

  /**
   * 运行任务
   * @param {string} instruction — 用户自然语言指令
   * @returns {Promise<{ok: boolean, summary: string, turns: number}>}
   */
  async run(instruction) {
    if (!this.apiKey) {
      return { ok: false, summary: '未配置 API Key, 请在面板中设置', turns: 0 };
    }
    if (!this.executor) {
      return { ok: false, summary: 'MCP 执行器未初始化', turns: 0 };
    }

    this.messages = [
      { role: 'system', content: SYSTEM_PROMPT },
      { role: 'user', content: instruction },
    ];

    let turns = 0;
    while (turns < this.maxTurns) {
      turns++;
      this._log(`[LLM] 第${turns}轮对话...`);

      // 调用大模型
      const response = await this._callAPI();
      if (!response.ok) {
        return { ok: false, summary: `API调用失败: ${response.error}`, turns };
      }

      const choice = response.data.choices?.[0];
      if (!choice) {
        return { ok: false, summary: 'API返回空响应', turns };
      }

      const msg = choice.message;
      this.messages.push(msg);

      // 如果有文本回复, 输出
      if (msg.content) {
        this._log(`[LLM] ${msg.content}`);
      }

      // 没有工具调用 → 任务结束
      if (!msg.tool_calls || msg.tool_calls.length === 0) {
        return {
          ok: true,
          summary: msg.content || '任务完成',
          turns,
        };
      }

      // 执行工具调用
      for (const tc of msg.tool_calls) {
        const toolName = tc.function.name;
        let params = {};
        try { params = JSON.parse(tc.function.arguments || '{}'); }
        catch (e) { this._log(`[LLM] 工具参数解析失败: ${e.message}`); }

        if (this.onToolCall) this.onToolCall(toolName, params);

        const result = await this.executor.execute(toolName, params);

        // 工具结果回传大模型
        this.messages.push({
          role: 'tool',
          tool_call_id: tc.id,
          content: JSON.stringify(result),
        });
      }
    }

    return { ok: false, summary: `达到最大轮数(${this.maxTurns})`, turns };
  }

  /**
   * 调用 OpenAI 兼容 API
   */
  async _callAPI() {
    try {
      const tools = MCP_TOOLS.map(t => ({
        type: 'function',
        function: {
          name: t.function.name,
          description: t.function.description,
          parameters: t.function.parameters,
        },
      }));

      const resp = await fetch(`${this.apiBase}/chat/completions`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${this.apiKey}`,
        },
        body: JSON.stringify({
          model: this.model,
          messages: this.messages,
          tools: tools,
          tool_choice: 'auto',
          temperature: 0.3,
          max_tokens: 2000,
        }),
      });

      if (!resp.ok) {
        const errText = await resp.text();
        return { ok: false, error: `HTTP ${resp.status}: ${errText.substring(0, 200)}` };
      }

      const data = await resp.json();
      return { ok: true, data };
    } catch (e) {
      return { ok: false, error: e.message };
    }
  }
}
