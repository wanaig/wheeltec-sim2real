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
- 双工作台: 左台料箱(3格), 右台工具, 机械臂小车大作业区覆盖两个工作区和中间区域
- 工作台X范围[0.225,0.375], 左台Y[-0.49,-0.11], 右台Y[0.11,0.49]
- 底盘可在大作业区内自由移动并规划最近路径, 但必须按小车footprint避开工作台; 优先使用工具返回的 suggested_chassis
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
- retract: 垂直抬升收回 (从上往下抓取/放置的逆操作, 抓取/放置后立即调用)

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
- ★ 抓取/放置均为从上往下: 规划器先到目标正上方再垂直下降; retract 为其逆操作(垂直抬升), 抓取/放置后立即调用
- ★ perceive/plan_arm_motion 返回 suggested_chassis 时, 直接 move_base 到该坐标
- ★ 抓取后必须 retract, 放置后必须 retract, move_base 前必须 retract
- ★ move_base 会自动先收回机械臂到安全高度 (多策略抬升), 无需手动 retract 后再 move_base
- ★ 底盘不再限制在窄过道内, 可进入覆盖双工作区的大作业区; move_base 会按小车footprint避障并优先走最近可达站位
- 可以在同一轮调用多个无依赖的工具 (如 perceive + get_scene_info)

## 失败恢复 (必须严格遵守)
- ik_unreachable: 按返回的 suggested_chassis 移动底盘后重试
- plan_arm_motion 碰撞 (trajectory_collision / goal_collision):
  ★ 检查 suggested_chassis 中的 no_collision_free 字段!
  ★ 如果 no_collision_free=true: 表示所有站位均无法避免碰撞, 此目标不可抓取! 必须放弃此目标, 换另一个同类工具或告知用户无法完成
  ★ 如果 no_collision_free 不存在或 false: 按 suggested_chassis move_base 换站位后再 plan_arm_motion
  ★ 禁止修改坐标重试! 必须先 move_base 换站位
  ★ 同一目标 plan_arm_motion 失败2次后, 必须换另一个同类目标, 不要在同一个目标上死循环
- move_base 失败 (path_not_found): 目标站位太近或不可达, 换一个不同角度的站位重试
- move_base 失败 (arm_stuck): 先 release 再 retract, 然后重试 move_base
- grasp失败: 偏移抓取点
- ★ move_base 成功后可以重试 plan_arm_motion (站位已变, 不算重复调用)
- ★ 同一目标 plan_arm_motion 失败3次后系统自动终止; 失败2次后应主动换目标
- ★ 连续5次工具失败自动终止 (move_base 成功不重置计数)
- 最多重试4次, 超过后总结失败原因并结束

## 输出要求
- 中文回复, 每步简要说明理由
- 工具调用给出明确坐标`;

// ─────────────── LLMAgent ───────────────

export class LLMAgent {
  constructor(config = {}) {
    this.apiBase = config.apiBase || 'https://api.openai.com/v1';
    this.apiKey = config.apiKey || '';
    this.model = config.model || 'gpt-4o';
    this.maxTurns = config.maxTurns || 100;  // 最大对话轮数
    this.executor = null;  // MCPToolExecutor
    this.onLog = null;
    this.onToolCall = null;
    this.messages = [];
    this._aborted = false;
    this._abortCtrl = null;
  }

  /** 用户手动停止对话 */
  stop() {
    this._aborted = true;
    if (this._abortCtrl) {
      try { this._abortCtrl.abort(); } catch (e) {}
    }
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

    this._aborted = false;
    this._abortCtrl = new AbortController();
    this.messages = [
      { role: 'system', content: SYSTEM_PROMPT },
      { role: 'user', content: instruction },
    ];

    // 循环检测状态
    this._failCount = 0;           // 连续失败计数
    this._planFailTarget = null;    // 最近失败的 plan_arm_motion 目标坐标
    this._planFailCount = 0;        // 同一目标 plan_arm_motion 失败次数
    this._callHistory = [];         // 近期工具调用 (用于重复检测)

    const MAX_CONSECUTIVE_FAILURES = 5;
    const WARN_THRESHOLD = 3;

    let turns = 0;
    while (turns < this.maxTurns && !this._aborted) {
      turns++;
      this._log(`[LLM] 第${turns}轮对话...`);

      // 调用大模型
      const response = await this._callAPI();
      if (this._aborted) {
        return { ok: false, summary: '用户已停止对话', turns };
      }
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

        // 重复调用检测: 同一 tool+params 在最近 3 次调用中出现过 → 跳过
        const callKey = `${toolName}:${JSON.stringify(params)}`;
        const recentKeys = this._callHistory.slice(-3);
        let result;
        if (recentKeys.includes(callKey)) {
          result = {
            ok: false, reason: 'duplicate_call',
            detail: '此调用与最近3次内的调用完全相同, 已跳过。请更换策略或目标, 不要重复调用。',
          };
          this._log(`[LLM] ⚠ 重复调用 ${toolName}, 跳过`);
        } else {
          this._callHistory.push(callKey);
          if (this._callHistory.length > 20) this._callHistory.shift();
          result = await this.executor.execute(toolName, params);
          // move_base 成功后机器人状态已变, 清除调用历史
          // (允许从新站位重试同参数 plan_arm_motion, 不算重复)
          if (toolName === 'move_base' && result.ok) {
            this._callHistory = [];
          }
        }

        // 连续失败追踪
        if (result.ok === false) {
          this._failCount++;
          if (toolName === 'plan_arm_motion') {
            const tk = `${(params.x ?? 0).toFixed(3)},${(params.y ?? 0).toFixed(3)},${(params.z ?? 0).toFixed(3)}`;
            if (tk === this._planFailTarget) {
              this._planFailCount++;
            } else {
              this._planFailTarget = tk;
              this._planFailCount = 1;
            }
          }
        } else {
          // move_base 成功不算"进展"(只是换站位), 不重置失败计数
          // 只有实际操作 (plan_arm_motion/grasp/release/retract/verify) 成功才重置
          if (toolName !== 'move_base') {
            this._failCount = 0;
            this._planFailTarget = null;
            this._planFailCount = 0;
          }
        }

        // 构建工具结果 (失败达阈值时追加警告)
        let content = JSON.stringify(result);
        if (this._failCount >= WARN_THRESHOLD) {
          const w = this._planFailCount >= 2
            ? `同一目标(${this._planFailTarget})plan_arm_motion已失败${this._planFailCount}次, 必须更换目标或放弃`
            : '请更换策略, 不要重复相同的操作';
          content += `\n\n⚠ [系统警告] 已连续${this._failCount}次工具调用失败。${w}。`;
          this._log(`[LLM] ⚠ 连续${this._failCount}次失败, 已向模型发出警告`);
        }

        this.messages.push({
          role: 'tool',
          tool_call_id: tc.id,
          content,
        });

        // 熔断: 同一目标 plan_arm_motion 失败3次 → 终止 (不管 move_base 是否成功)
        if (this._planFailCount >= 3) {
          this._log(`[LLM] 同一目标(${this._planFailTarget})plan_arm_motion失败${this._planFailCount}次, 自动终止`);
          return {
            ok: false,
            summary: `目标(${this._planFailTarget})无法到达: 碰撞或不可达, 已尝试${this._planFailCount}次`,
            turns,
          };
        }

        // 熔断: 连续失败达上限 → 终止
        if (this._failCount >= MAX_CONSECUTIVE_FAILURES) {
          this._log(`[LLM] 连续${this._failCount}次工具调用失败, 自动终止 (防止死循环)`);
          return {
            ok: false,
            summary: `连续${this._failCount}次工具调用失败, 自动终止`,
            turns,
          };
        }

        if (this._aborted) break;
      }
    }

    if (this._aborted) {
      return { ok: false, summary: '用户已停止对话', turns };
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
        signal: this._abortCtrl?.signal,
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
      if (e.name === 'AbortError') {
        return { ok: false, aborted: true, error: '已停止' };
      }
      return { ok: false, error: e.message };
    }
  }
}
