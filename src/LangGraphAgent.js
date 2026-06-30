/**
 * LangGraphAgent.js — 基于 LangGraph 的交互型智能体
 *
 * 用 @langchain/langgraph 的 StateGraph 重写 LLMAgent 的 function-calling 循环,
 * 替代 LLMAgent.js 中手写的 while-turn 轮询。接口与 LLMAgent 完全一致,
 * main.js / AgentPanel / MockAgent 无需改动即可热替换。
 *
 * 图结构 (LangGraph StateGraph):
 *   START ─► agent ─┬─(有 tool_calls)─────────► tools ─┬─(未熔断且未达上限)─► agent
 *                  ├─(无 tool_calls, 未verify, 已作业)─► guard ─► agent   (注入"继续"指令, ≤3次)
 *                  └─(无 tool_calls, 已verify/纯查询/达上限)─► END
 *   tools 节点: verify 成功 → verified=true; actuation 工具调用 → manipulationStarted=true
 *   guard 节点: LLM 提前结束但未 verify 时, 注入 HumanMessage 逼回工具循环, 防止中途伪报完成
 *
 *   - agent 节点: 调 ChatOpenAI.bindTools(9 个 MCP 工具) 生成下一动作
 *   - tools 节点: 路由到 MCPToolExecutor 执行, 内置
 *       · 重复调用检测 (近 3 次相同 tool+params 跳过)
 *       · 连续失败熔断 (5 次) / 同目标 plan_arm_motion 熔断 (3 次)
 *       · 阈值警告注入 ToolMessage, 引导模型换策略
 *   - run() 终判: 只有 verified=true (或纯信息查询) 才返回 ok:true, 否则 ok:false
 *
 * 9 个 MCP 工具以 DynamicStructuredTool(Zod schema) 形式注册, 既给模型
 * 提供函数签名, 又由 tools 节点统一调度 MCPToolExecutor, 保留全部恢复逻辑。
 *
 * 兼容: OpenAI / DeepSeek / 通义千问 / 智谱 / 本地 Ollama 等 OpenAI 格式 API。
 */
import { Annotation, StateGraph, START, END, MessagesAnnotation } from '@langchain/langgraph';
import { ChatOpenAI } from '@langchain/openai';
import {
  SystemMessage, HumanMessage, AIMessage, ToolMessage,
} from '@langchain/core/messages';
import { DynamicStructuredTool } from '@langchain/core/tools';
import { z } from 'zod';
import { SYSTEM_PROMPT } from './LLMAgent.js';

// ─────────────── 状态标注 (LangGraph channels) ───────────────

// 标量通道: 后写覆盖 (next 省略时保持原值)。必须用 Annotation(...) 包裹,
// 否则 LangGraph 不识别为真正的 channel (会静默丢弃, 节点读不到)。
const scal = (init) => Annotation({
  reducer: (prev, next) => (next === undefined ? prev : next),
  default: () => init,
});

const AgentState = Annotation.Root({
  ...MessagesAnnotation.spec,                 // messages: BaseMessage[] (追加 + id 去重)
  turns: scal(0),                             // 已完成的 agent 节点轮数
  failCount: scal(0),                         // 连续失败计数
  planFailTarget: scal(null),                 // 最近失败的 plan_arm_motion 目标键
  planFailCount: scal(0),                     // 同一目标 plan_arm_motion 连续失败次数
  callHistory: scal([]),                      // 近期工具调用键 (用于重复检测)
  aborted: scal(false),                       // 用户中止
  fuse: scal(null),                           // 熔断时 { reason, summary }
  verified: scal(false),                      // verify 工具曾成功 (允许 ok:true 收尾)
  manipulationStarted: scal(false),           // 曾调用 actuation 类工具 (plan_arm_motion/grasp/release/move_base)
  finishGuardAttempts: scal(0),               // guard 节点已注入"继续"指令的次数
});

// ─────────────── 常量 (与 LLMAgent 对齐) ───────────────

const MAX_CONSECUTIVE_FAILURES = 5;
const WARN_THRESHOLD = 3;
const PLAN_FAIL_LIMIT = 3;
const FINISH_GUARD_LIMIT = 3;                 // LLM 提前结束但未 verify 时, 最多注入 N 次继续指令
const API_RETRY_LIMIT = 2;                    // 模型 invoke 失败 (如 400 非法 function.arguments) 最多重试 N 次

// actuation 类工具: 调用过任一即视为"已开始作业", 收尾必须 verify
const MANIPULATION_TOOLS = new Set([
  'plan_arm_motion', 'grasp', 'release', 'move_base',
]);

// ─────────────── 历史消息 sanitize ───────────────
// 某些服务商 (阿里 Qwen/DashScope 等) 回放历史时, 若 assistant 消息的
// function.arguments 不是合法 JSON (no-arg 工具被模型写成空串 ""), 会以 400 拒绝。
// 调用前对历史做清洗: 保证每个 tool_call.args 为对象 (string→JSON.parse 失败→{}),
// 仅在需要替换时构造新 AIMessage, 避免无谓拷贝。
function sanitizeMessages(msgs) {
  const out = [];
  for (const m of msgs) {
    if (m instanceof AIMessage && m.tool_calls && m.tool_calls.length) {
      let dirty = false;
      const cleanCalls = m.tool_calls.map(tc => {
        let args = tc.args;
        if (typeof args === 'string') {
          try { args = JSON.parse(args || '{}'); }
          catch { args = {}; }
          dirty = true;
        } else if (!args || typeof args !== 'object') {
          args = {};
          dirty = true;
        }
        return dirty ? { ...tc, args } : tc;
      });
      if (dirty) {
        const nm = new AIMessage({
          content: m.content ?? '',
          tool_calls: cleanCalls,
        });
        if (m.id) nm.id = m.id;
        if (m.name) nm.name = m.name;
        out.push(nm);
        continue;
      }
    }
    out.push(m);
  }
  return out;
}

// ─────────────── MCP 工具 → LangChain Tool (Zod schema) ───────────────
// schema 仅用于向模型声明函数签名; 实际执行由 tools 节点经 MCPToolExecutor 调度,
// 以便集中实施重复检测/熔断/警告注入。

function buildToolSchemas() {
  const mk = (name, description, schema) =>
    new DynamicStructuredTool({
      name,
      description,
      schema,
      func: async () => '[]',  // 不会命中: tools 节点直接调 executor
    });
  return [
    mk('perceive',
      '感知当前场景中所有可见的工具和零件, 返回类别、世界坐标(x,y,z米)、置信度。仅在工具未被抓取时可见。',
      z.object({}).describe('无参数')),
    mk('move_base',
      '驱动底盘导航到指定世界坐标 (x, y) 米, 朝向 yaw 弧度。底盘移动后臂的可达范围改变。用于目标超出臂展时靠近目标。',
      z.object({
        x: z.number().describe('目标X坐标 (米), 前方为正'),
        y: z.number().describe('目标Y坐标 (米), 左为正'),
        yaw: z.number().optional().describe('目标朝向 (弧度), 默认0'),
      })),
    mk('plan_arm_motion',
      'MoveIt2规划并执行臂运动到目标坐标(x,y,z)。多起始点IK求解+碰撞检测+经由点轨迹, 自动避穿模。一次调用完成规划+执行, 无需单独execute。抓取时直接传目标高度即可, 规划器自动经安全高度绕行。',
      z.object({
        x: z.number().describe('目标TCP X坐标 (米)'),
        y: z.number().describe('目标TCP Y坐标 (米)'),
        z: z.number().describe('目标TCP Z坐标 (米)'),
      })),
    mk('grasp',
      '闭合夹爪, 抓取距离TCP最近且已贴近的工具(3.5cm内)。返回是否抓取成功。',
      z.object({}).describe('无参数')),
    mk('release',
      '张开夹爪, 松开当前抓取的工具, 在TCP位置放置。',
      z.object({}).describe('无参数')),
    mk('retract',
      '收回机械臂: 多策略垂直抬升(逆序/多解IK/后退脱离)到安全高度, 再收到安全姿态, 保持当前方向。抓取/放置后必须调用。碰撞时系统自动切换策略, 无需手动处理。',
      z.object({
        lift: z.number().optional().describe('额外抬升高度 (米), 默认0.08'),
      })),
    mk('verify',
      '验证操作结果。type=grasp检查工具是否被抓取(离开原位置), type=place检查工具是否在指定格子内。',
      z.object({
        type: z.enum(['grasp', 'place']).describe('验证类型'),
        slot: z.number().int().min(1).max(3).optional().describe('格子编号(1-3), 仅place时需要'),
      })),
    mk('get_robot_state',
      '获取机器人当前状态: 底盘位置(x,y,yaw)、臂关节角(joint1-5)、夹爪状态(open/close)、TCP世界坐标。',
      z.object({}).describe('无参数')),
    mk('get_scene_info',
      '获取场景信息: 双工作台位置、料箱格子坐标、臂展范围、工具台区域。',
      z.object({}).describe('无参数')),
  ];
}

// ─────────────── LangGraphAgent ───────────────

export class LangGraphAgent {
  constructor(config = {}) {
    this.apiBase = config.apiBase || 'https://api.openai.com/v1';
    this.apiKey = config.apiKey || '';
    this.model = config.model || 'gpt-4o';
    this.maxTurns = config.maxTurns || 100;
    this.executor = null;
    this.onLog = null;
    this.onToolCall = null;
    this._aborted = false;
    this._abortCtrl = null;
    this._toolSchemas = buildToolSchemas();
    this._graph = this._buildGraph();
  }

  stop() {
    this._aborted = true;
    if (this._abortCtrl) {
      try { this._abortCtrl.abort(); } catch (e) {}
    }
  }

  setExecutor(executor) { this.executor = executor; }
  setLogCb(cb) { this.onLog = cb; }
  _log(msg) { if (this.onLog) this.onLog(msg); }

  // ─── 构建编译后的图 (一次性, 节点闭包读取实例属性) ───
  _buildGraph() {
    const self = this;

    // agent 节点: 调用大模型生成下一动作
    //   - 调用前 sanitize 历史消息 (修复 no-arg 工具 arguments 为空串导致的回放 400)
    //   - invoke 失败 (如模型生成的 function.arguments 非法 JSON 被服务商 400) 时,
    //     注入纠正指令后重试, 最多 API_RETRY_LIMIT 次, 不让单次模型侧格式错误杀死整个任务
    const agentNode = async (state) => {
      if (state.aborted || state.fuse) return {};
      const turns = (state.turns || 0) + 1;
      self._log(`[LLM] 第${turns}轮对话...`);
      if (!self._boundModel) return { aborted: true, fuse: { reason: 'no_model', summary: '模型未初始化' }, turns };

      const added = [];        // 本轮追加的纠正 HumanMessage (写入 state 保持 history 一致)
      let lastErr = null;
      for (let attempt = 0; attempt <= API_RETRY_LIMIT; attempt++) {
        if (self._aborted) {
          return { aborted: true, fuse: { reason: 'aborted', summary: '用户已停止对话' }, turns };
        }
        const msgs = sanitizeMessages(state.messages.concat(added));
        try {
          const resp = await self._boundModel.invoke(msgs, {
            signal: self._abortCtrl?.signal,
          });
          if (resp.content) self._log(`[LLM] ${resp.content}`);
          return { messages: [...added, resp], turns };
        } catch (e) {
          if (e.name === 'AbortError' || self._aborted) {
            return { aborted: true, fuse: { reason: 'aborted', summary: '用户已停止对话' }, turns };
          }
          lastErr = e;
          const emsg = (e.message || String(e)).slice(0, 160);
          if (attempt < API_RETRY_LIMIT) {
            self._log(`[LLM] 模型调用异常 (尝试${attempt + 1}/${API_RETRY_LIMIT + 1}), 将重试: ${emsg}`);
            added.push(new HumanMessage({
              content: `⚠ 上一次请求失败: ${emsg}\n` +
                `请检查并重新发出合法的工具调用: function.arguments 必须是合法 JSON 对象 — ` +
                `无参数工具传 "{}", 有参数工具传如 {"x":0.3,"y":0.2,"z":0.06}。` +
                `若刚才的调用已导致失败, 请改用其他策略 (如先 move_base 换站位) 再重试, 不要重复同样的错误调用。`,
            }));
          } else {
            self._log(`[LLM] 模型调用异常 (重试${API_RETRY_LIMIT}次仍失败): ${emsg}`);
          }
        }
      }
      return {
        aborted: true,
        fuse: { reason: 'api_error', summary: `API调用失败: ${(lastErr.message || '').slice(0, 140)}` },
        turns,
      };
    };

    // tools 节点: 调度 MCPToolExecutor + 失败/重复/熔断
    const toolsNode = async (state) => {
      const last = state.messages[state.messages.length - 1];
      const calls = (last && last.tool_calls) || [];
      const toolMessages = [];
      let { failCount, planFailTarget, planFailCount, callHistory, verified, manipulationStarted } = state;
      callHistory = [...callHistory];
      let fuse = null;

      for (const tc of calls) {
        const name = tc.name;
        let params = {};
        try { params = (tc.args && typeof tc.args === 'object') ? tc.args : JSON.parse(tc.args || '{}'); }
        catch (e) { self._log(`[LLM] 工具参数解析失败: ${e.message}`); }

        if (self.onToolCall) self.onToolCall(name, params);

        // actuation 类工具一经调用即视为"已开始作业", 后续收尾必须 verify
        if (MANIPULATION_TOOLS.has(name)) manipulationStarted = true;

        const callKey = `${name}:${JSON.stringify(params)}`;
        const recentKeys = callHistory.slice(-3);
        let result;
        if (recentKeys.includes(callKey)) {
          result = {
            ok: false, reason: 'duplicate_call',
            detail: '此调用与最近3次内的调用完全相同, 已跳过。请更换策略或目标, 不要重复调用。',
          };
          self._log(`[LLM] ⚠ 重复调用 ${name}, 跳过`);
        } else {
          callHistory.push(callKey);
          if (callHistory.length > 20) callHistory.shift();
          result = await self.executor.execute(name, params);
          // move_base 成功后机器人状态已变, 清除调用历史 (允许从新站位重试同参数)
          if (name === 'move_base' && result.ok) callHistory = [];
          // verify 成功 → 标记任务通过验证 (允许 ok:true 收尾)
          if (name === 'verify' && result.ok) verified = true;
        }

        // 连续失败追踪
        if (result.ok === false) {
          failCount++;
          if (name === 'plan_arm_motion') {
            const tk = `${(params.x ?? 0).toFixed(3)},${(params.y ?? 0).toFixed(3)},${(params.z ?? 0).toFixed(3)}`;
            if (tk === planFailTarget) planFailCount++;
            else { planFailTarget = tk; planFailCount = 1; }
          }
        } else if (name !== 'move_base') {
          // 实际操作成功才重置失败计数 (move_base 仅换站位, 不算进展)
          failCount = 0; planFailTarget = null; planFailCount = 0;
        }

        // 构建工具结果 (达阈值追加系统警告)
        let content = JSON.stringify(result);
        if (failCount >= WARN_THRESHOLD) {
          const w = planFailCount >= 2
            ? `同一目标(${planFailTarget})plan_arm_motion已失败${planFailCount}次, 必须更换目标或放弃`
            : '请更换策略, 不要重复相同的操作';
          content += `\n\n⚠ [系统警告] 已连续${failCount}次工具调用失败。${w}。`;
          self._log(`[LLM] ⚠ 连续${failCount}次失败, 已向模型发出警告`);
        }

        toolMessages.push(new ToolMessage({
          tool_call_id: tc.id,
          name,
          content,
        }));

        // 熔断: 同一目标 plan_arm_motion 失败达上限
        if (planFailCount >= PLAN_FAIL_LIMIT) {
          self._log(`[LLM] 同一目标(${planFailTarget})plan_arm_motion失败${planFailCount}次, 自动终止`);
          fuse = {
            reason: 'plan_fail_limit',
            summary: `目标(${planFailTarget})无法到达: 碰撞或不可达, 已尝试${planFailCount}次`,
          };
          break;
        }
        // 熔断: 连续失败达上限
        if (failCount >= MAX_CONSECUTIVE_FAILURES) {
          self._log(`[LLM] 连续${failCount}次工具调用失败, 自动终止 (防止死循环)`);
          fuse = {
            reason: 'consecutive_fail_limit',
            summary: `连续${failCount}次工具调用失败, 自动终止`,
          };
          break;
        }
        if (self._aborted) break;
      }

      const patch = {
        messages: toolMessages,
        failCount, planFailTarget, planFailCount, callHistory,
        verified, manipulationStarted,
      };
      if (fuse) patch.fuse = fuse;
      return patch;
    };

    // guard 节点: LLM 提前结束 (无 tool_calls) 但未 verify 且已开始作业时,
    // 注入"继续"指令把模型逼回工具循环, 防止中途伪报完成。最多 FINISH_GUARD_LIMIT 次。
    const guardNode = (state) => {
      const n = (state.finishGuardAttempts || 0) + 1;
      const nudge = new HumanMessage({
        content: `⚠ 任务尚未通过 verify 验证完成, 你刚才未调用任何工具(空回复/直接声称完成)。请继续作业:\n` +
          `- 若上一步 plan_arm_motion 失败 (unsafe_approach/ik_unreachable/collision), 必须按其返回的 suggested_chassis 先调用 move_base 换站位, 再重试 plan_arm_motion; 禁止修改坐标原地重试!\n` +
          `- 若已抓取/放置, 必须调用 verify(type=grasp|place) 验证;\n` +
          `- 若确实无法完成 (如 no_collision_free=true), 调用 get_robot_state 后回复"无法完成: <原因>";\n` +
          `禁止返回空文本或未经 verify 即声称完成。(第 ${n}/${FINISH_GUARD_LIMIT} 次提醒)`,
      });
      self._log(`[LLM] ⚠ 大模型提前结束且未 verify, 注入继续指令 (第${n}/${FINISH_GUARD_LIMIT}次)`);
      return { messages: [nudge], finishGuardAttempts: n };
    };

    // agent → tools | guard | END
    const afterAgent = (state) => {
      if (state.aborted || state.fuse) return END;
      if ((state.turns || 0) >= self.maxTurns) return END;
      const last = state.messages[state.messages.length - 1];
      if (last instanceof AIMessage && last.tool_calls && last.tool_calls.length > 0) return 'tools';
      // LLM 想收尾 (无 tool_calls):
      if (state.verified) return END;                                 // 已 verify → 真正完成
      if (!state.manipulationStarted) return END;                     // 纯信息查询 (只 perceive/scene) → 允许结束
      if ((state.finishGuardAttempts || 0) >= FINISH_GUARD_LIMIT) return END;  // 已提醒上限 → 放弃
      return 'guard';                                                 // 逼回工具循环
    };

    // tools → agent | END
    const afterTools = (state) => {
      if (state.aborted || state.fuse) return END;
      if ((state.turns || 0) >= self.maxTurns) return END;
      return 'agent';
    };

    const graph = new StateGraph(AgentState)
      .addNode('agent', agentNode)
      .addNode('tools', toolsNode)
      .addNode('guard', guardNode)
      .addEdge(START, 'agent')
      .addConditionalEdges('agent', afterAgent)
      .addConditionalEdges('tools', afterTools)
      .addEdge('guard', 'agent')
      .compile();
    return graph;
  }

  // 每次运行前重建模型 (apiBase/key/model 可能被面板热修改)
  _rebuildModel() {
    const model = new ChatOpenAI({
      apiKey: this.apiKey,
      modelName: this.model,
      configuration: { baseURL: this.apiBase },
      temperature: 0.3,
      maxTokens: 2000,
      streaming: false,
    });
    this._boundModel = model.bindTools(this._toolSchemas);
  }

  /**
   * 运行任务 (接口与 LLMAgent.run 一致)
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
    this._rebuildModel();

    const init = {
      messages: [
        new SystemMessage(SYSTEM_PROMPT),
        new HumanMessage(instruction),
      ],
      turns: 0,
      failCount: 0,
      planFailTarget: null,
      planFailCount: 0,
      callHistory: [],
      aborted: false,
      fuse: null,
      verified: false,
      manipulationStarted: false,
      finishGuardAttempts: 0,
    };

    let finalState;
    try {
      finalState = await this._graph.invoke(init, {
        signal: this._abortCtrl.signal,
        // 每轮 agent+tools 2 跳, + guard 循环 (每次 guard+agent 2 跳) × FINISH_GUARD_LIMIT
        recursionLimit: (this.maxTurns + 1) * 2 + FINISH_GUARD_LIMIT * 2 + 4,
      });
    } catch (e) {
      if (this._aborted || e.name === 'AbortError') {
        return { ok: false, summary: '用户已停止对话', turns: init.turns };
      }
      // 递归超限 / 其他图错误
      this._log(`[LLM] 图执行异常: ${e.message}`);
      return { ok: false, summary: `图执行异常: ${e.message}`, turns: init.turns };
    }

    if (this._aborted) {
      return { ok: false, summary: '用户已停止对话', turns: finalState.turns || 0 };
    }
    if (finalState.fuse) {
      return { ok: false, summary: finalState.fuse.summary, turns: finalState.turns || 0 };
    }

    // 取最后一条 AIMessage 的文本作为总结
    const msgs = finalState.messages || [];
    let lastAiContent = '';
    for (let i = msgs.length - 1; i >= 0; i--) {
      if (msgs[i] instanceof AIMessage && msgs[i].content) {
        lastAiContent = msgs[i].content;
        break;
      }
    }

    // 终判: 只有 verify 成功 (或纯信息查询未启动作业) 才算 ok:true
    if (finalState.verified || !finalState.manipulationStarted) {
      return { ok: true, summary: lastAiContent || '任务完成', turns: finalState.turns || 0 };
    }
    // 已开始作业但未 verify → 中途伪报完成, 判失败
    const why = lastAiContent
      ? `任务未完成: 大模型提前结束且未通过 verify 验证 (最后回复: ${lastAiContent.slice(0, 80)})`
      : '任务未完成: 大模型提前结束且未通过 verify 验证';
    this._log(`[LLM] ⚠ ${why}`);
    return { ok: false, summary: why, turns: finalState.turns || 0 };
  }
}
