/**
 * LangGraphAgent.js — 基于 LangGraph 的交互型智能体
 *
 * 用 @langchain/langgraph 的 StateGraph 重写 LLMAgent 的 function-calling 循环,
 * 替代 LLMAgent.js 中手写的 while-turn 轮询。接口与 LLMAgent 完全一致,
 * main.js / AgentPanel / MockAgent 无需改动即可热替换。
 *
 * 图结构 (LangGraph StateGraph):
 *   START ─► agent ─┬─(有 tool_calls)───────────────────► tools ─┬─(未熔断且未达上限)─► agent
 *                  ├─(无 tool_calls, 未verify/失败未恢复)──────► guard ─► agent   (注入"继续"指令, ≤5次)
 *                  └─(无 tool_calls, 已verify+guard≥1/纯查询/达上限)─► END
 *   tools 节点: verify 成功 → verified=true; actuation 工具调用 → manipulationStarted=true
 *   guard 节点 (校验节点): LLM 输出无 tool_call 时, 不依赖 API 的 tool_choice 能力,
 *       纯 LangGraph 层校验 — 追加 HumanMessage 丢回 agent 重新生成, 防止中途伪报完成.
 *       guard 从最近 ToolMessage 提取具体下一步 (suggested_chassis 坐标等), 直接告诉
 *       LLM 调用什么工具+什么参数, 而非冗长说教.
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
import * as THREE from 'three';
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
const FINISH_GUARD_LIMIT = 5;                 // LLM 提前结束但未 verify 时, 最多注入 N 次继续指令
const API_RETRY_LIMIT = 2;                    // 模型 invoke 失败 (如 400 非法 function.arguments) 最多重试 N 次

// actuation 类工具: 调用过任一即视为"已开始作业", 收尾必须 verify
const MANIPULATION_TOOLS = new Set([
  'plan_arm_motion', 'grasp', 'release', 'move_base',
]);

// ─────────────── 历史消息 sanitize ───────────────
// 某些服务商 (阿里 Qwen/DashScope 等) 回放历史时, 若 assistant 消息的
// function.arguments 不是合法 JSON (no-arg 工具被模型写成空串 ""), 会以 400 拒绝。
// LangChain 解析模型响应时, 标准 tool_calls (args=对象) 与原始 additional_kwargs.tool_calls
// (arguments=JSON字符串, 可能是空串 "") 会同时存在于 AIMessage 上。回放时 LangChain
// 优先用标准 tool_calls, 但若 tool_calls 为空则回退到 additional_kwargs (含非法空串)。
// 清洗策略:
//   1. 保证每个 tool_call.args 为可序列化对象 (string→parse 失败→{}, null/undefined→{}, 确保能 stringify)
//   2. 清除 additional_kwargs.tool_calls 和 additional_kwargs.function_call (原始格式可能含非法 arguments)
//   3. 对有 tool_calls 的 AIMessage 总是重建 (确保 additional_kwargs 干净)
function sanitizeMessages(msgs) {
  const out = [];
  for (const m of msgs) {
    if (!(m instanceof AIMessage)) { out.push(m); continue; }
    const hasTC = m.tool_calls && m.tool_calls.length > 0;
    const akTC = m.additional_kwargs?.tool_calls;
    const akFC = m.additional_kwargs?.function_call;
    if (!hasTC && akTC == null && akFC == null) { out.push(m); continue; }

    let dirty = false;
    const cleanCalls = (m.tool_calls || []).map(tc => {
      let args = tc.args;
      if (typeof args === 'string') {
        try { args = JSON.parse(args || '{}'); }
        catch { args = {}; }
      } else if (!args || typeof args !== 'object' || Array.isArray(args)) {
        args = {};
      }
      try { JSON.stringify(args); }
      catch { args = {}; }
      if (args !== tc.args) dirty = true;
      return { ...tc, args };
    });
    const cleanKw = { ...(m.additional_kwargs || {}) };
    if (cleanKw.tool_calls != null) { delete cleanKw.tool_calls; dirty = true; }
    if (cleanKw.function_call != null) { delete cleanKw.function_call; dirty = true; }

    if (dirty) {
      const nm = new AIMessage({
        content: m.content ?? '',
        tool_calls: cleanCalls,
        additional_kwargs: cleanKw,
      });
      if (m.id) nm.id = m.id;
      if (m.name) nm.name = m.name;
      out.push(nm);
    } else {
      out.push(m);
    }
  }
  return out;
}

// ─────────────── 历史消息裁剪 ───────────────
// 长任务 (30+ 轮) 中, 完整历史会超出弱模型的上下文窗口, 导致空响应/混乱.
// 裁剪策略: 保留 SystemMessage + 首条 HumanMessage (指令) + 最近 N 条消息.
// 裁剪尾部开头可能落在孤立消息上 (ToolMessage 无对应 AIMessage, 或 AIMessage
// 有 tool_calls 但对应 ToolMessage 已被裁掉) → 逐个删除直到尾部开头安全.
const PRUNE_KEEP_LAST = 30;
function pruneMessages(msgs) {
  if (msgs.length <= PRUNE_KEEP_LAST + 2) return msgs;
  const head = msgs.slice(0, 2);  // SystemMessage + first HumanMessage
  let tail = msgs.slice(-PRUNE_KEEP_LAST);
  while (tail.length > 0) {
    const m = tail[0];
    // 孤立 ToolMessage (对应 AIMessage 已被裁掉)
    if (m instanceof ToolMessage) { tail = tail.slice(1); continue; }
    // 孤立 AIMessage 带 tool_calls (对应 ToolMessage 已被裁掉)
    if (m instanceof AIMessage && m.tool_calls && m.tool_calls.length > 0) {
      const ids = new Set(m.tool_calls.map(tc => tc.id));
      const hasResp = tail.slice(1).some(m2 =>
        m2 instanceof ToolMessage && ids.has(m2.tool_call_id));
      if (!hasResp) { tail = tail.slice(1); continue; }
    }
    break;
  }
  return [...head, ...tail];
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
    this._instruction = '';        // 原始指令 (用于解析目标料箱, 在 guard 提示中给出精确坐标)
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
        const msgs = pruneMessages(sanitizeMessages(state.messages.concat(added)));
        try {
          const resp = await self._boundModel.invoke(msgs, { signal: self._abortCtrl?.signal });
          if (resp.content) self._log(`[LLM] ${resp.content}`);

          // 检测空响应 (无文字无工具调用) → 注入智能提示重试, 不浪费 guard 配额
          // 弱模型在子任务切换点常返回空响应 — 不知下一步该调什么工具.
          // 扫描最近的 ToolMessage, 调用 _computeGuardHint 给出具体下一步指令.
          const hasToolCalls = resp.tool_calls && resp.tool_calls.length > 0;
          if (!resp.content && !hasToolCalls && attempt < API_RETRY_LIMIT) {
            self._log(`[LLM] 模型返回空响应, 注入智能提示重试 (尝试${attempt + 1}/${API_RETRY_LIMIT + 1})`);
            let ltName = null, ltResult = null;
            for (let i = state.messages.length - 1; i >= 0; i--) {
              if (state.messages[i] instanceof ToolMessage) {
                ltName = state.messages[i].name;
                try { ltResult = JSON.parse(state.messages[i].content); } catch (e) {}
                break;
              }
            }
            let hint = null;
            try { hint = await self._computeGuardHint(ltName, ltResult, ''); }
            catch (e) { self._log(`[LLM] 提示计算失败: ${e.message}`); }
            const msg = hint
              ? `⚠ 你的上一次回复为空。${hint}`
              : `⚠ 你的上一次回复为空 (既无文字也无工具调用)。请立即调用工具执行下一步, 不要返回空回复。`;
            added.push(new HumanMessage({ content: msg }));
            continue;
          }
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
      let { failCount, planFailTarget, planFailCount, callHistory, verified, manipulationStarted, finishGuardAttempts } = state;
      callHistory = [...callHistory];
      let fuse = null;

      for (const tc of calls) {
        const name = tc.name;
        let params = {};
        try { params = (tc.args && typeof tc.args === 'object') ? tc.args : JSON.parse(tc.args || '{}'); }
        catch (e) { self._log(`[LLM] 工具参数解析失败: ${e.message}`); }

        if (self.onToolCall) self.onToolCall(name, params);

        // actuation 类工具一经调用即视为"已开始作业", 后续收尾必须 verify
        if (MANIPULATION_TOOLS.has(name)) {
          manipulationStarted = true;
          // ★ 复合任务: 新的 actuation 操作会作废之前的 verify, 每个 子任务需独立 verify
          // (防止"放完扳手1后抓扳手2失败时, 仍因之前 verify 过而误判任务完成")
          verified = false;
        }

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

        // ★ actuation 工具成功 → 子任务有物理进展, 重置 guard 配额
        // (防止前面子任务中 LLM 良性"宣告意图但没调工具"消耗的 guard 次数,
        //   导致后续子任务需要 guard 时配额已耗尽而直接 END)
        // retract 也算物理进展 (臂已收回), 但不加入 MANIPULATION_TOOLS 以免作废 verify
        if ((MANIPULATION_TOOLS.has(name) || name === 'retract') && result.ok) {
          finishGuardAttempts = 0;
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
        verified, manipulationStarted, finishGuardAttempts,
      };
      if (fuse) patch.fuse = fuse;
      return patch;
    };

    // guard 节点 (校验节点): LLM 提前结束 (无 tool_calls) 时, 注入极简指令逼回工具循环.
    // 不依赖 API 的 tool_choice 能力, 纯 LangGraph 层校验: LLM 输出无 tool_call → 追加
    // HumanMessage 丢回 agent 重新生成. 策略: 从最近 ToolMessage 提取具体下一步动作
    // (suggested_chassis 坐标等), 直接告诉 LLM 调用什么工具+什么参数, 而非冗长说教.
    const guardNode = async (state) => {
      const n = (state.finishGuardAttempts || 0) + 1;

      // 解析最近 ToolMessage (跳过 AIMessage/HumanMessage, 向前查找)
      // 上一版在遇到 AIMessage 时 break, 导致空响应重试/guard 注入后 lastResult 恒为 null,
      // guard 永远走通用分支, 无法给出 verify-success 等精确提示.
      let lastResult = null, lastToolName = null;
      for (let i = state.messages.length - 1; i >= 0; i--) {
        const m = state.messages[i];
        if (m instanceof ToolMessage) {
          lastToolName = m.name;
          try { lastResult = JSON.parse(m.content); } catch (e) {}
          break;
        }
      }

      // 取最近 AIMessage 文本 (解析 LLM 意图, 给出精确工具调用坐标)
      let lastAiContent = '';
      for (let i = state.messages.length - 1; i >= 0; i--) {
        if (state.messages[i] instanceof AIMessage) {
          lastAiContent = state.messages[i].content || '';
          break;
        }
      }

      let content;
      if (lastResult && lastResult.ok === false && lastResult.suggested_chassis) {
        // plan_arm_motion 失败且有建议站位 → 直接给出 move_base 参数 (唯一需要格式化坐标的分支)
        const sc = lastResult.suggested_chassis;
        if (sc.no_collision_free) {
          content = `⚠ ${lastToolName}失败且所有站位均无法避免碰撞。请更换目标(调用perceive)或回复"无法完成"。(第${n}/${FINISH_GUARD_LIMIT}次)`;
        } else {
          content = `⚠ ${lastToolName}失败(${lastResult.reason})。立即调用 move_base(x=${sc.x.toFixed(3)}, y=${sc.y.toFixed(3)}, yaw=${sc.yaw.toFixed(3)})，不要输出文字。(第${n}/${FINISH_GUARD_LIMIT}次)`;
        }
      } else {
        // 所有其他情况 (成功/失败/verify/空) 统一走状态机提示
        let hint = null;
        try { hint = await self._computeGuardHint(lastToolName, lastResult, lastAiContent); }
        catch (e) { self._log(`[LLM] guard 提示计算失败: ${e.message}`); }
        content = hint
          ? `⚠ ${hint} (第${n}/${FINISH_GUARD_LIMIT}次)`
          : `⚠ 你未调用任何工具。请立即调用工具执行下一步, 不要只描述意图。(第${n}/${FINISH_GUARD_LIMIT}次)`;
      }

      const nudge = new HumanMessage({ content });
      self._log(`[LLM] ⚠ 大模型提前结束, 注入继续指令 (第${n}/${FINISH_GUARD_LIMIT}次)`);
      return { messages: [nudge], finishGuardAttempts: n };
    };

    // agent → tools | guard | END
    const afterAgent = (state) => {
      if (state.aborted || state.fuse) return END;
      if ((state.turns || 0) >= self.maxTurns) return END;
      const last = state.messages[state.messages.length - 1];
      if (last instanceof AIMessage && last.tool_calls && last.tool_calls.length > 0) return 'tools';
      // LLM 想收尾 (无 tool_calls): 查找最近的 ToolMessage
      // (跳过 AIMessage/HumanMessage — guard 注入/空响应重试产生的消息, 向前找到真正的工具结果)
      let lastToolMsg = null;
      for (let i = state.messages.length - 1; i >= 0; i--) {
        if (state.messages[i] instanceof ToolMessage) {
          lastToolMsg = state.messages[i];
          break;
        }
      }

      let lastToolFailed = false;
      let lastToolIsVerifyOk = false;
      if (lastToolMsg) {
        try {
          const r = JSON.parse(lastToolMsg.content);
          lastToolFailed = (r.ok === false);
          lastToolIsVerifyOk = (lastToolMsg.name === 'verify' && r.ok === true);
        } catch (e) {}
      }

      // ★ 未恢复的失败不允许直接结束 → guard 逼回工具循环
      if (lastToolFailed && (state.finishGuardAttempts || 0) < FINISH_GUARD_LIMIT) return 'guard';

      // 纯信息查询 (只 perceive/scene, 未启动作业) → 允许结束
      if (!state.manipulationStarted) return END;

      // ★ verify 通过后模型宣告完成:
      //   guard 已至少注入过 1 次 (给过模型重新考虑/补 verify 的机会) → 允许 END
      //   (防止 verify 一个子任务后模型伪报完成 — guard 先 nudge 一次让模型补 verify 或继续;
      //    也不至于无限 guard 到 5/5 — 之前两轮 verify 都 ok 后模型说"完成"仍被 guard 反复 nudge)
      if (state.verified && lastToolIsVerifyOk && (state.finishGuardAttempts || 0) >= 1) return END;

      // guard 上限 → END
      if ((state.finishGuardAttempts || 0) >= FINISH_GUARD_LIMIT) return END;
      return 'guard';
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
      maxTokens: 4096,
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
    this._instruction = instruction;
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

  /**
   * 计算下一步提示 (guard 节点 + 空响应重试共用).
   *
   * 弱模型 (glm 等) 在长任务中途常出现两类问题:
   *   1. 子任务切换点返回空响应 (不知下一步该调什么工具)
   *   2. 只描述意图而不调工具 ("我要移动到料箱前方" → 停下)
   *
   * 本方法用状态机 (lastToolName + gripper 状态) 推断下一步,
   * 并从原始指令解析目标料箱, 直接给出 move_base 精确坐标.
   *
   * @param {string} lastToolName - 最近执行的工具名
   * @param {object|null} lastResult - 最近工具结果 (解析后的 JSON)
   * @param {string} lastAiContent - LLM 最后一条 AIMessage 文本
   * @returns {Promise<string|null>} 具体提示文本; null=无法给出具体建议
   */
  async _computeGuardHint(lastToolName, lastResult, lastAiContent) {
    if (!this.executor) return null;

    // 解析目标料箱: 优先从 AI 文本, 其次从原始指令 (空响应时 lastAiContent 为空)
    const text = (lastAiContent || '') + ' ' + (this._instruction || '');
    let slotNum = null;
    if (/第\s*[一1]\s*[个]?(?:料箱|格|槽)/.test(text) || /(?:料箱|格|槽)\s*[1一]/.test(text) || /slot\s*1/i.test(text)) slotNum = 1;
    else if (/第\s*[二2]\s*[个]?(?:料箱|格|槽)/.test(text) || /(?:料箱|格|槽)\s*[2二]/.test(text) || /slot\s*2/i.test(text)) slotNum = 2;
    else if (/第\s*[三3]\s*[个]?(?:料箱|格|槽)/.test(text) || /(?:料箱|格|槽)\s*[3三]/.test(text) || /slot\s*3/i.test(text)) slotNum = 3;

    // 查询机器人状态 (夹爪闭合 = 抓有物体)
    let gripperClosed = false;
    try {
      const rs = this.executor._getRobotState?.();
      if (rs && rs.ok) gripperClosed = (rs.gripper === 'closed');
    } catch (e) {}

    const binSlots = this.executor.binSlots || {};
    const slot = slotNum && (binSlots[slotNum] || binSlots[String(slotNum)]);
    const ok = lastResult && lastResult.ok === true;

    // ── 失败恢复提示 (无 suggested_chassis 的失败, 由状态机给出具体恢复策略) ──
    if (lastResult && lastResult.ok === false) {
      if (lastToolName === 'grasp') {
        return '抓取失败(3.5cm内无物体)。立即调用 perceive 确认目标坐标; 若目标不可达(distance > arm_reach), 按 suggested_chassis 调用 move_base 靠近后重试。禁止只输出文字。';
      }
      if (lastToolName === 'release') {
        return '放置失败(夹爪未持物)。立即调用 get_robot_state 确认夹爪状态; 若夹爪已张开但未持物, 说明之前未抓取成功, 调用 perceive 找下一个目标。禁止只输出文字。';
      }
      if (lastToolName === 'retract') {
        return '收回失败(所有抬升策略均碰撞)。调用 move_base 稍微移开当前位置再重试, 或调用 perceive 检查场景。禁止只输出文字。';
      }
      if (lastToolName === 'move_base') {
        return '底盘移动失败。更换目标坐标(避开工作台占地范围)重试, 或调用 get_scene_info 获取可移动区域。禁止只输出文字。';
      }
      return `上一步${lastToolName}失败(${lastResult.reason})。请调用工具恢复, 不要只输出文字。`;
    }

    // ── 状态机: lastToolName + gripper → 下一步具体指令 ──
    // 覆盖子任务切换点的所有常见过渡, 防止模型返回空响应
    if (lastToolName === 'grasp' && ok) {
      return '已抓取物体, 立即调用 retract 收回机械臂, 然后 move_base 到目标料箱放置。禁止只输出文字。';
    }
    if (lastToolName === 'release' && ok) {
      return '已放置物体, 立即调用 retract 收回机械臂, 然后调用 perceive 查找下一个目标或 verify 验证。禁止只输出文字。';
    }
    if (lastToolName === 'plan_arm_motion' && ok) {
      if (gripperClosed)
        return '臂已到位 (已抓取物体), 立即调用 release 放置。禁止只输出文字。';
      return '臂已到位, 立即调用 grasp 抓取。禁止只输出文字。';
    }
    if (lastToolName === 'move_base' && ok) {
      if (gripperClosed)
        return '已到达站位 (已抓取物体), 立即调用 plan_arm_motion 到料箱位置放置, 然后 release。禁止只输出文字。';
      return '已到达站位, 立即调用 plan_arm_motion 到目标位置, 然后 grasp 抓取。禁止只输出文字。';
    }
    if (lastToolName === 'retract' && ok && !gripperClosed) {
      return '臂已收回 (未抓取物体), 立即调用 perceive 查找下一个目标, 或 verify 验证已完成的子任务。禁止只输出文字。';
    }
    if (lastToolName === 'perceive' && ok) {
      if (gripperClosed)
        return '已感知场景 (已抓取物体), 立即按感知结果中的 suggested_chassis 调用 move_base 到目标料箱前方放置。禁止只输出文字。';
      return '已感知场景, 立即按感知结果中的 suggested_chassis 调用 move_base 到目标附近, 然后 plan_arm_motion + grasp。禁止只输出文字。';
    }
    if (lastToolName === 'verify' && ok) {
      return '已验证通过。若复合指令还有子任务, 立即调用 perceive 或 move_base 继续; 若全部完成, 回复"全部完成"。';
    }
    if ((lastToolName === 'get_scene_info' || lastToolName === 'get_robot_state') && ok) {
      if (gripperClosed)
        return '已获取信息 (已抓取物体), 立即调用 move_base 到目标料箱前方放置。禁止只输出文字。';
      return '已获取信息, 立即调用 move_base 或 plan_arm_motion 执行下一步。禁止只输出文字。';
    }

    // ── 已抓取物体 + 已知目标料箱 → 给出 move_base + plan_arm_motion 精确坐标 ──
    if (gripperClosed && slot && typeof this.executor._suggestChassisFor === 'function') {
      const target = new THREE.Vector3(+slot[0], +slot[1], +slot[2]);
      let sc;
      try { sc = this.executor._suggestChassisFor(target); }
      catch (e) { return null; }
      if (!sc) return null;
      const mv = `move_base(x=${sc.x}, y=${sc.y}, yaw=${sc.yaw})`;
      const place = `plan_arm_motion(x=${(+slot[0]).toFixed(3)}, y=${(+slot[1]).toFixed(3)}, z=${(+slot[2]).toFixed(3)})`;
      if (sc.no_collision_free) {
        return `你已抓取物体, 放到第${slotNum}料箱所有站位均有碰撞风险。先调用 ${mv} 换站位, 再 ${place}; 若仍失败则更换目标。禁止只输出文字。`;
      }
      return `你已抓取物体, 立即调用 ${mv} 到第${slotNum}料箱前方, 到位后调用 ${place} 放置, 然后 release + retract + verify(type=place, slot=${slotNum})。禁止只输出文字。`;
    }

    // ── 已抓取物体但未知目标料箱 → 提示获取场景信息 ──
    if (gripperClosed) {
      return '你已抓取物体但未调用工具。立即调用 get_scene_info 获取料箱格子坐标, 然后 move_base 到目标料箱前方放置。禁止只输出文字。';
    }

    // ── 未抓取物体 → 提示感知场景找下一个目标 ──
    if (/抓|取|扳手|螺母|螺丝|工具/.test(text)) {
      return '未抓取物体。立即调用 perceive 获取工具坐标, 不可达时按返回的 suggested_chassis 调用 move_base。禁止只输出文字。';
    }
    return '未调用任何工具。立即调用 perceive 或 get_scene_info 获取场景信息后执行下一步。禁止只输出文字。';
  }
}
