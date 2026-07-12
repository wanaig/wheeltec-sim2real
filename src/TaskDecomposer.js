/**
 * TaskDecomposer.js — 任务序列分解模块
 *
 * 将 InstructionParser 解析后的结构化指令分解为显式任务步骤列表,
 * 每个步骤映射到对应的 MCP 工具, 供 AgentPanel 可视化展示与进度跟踪。
 *
 * 输入: InstructionParser.parse() 输出
 *   { ok, action, target_tool, side, slot, query, source }
 *
 * 输出: 步骤数组
 *   [{ id, title, tool, description, status, optional, conditional }]
 *
 *   status: pending | running | done | failed | skipped
 *   optional:    该步骤可能不执行 (如底盘导航, 目标已可达时跳过)
 *   conditional: 该步骤仅在特定条件下执行 (如失败重试)
 *
 * 用法:
 *   const decomposer = new TaskDecomposer();
 *   const steps = decomposer.decompose(plan);
 *   // → [{ id:1, title:'环境感知与目标定位', tool:'perceive', ... }, ...]
 */
import { TOOL_CN } from './SceneLayouts.js';

export class TaskDecomposer {
  /**
   * 根据解析后的指令生成显式任务序列
   * @param {Object} plan - InstructionParser 输出 {ok, action, target_tool, side, slot, query}
   * @returns {Array<{id, title, tool, description, status, optional?, conditional?}>}
   */
  decompose(plan) {
    if (!plan || !plan.ok) return [];

    const targetDesc = plan.target_tool
      ? (TOOL_CN[plan.target_tool] || plan.target_tool)
      : (plan.query || '目标物体');
    const sideDesc = plan.side === 'left' ? '左侧的'
      : plan.side === 'right' ? '右侧的' : '';
    const isPlace = plan.action === 'place' && plan.slot;

    const steps = [];
    let id = 1;

    // ── 步骤1: 环境感知与目标定位 ──
    steps.push({
      id: id++,
      title: '环境感知与目标定位',
      tool: 'perceive',
      description: plan.query
        ? `识别场景中所有工具/零件，按"${plan.query}"定位${sideDesc}${targetDesc}`
        : `识别场景中所有工具/零件，定位${sideDesc}${targetDesc}并获取世界坐标`,
      status: 'pending',
    });

    // ── 步骤2: 可达性评估与底盘站位规划 ──
    steps.push({
      id: id++,
      title: '可达性评估与底盘站位规划',
      tool: 'move_base',
      description: `评估${targetDesc}是否在机械臂臂展(0.40m)范围内；若不可达，A*路径规划移动底盘到最优抓取站位`,
      status: 'pending',
      optional: true,
    });

    // ── 步骤3: 抓取路径规划与避障 ──
    steps.push({
      id: id++,
      title: '抓取路径规划与避障',
      tool: 'plan_arm_motion',
      description: `多起始IK求解 + 碰撞检测 + 经由点轨迹，规划到${sideDesc}${targetDesc}正上方再垂直下降`,
      status: 'pending',
    });

    // ── 步骤4: 生成抓取姿态指令 ──
    steps.push({
      id: id++,
      title: '生成抓取姿态指令',
      tool: 'grasp',
      description: `下降到${targetDesc}位置，闭合夹爪执行抓取`,
      status: 'pending',
    });

    // ── 步骤5: 机械臂安全收回 ──
    steps.push({
      id: id++,
      title: '机械臂安全收回',
      tool: 'retract',
      description: '多策略垂直抬升到安全高度(0.16m)，避免与工作台/料箱壁碰撞',
      status: 'pending',
    });

    if (isPlace) {
      // ── 步骤6: 底盘导航至料箱区域 ──
      steps.push({
        id: id++,
        title: '底盘导航至料箱区域',
        tool: 'move_base',
        description: `A*路径规划移动底盘靠近料箱第${plan.slot}格区域`,
        status: 'pending',
        optional: true,
      });

      // ── 步骤7: 放置路径规划 ──
      steps.push({
        id: id++,
        title: '放置路径规划',
        tool: 'plan_arm_motion',
        description: `规划机械臂运动到料箱第${plan.slot}格坐标，经安全高度绕行避障`,
        status: 'pending',
      });

      // ── 步骤8: 生成放置姿态指令 ──
      steps.push({
        id: id++,
        title: '生成放置姿态指令',
        tool: 'release',
        description: `张开夹爪，将${targetDesc}放入料箱第${plan.slot}格`,
        status: 'pending',
      });

      // ── 步骤9: 机械臂安全收回 ──
      steps.push({
        id: id++,
        title: '机械臂安全收回',
        tool: 'retract',
        description: '放置后垂直抬升收回机械臂到安全姿态',
        status: 'pending',
      });

      // ── 步骤10: 放置结果验证 ──
      steps.push({
        id: id++,
        title: '放置结果验证',
        tool: 'verify',
        description: `检查${targetDesc}是否已正确放入料箱第${plan.slot}格`,
        status: 'pending',
      });

      // ── 步骤11: 失败自主感知与重试 ──
      steps.push({
        id: id++,
        title: '失败自主感知与重试',
        tool: 'replan',
        description: `若放置未入格，自主感知失败状态，重新抓取${targetDesc}并放置到指定格子`,
        status: 'pending',
        conditional: true,
      });
    } else {
      // ── 步骤6: 抓取结果验证 ──
      steps.push({
        id: id++,
        title: '抓取结果验证',
        tool: 'verify',
        description: `检查${targetDesc}是否已成功离开原位（被抓取）`,
        status: 'pending',
      });
    }

    return steps;
  }

  /**
   * NLU 模式步骤号 → 工具名映射
   * MockAgent.runNLU 的 [nlu] 步骤N 日志映射到 decomposer 的 tool
   */
  static NLU_STEP_TOOLS = {
    1: 'perceive',           // 感知场景物体
    2: null,                  // 筛选目标工具 (感知内部, 无独立工具)
    3: 'plan_arm_motion',     // 规划臂运动到目标位置
    4: 'grasp',               // 闭合夹爪抓取
    5: 'retract',             // 收回机械臂
    '6a': 'move_base',        // 底盘导航靠近料箱
    '6b': 'plan_arm_motion',  // 规划臂运动到格子
    '6c': 'release',          // 张开夹爪释放
    7: 'retract',             // 收回机械臂
  };

  /**
   * 从日志行解析 NLU 步骤号
   * @returns {string|null} 如 "1", "6a", 或 null
   */
  static parseNLUStep(line) {
    const m = line.match(/^\[nlu\]\s+步骤([0-9]+[a-c]?)\s*:/);
    return m ? m[1] : null;
  }

  /**
   * 从日志行解析 MCP 工具调用与结果
   * @returns {{type:'call'|'result', tool:string, ok?:boolean}|null}
   */
  static parseMCPLog(line) {
    // [MCP] 调用工具: perceive(...)
    const callMatch = line.match(/^\[MCP\]\s+调用工具:\s*(\w+)\s*\(/);
    if (callMatch) return { type: 'call', tool: callMatch[1] };

    // [MCP] perceive → ok  /  [MCP] perceive → fail: reason
    const resultMatch = line.match(/^\[MCP\]\s+(\w+)\s+→\s+(ok|fail)/);
    if (resultMatch) return { type: 'result', tool: resultMatch[1], ok: resultMatch[2] === 'ok' };

    return null;
  }
}
