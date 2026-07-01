/**
 * main.js — 入口
 * 初始化场景 → 加载机器人(URDF+STL) → 构建 ROS 桥 → 绑定 UI → 渲染循环
 */
import { SceneSetup } from './SceneSetup.js';
import { RobotModel } from './RobotModel.js';
import { IKSolver } from './IKSolver.js';
import { RosBridge } from './RosBridge.js';
import { ChassisController } from './ChassisController.js';
import { ArmKeyboardController } from './ArmKeyboardController.js';
import { CameraView } from './CameraView.js';
import { UIController } from './UIController.js';
import { AgentPanel } from './AgentPanel.js';
import { MockAgent } from './MockAgent.js';
import { DatasetGenerator } from './DatasetGenerator.js';
import { MCPToolExecutor, MCP_TOOLS } from './MCPTools.js';
import { LangGraphAgent } from './LangGraphAgent.js';
// 原生 fetch + function-calling 实现, 保留作 fallback (注释切换即可):
// import { LLMAgent } from './LLMAgent.js';

async function main() {
  // 1. 场景
  const canvas = document.getElementById('three-canvas');
  const scene = new SceneSetup(canvas);

  // 2. 加载机器人模型 (URDF 运动学树 + 真实 STL 网格)
  const robot = new RobotModel();
  await robot.load();
  scene.add(robot.root);

  // 3. ROS 桥接
  const ros = new RosBridge();
  // 真机 voice_joint_states 按顺序读 [joint1..joint5, joint6], 需当前关节状态补全发送帧
  ros.getCurrentJoints = () => {
    const ns = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    const m = {};
    for (const n of ns) m[n] = robot.getJoint(n);
    return m;
  };

  // 4. IK 求解器
  const ik = new IKSolver(robot);

  // 5. 底盘运动控制器 (仿真驱动 + 真机同步)
  const chassis = new ChassisController(robot, ros);

  // 6. 机械臂键盘控制器
  const armKey = new ArmKeyboardController(robot, ros);

  // 6.5 实时相机画面 (双槽, 由 UIController 绑定 DOM 控件)
  const cameras = [
    new CameraView(ros,
      document.querySelector('.cam-canvas[data-cam="0"]'),
      document.querySelector('.cam-status[data-cam="0"]')),
    new CameraView(ros,
      document.querySelector('.cam-canvas[data-cam="1"]'),
      document.querySelector('.cam-status[data-cam="1"]')),
  ];

  // 7. UI 控制器
  const ui = new UIController({ robot, ik, ros, scene, chassis, armKey, cameras });
  ui.init();

  // 7.5 交互型智能体状态面板 (订阅 /agent/status /agent/log, 发布 /agent/instruction)
  // 放在 ui.init() 之后, 自包含轮询 ros 连接/版本, 不侵入 UIController
  const agentPanel = new AgentPanel(ros);
  // 暴露给 UIController 版本切换时调用重建 (若 UIController 有 version 切换钩子)
  ros._agentPanel = agentPanel;

  // 7.6 仿真智能体 (LLM 大模型 + MCP 工具调用)
  const mockAgent = new MockAgent(robot, ik, scene, agentPanel, ros, chassis);
  agentPanel.setMockAgent(mockAgent);

  // 7.7 数据集生成器
  const datasetGen = new DatasetGenerator(scene, mockAgent);
  agentPanel.setDatasetGenerator(datasetGen);

  // 7.8 MCP 工具执行器 (大模型 function calling 的后端)
  const mcpExecutor = new MCPToolExecutor({
    mockAgent, robot, ik, chassis, ros,
    binSlots: mockAgent._binSlots || {},
  });
  mcpExecutor.onLog(msg => agentPanel._onLog(msg));

  // 7.9 LLM 大模型智能体 (基于 LangGraph StateGraph: agent↔tools 循环)
  // 接口与 LLMAgent 一致, 可直接换回 new LLMAgent({...}) 作原生模式
  const llmAgent = new LangGraphAgent({
    apiBase: localStorage.getItem('llm_api_base') || 'https://api.openai.com/v1',
    apiKey: localStorage.getItem('llm_api_key') || '',
    model: localStorage.getItem('llm_model') || 'gpt-4o',
  });
  llmAgent.setExecutor(mcpExecutor);
  llmAgent.setLogCb(msg => agentPanel._onLog(msg));
  llmAgent.onToolCall = (name, params) => agentPanel._onLog(`[tool] ${name}(${JSON.stringify(params).substring(0, 80)})`);

  // 注入到 MockAgent (双模式切换: 有API Key走LLM, 无则走正则)
  mockAgent.setLLMAgent(llmAgent);
  agentPanel.setLLMAgent(llmAgent);

  // 8. 渲染循环
  let lastJointPub = 0;
  function animate() {
    requestAnimationFrame(animate);
    const now = performance.now();
    chassis.update();
    armKey.update();
    robot.update();
    mockAgent.update();  // 被抓的工具跟随 TCP
    ui.tick();
    // 持续发布 /joint_states (~10Hz), 覆盖 MoveIt joint_state_publisher 的零值
    // joint_state_publisher 本身 10Hz, 喂更快无意义, 降低 WebSocket 负载
    if (ros.publishEnabled && ros.connected && now - lastJointPub > 100) {
      lastJointPub = now;
      ros.publishJointStates(robot.getJointStateArray());
    }
    scene.render();
  }
  animate();

  console.log('[Sim2Real] 初始化完成 — mini_4wd_six_arm 数字孪生已就绪');
}

main().catch(err => {
  console.error('[Sim2Real] 初始化失败:', err);
  document.getElementById('ros-status').textContent = '初始化失败: ' + err.message;
});
