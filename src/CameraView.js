/**
 * CameraView.js
 * 实时相机画面展示 — 通过 roslibjs 订阅图像话题, 解码到 <canvas>。
 *
 * 支持两种消息类型 (按 ROS 版本自动适配字符串):
 *   - CompressedImage (sensor_msgs/CompressedImage | sensor_msgs/msg/CompressedImage)
 *     带宽低 (jpeg/png), 推荐。data 为 base64, 直接 dataURL 绘制。
 *   - Image (sensor_msgs/Image | sensor_msgs/msg/Image) 原始图
 *     支持 rgb8 / bgr8 / mono8 编码, base64 解码后组装 ImageData。
 *
 * 每个实例对应一个相机槽: 一个 <canvas> + 一个状态元素。
 */
import ROSLIB from 'roslib';

// ROS1/ROS2 消息类型字符串
const TYPE_STR = {
  ros1: { compressed: 'sensor_msgs/CompressedImage', raw: 'sensor_msgs/Image' },
  ros2: { compressed: 'sensor_msgs/msg/CompressedImage', raw: 'sensor_msgs/msg/Image' },
};

// base64 → Uint8Array (浏览器 atob)
function b64ToUint8(b64) {
  const bin = atob(b64);
  const len = bin.length;
  const out = new Uint8Array(len);
  for (let i = 0; i < len; i++) out[i] = bin.charCodeAt(i);
  return out;
}

export class CameraView {
  /**
   * @param {Object} ros  RosBridge 实例 (需有 .ros, .version, .connected)
   * @param {HTMLCanvasElement} canvas 渲染目标
   * @param {HTMLElement} statusEl 状态文字元素
   */
  constructor(ros, canvas, statusEl) {
    this.ros = ros;
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.statusEl = statusEl;
    this.topicName = '';
    this.msgKind = 'compressed';   // 'compressed' | 'raw'
    this.enabled = false;
    this._sub = null;
    this._img = new Image();
    this._lastFrame = 0;
    this._fps = 0;
    this._fpsAcc = 0;
    this._fpsT0 = performance.now();
    // 默认尺寸
    this._placeholder();
  }

  _placeholder(text = '未启用') {
    const c = this.canvas;
    this.ctx.clearRect(0, 0, c.width, c.height);
    this.ctx.fillStyle = '#11121c';
    this.ctx.fillRect(0, 0, c.width, c.height);
    this.ctx.fillStyle = '#445';
    this.ctx.font = '12px Consolas, monospace';
    this.ctx.textAlign = 'center';
    this.ctx.fillText(text, c.width / 2, c.height / 2);
    this.ctx.textAlign = 'start';
  }

  _mt() {
    return TYPE_STR[this.ros.version || 'ros2'][this.msgKind];
  }

  /** 配置话题与类型 (未启用时仅记录, 启用时重建订阅) */
  configure(topicName, kind) {
    this.topicName = topicName;
    this.msgKind = (kind === 'raw') ? 'raw' : 'compressed';
    if (this.enabled) this._restart();
  }

  /** 启用: 创建订阅 */
  enable() {
    if (this.enabled) return;
    this.enabled = true;
    this._start();
  }

  /** 禁用: 取消订阅 */
  disable() {
    this.enabled = false;
    this._stop();
    this._placeholder('已停用');
    if (this.statusEl) this.statusEl.textContent = '已停用';
  }

  _start() {
    if (!this.ros.connected || !this.ros.ros || !this.topicName) {
      this._placeholder('等待连接');
      if (this.statusEl) this.statusEl.textContent = '等待连接';
      return;
    }
    this._stop();
    const throttle = this.msgKind === 'raw' ? 100 : 66; // raw 10Hz, compressed 15Hz
    this._sub = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: this.topicName,
      messageType: this._mt(),
      throttle_rate: throttle,
      queue_length: 1,
    });
    this._sub.subscribe((msg) => this._onFrame(msg));
    if (this.statusEl) this.statusEl.textContent = `订阅中: ${this.topicName}`;
    this._placeholder('等待画面…');
  }

  _stop() {
    if (this._sub) {
      try { this._sub.unsubscribe(); } catch (e) { /* ignore */ }
      this._sub = null;
    }
  }

  /** ROS 连接状态变化后重建订阅 */
  onRosStateChanged() {
    if (this.enabled) {
      this._stop();
      this._start();
    }
  }

  /** 版本切换后重建 (消息类型字符串变了) */
  rebuild() {
    if (this.enabled) {
      this._stop();
      this._start();
    }
  }

  _onFrame(msg) {
    try {
      if (this.msgKind === 'compressed') this._drawCompressed(msg);
      else this._drawRaw(msg);
      // FPS 估算
      const now = performance.now();
      this._fpsAcc++;
      if (now - this._fpsT0 >= 1000) {
        this._fps = this._fpsAcc;
        this._fpsAcc = 0;
        this._fpsT0 = now;
        if (this.statusEl) {
          this.statusEl.textContent = `${this.topicName} · ${this._fps} fps`;
        }
      }
      this._lastFrame = now;
    } catch (e) {
      if (this.statusEl) this.statusEl.textContent = `解码错误: ${e.message}`;
    }
  }

  _drawCompressed(msg) {
    // msg.format: 'jpeg' | 'png' | 'image/jpeg' ...
    let fmt = (msg.format || 'jpeg').toLowerCase();
    const mime = fmt.includes('png') ? 'image/png'
      : (fmt.includes('jpeg') || fmt.includes('jpg')) ? 'image/jpeg'
      : 'image/jpeg';
    const url = 'data:' + mime + ';base64,' + msg.data;
    const img = this._img;
    img.onload = () => {
      const c = this.canvas;
      // 保持宽高比铺满
      this._drawCover(img, img.width, img.height);
    };
    img.src = url;
  }

  _drawRaw(msg) {
    const w = msg.width | 0;
    const h = msg.height | 0;
    if (!w || !h) return;
    const enc = (msg.encoding || 'rgb8').toLowerCase();
    const u8 = b64ToUint8(msg.data || '');
    const c = this.canvas;
    // 内部绘制缓冲 = 原始分辨率
    c.width = w; c.height = h;
    const imgData = this.ctx.createImageData(w, h);
    const dst = imgData.data;
    if (enc === 'rgb8') {
      for (let i = 0, j = 0; i < u8.length && j < dst.length; i += 3, j += 4) {
        dst[j] = u8[i]; dst[j+1] = u8[i+1]; dst[j+2] = u8[i+2]; dst[j+3] = 255;
      }
    } else if (enc === 'bgr8') {
      for (let i = 0, j = 0; i < u8.length && j < dst.length; i += 3, j += 4) {
        dst[j] = u8[i+2]; dst[j+1] = u8[i+1]; dst[j+2] = u8[i]; dst[j+3] = 255;
      }
    } else if (enc === 'mono8' || enc === 'mono8uc3' || enc === '8uc1') {
      for (let i = 0, j = 0; i < u8.length && j < dst.length; i += 1, j += 4) {
        const v = u8[i]; dst[j] = v; dst[j+1] = v; dst[j+2] = v; dst[j+3] = 255;
      }
    } else {
      // 未知编码, 按 rgb8 兜底
      for (let i = 0, j = 0; i < u8.length && j < dst.length; i += 3, j += 4) {
        dst[j] = u8[i]; dst[j+1] = u8[i+1]; dst[j+2] = u8[i+2]; dst[j+3] = 255;
      }
    }
    this.ctx.putImageData(imgData, 0, 0);
    // 重置 canvas 显示尺寸 (CSS 控制), 但 width/height 已改为原图, 下次 _drawCover 之类不再用
    if (this.statusEl) this.statusEl.textContent = `${this.topicName} · ${enc} ${w}x${h}`;
  }

  _drawCover(img, iw, ih) {
    const c = this.canvas;
    c.width = iw; c.height = ih;
    this.ctx.drawImage(img, 0, 0, iw, ih);
  }
}
