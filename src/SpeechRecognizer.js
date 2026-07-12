/**
 * SpeechRecognizer.js — 语音识别模块 (PTT 按住说话)
 *
 * 浏览器端用 MediaRecorder 采集音频 (webm/opus), POST 到外置机 ASR 服务
 * (SenseVoice, :8766/asr), 收到识别文本后交给 InstructionParser 解析。
 *
 * 交互方式: 按住麦克风按钮开始录音, 松开停止并自动上传。
 * 外置机 ASR 推理跑在 RTX GPU 上, 与 LocateAnything / LLM serving 同理,
 * Jetson 不参与推理。
 *
 * 用法:
 *   const sr = new SpeechRecognizer({
 *     serverUrl: 'http://192.168.0.100:8766',
 *     onText: (text) => { instructionParser.parse(text); },
 *     onState: (state) => { updateMicButton(state); },
 *     onError: (msg) => { console.warn(msg); },
 *   });
 *   button.addEventListener('mousedown', () => sr.start());
 *   button.addEventListener('mouseup',   () => sr.stop());
 */

export class SpeechRecognizer {
  /**
   * @param {Object}   opts
   * @param {string}   opts.serverUrl     ASR 服务地址 (含端口, 不含 /asr)
   * @param {function} opts.onText        (text, meta) => void  识别成功
   * @param {function} opts.onState       (state) => void  idle | recording | processing
   * @param {function} opts.onError       (msg) => void
   * @param {number}   opts.timeout       上传超时 ms (默认 15000)
   */
  constructor({ serverUrl, onText, onState, onError, timeout } = {}) {
    this.serverUrl = (serverUrl || localStorage.getItem('asr_server_url') || 'http://192.168.0.100:8766').replace(/\/+$/, '');
    this.onText = onText || null;
    this.onState = onState || null;
    this.onError = onError || null;
    this.timeout = timeout || 15000;

    this._recorder = null;
    this._stream = null;
    this._chunks = [];
    this._recording = false;
    this._state = 'idle';
  }

  /** 浏览器是否支持 getUserMedia + MediaRecorder */
  isSupported() {
    return !!(navigator.mediaDevices && navigator.mediaDevices.getUserMedia && window.MediaRecorder);
  }

  setServerUrl(url) {
    this.serverUrl = (url || '').replace(/\/+$/, '');
    if (this.serverUrl) localStorage.setItem('asr_server_url', this.serverUrl);
  }

  get state() { return this._state; }

  _setState(s) {
    this._state = s;
    if (this.onState) this.onState(s);
  }

  /** 按下: 开始录音 */
  async start() {
    if (this._recording) return;
    if (!this.isSupported()) {
      this._err('浏览器不支持语音录制 (需要 getUserMedia + MediaRecorder)');
      return;
    }
    try {
      this._stream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
      });
    } catch (e) {
      this._err('麦克风访问失败: ' + (e.message || e.name || '权限被拒绝'));
      return;
    }
    this._chunks = [];
    const mime = this._pickMime();
    try {
      this._recorder = new MediaRecorder(this._stream, mime ? { mimeType: mime } : undefined);
    } catch {
      this._recorder = new MediaRecorder(this._stream);
    }
    this._recorder.ondataavailable = (e) => {
      if (e.data && e.data.size > 0) this._chunks.push(e.data);
    };
    this._recorder.onstop = () => this._onStop();
    this._recorder.start();
    this._recording = true;
    this._setState('recording');
  }

  /** 松开: 停止录音并上传 */
  stop() {
    if (!this._recording || !this._recorder) return;
    try { this._recorder.stop(); } catch { /* noop */ }
    if (this._stream) this._stream.getTracks().forEach((t) => t.stop());
    this._recording = false;
    this._setState('processing');
  }

  /** 强制取消 (录音中松开按钮外的取消, 如切换页面) */
  cancel() {
    if (this._recording && this._recorder) {
      this._chunks = [];
      try { this._recorder.stop(); } catch { /* noop */ }
      if (this._stream) this._stream.getTracks().forEach((t) => t.stop());
      this._recording = false;
    }
    this._setState('idle');
  }

  _pickMime() {
    const candidates = [
      'audio/webm;codecs=opus',
      'audio/webm',
      'audio/ogg;codecs=opus',
      'audio/mp4',
    ];
    for (const t of candidates) {
      if (window.MediaRecorder && MediaRecorder.isTypeSupported(t)) return t;
    }
    return null;
  }

  async _onStop() {
    if (!this._chunks.length) {
      this._setState('idle');
      return;
    }
    const mimeType = this._recorder ? this._recorder.mimeType : 'audio/webm';
    const blob = new Blob(this._chunks, { type: mimeType || 'audio/webm' });
    const ext = (mimeType || '').includes('webm') ? 'webm'
      : (mimeType || '').includes('ogg') ? 'ogg'
      : (mimeType || '').includes('mp4') ? 'm4a'
      : 'webm';

    const fd = new FormData();
    fd.append('file', blob, `speech.${ext}`);
    fd.append('language', 'auto');

    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeout);
    try {
      const resp = await fetch(`${this.serverUrl}/asr`, {
        method: 'POST',
        body: fd,
        signal: controller.signal,
      });
      if (!resp.ok) {
        const txt = await resp.text().catch(() => '');
        this._err(`ASR 服务错误 (HTTP ${resp.status})${txt ? ': ' + txt.slice(0, 100) : ''}`);
        return;
      }
      const data = await resp.json();
      const text = (data.text || '').trim();
      if (text) {
        if (this.onText) this.onText(text, data);
      } else {
        this._err('识别结果为空, 请重试');
      }
    } catch (e) {
      if (e.name === 'AbortError') this._err('ASR 请求超时');
      else this._err('ASR 请求失败: ' + (e.message || '网络错误'));
    } finally {
      clearTimeout(timer);
      this._setState('idle');
    }
  }

  _err(msg) {
    this._setState('idle');
    if (this.onError) this.onError(msg);
  }
}
