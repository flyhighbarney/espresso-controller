'use strict';

(function() {

// ============================================================
// UTILITIES
// ============================================================

function getId(id) { return document.getElementById(id); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

function gaussRand() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function setStatValue(id, value, unit) {
  const el = getId(id);
  el.textContent = '';
  el.appendChild(document.createTextNode(value + ' '));
  const span = document.createElement('span');
  span.className = 'stat-unit';
  span.textContent = unit;
  el.appendChild(span);
}

function safeSliderValue(id, min, max, fallback) {
  const raw = +getId(id).value;
  if (!Number.isFinite(raw)) return fallback;
  return clamp(raw, min, max);
}

// 3x3 matrix ops for Kalman
function matMul(A, B) {
  const n = A.length, m = B[0].length, p = B.length;
  const C = Array.from({length: n}, () => new Array(m).fill(0));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++)
      for (let k = 0; k < p; k++) C[i][j] += A[i][k] * B[k][j];
  return C;
}

function matMul3(A, B, C) { return matMul(matMul(A, B), C); }

function matT(A) {
  const n = A.length, m = A[0].length;
  const T = Array.from({length: m}, () => new Array(n));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++) T[j][i] = A[i][j];
  return T;
}

function matAdd(A, B) {
  const n = A.length, m = A[0].length;
  const C = Array.from({length: n}, () => new Array(m));
  for (let i = 0; i < n; i++)
    for (let j = 0; j < m; j++) C[i][j] = A[i][j] + B[i][j];
  return C;
}

// ============================================================
// PLANT MODEL
// ============================================================

class PlantModel {
  constructor() { this.reset(); }

  reset() {
    this.pressure = 0;
    this.flow = 0;
    this.puckResistance = 10;
    this.flowIntegral = 0;
    this.lineBuffer = new Float32Array(3000);
    this.lineHead = 0;
    this.temperature = 93;
  }

  step(pumpDuty, dt, params) {
    const { r0, alpha, beta, delaySamples, sensorNoise } = params;
    const K_PUMP = 12.0;
    const TAU_PUMP = 0.3;

    this.lineBuffer[this.lineHead] = pumpDuty;
    const delayIdx = (this.lineHead - delaySamples + this.lineBuffer.length) % this.lineBuffer.length;
    const delayedDuty = this.lineBuffer[delayIdx];
    this.lineHead = (this.lineHead + 1) % this.lineBuffer.length;

    const pumpPressure = K_PUMP * delayedDuty;
    const dpump = (pumpPressure - this.pressure) / TAU_PUMP;

    const noise = 1 + beta * (Math.random() - 0.5) * 2;
    this.puckResistance = r0 * (1 + alpha * this.flowIntegral) * noise;
    if (this.puckResistance < 0.5) this.puckResistance = 0.5;

    if (this.puckResistance > 0.1) {
      this.flow = Math.max(0, this.pressure / this.puckResistance);
    }
    this.flowIntegral += this.flow * dt;

    this.pressure += (dpump - this.flow * this.puckResistance * 0.1) * dt;
    if (this.pressure < 0) this.pressure = 0;
    if (this.pressure > 15) this.pressure = 15;

    this.temperature += (93 - this.temperature) * 0.01 * dt + (Math.random() - 0.5) * 0.02;

    return {
      pressure: this.pressure + gaussRand() * sensorNoise,
      flow: Math.max(0, this.flow + gaussRand() * sensorNoise * 0.5),
      temperature: this.temperature + gaussRand() * 0.3,
      trueResistance: this.puckResistance,
      truePressure: this.pressure,
      trueFlow: this.flow,
    };
  }
}

// ============================================================
// PID CONTROLLER
// ============================================================

class PIDController {
  constructor(kp, ki, kd, dt) {
    this.kp = kp; this.ki = ki; this.kd = kd; this.dt = dt;
    this.reset();
  }

  reset() {
    this.integral = 0;
    this.prevError = 0;
  }

  update(setpoint, measurement) {
    const error = setpoint - measurement;
    this.integral += error * this.dt;
    this.integral = clamp(this.integral, -50, 50);
    const derivative = (error - this.prevError) / this.dt;
    this.prevError = error;
    return clamp(this.kp * error + this.ki * this.integral + this.kd * derivative, 0, 1);
  }

  setGains(kp, ki, kd) { this.kp = kp; this.ki = ki; this.kd = kd; }
}

// ============================================================
// KALMAN FILTER
// ============================================================

class KalmanFilter {
  constructor(dt) {
    this.dt = dt;
    this.reset();
  }

  reset() {
    this.x = [0, 0, 10];
    this.P = [[1,0,0],[0,1,0],[0,0,50]];
    this.Q = [[0.1,0,0],[0,0.05,0],[0,0,0.5]];
    this.R = [[0.2,0],[0,0.1]];
  }

  setProcessNoise(qr) { this.Q[2][2] = qr; }
  setMeasurementNoise(rp) { this.R[0][0] = rp; }

  predict(pumpDuty) {
    const K_PUMP = 12, INERTIA = 0.5, dt = this.dt;
    const [p, f, r] = this.x;

    let pNew = p + dt * (K_PUMP * pumpDuty - f * r);
    let fNew = f + dt * (p - f * r) / INERTIA;
    let rNew = r;
    if (pNew < 0) pNew = 0;
    if (fNew < 0) fNew = 0;
    if (rNew < 0.1) rNew = 0.1;
    this.x = [pNew, fNew, rNew];

    const F = [
      [1, -r*dt, -f*dt],
      [dt/INERTIA, 1-r*dt/INERTIA, -f*dt/INERTIA],
      [0, 0, 1]
    ];
    this.P = matAdd(matMul3(F, this.P, matT(F)), this.Q);
  }

  update(zPressure, zFlow) {
    const y = [zPressure - this.x[0], zFlow - this.x[1]];

    const S = [
      [this.P[0][0]+this.R[0][0], this.P[0][1]],
      [this.P[1][0], this.P[1][1]+this.R[1][1]]
    ];
    const det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (Math.abs(det) < 1e-10) return;
    const Si = [[S[1][1]/det, -S[0][1]/det],[-S[1][0]/det, S[0][0]/det]];

    const K = [];
    for (let i = 0; i < 3; i++) {
      K[i] = [];
      for (let j = 0; j < 2; j++) {
        K[i][j] = 0;
        for (let k = 0; k < 2; k++) K[i][j] += this.P[i][k] * Si[k][j];
      }
    }

    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 2; j++) this.x[i] += K[i][j] * y[j];
    if (this.x[0] < 0) this.x[0] = 0;
    if (this.x[1] < 0) this.x[1] = 0;
    if (this.x[2] < 0.1) this.x[2] = 0.1;

    const KH = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 2; j++) KH[i][j] = K[i][j];
    const IKH = [[1-KH[0][0],-KH[0][1],-KH[0][2]],
                  [-KH[1][0],1-KH[1][1],-KH[1][2]],
                  [-KH[2][0],-KH[2][1],1-KH[2][2]]];
    this.P = matMul(IKH, this.P);
  }

  getPressure() { return this.x[0]; }
  getFlow() { return this.x[1]; }
  getResistance() { return this.x[2]; }
}

// ============================================================
// ADAPTIVE CONTROLLER
// ============================================================

class AdaptiveController {
  constructor(kf, dt) {
    this.kf = kf;
    this.dt = dt;
    this.pid = new PIDController(1.2, 0.6, 0.08, dt);
    this.shotTime = 0;
    this.phase = 0;
    this.rNominal = 10;

    this.delayBuf = new Float32Array(64);
    this.delayHead = 0;
    this.delaySamples = 0;
    this.modelPressure = 0;

    this.gains = [
      [0.8, 0.3, 0.05],
      [1.5, 0.8, 0.10],
      [1.2, 0.6, 0.08],
      [0.6, 0.2, 0.03],
    ];
  }

  reset() {
    this.pid.reset();
    this.shotTime = 0;
    this.phase = 0;
    this.modelPressure = 0;
    this.delayHead = 0;
    this.delayBuf.fill(0);
  }

  setDelay(delaySec) {
    this.delaySamples = Math.min(63, Math.round(delaySec / this.dt));
  }

  setMLResistance(r) { this.rNominal = r; }

  update(setpoint, pressure, flow, prevDuty) {
    this.shotTime += this.dt;

    let newPhase = 0;
    if (this.shotTime < 3) newPhase = 0;
    else if (pressure < 7 && this.shotTime < 10) newPhase = 1;
    else if (this.shotTime < 25) newPhase = 2;
    else newPhase = 3;

    if (newPhase !== this.phase) {
      this.phase = newPhase;
      const g = this.gains[this.phase];
      let scale = this.rNominal / Math.max(0.1, this.kf.getResistance());
      scale = clamp(scale, 0.3, 3.0);
      this.pid.setGains(g[0] * scale, g[1] * scale, g[2]);
    }

    if (this.delaySamples > 0) {
      const MODEL_GAIN = 10, MODEL_TAU = 0.3;
      this.modelPressure += this.dt / MODEL_TAU * (MODEL_GAIN * prevDuty - this.modelPressure);

      const delayIdx = (this.delayHead + 64 - this.delaySamples) % 64;
      const delayed = this.delayBuf[delayIdx];
      this.delayBuf[this.delayHead] = this.modelPressure;
      this.delayHead = (this.delayHead + 1) % 64;

      const compensated = pressure - delayed + this.modelPressure;
      return this.pid.update(setpoint, compensated);
    }

    return this.pid.update(setpoint, pressure);
  }
}

// ============================================================
// ML PREDICTOR
// ============================================================

class MLPredictor {
  constructor() { this.reset(); }

  reset() {
    this.samples = [];
    this.prediction = null;
  }

  feed(pressure, flow) {
    if (this.samples.length < 300)
      this.samples.push({ pressure, flow });
  }

  isReady() { return this.samples.length >= 300 && !this.prediction; }

  predict() {
    if (this.samples.length < 300) return null;
    const n = this.samples.length;
    let sumP = 0, sumF = 0, pStart = 0, pEnd = 0;
    for (let i = 0; i < n; i++) {
      sumP += this.samples[i].pressure;
      sumF += this.samples[i].flow;
      if (i < 10) pStart += this.samples[i].pressure;
      if (i >= n - 10) pEnd += this.samples[i].pressure;
    }
    const meanP = sumP / n, meanF = Math.max(0.01, sumF / n);
    pStart /= 10; pEnd /= 10;

    this.prediction = {
      rInitial: meanP / meanF,
      alpha: (pEnd - pStart) / (meanP + 0.01) * 0.1,
    };
    return this.prediction;
  }
}

// ============================================================
// SIMULATION ENGINE
// ============================================================

const DT = 0.001;
const PLOT_HISTORY = 4000;

const state = {
  running: false,
  shotActive: false,
  shotTime: 0,
  totalVolume: 0,
  machineState: 'idle',
  plant: new PlantModel(),
  kf: new KalmanFilter(DT),
  pid: new PIDController(1.2, 0.6, 0.08, DT),
  adaptive: null,
  ml: new MLPredictor(),
  pumpDuty: 0,
  plotTime: [], plotPressure: [], plotSetpoint: [],
  plotFlow: [], plotResistance: [], plotTrueResistance: [],
  plotDuty: [], plotKfResistance: [],
};

state.adaptive = new AdaptiveController(state.kf, DT);

function getParams() {
  return {
    r0: safeSliderValue('r0', 40, 200, 100) / 10,
    alpha: safeSliderValue('alpha', 0, 100, 20) / 1000,
    beta: safeSliderValue('beta', 0, 100, 30) / 1000,
    delaySamples: Math.round(safeSliderValue('delay', 1, 30, 10) / 10 / DT),
    sensorNoise: safeSliderValue('noise', 0, 50, 15) / 100,
    preinfusePressure: safeSliderValue('preinf', 10, 50, 30) / 10,
    extractPressure: safeSliderValue('extract', 60, 120, 90) / 10,
    preinfuseTime: safeSliderValue('pi-time', 10, 100, 50) / 10,
  };
}

function getSetpoint(params) {
  if (!state.shotActive) return 0;
  if (state.shotTime < params.preinfuseTime)
    return params.preinfusePressure;
  return params.extractPressure;
}

function simStep() {
  const params = getParams();
  const mode = getId('ctrl-mode').value;
  const setpoint = getSetpoint(params);

  const sensors = state.plant.step(state.pumpDuty, DT, params);

  if (state.shotActive) {
    state.shotTime += DT;
    state.totalVolume += sensors.trueFlow * DT;

    if (state.shotTime < params.preinfuseTime) {
      state.machineState = 'preinfuse';
    } else {
      state.machineState = 'extract';
    }
  }

  state.kf.predict(state.pumpDuty);
  state.kf.update(sensors.pressure, sensors.flow);

  if (mode === 'adaptive_ml' && state.shotActive) {
    state.ml.feed(sensors.pressure, sensors.flow);
    if (state.ml.isReady()) {
      const pred = state.ml.predict();
      if (pred) state.adaptive.setMLResistance(pred.rInitial);
    }
  }

  let duty = 0;
  if (state.shotActive) {
    switch (mode) {
      case 'pid':
        state.pid.setGains(
          safeSliderValue('kp', 0, 50, 12) / 10,
          safeSliderValue('ki', 0, 30, 6) / 10,
          safeSliderValue('kd', 0, 20, 8) / 100
        );
        duty = state.pid.update(setpoint, sensors.pressure);
        break;
      case 'adaptive':
      case 'adaptive_ml':
        state.adaptive.setDelay(safeSliderValue('delay', 1, 30, 10) / 10);
        duty = state.adaptive.update(setpoint, sensors.pressure, sensors.flow, state.pumpDuty);
        break;
    }
  }

  state.pumpDuty = duty;
  return { sensors, setpoint, duty };
}

// ============================================================
// PLOTTING
// ============================================================

class Plot {
  constructor(canvasId, yMin, yMax) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    this.yMin = yMin;
    this.yMax = yMax;
  }

  resize() {
    const rect = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = rect.width;
    this.h = rect.height;
  }

  draw(datasets) {
    const ctx = this.ctx;
    const w = this.w, h = this.h;
    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = '#1e2030';
    ctx.lineWidth = 1;
    const ySteps = 5;
    for (let i = 0; i <= ySteps; i++) {
      const y = h * i / ySteps;
      ctx.beginPath(); ctx.moveTo(40, y); ctx.lineTo(w, y); ctx.stroke();
      ctx.fillStyle = '#555';
      ctx.font = '10px Consolas, monospace';
      ctx.textAlign = 'right';
      const val = this.yMax - (this.yMax - this.yMin) * i / ySteps;
      ctx.fillText(val.toFixed(1), 36, y + 4);
    }

    for (const ds of datasets) {
      if (!ds.data || ds.data.length < 2) continue;
      const n = ds.data.length;
      ctx.strokeStyle = ds.color;
      ctx.lineWidth = ds.width || 1.5;
      if (ds.dash) ctx.setLineDash(ds.dash);
      else ctx.setLineDash([]);

      ctx.beginPath();
      for (let i = 0; i < n; i++) {
        const x = 40 + (w - 44) * i / (PLOT_HISTORY - 1);
        const val = clamp(ds.data[i], this.yMin, this.yMax);
        const y = h - (val - this.yMin) / (this.yMax - this.yMin) * h;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
}

const plots = {
  pressure: new Plot('plot-pressure', 0, 14),
  flow: new Plot('plot-flow', 0, 4),
  resistance: new Plot('plot-resistance', 0, 25),
  duty: new Plot('plot-duty', 0, 1),
};

function resizePlots() {
  Object.values(plots).forEach(p => p.resize());
}
window.addEventListener('resize', resizePlots);

// ============================================================
// RENDER LOOP
// ============================================================

let decimator = 0;
const DECIMATE = 10;

function renderFrame() {
  if (!state.running) return;

  const speed = clamp(safeSliderValue('speed', 1, 10, 1), 1, 10);
  const stepsPerFrame = Math.round(1000 / 60 * speed);

  let lastSensors, lastSetpoint, lastDuty;
  for (let i = 0; i < stepsPerFrame; i++) {
    const result = simStep();
    lastSensors = result.sensors;
    lastSetpoint = result.setpoint;
    lastDuty = result.duty;

    decimator++;
    if (decimator >= DECIMATE) {
      decimator = 0;
      pushPlotData(result);
    }
  }

  plots.pressure.draw([
    { data: state.plotPressure, color: '#f59e0b', width: 2 },
    { data: state.plotSetpoint, color: '#ef4444', width: 1.5, dash: [6, 4] },
  ]);
  plots.flow.draw([
    { data: state.plotFlow, color: '#06b6d4', width: 2 },
  ]);
  plots.resistance.draw([
    { data: state.plotTrueResistance, color: '#22c55e', width: 2 },
    { data: state.plotKfResistance, color: '#a855f7', width: 1.5, dash: [4, 3] },
  ]);
  plots.duty.draw([
    { data: state.plotDuty, color: '#f97316', width: 2 },
  ]);

  if (lastSensors) {
    setStatValue('stat-pressure', lastSensors.pressure.toFixed(2), 'bar');
    setStatValue('stat-flow', lastSensors.flow.toFixed(2), 'mL/s');
    setStatValue('stat-resistance', state.kf.getResistance().toFixed(1), 'bar·s/mL');
    setStatValue('stat-time', state.shotTime.toFixed(1), 's');
    setStatValue('stat-duty', (lastDuty * 100).toFixed(0), '%');
    setStatValue('stat-volume', state.totalVolume.toFixed(1), 'mL');

    const kfP = state.kf.getPressure();
    const kfF = state.kf.getFlow();
    const kfR = state.kf.getResistance();
    getId('kf-p').textContent = kfP.toFixed(2);
    getId('kf-f').textContent = kfF.toFixed(2);
    getId('kf-r').textContent = kfR.toFixed(1);
    getId('kf-p-bar').style.width = clamp(kfP / 14 * 100, 0, 100) + '%';
    getId('kf-f-bar').style.width = clamp(kfF / 4 * 100, 0, 100) + '%';
    getId('kf-r-bar').style.width = clamp(kfR / 25 * 100, 0, 100) + '%';

    const badge = getId('state-badge');
    badge.textContent = state.machineState.toUpperCase();
    badge.className = 'state-badge state-' + state.machineState;

    if (state.shotActive) {
      const params = getParams();
      const total = Math.max(state.shotTime, 1);
      const piPct = Math.min(params.preinfuseTime / total * 100, 100);
      const exPct = state.machineState === 'extract' ?
        (state.shotTime - params.preinfuseTime) / total * 100 : 0;
      const segments = getId('phase-bar').children;
      segments[0].style.width = piPct + '%';
      segments[1].style.width = Math.max(0, exPct) + '%';
    }
  }

  requestAnimationFrame(renderFrame);
}

function pushPlotData(result) {
  const { sensors, setpoint, duty } = result;
  state.plotPressure.push(sensors.pressure);
  state.plotSetpoint.push(setpoint);
  state.plotFlow.push(sensors.flow);
  state.plotTrueResistance.push(sensors.trueResistance);
  state.plotKfResistance.push(state.kf.getResistance());
  state.plotDuty.push(duty);

  const max = PLOT_HISTORY;
  if (state.plotPressure.length > max) {
    state.plotPressure.splice(0, state.plotPressure.length - max);
    state.plotSetpoint.splice(0, state.plotSetpoint.length - max);
    state.plotFlow.splice(0, state.plotFlow.length - max);
    state.plotTrueResistance.splice(0, state.plotTrueResistance.length - max);
    state.plotKfResistance.splice(0, state.plotKfResistance.length - max);
    state.plotDuty.splice(0, state.plotDuty.length - max);
  }
}

// ============================================================
// CONTROLS
// ============================================================

function startShot() {
  clearPlotData();
  state.plant.reset();
  state.kf.reset();
  state.pid.reset();
  state.adaptive.reset();
  state.ml.reset();
  state.shotTime = 0;
  state.totalVolume = 0;
  state.pumpDuty = 0;
  state.shotActive = true;
  state.machineState = 'preinfuse';

  if (!state.running) {
    state.running = true;
    resizePlots();
    requestAnimationFrame(renderFrame);
  }
}

function stopShot() {
  state.shotActive = false;
  state.machineState = 'idle';
  state.pumpDuty = 0;
}

function resetSim() {
  state.running = false;
  state.shotActive = false;
  state.machineState = 'idle';
  state.plant.reset();
  state.kf.reset();
  state.pid.reset();
  state.adaptive.reset();
  state.ml.reset();
  state.shotTime = 0;
  state.totalVolume = 0;
  state.pumpDuty = 0;
  clearPlotData();
  resizePlots();
  Object.values(plots).forEach(p => p.draw([]));
}

function clearPlotData() {
  state.plotPressure = [];
  state.plotSetpoint = [];
  state.plotFlow = [];
  state.plotTrueResistance = [];
  state.plotKfResistance = [];
  state.plotDuty = [];
}

// ============================================================
// 20-SHOT COMPARISON
// ============================================================

async function runComparison() {
  const btn = document.querySelector('.btn-compare');
  btn.disabled = true;
  btn.textContent = 'Running...';

  const modes = ['pid', 'adaptive', 'adaptive_ml'];
  const modeNames = ['Baseline PID', 'Adaptive', 'Adaptive + ML'];
  const results = {};

  for (let mi = 0; mi < modes.length; mi++) {
    const mode = modes[mi];
    const times = [];

    for (let shot = 0; shot < 20; shot++) {
      const time = runSingleShotSilent(mode);
      times.push(time);
      btn.textContent = modeNames[mi] + ': ' + (shot + 1) + '/20';
      await sleep(5);
    }

    const mean = times.reduce((a, b) => a + b) / times.length;
    const std = Math.sqrt(times.map(t => (t - mean) ** 2).reduce((a, b) => a + b) / times.length);
    results[mode] = { mean, std, times, name: modeNames[mi] };
  }

  const div = getId('compare-results');
  div.style.display = 'block';
  const body = getId('compare-body');
  while (body.firstChild) body.removeChild(body.firstChild);

  const baseline = results.pid;
  for (const mode of modes) {
    const r = results[mode];
    const reduction = mode !== 'pid' ?
      ' (' + ((1 - r.std / baseline.std) * 100).toFixed(0) + '% reduction)' : '';

    const row = document.createElement('div');
    row.className = 'compare-row';

    const nameSpan = document.createElement('span');
    nameSpan.className = 'mode-name';
    nameSpan.textContent = r.name;
    row.appendChild(nameSpan);

    const valueSpan = document.createElement('span');
    valueSpan.className = 'mode-value';
    valueSpan.textContent = r.mean.toFixed(1) + ' ± ' + r.std.toFixed(2) + ' s' + reduction;
    row.appendChild(valueSpan);

    body.appendChild(row);
  }

  btn.disabled = false;
  btn.textContent = 'Run 20-Shot Comparison';
}

function runSingleShotSilent(mode) {
  const plant = new PlantModel();
  const kf = new KalmanFilter(DT);
  const pid = new PIDController(
    safeSliderValue('kp', 0, 50, 12) / 10,
    safeSliderValue('ki', 0, 30, 6) / 10,
    safeSliderValue('kd', 0, 20, 8) / 100,
    DT
  );
  const adaptive = new AdaptiveController(kf, DT);
  const ml = new MLPredictor();
  const params = getParams();

  const shotParams = {
    ...params,
    r0: params.r0 * (0.9 + Math.random() * 0.2),
    alpha: params.alpha * (0.8 + Math.random() * 0.4),
  };

  adaptive.setDelay(safeSliderValue('delay', 1, 30, 10) / 10);

  let pumpDuty = 0;
  let shotTime = 0;
  let volume = 0;
  const TARGET_VOLUME = 36;
  const MAX_TIME = 45;

  let mlDecimator = 0;

  while (shotTime < MAX_TIME && volume < TARGET_VOLUME) {
    const sensors = plant.step(pumpDuty, DT, shotParams);
    kf.predict(pumpDuty);
    kf.update(sensors.pressure, sensors.flow);

    let setpoint;
    if (shotTime < shotParams.preinfuseTime) setpoint = shotParams.preinfusePressure;
    else setpoint = shotParams.extractPressure;

    if (mode === 'adaptive_ml') {
      mlDecimator++;
      if (mlDecimator >= 10) {
        mlDecimator = 0;
        ml.feed(sensors.pressure, sensors.flow);
        if (ml.isReady()) {
          const pred = ml.predict();
          if (pred) adaptive.setMLResistance(pred.rInitial);
        }
      }
    }

    switch (mode) {
      case 'pid':
        pumpDuty = pid.update(setpoint, sensors.pressure);
        break;
      case 'adaptive':
      case 'adaptive_ml':
        pumpDuty = adaptive.update(setpoint, sensors.pressure, sensors.flow, pumpDuty);
        break;
    }

    shotTime += DT;
    volume += sensors.trueFlow * DT;
  }

  return shotTime;
}

// ============================================================
// SLIDER BINDINGS + EVENT LISTENERS
// ============================================================

const sliderBindings = [
  ['kp', 'kp-val', v => (v/10).toFixed(1)],
  ['ki', 'ki-val', v => (v/10).toFixed(1)],
  ['kd', 'kd-val', v => (v/100).toFixed(2)],
  ['r0', 'r0-val', v => (v/10).toFixed(1)],
  ['dose', 'dose-val', v => (v/10).toFixed(1)],
  ['alpha', 'alpha-val', v => (v/1000).toFixed(3)],
  ['beta', 'beta-val', v => (v/1000).toFixed(2)],
  ['delay', 'delay-val', v => (v/10).toFixed(1)],
  ['noise', 'noise-val', v => (v/100).toFixed(2)],
  ['preinf', 'preinf-val', v => (v/10).toFixed(1)],
  ['extract', 'extract-val', v => (v/10).toFixed(1)],
  ['pi-time', 'pi-time-val', v => (v/10).toFixed(1)],
  ['q-res', 'q-res-val', v => (v/100).toFixed(2)],
  ['r-pres', 'r-pres-val', v => (v/100).toFixed(2)],
  ['speed', 'speed-val', v => v + 'x'],
];

sliderBindings.forEach(([sliderId, labelId, fmt]) => {
  const slider = getId(sliderId);
  const label = getId(labelId);
  slider.addEventListener('input', () => {
    label.textContent = fmt(+slider.value);
    if (sliderId === 'q-res') state.kf.setProcessNoise(+slider.value / 100);
    if (sliderId === 'r-pres') state.kf.setMeasurementNoise(+slider.value / 100);
  });
});

getId('btn-start').addEventListener('click', startShot);
getId('btn-stop').addEventListener('click', stopShot);
getId('btn-reset').addEventListener('click', resetSim);
getId('btn-compare').addEventListener('click', runComparison);

// Init
resizePlots();
Object.values(plots).forEach(p => p.draw([]));

})();
