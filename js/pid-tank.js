// ============================================
// PID Tank Level Control - Real-time Simulation
// ============================================

class PIDTankSimulator {
    constructor() {
        this.params = {
            kp: 2.0, ki: 0.5, kd: 0.0,
            setpoint: 5.0, area: 2.0, resistance: 1.0, disturbance: 0.0
        };
        this.speed = 1; // 1x, 10x, 50x, 100x
        this.running = false;
        this.simulationTime = 0;
        this.dt = 0.01; // base timestep
        this.maxTime = 30;

        // State
        this.h = 0;
        this.prevH = 0;
        this.integral = 0;
        this.prevError = this.params.setpoint;
        this.prevControl = 0;
        this.lastStepTime = 0; // time of last setpoint change (for per-step metrics)

        // PID controller (2-DOF + anti-reset windup) from js/pid-controller.js
        this.pid = new PIDController({
            kp: this.params.kp, ki: this.params.ki, kd: this.params.kd,
            bias: 0, min: 0, max: 20, iMin: -100, iMax: 100,
            beta: 1, gamma: 0, antiWindup: true
        });
        this.noiseStd = 0; // PV sensor noise std-dev (m)

        // Data buffers (rolling window)
        this.bufferSize = 3000;
        this.timeData = [];
        this.levelData = [];
        this.spData = [];
        this.controlData = [];
        this.errorData = [];

        // Animation
        this.animationId = null;
        this.lastFrameTime = 0;
        this.accumulator = 0;

        // DOM elements
        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.initTankSVG();
    }

    initDOM() {
        // Sliders
        this.sliders = {
            kp: document.getElementById('kp'),
            ki: document.getElementById('ki'),
            kd: document.getElementById('kd'),
            setpoint: document.getElementById('setpoint'),
            area: document.getElementById('tank-area'),
            resistance: document.getElementById('resistance'),
            disturbance: document.getElementById('disturbance')
        };
        // Controller options (2-DOF + anti-windup + PV noise)
        this.betaSlider = document.getElementById('beta');
        this.gammaSlider = document.getElementById('gamma');
        this.noiseSlider = document.getElementById('noise');
        this.alphaSlider = document.getElementById('alpha');
        this.awCheck = document.getElementById('antiwindup');
        this.values = {
            kp: document.getElementById('kp-val'),
            ki: document.getElementById('ki-val'),
            kd: document.getElementById('kd-val'),
            setpoint: document.getElementById('sp-val'),
            area: document.getElementById('area-val'),
            resistance: document.getElementById('resistance-val'),
            disturbance: document.getElementById('dist-val')
        };

        // Controls
        this.runPauseBtn = document.getElementById('run-pause-btn');
        this.runPauseIcon = document.getElementById('run-pause-icon');
        this.runPauseText = document.getElementById('run-pause-text');
        this.speedBtns = document.querySelectorAll('.speed-btn');
        this.resetBtn = document.getElementById('reset-btn');

        // Metrics
        this.metrics = {
            overshoot: document.getElementById('metric-overshoot'),
            settling: document.getElementById('metric-settling'),
            sse: document.getElementById('metric-sse'),
            currentLevel: document.getElementById('metric-current'),
            currentControl: document.getElementById('metric-control'),
            currentError: document.getElementById('metric-error')
        };
    }

    bindEvents() {
        // Parameter sliders — all changes apply LIVE (no reset), so a
        // setpoint change mid-run acts as a genuine step disturbance.
        Object.keys(this.sliders).forEach(key => {
            this.sliders[key].addEventListener('input', (e) => {
                const val = parseFloat(e.target.value);
                this.params[key] = val;
                this.values[key].textContent = val.toFixed(1);

                // Update SVG area label live
                if (key === 'area') {
                    const areaLabel = document.getElementById('svg-tank-area');
                    if (areaLabel) areaLabel.textContent = val.toFixed(1);
                }

                // A setpoint change is a step input — restart per-step metrics
                if (key === 'setpoint') {
                    this.lastStepTime = this.simulationTime;
                }

                // Refresh visuals immediately (works while paused too)
                this.updateTankSVG(this.h, this.prevControl);
                if (!this.running) this.updatePlots();
            });
        });

        // Controller options: 2-DOF weights, PV noise, anti-reset windup
        const bindOpt = (slider, valId, apply, digits) => {
            if (!slider) return;
            const valEl = document.getElementById(valId);
            const update = () => {
                const v = parseFloat(slider.value);
                apply(v);
                if (valEl) valEl.textContent = v.toFixed(digits);
            };
            slider.addEventListener('input', update);
            update();
        };
        bindOpt(this.betaSlider, 'beta-val', v => { this.pid.beta = v; }, 2);
        bindOpt(this.gammaSlider, 'gamma-val', v => { this.pid.gamma = v; }, 2);
        bindOpt(this.noiseSlider, 'noise-val', v => { this.noiseStd = v; }, 3);
        bindOpt(this.alphaSlider, 'alpha-val', v => { this.pid.alpha = v; }, 2);
        if (this.awCheck) {
            this.awCheck.addEventListener('change', () => { this.pid.antiWindup = this.awCheck.checked; });
            this.pid.antiWindup = this.awCheck.checked;
        }

        // Setpoint step buttons
        this.stepBtns = document.querySelectorAll('.step-btn');
        this.stepBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const delta = parseFloat(btn.dataset.step);
                this.stepSetpoint(delta);
            });
        });

        // Run/Pause
        this.runPauseBtn.addEventListener('click', () => this.toggleRunPause());

        // Speed buttons
        this.speedBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                this.setSpeed(parseInt(btn.dataset.speed));
            });
        });

        // Reset
        this.resetBtn.addEventListener('click', () => this.reset());

        // Presets
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', () => this.setPreset(btn.dataset.preset));
        });
    }

    setPreset(preset) {
        const presets = {
            'p-only': { kp: 2, ki: 0, kd: 0 },
            'pi': { kp: 2, ki: 0.5, kd: 0 },
            'pid': { kp: 2, ki: 0.5, kd: 1 },
            'aggressive': { kp: 8, ki: 2, kd: 1 },
            'conservative': { kp: 0.5, ki: 0.1, kd: 0 }
        };
        const p = presets[preset];
        if (!p) return;

        this.params.kp = p.kp; this.params.ki = p.ki; this.params.kd = p.kd;
        this.sliders.kp.value = p.kp; this.values.kp.textContent = p.kp.toFixed(1);
        this.sliders.ki.value = p.ki; this.values.ki.textContent = p.ki.toFixed(1);
        this.sliders.kd.value = p.kd; this.values.kd.textContent = p.kd.toFixed(1);

        document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
        document.querySelector(`[data-preset="${preset}"]`).classList.add('active');
    }

    setSpeed(speed) {
        this.speed = speed;
        this.speedBtns.forEach(btn => {
            btn.classList.toggle('active', parseInt(btn.dataset.speed) === speed);
        });
    }

    // Apply a step change to the setpoint while the simulation keeps running.
    // The controller then drives the tank to the new level, producing a real
    // transient (rise / overshoot / settling) in the response.
    stepSetpoint(delta) {
        let newSp = this.params.setpoint + delta;
        newSp = Math.max(0, Math.min(10, newSp));
        this.params.setpoint = parseFloat(newSp.toFixed(1));

        // Mark the step time so performance metrics restart from here
        this.lastStepTime = this.simulationTime;

        // Reflect the new value on the slider + label
        this.sliders.setpoint.value = this.params.setpoint;
        this.values.setpoint.textContent = this.params.setpoint.toFixed(1);

        // Immediate visual feedback even if paused
        this.updateTankSVG(this.h, this.prevControl);
        if (!this.running) this.updatePlots();
    }

    toggleRunPause() {
        this.running = !this.running;
        this.runPauseIcon.className = this.running ? 'fas fa-pause' : 'fas fa-play';
        this.runPauseText.textContent = this.running ? 'Pause' : 'Run';
        this.runPauseBtn.classList.toggle('running', this.running);

        if (this.running) {
            this.lastFrameTime = performance.now();
            this.animate();
        } else if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
    }

    reset() {
        this.running = false;
        if (this.animationId) cancelAnimationFrame(this.animationId);
        this.runPauseIcon.className = 'fas fa-play';
        this.runPauseText.textContent = 'Run';
        this.runPauseBtn.classList.remove('running');

        this.simulationTime = 0;
        this.h = 0;
        this.prevH = 0;
        this.integral = 0;
        this.prevError = this.params.setpoint;
        this.prevControl = 0;
        this.lastStepTime = 0;
        this.pid.reset();
        this.timeData = [];
        this.levelData = [];
        this.spData = [];
        this.controlData = [];
        this.errorData = [];

        this.updatePlots();
        this.updateTankSVG(0, 0);
        this.updateMetrics(true);
    }

    animate() {
        if (!this.running) return;

        const now = performance.now();
        const delta = (now - this.lastFrameTime) / 1000; // seconds
        this.lastFrameTime = now;

        // Run simulation steps based on speed (cap to avoid freezing)
        const stepsThisFrame = Math.min(
            Math.max(1, Math.floor(delta * this.speed / this.dt)),
            500
        );

        for (let i = 0; i < stepsThisFrame; i++) {
            this.stepSimulation();
        }

        this.updatePlots();
        this.updateTankSVG(this.h, this.prevControl);
        this.updateMetrics();

        this.animationId = requestAnimationFrame(() => this.animate());
    }

    stepSimulation() {
        // Controller gains mirror the live parameters.
        this.pid.kp = this.params.kp;
        this.pid.ki = this.params.ki;
        this.pid.kd = this.params.kd;

        // Measured level (optional additive Gaussian sensor noise).
        const pv = this.noiseStd > 0 ? this.h + this.noiseStd * gaussianNoise() : this.h;

        const u = this.pid.update(this.params.setpoint, pv, this.dt);
        const e = this.params.setpoint - this.h;
        this.prevError = e;

        // Tank dynamics: A * dh/dt = u - h/R + disturbance
        const dhdt = (u - this.h / this.params.resistance + this.params.disturbance) / this.params.area;
        this.h += dhdt * this.dt;
        this.h = Math.max(0, this.h);

        this.simulationTime += this.dt;

        // Store data (every 5th step for performance)
        if (this.timeData.length === 0 || this.simulationTime - this.timeData[this.timeData.length - 1] >= 0.05) {
            this.timeData.push(this.simulationTime);
            this.levelData.push(this.h);
            this.spData.push(this.params.setpoint);
            this.controlData.push(u);
            this.errorData.push(e);

            // Rolling buffer
            if (this.timeData.length > this.bufferSize) {
                this.timeData.shift();
                this.levelData.shift();
                this.spData.shift();
                this.controlData.shift();
                this.errorData.shift();
            }
        }

        this.prevControl = u;
    }

    initPlots() {
        const layout = {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 50, r: 20, t: 10, b: 40 },
            xaxis: { title: 'Time (s)', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.01, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified',
            uirevision: 'true'
        };

        // Tank Level Plot. The setpoint is drawn last (on top) as a dashed
        // staircase so every setpoint move stays visible on the figure.
        Plotly.newPlot('chart-tank', [
            { x: [], y: [], name: 'Tank Level', line: { color: '#06b6d4', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(6,182,212,0.1)' },
            { x: [], y: [], name: 'Setpoint', line: { color: '#fbbf24', width: 2.5, dash: 'dash' } }
        ], { ...layout, yaxis: { ...layout.yaxis, title: 'Level (m)' } }, { responsive: true, displayModeBar: false });

        // Control Signal Plot
        Plotly.newPlot('chart-control', [
            { x: [], y: [], name: 'Inflow (q_in)', line: { color: '#a78bfa', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(167,139,250,0.1)' },
            { x: [], y: [], name: 'Outflow (q_out)', line: { color: '#f87171', width: 2, dash: 'dot' } }
        ], { ...layout, yaxis: { ...layout.yaxis, title: 'Flow (m³/s)' } }, { responsive: true, displayModeBar: false });

        // Error Plot
        Plotly.newPlot('chart-error', [
            { x: [], y: [], name: 'Error', line: { color: '#f87171', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(248,113,113,0.1)' }
        ], { ...layout, yaxis: { ...layout.yaxis, title: 'Error (m)' } }, { responsive: true, displayModeBar: false });
    }

    updatePlots() {
        const outflowData = this.levelData.map(h => h / this.params.resistance);

        // Recorded setpoint history -> a dashed staircase that moves with every
        // setpoint change. A pending point is appended when the setpoint has
        // just changed but the next sample has not been stored yet (e.g. paused),
        // so the move is visible immediately.
        let spX = this.timeData;
        let spY = this.spData;
        const lastSP = this.spData.length ? this.spData[this.spData.length - 1] : null;
        if (this.timeData.length && lastSP !== this.params.setpoint) {
            spX = this.timeData.concat([this.simulationTime]);
            spY = this.spData.concat([this.params.setpoint]);
        }

        // Rolling time window (keep last 20 seconds visible)
        const windowSpan = 20;
        const tEnd = Math.max(
            this.timeData.length ? this.timeData[this.timeData.length - 1] : 0,
            this.simulationTime
        ) || windowSpan;
        const tStart = Math.max(0, tEnd - windowSpan);

        // Vertical marker at the last setpoint step (if it is in view)
        const shapes = [];
        if (this.lastStepTime > tStart && this.lastStepTime <= tEnd) {
            shapes.push({
                type: 'line',
                x0: this.lastStepTime, x1: this.lastStepTime,
                yref: 'paper', y0: 0, y1: 1,
                line: { color: '#fbbf24', width: 1.5, dash: 'dot' }
            });
        }

        const layoutUpdate = {
            xaxis: { range: [tStart, Math.max(tEnd, windowSpan)] },
            shapes: shapes
        };

        // Update every chart: PV on top, its MV directly below.
        Plotly.update('chart-tank', {
            x: [this.timeData, spX],
            y: [this.levelData, spY]
        }, layoutUpdate);
        Plotly.update('chart-control', {
            x: [this.timeData, this.timeData],
            y: [this.controlData, outflowData]
        }, layoutUpdate);
        Plotly.update('chart-error', {
            x: [this.timeData],
            y: [this.errorData]
        }, layoutUpdate);
    }

    initTankSVG() {
        this.tankSVG = document.getElementById('tank-svg');
        this.waterRect = document.getElementById('water-level');
        this.inflowArrow = document.getElementById('inflow-arrow');
        this.outflowArrow = document.getElementById('outflow-arrow');
        this.setpointLine = document.getElementById('setpoint-line');
        this.setpointLabel = document.getElementById('setpoint-label');
        this.valveOpening = document.getElementById('valve-opening');
        this.waterWave = document.getElementById('water-wave');
    }

    updateTankSVG(level, control) {
        if (!this.tankSVG) return;

        // Tank dimensions in SVG coords (must match pid-tank.html inner tank rect)
        const tankBottom = 357;
        const tankTop = 58;
        const tankHeight = tankBottom - tankTop;
        const maxLevel = 10; // meters

        // Water level
        const waterHeight = Math.min(level / maxLevel, 1) * tankHeight;
        const waterY = tankBottom - waterHeight;
        if (this.waterRect) {
            this.waterRect.setAttribute('y', waterY);
            this.waterRect.setAttribute('height', waterHeight);
        }

        // Move wave ellipse with the surface
        if (this.waterWave) {
            this.waterWave.setAttribute('cy', waterY);
            this.waterWave.style.opacity = level > 0.05 ? '1' : '0';
        }

        // Setpoint line
        const spY = tankBottom - (this.params.setpoint / maxLevel) * tankHeight;
        if (this.setpointLine) {
            this.setpointLine.setAttribute('y1', spY);
            this.setpointLine.setAttribute('y2', spY);
        }
        // Keep the "SP" label riding on the dashed line.
        if (this.setpointLabel) {
            this.setpointLabel.setAttribute('y', spY + 3);
        }

        // Inflow arrow animation (size based on control)
        if (this.inflowArrow) {
            const arrowScale = Math.min(control / 10, 1.5);
            this.inflowArrow.style.opacity = control > 0.1 ? '1' : '0.3';
            this.inflowArrow.style.transform = `scaleY(${0.5 + arrowScale})`;
        }

        // Outflow arrow (based on level)
        if (this.outflowArrow) {
            const outFlow = level / this.params.resistance;
            const arrowScale = Math.min(outFlow / 5, 1.5);
            this.outflowArrow.style.opacity = outFlow > 0.1 ? '1' : '0.3';
            this.outflowArrow.style.transform = `scaleY(${0.5 + arrowScale})`;
        }

        // Valve opening visualization (valve body spans y=72..104)
        if (this.valveOpening) {
            const valveOpen = Math.min(control / 10, 1);
            const h = 26 * valveOpen;
            this.valveOpening.setAttribute('height', h);
            this.valveOpening.setAttribute('y', 100 - h);
        }
    }

    updateMetrics(reset = false) {
        if (reset) {
            this.metrics.currentLevel.textContent = '0.00 m';
            this.metrics.currentControl.textContent = '0.00';
            this.metrics.currentError.textContent = '0.00 m';
            this.metrics.overshoot.textContent = '0%';
            this.metrics.settling.textContent = '— s';
            this.metrics.sse.textContent = '—';
            return;
        }

        this.metrics.currentLevel.textContent = this.h.toFixed(2) + ' m';
        this.metrics.currentControl.textContent = this.prevControl.toFixed(2);
        this.metrics.currentError.textContent = (this.params.setpoint - this.h).toFixed(2) + ' m';

        // Consider only data since the last setpoint step, so metrics describe
        // the transient of the most recent step.
        let startIdx = 0;
        for (let i = 0; i < this.timeData.length; i++) {
            if (this.timeData[i] >= this.lastStepTime) { startIdx = i; break; }
        }
        const lv = this.levelData.slice(startIdx);
        const tm = this.timeData.slice(startIdx);

        if (lv.length > 10) {
            const sp = this.params.setpoint;
            const peak = Math.max(...lv);
            const overshoot = sp > 0 ? Math.max(0, ((peak - sp) / sp) * 100) : 0;
            this.metrics.overshoot.textContent = overshoot.toFixed(1) + '%';

            // Settling time (2% band) since the step
            let settling = tm[tm.length - 1];
            for (let i = lv.length - 1; i >= 0; i--) {
                if (Math.abs(lv[i] - sp) > 0.02 * sp && sp > 0) {
                    settling = tm[Math.min(i + 1, tm.length - 1)];
                    break;
                }
            }
            const settlingFromStep = Math.max(0, settling - this.lastStepTime);
            this.metrics.settling.textContent = settlingFromStep.toFixed(1) + ' s';

            // SSE (current steady-state error)
            const finalLevel = lv[lv.length - 1];
            const sse = Math.abs(sp - finalLevel);
            this.metrics.sse.textContent = sse.toFixed(3) + ' m';
        }
    }
}

// Initialize when DOM ready
document.addEventListener('DOMContentLoaded', () => {
    window.pidTank = new PIDTankSimulator();
});