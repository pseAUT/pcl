// ============================================
// CSTR Closed-Loop Control - Two Decentralized PIDs
//   Temperature loop      -> manipulates coolant temperature Tc
//   Concentration loop    -> manipulates feed flow rate q
// Real-time simulation (like the PID tank example).
// PIDController (2-DOF + anti-windup) is provided by js/pid-controller.js
// ============================================

class CSTRControlSimulator {
    constructor() {
        this.params = {
            cain: 1.0, vol: 100, ti: 350, ua: 50000,
            k0: 7.2e10, eoverr: 8750, dhr: -50000, rho: 1000, cp: 0.239,
            ca0: 0.87725, t0: 324.475
        };
        this.tempSP = 350;
        this.concSP = 0.5;
        this.tcBias = 300;
        this.qBias = 100;
        this.tcMin = 250; this.tcMax = 400;
        this.qMin = 10; this.qMax = 300;

        this.dt = 0.002;              // plant integration step (RK4)
        this.speed = 5;
        this.maxStepsPerFrame = 1200;
        this.running = false;
        this.simTime = 0;

        this.CA = this.params.ca0;
        this.T = this.params.t0;
        this.Tc = this.tcBias;
        this.q = this.qBias;

        this.noiseT = 0;     // std-dev of temperature sensor noise (K)
        this.noiseCA = 0;    // std-dev of concentration sensor noise (mol/L)

        this.bufferSize = 4000;
        this.timeData = [];
        this.tData = [];
        this.caData = [];
        this.tcData = [];
        this.qData = [];

        this.animationId = null;
        this.lastFrame = 0;

        this.tempController = new PIDController({
            kp: 3, ki: 1, kd: 0.3, bias: this.tcBias,
            min: this.tcMin, max: this.tcMax, iMin: -40, iMax: 40,
            beta: 1, gamma: 0, antiWindup: true
        });
        this.concController = new PIDController({
            kp: 30, ki: 8, kd: 1.5, bias: this.qBias,
            min: this.qMin, max: this.qMax, iMin: -80, iMax: 80,
            beta: 1, gamma: 0, antiWindup: true
        });

        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.seedTraces();
        this.reset();
    }

    /* ---------------------------------------------------------------- */
    initDOM() {
        this.runPauseBtn = document.getElementById('run-pause-btn');
        this.runPauseIcon = document.getElementById('run-pause-icon');
        this.runPauseText = document.getElementById('run-pause-text');
        this.resetBtn = document.getElementById('reset-btn');
        this.speedBtns = document.querySelectorAll('.speed-btn');
        this.chartTabs = document.querySelectorAll('.chart-tab');
        this.currentChart = 'temp';

        this.liquid = document.getElementById('cstrc-liquid');
        this.agitator = document.getElementById('cstrc-agitator');
        this.caReadout = document.getElementById('cstrc-ca-readout');
        this.tReadout = document.getElementById('cstrc-t-readout');
        this.feedValve = document.getElementById('cstrc-feed-valve');
        this.coolantValve = document.getElementById('cstrc-coolant-valve');
        this.tcLabel = document.getElementById('cstrc-tc-label');
        this.qLabel = document.getElementById('cstrc-q-label');
        this.feedFlow = document.getElementById('cstrc-feed-flow');
        this.coolantFlow = document.getElementById('cstrc-coolant-flow');
        this.productFlow = document.getElementById('cstrc-product-flow');
        this.coolantOutFlow = document.getElementById('cstrc-coolout-flow');

        this.readouts = {
            time: document.getElementById('roc-time'),
            t: document.getElementById('roc-t'),
            ca: document.getElementById('roc-ca'),
            tc: document.getElementById('roc-tc'),
            q: document.getElementById('roc-q'),
            et: document.getElementById('roc-et'),
            eca: document.getElementById('roc-eca')
        };
        this.metrics = {
            t: document.getElementById('metric-t'),
            ca: document.getElementById('metric-ca'),
            tc: document.getElementById('metric-tc'),
            q: document.getElementById('metric-q'),
            et: document.getElementById('metric-et'),
            eca: document.getElementById('metric-eca')
        };
    }

    formatExp(x) {
        if (x === 0) return '0';
        const e = Math.floor(Math.log10(Math.abs(x)));
        const m = x / Math.pow(10, e);
        return m.toFixed(1) + 'e' + e;
    }

    // Generic slider binding. `resetOnChange` restarts the run (for ICs).
    bind(id, apply, fmt, resetOnChange) {
        const el = document.getElementById(id);
        const valEl = document.getElementById(id + '-val');
        if (!el) return;
        const update = () => {
            const v = parseFloat(el.value);
            apply(v);
            if (valEl) valEl.textContent = fmt ? fmt(v) : String(v);
        };
        el.addEventListener('input', () => {
            update();
            if (resetOnChange) this.reset();
            else if (!this.running) { this.updateCharts(); this.updateMetrics(); }
        });
        update();
    }

    bindCheckbox(id, apply) {
        const el = document.getElementById(id);
        if (!el) return;
        el.addEventListener('change', () => apply(el.checked));
        apply(el.checked);
    }

    bindEvents() {
        // Temperature controller
        this.bind('t-kp', v => { this.tempController.kp = v; }, v => v.toFixed(2));
        this.bind('t-ki', v => { this.tempController.ki = v; }, v => v.toFixed(2));
        this.bind('t-kd', v => { this.tempController.kd = v; }, v => v.toFixed(2));
        this.bind('t-sp', v => { this.tempSP = v; }, v => v.toFixed(1));
        this.bind('tc-bias', v => { this.tcBias = v; this.tempController.bias = v; }, v => v.toFixed(0));
        this.bind('t-beta', v => { this.tempController.beta = v; }, v => v.toFixed(2));
        this.bind('t-gamma', v => { this.tempController.gamma = v; }, v => v.toFixed(2));
        this.bind('t-alpha', v => { this.tempController.alpha = v; }, v => v.toFixed(2));
        this.bind('noise-t', v => { this.noiseT = v; }, v => v.toFixed(3));
        this.bindCheckbox('t-aw', v => { this.tempController.antiWindup = v; });

        // Concentration controller
        this.bind('c-kp', v => { this.concController.kp = v; }, v => v.toFixed(1));
        this.bind('c-ki', v => { this.concController.ki = v; }, v => v.toFixed(1));
        this.bind('c-kd', v => { this.concController.kd = v; }, v => v.toFixed(2));
        this.bind('c-sp', v => { this.concSP = v; }, v => v.toFixed(2));
        this.bind('q-bias', v => { this.qBias = v; this.concController.bias = v; }, v => v.toFixed(0));
        this.bind('c-beta', v => { this.concController.beta = v; }, v => v.toFixed(2));
        this.bind('c-gamma', v => { this.concController.gamma = v; }, v => v.toFixed(2));
        this.bind('c-alpha', v => { this.concController.alpha = v; }, v => v.toFixed(2));
        this.bind('noise-c', v => { this.noiseCA = v; }, v => v.toFixed(3));
        this.bindCheckbox('c-aw', v => { this.concController.antiWindup = v; });

        // Disturbances
        this.bind('cain', v => { this.params.cain = v; }, v => v.toFixed(2));
        this.bind('ti', v => { this.params.ti = v; }, v => v.toFixed(0));

        // Plant
        this.bind('vol', v => { this.params.vol = v; }, v => v.toFixed(0));
        this.bind('ua', v => { this.params.ua = v; }, v => (v / 1000).toFixed(1) + 'k');
        this.bind('k0', v => { this.params.k0 = Math.pow(10, v); }, v => this.formatExp(Math.pow(10, v)));
        this.bind('eoverr', v => { this.params.eoverr = v; }, v => v.toFixed(0));
        this.bind('dhr', v => { this.params.dhr = v; }, v => this.formatExp(v));
        this.bind('rho', v => { this.params.rho = v; }, v => v.toFixed(0));
        this.bind('cp', v => { this.params.cp = v; }, v => v.toFixed(3));

        // Initial conditions (restart the run)
        this.bind('ca0', v => { this.params.ca0 = v; }, v => v.toFixed(5), true);
        this.bind('t0', v => { this.params.t0 = v; }, v => v.toFixed(3), true);

        this.runPauseBtn.addEventListener('click', () => this.toggleRunPause());
        this.resetBtn.addEventListener('click', () => this.reset());
        this.speedBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                this.speed = parseInt(btn.dataset.speed);
                this.speedBtns.forEach(b => b.classList.toggle('active', b === btn));
            });
        });

        // One tab per control loop (PV on top, its MV below).
        this.chartTabs.forEach(tab => {
            tab.addEventListener('click', () => this.switchChart(tab.dataset.chart));
        });
    }

    // Show the temperature-loop or concentration-loop pair of charts.
    switchChart(chart) {
        this.currentChart = chart;
        this.chartTabs.forEach(t => t.classList.toggle('active', t.dataset.chart === chart));
        const tempTab = document.getElementById('tab-temp');
        const concTab = document.getElementById('tab-conc');
        if (tempTab) tempTab.style.display = chart === 'temp' ? 'block' : 'none';
        if (concTab) concTab.style.display = chart === 'conc' ? 'block' : 'none';
        // Hidden plots are created with zero size; resize once they are shown.
        ['chart-temp', 'chart-tc', 'chart-conc', 'chart-q'].forEach(id => {
            const el = document.getElementById(id);
            if (el && el.offsetParent !== null) {
                try { Plotly.Plots.resize(el); } catch (e) { /* not ready yet */ }
            }
        });
    }

    /* ---------------------------------------------------------------- */
    /* Plant model                                                      */
    /* ---------------------------------------------------------------- */
    derivs(CA, T, Tc, q) {
        const p = this.params;
        const Ts = Math.max(T, 1e-6);
        const k = p.k0 * Math.exp(-p.eoverr / Ts);
        const dCA = q / p.vol * (p.cain - CA) - k * CA;
        const dT = q / p.vol * (p.ti - Ts)
                 - p.dhr * k * CA / (p.rho * p.cp)
                 + p.ua * (Tc - Ts) / (p.rho * p.cp * p.vol);
        return [dCA, dT];
    }

    rk4(CA, T, Tc, q, h) {
        const a = this.derivs(CA, T, Tc, q);
        const b = this.derivs(CA + h / 2 * a[0], T + h / 2 * a[1], Tc, q);
        const c = this.derivs(CA + h / 2 * b[0], T + h / 2 * b[1], Tc, q);
        const d = this.derivs(CA + h * c[0], T + h * c[1], Tc, q);
        return [
            CA + h / 6 * (a[0] + 2 * b[0] + 2 * c[0] + d[0]),
            T + h / 6 * (a[1] + 2 * b[1] + 2 * c[1] + d[1])
        ];
    }

    stepSimulation() {
        // Sensor measurements (optional additive Gaussian noise).
        const Tmeas = this.noiseT > 0 ? this.T + this.noiseT * gaussianNoise() : this.T;
        const CAmeas = this.noiseCA > 0 ? this.CA + this.noiseCA * gaussianNoise() : this.CA;

        // Controllers sample the (noisy) measurements (zero-order hold).
        this.Tc = this.tempController.update(this.tempSP, Tmeas, this.dt);
        this.q = this.concController.update(this.concSP, CAmeas, this.dt);

        const [CA, T] = this.rk4(this.CA, this.T, this.Tc, this.q, this.dt);
        this.CA = CA;
        this.T = T;
        this.simTime += this.dt;

        if (this.timeData.length === 0 || this.simTime - this.timeData[this.timeData.length - 1] >= 0.05) {
            this.timeData.push(this.simTime);
            this.tData.push(this.T);
            this.caData.push(this.CA);
            this.tcData.push(this.Tc);
            this.qData.push(this.q);
            if (this.timeData.length > this.bufferSize) {
                this.timeData.shift(); this.tData.shift(); this.caData.shift();
                this.tcData.shift(); this.qData.shift();
            }
        }
    }

    /* ---------------------------------------------------------------- */
    /* Run / reset                                                      */
    /* ---------------------------------------------------------------- */
    reset() {
        this.running = false;
        if (this.animationId) cancelAnimationFrame(this.animationId);
        this.animationId = null;
        this.setRunButton(false);

        this.simTime = 0;
        this.CA = this.params.ca0;
        this.T = this.params.t0;
        this.tempController.reset();
        this.concController.reset();
        this.Tc = this.tempController.bias;
        this.q = this.concController.bias;
        this.timeData = []; this.tData = []; this.caData = []; this.tcData = []; this.qData = [];

        this.updateCharts();
        this.updateSchematic();
        this.updateMetrics();
    }

    setRunButton(running) {
        this.runPauseIcon.className = running ? 'fas fa-pause' : 'fas fa-play';
        this.runPauseText.textContent = running ? 'Pause' : 'Run';
        this.runPauseBtn.classList.toggle('running', running);
    }

    toggleRunPause() {
        this.running = !this.running;
        this.setRunButton(this.running);
        if (this.running) {
            this.lastFrame = performance.now();
            this.animate();
        } else if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
    }

    animate() {
        if (!this.running) return;
        const now = performance.now();
        const delta = (now - this.lastFrame) / 1000;
        this.lastFrame = now;

        let steps = Math.floor(delta * this.speed / this.dt);
        steps = Math.min(Math.max(1, steps), this.maxStepsPerFrame);
        for (let i = 0; i < steps; i++) this.stepSimulation();

        this.updateCharts();
        this.updateSchematic();
        this.updateMetrics();

        this.animationId = requestAnimationFrame(() => this.animate());
    }

    /* ---------------------------------------------------------------- */
    /* Charts                                                           */
    /* ---------------------------------------------------------------- */
    baseLayout(xTitle, yTitle) {
        return {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 55, r: 55, t: 10, b: 40 },
            xaxis: { title: xTitle, gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { title: yTitle, gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.01, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified',
            uirevision: 'true'
        };
    }

    initPlots() {
        Plotly.newPlot('chart-temp', [], this.baseLayout('Time (s)', 'Temperature (K)'), { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-tc', [], this.baseLayout('Time (s)', 'Coolant Tc (K)'), { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-conc', [], this.baseLayout('Time (s)', 'Concentration CA (mol/L)'), { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-q', [], this.baseLayout('Time (s)', 'Feed q (m³/s)'), { responsive: true, displayModeBar: false });
    }

    windowRange() {
        const span = 60;
        const tEnd = this.timeData.length ? this.timeData[this.timeData.length - 1] : span;
        const tStart = Math.max(0, tEnd - span);
        return { range: [tStart, Math.max(tEnd, span)] };
    }

    updateCharts() {
        const xr = this.windowRange();
        const spT = this.timeData.map(() => this.tempSP);
        const spC = this.timeData.map(() => this.concSP);

        // Each MV chart sits directly below its PV chart.
        Plotly.update('chart-temp', { x: [this.timeData, this.timeData], y: [this.tData, spT] }, { xaxis: xr });
        Plotly.update('chart-tc', { x: [this.timeData], y: [this.tcData] }, { xaxis: xr });
        Plotly.update('chart-conc', { x: [this.timeData, this.timeData], y: [this.caData, spC] }, { xaxis: xr });
        Plotly.update('chart-q', { x: [this.timeData], y: [this.qData] }, { xaxis: xr });
    }

    // Build the initial traces once, then update() only changes the data.
    seedTraces() {
        const trace = (name, color, dash) => ({ x: [], y: [], name, line: { color, width: dash ? 2 : 3, dash: dash || 'solid' } });
        Plotly.react('chart-temp', [
            trace('Reactor T', '#ef4444'),
            trace('Setpoint T', '#fbbf24', 'dash')
        ], this.baseLayout('Time (s)', 'Temperature (K)'), { responsive: true, displayModeBar: false });
        Plotly.react('chart-tc', [
            trace('Coolant Tc', '#38bdf8')
        ], this.baseLayout('Time (s)', 'Coolant Tc (K)'), { responsive: true, displayModeBar: false });
        Plotly.react('chart-conc', [
            trace('CA', '#0ea5e9'),
            trace('Setpoint CA', '#fbbf24', 'dash')
        ], this.baseLayout('Time (s)', 'Concentration CA (mol/L)'), { responsive: true, displayModeBar: false });
        Plotly.react('chart-q', [
            trace('Feed q', '#a78bfa')
        ], this.baseLayout('Time (s)', 'Feed q (m³/s)'), { responsive: true, displayModeBar: false });
    }

    /* ---------------------------------------------------------------- */
    /* Schematic                                                        */
    /* ---------------------------------------------------------------- */
    updateSchematic() {
        const T = this.T, CA = this.CA, Tc = this.Tc, q = this.q;
        const frac = Math.max(0, Math.min(1, (T - 280) / (520 - 280)));
        const hue = 210 * (1 - frac);
        const color = `hsl(${hue}, 85%, 52%)`;
        if (this.liquid) this.liquid.setAttribute('fill', color);

        if (this.caReadout) this.caReadout.textContent = 'CA = ' + CA.toFixed(3);
        if (this.tReadout) this.tReadout.textContent = 'T = ' + T.toFixed(1) + ' K';

        // Valve openings
        if (this.feedValve) {
            const open = Math.max(0, Math.min(1, (q - this.qMin) / (this.qMax - this.qMin)));
            this.feedValve.setAttribute('opacity', (0.25 + 0.75 * open).toFixed(2));
        }
        if (this.coolantValve) {
            const open = Math.max(0, Math.min(1, (this.tcMax - Tc) / (this.tcMax - this.tcMin)));
            this.coolantValve.setAttribute('opacity', (0.25 + 0.75 * open).toFixed(2));
        }
        if (this.tcLabel) this.tcLabel.textContent = 'Tc = ' + Tc.toFixed(0) + ' K';
        if (this.qLabel) this.qLabel.textContent = 'q = ' + q.toFixed(0);

        // Animate the process streams: dash speed tracks the manipulated
        // variables, so a larger feed flow or colder coolant visibly flows faster.
        const feedDur = Math.max(0.25, Math.min(2.5, 40 / Math.max(q, 1)));
        if (this.feedFlow) this.feedFlow.style.animationDuration = feedDur.toFixed(2) + 's';
        if (this.productFlow) this.productFlow.style.animationDuration = feedDur.toFixed(2) + 's';
        const coolOpen = Math.max(0, Math.min(1, (this.tcMax - Tc) / (this.tcMax - this.tcMin)));
        const coolDur = Math.max(0.3, 1.8 - coolOpen * 1.5);
        if (this.coolantFlow) this.coolantFlow.style.animationDuration = coolDur.toFixed(2) + 's';
        if (this.coolantOutFlow) this.coolantOutFlow.style.animationDuration = coolDur.toFixed(2) + 's';

        // Agitator speed with reaction rate
        const k = this.params.k0 * Math.exp(-this.params.eoverr / Math.max(T, 1e-6));
        const rr = k * CA;
        if (this.agitator) this.agitator.style.animationDuration = (1.8 / (1 + Math.min(rr, 40)) + 0.15).toFixed(2) + 's';

        const eT = this.tempSP - T, eCA = this.concSP - CA;
        if (this.readouts.time) this.readouts.time.textContent = this.simTime.toFixed(1) + ' s';
        if (this.readouts.t) this.readouts.t.textContent = T.toFixed(1) + ' K';
        if (this.readouts.ca) this.readouts.ca.textContent = CA.toFixed(3);
        if (this.readouts.tc) this.readouts.tc.textContent = Tc.toFixed(1) + ' K';
        if (this.readouts.q) this.readouts.q.textContent = q.toFixed(1);
        if (this.readouts.et) this.readouts.et.textContent = (eT >= 0 ? '+' : '') + eT.toFixed(2) + ' K';
        if (this.readouts.eca) this.readouts.eca.textContent = (eCA >= 0 ? '+' : '') + eCA.toFixed(4);
    }

    updateMetrics() {
        const eT = this.tempSP - this.T, eCA = this.concSP - this.CA;
        if (this.metrics.t) this.metrics.t.textContent = this.T.toFixed(1) + ' K';
        if (this.metrics.ca) this.metrics.ca.textContent = this.CA.toFixed(3);
        if (this.metrics.tc) this.metrics.tc.textContent = this.Tc.toFixed(1) + ' K';
        if (this.metrics.q) this.metrics.q.textContent = this.q.toFixed(1);
        if (this.metrics.et) this.metrics.et.textContent = eT.toFixed(2) + ' K';
        if (this.metrics.eca) this.metrics.eca.textContent = eCA.toFixed(4);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.cstrControl = new CSTRControlSimulator();
});
