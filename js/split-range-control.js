// ============================================
// Split-Range Control - Surge Drum Level
// One PID controller drives two opposing valves:
//   output 0-50 %  -> drain valve V1 opens (0 -> 100 %)
//   output 50-100% -> make-up valve V2 opens (0 -> 100 %)
// Both valves are shut exactly at the 50 % split point.
//
// Plant (surge drum mass balance):
//   A dh/dt = q_f + q_in - q_out - q_L
//   q_out = Kv1 (V1/100) sqrt(h)      gravity drain through V1
//   q_in  = Kv2 (V2/100)              pumped make-up through V2
//   q_f   = constant feed
//   q_L   = draw-off load (the disturbance)
//
// The drain and make-up capacities are matched at the setpoint
// (Kv1 sqrt(h_sp)/50 = Kv2/50) so both branches have the same
// flow authority per % of controller output.
// ============================================

class SplitRangeControlSimulator {
    constructor() {
        // ---- Plant parameters -------------------------------------------
        this.params = {
            area: 4.0,      // A  - drum cross-section (m^2)
            kv1: 1.1,       // Kv1 - drain capacity coefficient
            kv2: 2.5,       // Kv2 - make-up capacity coefficient
            qFeed: 1.0,     // q_f - constant feed (m^3/s)
            qLoad: 1.6,     // q_L - draw-off load (m^3/s)
            setpoint: 13.0, // SP  - level setpoint (m)
            hMax: 15.0      // drum height (m)
        };

        // ---- Controller -------------------------------------------------
        this.pid = new PIDController({
            kp: 0.5, ki: 15.0, kd: 0.0,
            bias: 50,          // 50 % = split point = both valves shut
            min: 0, max: 100,
            iMin: -60, iMax: 60,
            beta: 1, gamma: 0, antiWindup: true
        });

        // ---- Simulation state -------------------------------------------
        // Must match the .speed-btn marked active in split-range-control.html.
        this.speed = 10;
        this.running = false;
        this.simulationTime = 0;
        this.dt = 0.01;
        this.tEnd = 600;
        this.splitPoint = 50;   // % of controller output

        this.h = 0;             // drum level (m)
        this.pidOutput = 50;    // controller output (%)
        this.v1 = 0;            // drain valve opening (%)
        this.v2 = 0;            // make-up valve opening (%)
        this.qIn = 0;           // make-up flow (m^3/s)
        this.qOut = 0;          // drain flow (m^3/s)
        this.qOverflow = 0;     // overflow past the rim (m^3/s, open drum)
        this.inDeadBand = true;

        this.noiseStd = 0;      // PV sensor noise std-dev (m)
        this.lastEventTime = 0; // last setpoint/load change (for metrics)

        // ---- Buffers ----------------------------------------------------
        this.bufferSize = 6000;
        this.timeData = [];
        this.levelData = [];
        this.spData = [];
        this.pidData = [];
        this.v1Data = [];
        this.v2Data = [];
        this.qinData = [];
        this.qoutData = [];
        this.qfeedData = [];
        this.qloadData = [];

        this.animationId = null;
        this.lastFrameTime = 0;

        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.initSchematic();
        this.reset();
    }

    // ---------------------------------------------------------------- DOM
    initDOM() {
        const $ = (id) => document.getElementById(id);

        this.sliders = {
            kp: $('kp'), ki: $('ki'), kd: $('kd'),
            setpoint: $('setpoint'),
            area: $('tank-area'),
            kv1: $('kv1'), kv2: $('kv2'),
            qfeed: $('qfeed'), qload: $('qload'),
            noise: $('noise')
        };
        this.values = {
            kp: $('kp-val'), ki: $('ki-val'), kd: $('kd-val'),
            setpoint: $('sp-val'),
            area: $('area-val'),
            kv1: $('kv1-val'), kv2: $('kv2-val'),
            qfeed: $('qfeed-val'), qload: $('qload-val'),
            noise: $('noise-val')
        };
        this.awCheck = $('antiwindup');
        this.tEndInput = $('tend');

        this.runPauseBtn = $('run-pause-btn');
        this.runPauseIcon = $('run-pause-icon');
        this.runPauseText = $('run-pause-text');
        this.resetBtn = $('reset-btn');
        this.speedBtns = document.querySelectorAll('.speed-btn');
        this.stepBtns = document.querySelectorAll('.step-btn');
        this.presetBtns = document.querySelectorAll('.preset-btn');

        this.readouts = {
            h: $('ro-h'), u: $('ro-u'), v1: $('ro-v1'), v2: $('ro-v2'),
            qin: $('ro-qin'), qout: $('ro-qout')
        };
        this.metrics = {
            level: $('metric-level'),
            pid: $('metric-pid'),
            v1: $('metric-v1'),
            v2: $('metric-v2'),
            branch: $('metric-branch'),
            sse: $('metric-sse')
        };
    }

    bindEvents() {
        const bindSlider = (el, out, fmt, apply) => {
            if (!el) return;
            const update = () => {
                const v = parseFloat(el.value);
                if (out) out.textContent = fmt(v);
                apply(v);
            };
            el.addEventListener('input', update);
            update();
        };

        const p = this.params;
        bindSlider(this.sliders.kp, this.values.kp, v => v.toFixed(1), v => { this.pid.kp = v; });
        bindSlider(this.sliders.ki, this.values.ki, v => v.toFixed(3), v => { this.pid.ki = v; });
        bindSlider(this.sliders.kd, this.values.kd, v => v.toFixed(2), v => { this.pid.kd = v; });

        bindSlider(this.sliders.setpoint, this.values.setpoint, v => v.toFixed(1),
            v => { if (v !== p.setpoint) { p.setpoint = v; this.lastEventTime = this.simulationTime; } });
        bindSlider(this.sliders.area, this.values.area, v => v.toFixed(1), v => { p.area = v; });
        bindSlider(this.sliders.kv1, this.values.kv1, v => v.toFixed(2), v => { p.kv1 = v; });
        bindSlider(this.sliders.kv2, this.values.kv2, v => v.toFixed(2), v => { p.kv2 = v; });
        bindSlider(this.sliders.qfeed, this.values.qfeed, v => v.toFixed(2), v => { p.qFeed = v; });
        bindSlider(this.sliders.qload, this.values.qload, v => v.toFixed(2),
            v => { if (v !== p.qLoad) { p.qLoad = v; this.lastEventTime = this.simulationTime; } });
        bindSlider(this.sliders.noise, this.values.noise, v => v.toFixed(3), v => { this.noiseStd = v; });

        if (this.awCheck) {
            this.awCheck.addEventListener('change', () => { this.pid.antiWindup = this.awCheck.checked; });
        }
        // The PID gains are mirrored onto the controller before each step, so a
        // slider move is picked up immediately (see stepSimulation).

        if (this.tEndInput) {
            const apply = () => {
                const v = Math.max(10, Math.min(5000, parseFloat(this.tEndInput.value) || 600));
                this.tEnd = v;
                this.tEndInput.value = v;
                if (this.simulationTime >= this.tEnd) this.stopRun();
            };
            this.tEndInput.addEventListener('change', apply);
            this.tEndInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') apply(); });
        }

        if (this.runPauseBtn) {
            this.runPauseBtn.addEventListener('click', () => this.running ? this.stopRun() : this.startRun());
        }
        if (this.resetBtn) this.resetBtn.addEventListener('click', () => this.reset());

        this.speedBtns.forEach(btn => {
            btn.addEventListener('click', () => this.setSpeed(parseFloat(btn.dataset.speed)));
        });

        // Live setpoint steps
        this.stepBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const step = parseFloat(btn.dataset.step);
                const next = Math.max(1, Math.min(this.params.hMax,
                    +(this.params.setpoint + step).toFixed(2)));
                this.params.setpoint = next;
                if (this.sliders.setpoint) this.sliders.setpoint.value = next;
                if (this.values.setpoint) this.values.setpoint.textContent = next.toFixed(1);
                this.lastEventTime = this.simulationTime;
            });
        });

        // Presets
        const presets = {
            pi: { kp: 12.0, ki: 1.2245, kd: 0.0 },
            sluggish: { kp: 4.0, ki: 0.15, kd: 0.0 },
            aggressive: { kp: 25.0, ki: 3.0, kd: 0.0 },
            pOnly: { kp: 12.0, ki: 0.0, kd: 0.0 }
        };
        this.presetBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const g = presets[btn.dataset.preset];
                if (!g) return;
                this.presetBtns.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.setGains(g);
                this.lastEventTime = this.simulationTime;
            });
        });
    }

    setGains(g) {
        if (this.sliders.kp) this.sliders.kp.value = g.kp;
        if (this.sliders.ki) this.sliders.ki.value = g.ki;
        if (this.sliders.kd) this.sliders.kd.value = g.kd;
        if (this.values.kp) this.values.kp.textContent = g.kp.toFixed(1);
        if (this.values.ki) this.values.ki.textContent = g.ki.toFixed(3);
        if (this.values.kd) this.values.kd.textContent = g.kd.toFixed(2);
        this.pid.kp = g.kp; this.pid.ki = g.ki; this.pid.kd = g.kd;
    }

    setSpeed(speed) {
        this.speed = speed;
        this.speedBtns.forEach(btn => {
            btn.classList.toggle('active', parseFloat(btn.dataset.speed) === speed);
        });
    }

    // ---------------------------------------------------------- lifecycle
    reset() {
        this.stopRun();
        this.simulationTime = 0;
        this.h = 3.0;
        this.pidOutput = 50;
        this.v1 = 0; this.v2 = 0;
        this.qIn = 0; this.qOut = 0; this.qOverflow = 0;
        this.inDeadBand = true;
        this.pid.reset();
        this.lastEventTime = 0;

        this.timeData = []; this.levelData = []; this.spData = [];
        this.pidData = []; this.v1Data = []; this.v2Data = [];
        this.qinData = []; this.qoutData = []; this.qfeedData = []; this.qloadData = [];

        this.updatePlots();
        this.updateSchematic();
        this.updateReadouts();
        this.updateMetrics(true);
    }

    startRun() {
        if (this.simulationTime >= this.tEnd) this.reset();
        this.running = true;
        if (this.runPauseIcon) this.runPauseIcon.className = 'fas fa-pause';
        if (this.runPauseText) this.runPauseText.textContent = 'Pause';
        this.lastFrameTime = performance.now();
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    stopRun() {
        this.running = false;
        if (this.animationId !== null) { cancelAnimationFrame(this.animationId); this.animationId = null; }
        if (this.runPauseIcon) this.runPauseIcon.className = 'fas fa-play';
        if (this.runPauseText) this.runPauseText.textContent = 'Run';
    }

    animate() {
        if (!this.running) return;

        const now = performance.now();
        const delta = (now - this.lastFrameTime) / 1000;
        this.lastFrameTime = now;

        let steps = Math.min(Math.max(1, Math.floor(delta * this.speed / this.dt)), 500);
        const remaining = Math.round((this.tEnd - this.simulationTime) / this.dt);
        if (remaining <= 0) { this.stopRun(); return; }
        steps = Math.min(steps, remaining);

        for (let i = 0; i < steps; i++) this.stepSimulation();

        this.updatePlots();
        this.updateSchematic();
        this.updateReadouts();
        this.updateMetrics();

        if (this.simulationTime >= this.tEnd - 1e-9) { this.stopRun(); return; }
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    // ------------------------------------------------------------- physics
    stepSimulation() {
        const p = this.params;

        // Measured level (optional Gaussian sensor noise on the measurement only).
        const pv = this.noiseStd > 0 ? this.h + this.noiseStd * gaussianNoise() : this.h;

        const u = this.pid.update(p.setpoint, pv, this.dt);
        this.pidOutput = u;

        // ---- Split-range characteristic -------------------------------
        // Complementary split: the drain opens as the output falls below the
        // split point; the make-up opens as it rises above it. At exactly the
        // split point both valves are shut (the split dead band).
        let v1, v2;
        if (u <= this.splitPoint) {
            v1 = (this.splitPoint - u) / this.splitPoint * 100;
            v2 = 0;
        } else {
            v1 = 0;
            v2 = (u - this.splitPoint) / (100 - this.splitPoint) * 100;
        }
        // Ignore a sub-0.05 % stroke: at exactly u = 50 both valves are shut.
        if (v1 < 0.05) v1 = 0;
        if (v2 < 0.05) v2 = 0;
        this.v1 = v1;
        this.v2 = v2;
        this.inDeadBand = (v1 === 0 && v2 === 0);

        // ---- Flows ----------------------------------------------------
        this.qOut = p.kv1 * (v1 / 100) * Math.sqrt(Math.max(this.h, 0));  // gravity drain
        this.qIn = p.kv2 * (v2 / 100);                                    // pumped make-up

        // ---- Drum mass balance ----------------------------------------
        // An open drum cannot store above its rim: when the inflow exceeds
        // what the drum can hold, the level saturates at hMax and the excess
        // leaves as overflow qOv. Keeping the overflow term makes the mass
        // balance close exactly in EVERY state (also while the level is
        // pinned at the brim), matching the physics of a real overflowing
        // open vessel.
        const dhdt = (p.qFeed + this.qIn - this.qOut - p.qLoad) / p.area;
        let hNext = this.h + dhdt * this.dt;
        let qOv = 0;
        if (hNext > p.hMax) {
            qOv = (hNext - p.hMax) * p.area / this.dt;
            hNext = p.hMax;
        }
        if (hNext < 0) hNext = 0;
        this.h = hNext;
        this.qOverflow = qOv;

        this.simulationTime += this.dt;

        // ---- Record ----------------------------------------------------
        if (this.timeData.length === 0 ||
            this.simulationTime - this.timeData[this.timeData.length - 1] >= 0.05) {
            this.timeData.push(this.simulationTime);
            this.levelData.push(this.h);
            this.spData.push(p.setpoint);
            this.pidData.push(this.pidOutput);
            this.v1Data.push(this.v1);
            this.v2Data.push(this.v2);
            this.qinData.push(this.qIn);
            this.qoutData.push(this.qOut);
            this.qfeedData.push(p.qFeed);
            this.qloadData.push(p.qLoad);
            if (this.timeData.length > this.bufferSize) {
                [this.timeData, this.levelData, this.spData, this.pidData,
                 this.v1Data, this.v2Data, this.qinData, this.qoutData,
                 this.qfeedData, this.qloadData].forEach(a => a.shift());
            }
        }
    }

    // -------------------------------------------------------------- plots
    baseLayout(yTitle) {
        return {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { family: 'Inter' },
            margin: { l: 52, r: 18, t: 10, b: 40 },
            xaxis: { title: 'Time (s)' },
            yaxis: { title: yTitle },
            legend: { x: 0.99, y: 0.99 },
            hovermode: 'x unified',
            uirevision: 'true'
        };
    }

    initPlots() {
        // PV: drum level
        Plotly.newPlot('chart-level', [
            { x: [], y: [], name: 'Level h', line: { color: '#06b6d4', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(6,182,212,0.10)' },
            { x: [], y: [], name: 'Setpoint', line: { color: '#fbbf24', width: 2.5, dash: 'dash' } }
        ], this.baseLayout('Level (m)'), { responsive: true, displayModeBar: false });

        // MV: the two valve openings, with the controller output behind them
        Plotly.newPlot('chart-valves', [
            { x: [], y: [], name: 'V₁ drain', line: { color: '#f87171', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(248,113,113,0.10)' },
            { x: [], y: [], name: 'V₂ make-up', line: { color: '#38bdf8', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(56,189,248,0.10)' },
            { x: [], y: [], name: 'PID output u', line: { color: '#94a3b8', width: 1.5, dash: 'dot' } }
        ], this.baseLayout('Valve opening (%)'), { responsive: true, displayModeBar: false });

        // Flows
        Plotly.newPlot('chart-flows', [
            { x: [], y: [], name: 'Make-up qᵢₙ', line: { color: '#38bdf8', width: 2.5 } },
            { x: [], y: [], name: 'Drain qₒᵤₜ', line: { color: '#f87171', width: 2.5 } },
            { x: [], y: [], name: 'Feed q_f', line: { color: '#22c55e', width: 2, dash: 'dash' } },
            { x: [], y: [], name: 'Draw-off q_L', line: { color: '#fbbf24', width: 2, dash: 'dash' } }
        ], this.baseLayout('Flow (m³/s)'), { responsive: true, displayModeBar: false });
    }

    updatePlots() {
        const windowSpan = 60;
        const tEnd = Math.max(
            this.timeData.length ? this.timeData[this.timeData.length - 1] : 0,
            this.simulationTime
        ) || windowSpan;
        const tStart = Math.max(0, tEnd - windowSpan);

        // A pending setpoint point so a step is visible the moment it is made.
        let spX = this.timeData, spY = this.spData;
        const lastSP = this.spData.length ? this.spData[this.spData.length - 1] : null;
        if (this.timeData.length && lastSP !== this.params.setpoint) {
            spX = this.timeData.concat([this.simulationTime]);
            spY = this.spData.concat([this.params.setpoint]);
        }

        const shapes = [];
        if (this.lastEventTime > tStart && this.lastEventTime <= tEnd) {
            shapes.push({
                type: 'line', x0: this.lastEventTime, x1: this.lastEventTime,
                yref: 'paper', y0: 0, y1: 1,
                line: { color: '#fbbf24', width: 1.5, dash: 'dot' }
            });
        }
        // Mark the split point on the valve chart so the handover is obvious.
        const layoutUpdate = {
            xaxis: { range: [tStart, Math.max(tEnd, windowSpan)] },
            shapes: shapes
        };

        Plotly.update('chart-level', { x: [this.timeData, spX], y: [this.levelData, spY] }, layoutUpdate);
        Plotly.update('chart-valves', {
            x: [this.timeData, this.timeData, this.timeData],
            y: [this.v1Data, this.v2Data, this.pidData]
        }, layoutUpdate);
        Plotly.update('chart-flows', {
            x: [this.timeData, this.timeData, this.timeData, this.timeData],
            y: [this.qinData, this.qoutData, this.qfeedData, this.qloadData]
        }, layoutUpdate);
    }

    // ---------------------------------------------------------- schematic
    initSchematic() {
        const $ = (id) => document.getElementById(id);
        this.svg = {
            water: $('water-level'),
            spLine: $('setpoint-line'),
            spLabel: $('setpoint-label'),
            hLabel: $('svg-level-label'),
            v1Stroke: $('v1-stroke'),
            v2Stroke: $('v2-stroke'),
            v1Label: $('v1-open-label'),
            v2Label: $('v2-open-label'),
            barU: $('bar-u-fill'),
            barU2: $('bar-u-needle'),
            barV1: $('bar-v1-fill'),
            barV2: $('bar-v2-fill'),
            branchText: $('svg-branch-label'),
            feedArrow: $('feed-arrow'),
            makeupArrow: $('makeup-arrow'),
            drainArrow: $('drain-arrow'),
            drawArrow: $('draw-arrow')
        };
        this.geom = {
            tankX: 383, tankW: 164,
            yZero: 358,      // y of h = 0
            pxPerM: 16,      // 15 m over 240 px
            barX: 150, barW: 100
        };
    }

    updateSchematic() {
        const g = this.geom;
        const s = this.svg;
        if (!s.water) return;

        const h = Math.max(0, Math.min(this.params.hMax, this.h));
        const wh = h * g.pxPerM;
        s.water.setAttribute('y', (g.yZero - wh).toFixed(1));
        s.water.setAttribute('height', wh.toFixed(1));

        // Setpoint line
        if (s.spLine) {
            const y = g.yZero - this.params.setpoint * g.pxPerM;
            s.spLine.setAttribute('y1', y.toFixed(1));
            s.spLine.setAttribute('y2', y.toFixed(1));
            if (s.spLabel) {
                s.spLabel.setAttribute('y', (y + 3.5).toFixed(1));
                s.spLabel.textContent = 'SP ' + this.params.setpoint.toFixed(1) + ' m';
            }
        }
        if (s.hLabel) {
            const y = g.yZero - wh;
            s.hLabel.setAttribute('y', Math.max(126, Math.min(g.yZero - 6, y - 6)).toFixed(1));
            s.hLabel.textContent = 'h = ' + this.h.toFixed(2) + ' m';
        }

        // Valve stroke indicators inside the valve bodies. Each body has its own
        // baseline y, so the bar grows upward from the bottom of its own body.
        const stroke = (el, opening, baseY) => {
            if (!el) return;
            const max = 22, hh = Math.max(0, Math.min(1, opening / 100)) * max;
            el.setAttribute('height', hh.toFixed(1));
            el.setAttribute('y', (baseY - hh).toFixed(1));
        };
        stroke(s.v1Stroke, this.v1, 391);
        stroke(s.v2Stroke, this.v2, 87);
        if (s.v1Label) s.v1Label.textContent = this.v1.toFixed(0) + '%';
        if (s.v2Label) s.v2Label.textContent = this.v2.toFixed(0) + '%';

        // Splitter bars: fill = opening, so all three share one scale
        const barW = (el, pct) => {
            if (el) el.setAttribute('width', (Math.max(0, Math.min(100, pct)) / 100 * g.barW).toFixed(1));
        };
        barW(s.barU, this.pidOutput);
        barW(s.barV1, this.v1);
        barW(s.barV2, this.v2);
        if (s.barU2) {
            s.barU2.setAttribute('x1', (g.barX + this.pidOutput / 100 * g.barW).toFixed(1));
            s.barU2.setAttribute('x2', (g.barX + this.pidOutput / 100 * g.barW).toFixed(1));
        }

        if (s.branchText) {
            const txt = this.inDeadBand ? 'SPLIT DEAD BAND'
                : (this.v1 > 0 ? 'DRAIN BRANCH (V₁)' : 'MAKE-UP BRANCH (V₂)');
            s.branchText.textContent = txt;
            s.branchText.setAttribute('fill', this.inDeadBand ? '#fbbf24' : (this.v1 > 0 ? '#f87171' : '#38bdf8'));
        }

        // Flow arrows: fade a stream out when it is shut. (Opacity only -- a
        // transform on a <g> would scale about the SVG origin, not the group.)
        const arrow = (el, flow, ref) => {
            if (!el) return;
            const on = flow > 1e-4;
            const strength = on ? 0.45 + 0.55 * Math.min(1, flow / ref) : 0.15;
            el.setAttribute('opacity', strength.toFixed(3));
        };
        arrow(s.feedArrow, this.params.qFeed, 1.5);
        arrow(s.drawArrow, this.params.qLoad, 1.5);
        arrow(s.makeupArrow, this.qIn, 1.5);
        arrow(s.drainArrow, this.qOut, 2.0);
    }

    updateReadouts() {
        const r = this.readouts;
        const set = (el, txt) => { if (el) el.textContent = txt; };
        set(r.h, this.h.toFixed(2) + ' m');
        set(r.u, this.pidOutput.toFixed(1) + ' %');
        set(r.v1, this.v1.toFixed(1) + ' %');
        set(r.v2, this.v2.toFixed(1) + ' %');
        set(r.qin, this.qIn.toFixed(3) + ' m³/s');
        set(r.qout, this.qOut.toFixed(3) + ' m³/s');
    }

    updateMetrics(reset) {
        const m = this.metrics;
        const set = (el, txt) => { if (el) el.textContent = txt; };

        if (reset) {
            set(m.level, '3.00 m');
            set(m.pid, '50.0 %');
            set(m.v1, '0.0 %');
            set(m.v2, '0.0 %');
            set(m.branch, 'dead band');
            set(m.sse, '— m');
            return;
        }

        set(m.level, this.h.toFixed(2) + ' m');
        set(m.pid, this.pidOutput.toFixed(1) + ' %');
        set(m.v1, this.v1.toFixed(1) + ' %');
        set(m.v2, this.v2.toFixed(1) + ' %');
        set(m.branch, this.inDeadBand ? 'dead band'
            : (this.v1 > 0 ? 'V₁ drain open' : 'V₂ make-up open'));
        set(m.sse, (this.params.setpoint - this.h >= 0 ? '+' : '') +
            (this.params.setpoint - this.h).toFixed(3) + ' m');
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.splitRangeControl = new SplitRangeControlSimulator();
});
