// ============================================
// CSTR Thermal Runaway - Stiff ODE Simulation
// Uses the ODELib adaptive solver (RK45 / TRBDF2 / auto)
// ============================================

const CSTR_BUILD = 'v5';

class CSTRRunawaySimulator {
    constructor() {
        // Reactor / kinetic parameters (MATLAB CSTR_runaway defaults)
        this.params = {
            cain: 1.0,
            q: 100,
            vol: 100,
            ti: 350,
            tc: 305,
            ua: 50000,
            k0: 7.2e10,
            eoverr: 8750,
            dhr: -50000,
            rho: 1000,
            cp: 0.239,
            ca0: 0.87725,
            t0: 324.475
        };
        this.tEnd = 20;
        this.method = 'auto';
        this.accuracy = 'balanced';

        this.speed = 5;              // playback multiplier
        this.running = false;
        this.solution = null;        // { t:[], y:[[]], stats }
        this.playbackIndex = 0;
        this.playbackTime = 0;
        this.animationId = null;
        this.lastFrame = 0;
        this.solveTimer = null;
        this.currentChart = 'temp';
        this.lastLiveIdx = -1;   // last index drawn on the live chart
        this.curveFull = false;  // never pre-plot; the trajectory fills in as it runs
        this.autoPlay = true;    // allow replay (started by the Run button)
        this.icLocked = false;   // initial conditions freeze once a run starts
        this.started = false;    // true once the user has pressed Run

        this.initDOM();
        this.bindEvents();
        this.initPlots();
        // Solve on load but do NOT start: the run begins when the user presses Run.
        this.solve({ autoPlay: false });
    }

    /* ---------------------------------------------------------------- */
    /* DOM / events                                                     */
    /* ---------------------------------------------------------------- */

    initDOM() {
        const sliderIds = ['cain', 'q', 'vol', 'ti', 'tc', 'ua', 'k0', 'eoverr', 'dhr', 'rho', 'cp', 'ca0', 't0'];
        this.sliders = {};
        sliderIds.forEach(id => {
            this.sliders[id] = document.getElementById(id);
        });
        // Text badges for the parameters; the initial conditions use numeric
        // inputs (see below) so exact values can be typed in.
        this.values = {};
        ['cain', 'q', 'vol', 'ti', 'tc', 'ua', 'k0', 'eoverr', 'dhr', 'rho', 'cp'].forEach(id => {
            this.values[id] = document.getElementById(id + '-val');
        });
        this.ca0Num = document.getElementById('ca0-num');
        this.t0Num = document.getElementById('t0-num');
        this.tendInput = document.getElementById('tend');
        this.methodSelect = document.getElementById('method');
        this.accuracySelect = document.getElementById('accuracy');
        this.statusEl = document.getElementById('solver-status');

        this.runPauseBtn = document.getElementById('run-pause-btn');
        this.runPauseIcon = document.getElementById('run-pause-icon');
        this.runPauseText = document.getElementById('run-pause-text');
        this.resetBtn = document.getElementById('reset-btn');
        this.solveBtn = document.getElementById('solve-btn');
        this.speedBtns = document.querySelectorAll('.speed-btn');
        this.chartTabs = document.querySelectorAll('.chart-tab');

        // Schematic elements
        this.liquid = document.getElementById('cstr-liquid');
        this.agitator = document.getElementById('cstr-agitator');
        this.thermoFill = document.getElementById('cstr-thermo-fill');
        this.thermoBulb = document.getElementById('cstr-thermo-bulb');
        this.thermoLabel = document.getElementById('cstr-thermo-label');
        this.caReadout = document.getElementById('cstr-ca-readout');
        this.tReadout = document.getElementById('cstr-t-readout');

        this.readouts = {
            time: document.getElementById('ro-time'),
            ca: document.getElementById('ro-ca'),
            t: document.getElementById('ro-t'),
            rate: document.getElementById('ro-rate')
        };

        this.metrics = {
            peakT: document.getElementById('metric-peak-t'),
            peakTime: document.getElementById('metric-peak-time'),
            finalT: document.getElementById('metric-final-t'),
            finalCA: document.getElementById('metric-final-ca'),
            conv: document.getElementById('metric-conv'),
            rate: document.getElementById('metric-rate'),
            method: document.getElementById('metric-method'),
            steps: document.getElementById('metric-steps')
        };
    }

    bindEvents() {
        // Reactor parameters can be changed while the simulation is running:
        // the ODE is re-integrated from the current state and the replay keeps
        // going in real time. Initial conditions are frozen once a run starts.
        const isInitialCondition = key => key === 'ca0' || key === 't0';
        const applyChange = (immediate) => {
            this.readParamsFromUI();
            if (this.isLiveSession()) {
                if (immediate) { this.clearSolveTimer(); this.resolveLive(); }
                else this.scheduleLive();
            } else if (immediate) {
                this.solve({ autoPlay: false });
            } else {
                this.scheduleSolve();
            }
        };
        Object.keys(this.sliders).forEach(key => {
            this.sliders[key].addEventListener('input', () => {
                if (isInitialCondition(key) && this.icLocked) return;
                applyChange(false);
            });
            // Solve immediately when the slider is released, so the new result
            // is always shown even if a debounce timer is interrupted.
            this.sliders[key].addEventListener('change', () => {
                if (isInitialCondition(key) && this.icLocked) return;
                applyChange(true);
            });
        });

        // Exact numeric entry for the initial conditions.
        const bindNumber = (numEl, rangeEl) => {
            if (!numEl || !rangeEl) return;
            const apply = () => {
                let v = parseFloat(numEl.value);
                if (!isFinite(v)) return null;
                const lo = parseFloat(rangeEl.min);
                const hi = parseFloat(rangeEl.max);
                if (isFinite(lo) && v < lo) v = lo;
                if (isFinite(hi) && v > hi) v = hi;
                rangeEl.value = v;
                this.readParamsFromUI(false);
                return v;
            };
            numEl.addEventListener('input', () => {
                if (this.icLocked) return;
                if (apply() === null) return;
                this.scheduleSolve();
            });
            numEl.addEventListener('change', () => {
                if (this.icLocked) return;
                if (apply() === null) return;
                this.solve({ autoPlay: false });
            });
        };
        bindNumber(this.ca0Num, this.sliders.ca0);
        bindNumber(this.t0Num, this.sliders.t0);

        // t_end is a text box: the new value is committed on Enter (and on
        // blur), never on every keystroke.
        const commitTend = () => {
            const raw = parseFloat(this.tendInput.value);
            if (!isFinite(raw)) { this.tendInput.value = this.tEnd; return; }
            const lo = parseFloat(this.tendInput.min);
            const hi = parseFloat(this.tendInput.max);
            let v = raw;
            if (isFinite(lo) && v < lo) v = lo;
            if (isFinite(hi) && v > hi) v = hi;
            this.tendInput.value = v;
            if (v === this.tEnd) return;   // nothing changed: skip a re-solve
            this.tEnd = v;
            if (this.isLiveSession()) {
                this.clearSolveTimer();
                this.resolveLive();
            } else {
                this.solve({ autoPlay: false });
            }
        };
        this.tendInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                commitTend();
                this.tendInput.blur();
            }
        });
        this.tendInput.addEventListener('change', () => commitTend());

        const onSolverChange = () => {
            this.method = this.methodSelect.value;
            this.accuracy = this.accuracySelect.value;
            // Immediate feedback so the panel never shows a stale solver.
            this.statusEl.innerHTML = `<strong>Selected:</strong> ${this.methodLabel(this.method)}<br>solving…`;
            this.solve({ autoPlay: false });
        };
        ['change', 'input'].forEach(ev => {
            this.methodSelect.addEventListener(ev, onSolverChange);
            this.accuracySelect.addEventListener(ev, onSolverChange);
        });

        this.runPauseBtn.addEventListener('click', () => this.togglePlay());
        this.resetBtn.addEventListener('click', () => this.resetPlayback());
        this.solveBtn.addEventListener('click', () => this.solve({ autoPlay: false, showFull: true }));

        this.speedBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                this.speed = parseInt(btn.dataset.speed);
                this.speedBtns.forEach(b => b.classList.toggle('active', b === btn));
            });
        });

        this.chartTabs.forEach(tab => {
            tab.addEventListener('click', () => this.switchChart(tab.dataset.chart));
        });
    }

    /* ---------------------------------------------------------------- */
    /* Parameter <-> UI plumbing                                        */
    /* ---------------------------------------------------------------- */

    formatValue(key, val) {
        switch (key) {
            case 'cain': return val.toFixed(2);
            case 'q':
            case 'vol':
            case 'ti':
            case 'tc':
            case 'rho':
            case 'eoverr': return String(Math.round(val));
            case 'ua': return (val / 1000).toFixed(1) + 'k';
            case 'k0': return this.formatExp(val);
            case 'dhr': return this.formatExp(val);
            case 'cp': return val.toFixed(3);
            case 'ca0': return val.toFixed(5);
            case 't0': return val.toFixed(3);
            default: return String(val);
        }
    }

    formatExp(x) {
        if (x === 0) return '0';
        const e = Math.floor(Math.log10(Math.abs(x)));
        const m = x / Math.pow(10, e);
        return m.toFixed(1) + 'e' + e;
    }

    readParamsFromUI(syncNumbers) {
        this.params.cain = parseFloat(this.sliders.cain.value);
        this.params.q = parseFloat(this.sliders.q.value);
        this.params.vol = parseFloat(this.sliders.vol.value);
        this.params.ti = parseFloat(this.sliders.ti.value);
        this.params.tc = parseFloat(this.sliders.tc.value);
        this.params.ua = parseFloat(this.sliders.ua.value);
        this.params.k0 = Math.pow(10, parseFloat(this.sliders.k0.value));
        this.params.eoverr = parseFloat(this.sliders.eoverr.value);
        this.params.dhr = parseFloat(this.sliders.dhr.value);
        this.params.rho = parseFloat(this.sliders.rho.value);
        this.params.cp = parseFloat(this.sliders.cp.value);
        this.params.ca0 = parseFloat(this.sliders.ca0.value);
        this.params.t0 = parseFloat(this.sliders.t0.value);

        Object.keys(this.values).forEach(key => {
            this.values[key].textContent = this.formatValue(key, this.params[key]);
        });
        // Do not write back to the number fields while the user is typing in
        // them (that would fight the cursor / swallow decimal points).
        if (syncNumbers !== false) {
            if (this.ca0Num) this.ca0Num.value = this.params.ca0;
            if (this.t0Num) this.t0Num.value = this.params.t0;
        }
    }

    /* ---------------------------------------------------------------- */
    /* Model + solve                                                    */
    /* ---------------------------------------------------------------- */

    rhs(t, y) {
        const p = this.params;
        const CA = y[0];
        const T = Math.max(y[1], 1e-6);
        const w = p.rho * p.q;
        const k = p.k0 * Math.exp(-p.eoverr / T);
        const dCA = p.q / p.vol * (p.cain - CA) - k * CA;
        const dT = w * (p.ti - T) / (p.rho * p.vol)
                 - p.dhr * k * CA / (p.rho * p.cp)
                 + p.ua * (p.tc - T) / (p.rho * p.cp * p.vol);
        return [dCA, dT];
    }

    scheduleSolve() {
        if (this.solveTimer) clearTimeout(this.solveTimer);
        this.solveTimer = setTimeout(() => this.solve({ autoPlay: false }), 180);
    }

    // Coalesce live parameter edits while a run is in progress.
    scheduleLive() {
        if (this.solveTimer) clearTimeout(this.solveTimer);
        this.solveTimer = setTimeout(() => {
            this.solveTimer = null;
            this.resolveLive();
        }, 120);
    }

    clearSolveTimer() {
        if (this.solveTimer) { clearTimeout(this.solveTimer); this.solveTimer = null; }
    }

    accuracySettings() {
        return {
            fast: { rtol: 1e-4, atol: 1e-6 },
            balanced: { rtol: 1e-6, atol: 1e-8 },
            accurate: { rtol: 1e-7, atol: 1e-9 }
        }[this.accuracy] || { rtol: 1e-6, atol: 1e-8 };
    }

    // A run is "live" while replaying or paused part-way through a run the user
    // actually started. Idle edits (before Run, or after the run finished) go
    // through a normal re-solve instead.
    isLiveSession() {
        if (!this.solution) return false;
        if (this.running) return true;
        return this.started && this.playbackIndex < this.solution.t.length - 1;
    }

    setICLocked(locked) {
        this.icLocked = locked;
        [this.sliders.ca0, this.sliders.t0, this.ca0Num, this.t0Num].forEach(el => {
            if (el) el.disabled = locked;
        });
    }

    // Re-integrate from the current playhead state with the (possibly changed)
    // parameters, keep the already-played history, and let the replay continue
    // in real time instead of jumping to the full solved curve.
    resolveLive() {
        if (this.solveTimer) { clearTimeout(this.solveTimer); this.solveTimer = null; }
        const sol = this.solution;
        if (!sol || sol.t.length < 2) { this.solve({ autoPlay: false }); return; }

        const n = sol.t.length;
        const idx = Math.max(0, Math.min(this.playbackIndex, n - 1));
        const tCur = sol.t[idx];
        if (!(this.tEnd > tCur)) { this.solve({ autoPlay: false }); return; }
        const yCur = [sol.y[idx][0], sol.y[idx][1]];
        const acc = this.accuracySettings();
        const span = this.tEnd - tCur;

        let res;
        try {
            res = ODELib.solve(
                (t, y) => this.rhs(t, y),
                [tCur, this.tEnd],
                yCur,
                {
                    method: this.method,
                    rtol: acc.rtol,
                    atol: acc.atol,
                    dtMax: span / 500,
                    minPoints: 500,
                    maxSteps: 20000
                }
            );
        } catch (err) {
            this.statusEl.textContent = 'Solver error: ' + err.message;
            return;
        }

        // Stitch: history up to the playhead + the newly integrated future.
        const combT = sol.t.slice(0, idx + 1).concat(res.t.slice(1));
        const combY = sol.y.slice(0, idx + 1).concat(res.y.slice(1));
        this.solution = { t: combT, y: combY, stats: res.stats };

        this.computeMetrics();
        this.playbackIndex = idx;
        this.lastLiveIdx = -1;
        this.curveFull = false;

        this.updateStatus();
        this.updateMetrics();
        try {
            this.updateCharts();
            this.updateSchematic(idx);
        } catch (e) {
            console.error('chart update failed', e);
        }
        // The animation loop (if running) picks up the new trajectory and keeps
        // revealing it; if paused, the new future waits until the user resumes.
    }

    solve(opts) {
        opts = opts || {};
        if (this.solveTimer) { clearTimeout(this.solveTimer); this.solveTimer = null; }
        this.pause();

        const acc = this.accuracySettings();

        const x0 = [this.params.ca0, this.params.t0];
        const tspan = [0, this.tEnd];

        let res;
        try {
            res = ODELib.solve(
                (t, y) => this.rhs(t, y),
                tspan,
                x0,
                {
                    method: this.method,
                    rtol: acc.rtol,
                    atol: acc.atol,
                    dtMax: this.tEnd / 500,
                    minPoints: 500,
                    maxSteps: 20000
                }
            );
        } catch (err) {
            this.statusEl.textContent = 'Solver error: ' + err.message;
            return;
        }

        this.solution = res;
        this.computeMetrics();
        this.playbackIndex = 0;
        this.playbackTime = 0;
        this.lastLiveIdx = -1;
        this.started = false;
        const autoPlay = opts.autoPlay === true;
        // Never pre-plot: unless explicitly asked (the Solve button), the chart
        // only shows the initial state and fills in as the run plays.
        this.curveFull = autoPlay ? false : (opts.showFull === true);

        // Status + metrics first: a chart error must never hide solver info.
        this.updateStatus();
        this.updateMetrics();
        try {
            this.updateCharts();
            this.updateSchematic(0);
        } catch (e) {
            console.error('chart update failed', e);
        }

        if (autoPlay) this.startPlayback();
        else { this.setPlayButton(false); this.setICLocked(false); }
    }

    computeMetrics() {
        const sol = this.solution;
        if (!sol || sol.t.length < 2) {
            this.metricsData = null;
            return;
        }
        const n = sol.t.length;
        const t = sol.t;
        const CA = sol.y.map(r => r[0]);
        const T = sol.y.map(r => r[1]);

        let peakIdx = 0;
        for (let i = 1; i < n; i++) if (T[i] > T[peakIdx]) peakIdx = i;

        const rate = new Array(n);
        for (let i = 0; i < n; i++) {
            if (i === 0) rate[i] = (T[1] - T[0]) / (t[1] - t[0]);
            else if (i === n - 1) rate[i] = (T[n - 1] - T[n - 2]) / (t[n - 1] - t[n - 2]);
            else rate[i] = (T[i + 1] - T[i - 1]) / (t[i + 1] - t[i - 1]);
        }
        let maxRate = 0;
        for (let i = 0; i < n; i++) maxRate = Math.max(maxRate, Math.abs(rate[i]));

        const finalCA = CA[n - 1];
        const conv = this.params.cain > 0 ? (this.params.cain - finalCA) / this.params.cain * 100 : 0;

        this.metricsData = {
            t, CA, T, rate,
            peakIdx,
            peakT: T[peakIdx],
            peakTime: t[peakIdx],
            finalT: T[n - 1],
            finalCA,
            conv,
            maxRate
        };
    }

    methodLabel(m) {
        return { rk45: 'RK45 (explicit)', trbdf2: 'TR-BDF2 (implicit)', auto: 'Auto (adaptive)' }[m] || m;
    }

    updateStatus() {
        const s = this.solution.stats;
        let used = this.methodLabel(s.method);
        if (s.fallbackFrom) used += ' ← from ' + this.methodLabel(s.fallbackFrom);
        const state = s.success ? 'converged' : 'stopped';
        this.statusEl.innerHTML =
            `<strong>Selected:</strong> ${this.methodLabel(this.method)}<br>` +
            `<strong>Used:</strong> ${used} · ${state}<br>` +
            `<strong>IC:</strong> CA0=${this.params.ca0}, T0=${this.params.t0} K<br>` +
            `${s.steps} steps · ${s.rejected} rejected · ${s.fEvals.toLocaleString()} f-evals<br>` +
            `wall time ${s.wallTime.toFixed(1)} ms · <span style="opacity:.6">build ${CSTR_BUILD}</span>`;
    }

    /* ---------------------------------------------------------------- */
    /* Playback                                                         */
    /* ---------------------------------------------------------------- */

    setPlayButton(isRunning) {
        this.runPauseIcon.className = isRunning ? 'fas fa-pause' : 'fas fa-play';
        this.runPauseText.textContent = isRunning ? 'Pause' : 'Play';
        this.runPauseBtn.classList.toggle('running', isRunning);
    }

    startPlayback() {
        if (!this.solution) return;
        if (this.animationId) cancelAnimationFrame(this.animationId);
        this.running = true;
        this.started = true;      // a real run is now in progress
        this.setICLocked(true);   // initial conditions freeze for this run
        this.setPlayButton(true);
        this.lastFrame = performance.now();
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    togglePlay() {
        if (!this.solution) { this.solve({ autoPlay: true }); return; }
        if (this.running) { this.pause(); return; }
        // Starting the run: if the full curve is shown (idle) or the replay has
        // finished, rewind to the initial condition and animate from there.
        if (this.curveFull || this.playbackIndex >= this.solution.t.length - 1) {
            this.curveFull = false;
            this.playbackIndex = 0;
            this.playbackTime = 0;
            this.lastLiveIdx = -1;
            try { this.updateCharts(); this.updateSchematic(0); } catch (e) { /* ignore */ }
        }
        this.startPlayback();
    }

    pause() {
        this.running = false;
        if (this.animationId) cancelAnimationFrame(this.animationId);
        this.animationId = null;
        this.setPlayButton(false);
    }

    resetPlayback() {
        this.pause();
        this.playbackIndex = 0;
        this.playbackTime = 0;
        this.lastLiveIdx = -1;
        this.curveFull = false;   // back to the initial state; Run plays it
        this.started = false;
        this.setICLocked(false);
        try {
            this.updateCharts();
            this.updateSchematic(0);
        } catch (e) { /* ignore */ }
    }

    animate() {
        if (!this.running || !this.solution) return;
        const now = performance.now();
        const dt = Math.min((now - this.lastFrame) / 1000, 0.1);
        this.lastFrame = now;

        const sol = this.solution;
        const n = sol.t.length;
        this.playbackTime += dt * this.speed;

        let idx = this.playbackIndex;
        while (idx < n - 1 && sol.t[idx + 1] <= this.playbackTime) idx++;
        this.playbackIndex = idx;

        this.updateSchematic(idx);
        this.updateChartsLive();

        if (idx >= n - 1 || this.playbackTime >= sol.t[n - 1]) {
            this.updateSchematic(n - 1);
            this.updateChartsLive(true);
            this.pause();
            this.setICLocked(false);   // run finished: ICs editable again
            return;
        }
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    /* ---------------------------------------------------------------- */
    /* Schematic                                                        */
    /* ---------------------------------------------------------------- */

    updateSchematic(idx) {
        const sol = this.solution;
        if (!sol) return;
        const n = sol.t.length;
        idx = Math.max(0, Math.min(idx, n - 1));
        const t = sol.t[idx];
        const CA = sol.y[idx][0];
        const T = sol.y[idx][1];

        // Temperature -> colour (blue ~280 K to red ~520 K)
        const frac = Math.max(0, Math.min(1, (T - 280) / (520 - 280)));
        const hue = 210 * (1 - frac);
        const color = `hsl(${hue}, 85%, 52%)`;
        if (this.liquid) this.liquid.setAttribute('fill', color);
        if (this.thermoBulb) this.thermoBulb.setAttribute('fill', color);

        // Thermometer fill (280 .. 550 K)
        const tFrac = Math.max(0, Math.min(1, (T - 280) / (550 - 280)));
        const maxH = 208;
        const h = maxH * tFrac;
        if (this.thermoFill) {
            this.thermoFill.setAttribute('y', 310 - h);
            this.thermoFill.setAttribute('height', h);
            this.thermoFill.setAttribute('fill', color);
        }
        if (this.thermoLabel) this.thermoLabel.textContent = T.toFixed(0);

        // In-vessel readouts
        if (this.caReadout) this.caReadout.textContent = 'CA = ' + CA.toFixed(3);
        if (this.tReadout) this.tReadout.textContent = 'T = ' + T.toFixed(1) + ' K';

        // Agitator speed scales with reaction rate k*CA
        const k = this.params.k0 * Math.exp(-this.params.eoverr / Math.max(T, 1e-6));
        const rr = k * CA;
        if (this.agitator) {
            const dur = 1.8 / (1 + Math.min(rr, 40)) + 0.15;
            this.agitator.style.animationDuration = dur.toFixed(2) + 's';
        }

        // Live readouts
        const rate = this.metricsData ? this.metricsData.rate[idx] : 0;
        if (this.readouts.time) this.readouts.time.textContent = t.toFixed(2) + ' s';
        if (this.readouts.ca) this.readouts.ca.textContent = CA.toFixed(3);
        if (this.readouts.t) this.readouts.t.textContent = T.toFixed(1) + ' K';
        if (this.readouts.rate) this.readouts.rate.textContent = (rate >= 0 ? '+' : '') + rate.toFixed(1) + ' K/s';
    }

    /* ---------------------------------------------------------------- */
    /* Charts                                                           */
    /* ---------------------------------------------------------------- */

    chartLayout(xTitle, yTitle) {
        return {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { family: 'Inter' },
            margin: { l: 55, r: 20, t: 10, b: 40 },
            xaxis: { title: xTitle },
            yaxis: { title: yTitle },
            legend: { x: 0.99, y: 0.99 },
            hovermode: 'x unified',
            uirevision: 'true'
        };
    }

    initPlots() {
        Plotly.newPlot('chart-temp', [], this.chartLayout('Time (s)', 'Temperature (K)'), { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-conc', [], this.chartLayout('Time (s)', 'Concentration CA (mol/L)'), { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-phase', [], this.chartLayout('Concentration CA (mol/L)', 'Temperature (K)'), { responsive: true, displayModeBar: false });
    }

    // Full rebuild of all three charts, drawn up to the current playhead.
    updateCharts() {
        const m = this.metricsData;
        if (!m) return;
        const n = m.t.length;
        const k = this.curveFull ? n : Math.max(1, Math.min(this.playbackIndex + 1, n));
        const tt = m.t.slice(0, k);
        const TT = m.T.slice(0, k);
        const CC = m.CA.slice(0, k);
        const p = this.params;
        const tEnd = m.t[n - 1];
        const last = k - 1;
        const peakShown = m.peakIdx <= last;
        const endShown = k >= n;

        // --- Temperature ---
        Plotly.react('chart-temp', [
            { x: [0, tEnd], y: [p.tc, p.tc], name: 'Coolant Tc', line: { color: '#38bdf8', width: 2, dash: 'dash' } },
            { x: tt, y: TT, name: 'Reactor T', line: { color: '#ef4444', width: 3 } },
            { x: peakShown ? [m.t[m.peakIdx]] : [], y: peakShown ? [m.T[m.peakIdx]] : [], mode: 'markers', name: 'Peak', marker: { color: '#f59e0b', size: 13, symbol: 'star' } },
            { x: [tt[last]], y: [TT[last]], mode: 'markers', name: 'Playhead', marker: { color: '#fbbf24', size: 11, line: { color: '#fff', width: 2 } } }
        ], this.chartLayout('Time (s)', 'Temperature (K)'), { responsive: true, displayModeBar: false });

        // --- Concentration ---
        Plotly.react('chart-conc', [
            { x: [0, tEnd], y: [p.cain, p.cain], name: 'Feed CAin', line: { color: '#64748b', width: 2, dash: 'dash' } },
            { x: tt, y: CC, name: 'CA', line: { color: '#0ea5e9', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(14,165,233,0.08)' },
            { x: [tt[last]], y: [CC[last]], mode: 'markers', name: 'Playhead', marker: { color: '#fbbf24', size: 11, line: { color: '#fff', width: 2 } } }
        ], this.chartLayout('Time (s)', 'Concentration CA (mol/L)'), { responsive: true, displayModeBar: false });

        // --- Phase plane ---
        Plotly.react('chart-phase', [
            { x: CC, y: TT, name: 'Trajectory', line: { color: '#a78bfa', width: 3 } },
            { x: [m.CA[0]], y: [m.T[0]], mode: 'markers', name: 'Start', marker: { color: '#10b981', size: 12 } },
            { x: endShown ? [m.CA[n - 1]] : [], y: endShown ? [m.T[n - 1]] : [], mode: 'markers', name: 'End', marker: { color: '#ef4444', size: 12 } },
            { x: [CC[last]], y: [TT[last]], mode: 'markers', name: 'Playhead', marker: { color: '#fbbf24', size: 11, line: { color: '#fff', width: 2 } } }
        ], this.chartLayout('Concentration CA (mol/L)', 'Temperature (K)'), { responsive: true, displayModeBar: false });
    }

    // Incremental live update of the visible chart while the replay runs,
    // so the curves are drawn in real time like the tank example.
    updateChartsLive(force) {
        const m = this.metricsData;
        if (!m) return;
        const n = m.t.length;
        const idx = Math.max(0, Math.min(this.playbackIndex, n - 1));
        if (!force && idx === this.lastLiveIdx) return;
        this.lastLiveIdx = idx;

        const k = idx + 1;
        const tt = m.t.slice(0, k);
        const TT = m.T.slice(0, k);
        const CC = m.CA.slice(0, k);
        const peakShown = m.peakIdx <= idx;
        const endShown = k >= n;

        try {
            if (this.currentChart === 'temp') {
                Plotly.restyle('chart-temp', { x: [tt], y: [TT] }, [1]);
                Plotly.restyle('chart-temp', { x: [[tt[k - 1]]], y: [[TT[k - 1]]] }, [3]);
                Plotly.restyle('chart-temp', {
                    x: [peakShown ? [m.t[m.peakIdx]] : []],
                    y: [peakShown ? [m.T[m.peakIdx]] : []]
                }, [2]);
            } else if (this.currentChart === 'conc') {
                Plotly.restyle('chart-conc', { x: [tt], y: [CC] }, [1]);
                Plotly.restyle('chart-conc', { x: [[tt[k - 1]]], y: [[CC[k - 1]]] }, [2]);
            } else {
                Plotly.restyle('chart-phase', { x: [CC], y: [TT] }, [0]);
                Plotly.restyle('chart-phase', { x: [[CC[k - 1]]], y: [[TT[k - 1]]] }, [3]);
                Plotly.restyle('chart-phase', {
                    x: [endShown ? [m.CA[n - 1]] : []],
                    y: [endShown ? [m.T[n - 1]] : []]
                }, [2]);
            }
        } catch (e) { /* chart not ready */ }
    }

    switchChart(chart) {
        this.currentChart = chart;
        this.chartTabs.forEach(t => t.classList.toggle('active', t.dataset.chart === chart));
        document.getElementById('chart-temp').style.display = chart === 'temp' ? 'block' : 'none';
        document.getElementById('chart-conc').style.display = chart === 'conc' ? 'block' : 'none';
        document.getElementById('chart-phase').style.display = chart === 'phase' ? 'block' : 'none';
        try { this.updateCharts(); } catch (e) { /* ignore */ }
        const el = document.getElementById('chart-' + chart);
        if (el) Plotly.Plots.resize(el);
        this.lastLiveIdx = -1;
    }

    /* ---------------------------------------------------------------- */
    /* Metrics panel                                                    */
    /* ---------------------------------------------------------------- */

    updateMetrics() {
        const m = this.metricsData;
        if (!m) return;
        this.metrics.peakT.textContent = m.peakT.toFixed(1) + ' K';
        this.metrics.peakTime.textContent = m.peakTime.toFixed(2) + ' s';
        this.metrics.finalT.textContent = m.finalT.toFixed(1) + ' K';
        this.metrics.finalCA.textContent = m.finalCA.toFixed(4);
        this.metrics.conv.textContent = m.conv.toFixed(1) + ' %';
        this.metrics.rate.textContent = m.maxRate.toFixed(1) + ' K/s';

        const s = this.solution.stats;
        const name = { rk45: 'RK45', trbdf2: 'TR-BDF2', auto: 'auto' }[s.method] || s.method;
        this.metrics.method.textContent = s.fallbackFrom ? name + '*' : name;
        this.metrics.steps.textContent = s.steps.toLocaleString();
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.cstrSim = new CSTRRunawaySimulator();
});
