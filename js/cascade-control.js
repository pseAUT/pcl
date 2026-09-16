// ============================================================================
// Cascade Control - Jacketed Stirred Tank Heater
//
// One plant, two structures. A jacketed vessel is fed cold product at Tin.
// The outer (master) temperature controller TIC holds the outlet temperature
// T2 by trimming the setpoint of an inner (slave) controller TjC, which drives
// the steam valve. In single-loop mode the TIC drives the steam valve
// directly, so the two structures can be compared under the identical
// disturbance at the same closed-loop speed.
//
// ---- Plant ----------------------------------------------------------------
// Two well-mixed thermal capacities: the vessel contents and the jacket.
// They exchange the SAME heat rate UA*(Tj - T2), which leaves the jacket and
// enters the vessel with opposite signs, so the pair conserves energy. The
// vessel additionally loses sensible heat to the through-flow w*cp*(T2 - Tin).
//
//   m2 cp dT2/dt = UA (Tj - T2) - w cp (T2 - Tin)
//   mj cp dTj/dt = -(UA (Tj - T2)) + UA_s (u/100) (Tsteam - Tj)
//
// Dividing by UA gives the dimensionless form the code integrates:
//
//   tau2 dT2/dt = (Tj - T2) - K1 (T2 - Tin)
//   tauj dTj/dt = (T2 - Tj) + Ks (u/100) (Tsteam - Tj)
//
//   tau2 = m2 cp / UA   [s]      K1 = w cp / UA   [-]  (through-flow number)
//   tauj = mj cp / UA   [s]      Ks = UA_s / UA   [-]  (steam authority)
//
// ---- Why this model (the "physics") ---------------------------------------
//   * Energy conservation: the coupling term appears with opposite signs in
//     the two equations, so whatever the jacket loses the vessel gains. The
//     steady state is therefore exact and the integrator cannot drift.
//   * Bounded by Tsteam: the jacket term is (Tsteam - Tj), so as Tj approaches
//     Tsteam the jacket stops accepting heat. Vessel temperature can never
//     reach Tsteam - this is the physical authority limit that makes the
//     single loop slow to recover.
//   * Steady state: setting both derivatives to zero gives
//       Tj - T2 = K1 (T2 - Tin)        (the offset the through-flow demands)
//       Ks (u/100) (Tsteam - Tj) = K1 (T2 - Tin)   (the steam must supply it)
//     so the outer controller's integral must be free to wander over the whole
//     jacket window Tin..Tsteam. That is why the TIC bias is the product
//     setpoint and its output limits are Tin..Tsteam.
//   * Why cascade helps: the inner loop is FASTER (tauj < tau2), so the steam
//     pressure / jacket disturbances are corrected before they reach T2.
//
// A transport delay theta sits on the steam valve (steam line + jacket fill),
// implemented as a FIFO of exactly theta/dt steps, so it is a real dead time
// and not a filter.
// ============================================================================

class CascadeControlSimulator {
    constructor() {
        // ---- Plant parameters ---------------------------------------------
        this.params = {
            tau2: 8.0,      // tau2  - vessel capacity / UA (s)
            tauj: 3.0,      // tauj  - jacket capacity / UA (s)
            K1: 0.5,        // K1    - w cp / UA (through-flow number)
            Ks: 2.0,        // Ks    - UA_s / UA (steam authority)
            theta: 1.0,     // theta - steam-valve transport delay (s)
            Tin: 25.0,      // feed temperature (degC)
            Tsteam: 140.0,  // steam temperature (degC)
            setpoint: 80.0, // product temperature setpoint (degC)
            tauD: 1.0       // derivative filter time constant (s)
        };

        // ---- Timestep -----------------------------------------------------
        // dt MUST exist before the PID controllers are constructed: they take
        // it as the reference step that turns the derivative-filter alpha into
        // a physical time constant. (Reading this.dt before assignment was a
        // real bug in the original page: dtRef came out undefined.)
        this.dt = 0.01;
        // Must match the .speed-btn marked active in cascade-control.html.
        this.speed = 5;
        this.running = false;
        this.simulationTime = 0;
        this.tEnd = 60;

        // ---- Tuned gain sets ----------------------------------------------
        // Both structures are tuned to the same closed-loop speed so the
        // comparison at the same disturbance is fair. The single loop needs a
        // much larger Kd to get there - see the "What to Try" card.
        this.gainSets = {
            cascade: {
                outer: { kp: 1.8, ki: 0.2, kd: 0.5 },
                inner: { kp: 2.5, ki: 0.8, kd: 0.2 }
            },
            single: {
                outer: { kp: 0.3, ki: 0.5, kd: 2.5 },
                inner: { kp: 2.5, ki: 0.8, kd: 0.2 }
            }
        };
        this.mode = 'cascade';
        this.gains = {
            outer: Object.assign({}, this.gainSets.cascade.outer),
            inner: Object.assign({}, this.gainSets.cascade.inner)
        };

        // ---- Controllers --------------------------------------------------
        // Outer TIC: its output IS a temperature (the jacket setpoint), so its
        // bias is the product setpoint and its limits span the jacket window.
        // beta/gamma are exposed on the sliders in the markup.
        this.outerPID = new PIDController({
            kp: this.gains.outer.kp, ki: this.gains.outer.ki, kd: this.gains.outer.kd,
            bias: this.params.setpoint,
            min: this.params.Tin, max: this.params.Tsteam,
            iMin: -400, iMax: 400, beta: 1, gamma: 0, antiWindup: true,
            dtRef: this.dt
        });
        // Inner TjC: jacket temperature -> steam valve (%).
        this.innerPID = new PIDController({
            kp: this.gains.inner.kp, ki: this.gains.inner.ki, kd: this.gains.inner.kd,
            bias: 50, min: 0, max: 100,
            iMin: -100, iMax: 100, beta: 1, gamma: 0, antiWindup: true,
            dtRef: this.dt
        });
        // Stand-alone single loop: product temperature -> steam valve (%).
        const sg = this.gainSets.single.outer;
        this.singlePID = new PIDController({
            kp: sg.kp, ki: sg.ki, kd: sg.kd,
            bias: 50, min: 0, max: 100,
            iMin: -100, iMax: 100, beta: 1, gamma: 0, antiWindup: true,
            dtRef: this.dt
        });

        // ---- Simulation state ---------------------------------------------
        this.T2 = this.params.Tin;      // vessel outlet temperature
        this.Tj = this.params.Tin;      // jacket temperature
        this.TjSP = this.params.setpoint;
        this.u = 50;                    // steam valve (%)
        this.uApplied = 50;             // after the transport delay
        this.delayLine = [];
        this.noiseStd = 0;
        this.lastStepTime = 0;

        // Set to the ideal jacket temperature during a mode switch, so the
        // incoming controller is seeded for a bumpless transfer.
        this.steadyTj = null;

        // ---- Buffers ------------------------------------------------------
        this.bufferSize = 8000;
        this.timeData = [];
        this.t2Data = [];
        this.tjData = [];
        this.spData = [];
        this.tjspData = [];
        this.uData = [];

        this.animationId = null;
        this.lastFrameTime = 0;

        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.initSchematic();
        this.reset();
    }

    delaySteps() {
        return Math.max(0, Math.round(this.params.theta / this.dt));
    }

    // Per-step retention of the derivative filter, from a time constant.
    derivativeRetention() {
        const tau = this.params.tauD;
        return tau > 0 ? Math.exp(-this.dt / tau) : 0;
    }


    // ---------------------------------------------------------------- DOM
    initDOM() {
        const $ = (id) => document.getElementById(id);

        this.sliders = {
            outKp: $('out-kp'), outKi: $('out-ki'), outKd: $('out-kd'),
            innKp: $('inn-kp'), innKi: $('inn-ki'), innKd: $('inn-kd'),
            setpoint: $('setpoint'),
            beta: $('o-beta'), gamma: $('o-gamma'),
            tsteam: $('tsteam'), tin: $('tin'),
            theta: $('theta'), tau2: $('tau2'), tauj: $('tauj'), ks: $('ks'),
            tauD: $('tau-d'), noise: $('noise')
        };
        this.values = {
            outKp: $('out-kp-val'), outKi: $('out-ki-val'), outKd: $('out-kd-val'),
            innKp: $('inn-kp-val'), innKi: $('inn-ki-val'), innKd: $('inn-kd-val'),
            setpoint: $('sp-val'),
            beta: $('o-beta-val'), gamma: $('o-gamma-val'),
            tsteam: $('tsteam-val'), tin: $('tin-val'),
            theta: $('theta-val'), tau2: $('tau2-val'), tauj: $('tauj-val'), ks: $('ks-val'),
            tauD: $('tau-d-val'), noise: $('noise-val')
        };
        this.awCheck = $('antiwindup');
        this.tEndInput = $('tend');
        this.innerPanel = $('inner-panel');
        this.outerSub = $('outer-sub');
        this.modeBtns = document.querySelectorAll('.mode-btn');

        this.runPauseBtn = $('run-pause-btn');
        this.runPauseIcon = $('run-pause-icon');
        this.runPauseText = $('run-pause-text');
        this.resetBtn = $('reset-btn');
        this.speedBtns = document.querySelectorAll('.speed-btn');
        this.stepBtns = document.querySelectorAll('.step-btn');
        this.presetBtns = document.querySelectorAll('.preset-btn');
        this.tabBtns = document.querySelectorAll('.chart-tab');

        this.readouts = {
            t2: $('ro-t2'), tj: $('ro-tj'), tjsp: $('ro-tjsp'),
            u: $('ro-u'), mode: $('ro-mode'), err: $('ro-err')
        };
        this.metrics = {
            t2: $('metric-t2'), tj: $('metric-tj'), u: $('metric-u'),
            ep: $('metric-ep'), ej: $('metric-ej'), iae: $('metric-iae')
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

        // Loop gains. The gains are mirrored onto the controllers before every
        // integration step (see stepSimulation), so a slider move is live.
        bindSlider(this.sliders.outKp, this.values.outKp, v => v.toFixed(2),
            v => { this.gains.outer.kp = v; this.markEdit(); });
        bindSlider(this.sliders.outKi, this.values.outKi, v => v.toFixed(3),
            v => { this.gains.outer.ki = v; this.markEdit(); });
        bindSlider(this.sliders.outKd, this.values.outKd, v => v.toFixed(2),
            v => { this.gains.outer.kd = v; this.markEdit(); });
        bindSlider(this.sliders.innKp, this.values.innKp, v => v.toFixed(2),
            v => { this.gains.inner.kp = v; this.markEdit(); });
        bindSlider(this.sliders.innKi, this.values.innKi, v => v.toFixed(3),
            v => { this.gains.inner.ki = v; this.markEdit(); });
        bindSlider(this.sliders.innKd, this.values.innKd, v => v.toFixed(2),
            v => { this.gains.inner.kd = v; this.markEdit(); });

        bindSlider(this.sliders.setpoint, this.values.setpoint, v => v.toFixed(1),
            v => {
                if (v !== this.params.setpoint) {
                    this.params.setpoint = v;
                    this.lastStepTime = this.simulationTime;
                    this.steadyTj = null;
                }
            });
        // Live disturbances.
        bindSlider(this.sliders.tsteam, this.values.tsteam, v => v.toFixed(0),
            v => { this.params.Tsteam = v; this.steadyTj = null; });
        bindSlider(this.sliders.tin, this.values.tin, v => v.toFixed(0),
            v => { this.params.Tin = v; this.steadyTj = null; });
        bindSlider(this.sliders.theta, this.values.theta, v => v.toFixed(1),
            v => { this.params.theta = v; });
        bindSlider(this.sliders.tau2, this.values.tau2, v => v.toFixed(1),
            v => { this.params.tau2 = v; });
        bindSlider(this.sliders.tauj, this.values.tauj, v => v.toFixed(1),
            v => { this.params.tauj = v; });
        bindSlider(this.sliders.ks, this.values.ks, v => v.toFixed(2),
            v => { this.params.Ks = v; });
        bindSlider(this.sliders.tauD, this.values.tauD, v => v.toFixed(2),
            v => { this.params.tauD = v; });
        bindSlider(this.sliders.noise, this.values.noise, v => v.toFixed(2),
            v => { this.noiseStd = v; });
        // 2-DOF setpoint weights on the outer loop.
        bindSlider(this.sliders.beta, this.values.beta, v => v.toFixed(2),
            v => { this.outerPID.beta = v; });
        bindSlider(this.sliders.gamma, this.values.gamma, v => v.toFixed(2),
            v => { this.outerPID.gamma = v; });

        if (this.awCheck) {
            this.awCheck.addEventListener('change', () => this.setAntiWindup(this.awCheck.checked));
        }

        if (this.tEndInput) {
            const apply = () => {
                const v = Math.max(10, Math.min(5000, parseFloat(this.tEndInput.value) || 60));
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

        this.stepBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const step = parseFloat(btn.dataset.step);
                const next = Math.max(40, Math.min(this.params.Tsteam - 5,
                    +(this.params.setpoint + step).toFixed(1)));
                this.params.setpoint = next;
                if (this.sliders.setpoint) this.sliders.setpoint.value = next;
                if (this.values.setpoint) this.values.setpoint.textContent = next.toFixed(1);
                this.lastStepTime = this.simulationTime;
                this.steadyTj = null;
            });
        });

        this.tabBtns.forEach(btn => {
            btn.addEventListener('click', () => this.setTab(btn.dataset.chart));
        });

        // Mode selector: each structure loads its own tuned gains.
        this.modeBtns.forEach(btn => {
            btn.addEventListener('click', () => this.setMode(btn.dataset.mode));
        });

        const presets = {
            cascadeTuned: { mode: 'cascade', outer: { kp: 1.8, ki: 0.2, kd: 0.5 }, inner: { kp: 2.5, ki: 0.8, kd: 0.2 } },
            cascadeSlow: { mode: 'cascade', outer: { kp: 0.8, ki: 0.05, kd: 0.2 }, inner: { kp: 1.5, ki: 0.4, kd: 0.1 } },
            cascadeFast: { mode: 'cascade', outer: { kp: 3.5, ki: 0.5, kd: 1.0 }, inner: { kp: 6.0, ki: 2.0, kd: 0.3 } },
            singleTuned: { mode: 'single', outer: { kp: 0.3, ki: 0.5, kd: 2.5 } }
        };
        this.presetBtns.forEach(btn => {
            btn.addEventListener('click', () => {
                const p = presets[btn.dataset.preset];
                if (!p) return;
                this.presetBtns.forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                if (p.inner) this.gainSets.cascade.inner = Object.assign({}, p.inner);
                if (p.mode === 'cascade') this.gainSets.cascade.outer = Object.assign({}, p.outer);
                else this.gainSets.single.outer = Object.assign({}, p.outer);
                this.setMode(p.mode, true);
                this.lastStepTime = this.simulationTime;
            });
        });
    }

    // A manual gain edit invalidates the auto-preset highlight.
    markEdit() {
        this.presetBtns.forEach(b => b.classList.remove('active'));
    }

    setAntiWindup(on) {
        [this.outerPID, this.innerPID, this.singlePID].forEach(p => { p.antiWindup = on; });
    }

    setTab(name) {
        this.tabBtns.forEach(b => b.classList.toggle('active', b.dataset.chart === name));
        const outer = document.getElementById('tab-outer');
        const inner = document.getElementById('tab-inner');
        if (outer) outer.style.display = name === 'inner' ? 'none' : '';
        if (inner) inner.style.display = name === 'inner' ? '' : 'none';
        // A chart inside a display:none box reports clientWidth 0 and skipped
        // its render; re-lay it out now that it is visible.
        setTimeout(() => {
            Plotly.Plots.resize(name === 'inner' ? 'chart-u' : 'chart-t2');
            Plotly.Plots.resize(name === 'inner' ? 'chart-u' : 'chart-tj');
        }, 0);
    }

    // Switch between the cascade and the single-loop structure.
    setMode(mode, force) {
        if (!mode) return;
        if (mode === this.mode && !force) {
            this.modeBtns.forEach(b => b.classList.toggle('active', b.dataset.mode === this.mode));
            return;
        }
        // Stash the gains under the mode we are leaving -- but only when the
        // mode actually changes. A preset click on the ALREADY-ACTIVE mode
        // (e.g. "Cascade (fast)" while in cascade) must not first overwrite
        // the slot with the current gains; the preset handler has just filled
        // that slot, and stashing here would clobber it before it is loaded.
        if (mode !== this.mode) {
            this.gainSets[this.mode].outer = Object.assign({}, this.gains.outer);
            if (this.mode === 'cascade') this.gainSets.cascade.inner = Object.assign({}, this.gains.inner);
        }

        this.mode = mode;
        const g = this.gainSets[mode];
        this.gains.outer = Object.assign({}, g.outer);
        this.gains.inner = Object.assign({}, g.inner);

        // ---- Bumpless transfer, both directions ---------------------------
        // At steady state both structures must hold the SAME valve position,
        // but their natural biases differ (the TIC biases at the product
        // setpoint, the single loop biases at 50 %). Seed the controller that
        // is coming on-line so its output equals the signal actually in
        // service. Without this the outer TIC would instantly demand
        // Tj,sp = 80 C and crash the jacket from its steady 107.5 C.
        const t2 = this.T2;
        if (mode === 'cascade') {
            const o = this.outerPID;
            o.integral = this.Tj - o.bias - o.kp * (o.beta * this.params.setpoint - t2);
            // The inner loop keeps its integral; it simply tracks the new
            // jacket setpoint from where the jacket already is.
        } else {
            const s = this.singlePID;
            s.bias = this.u;
            s.integral = 0;
            s.prevPv = null;
            s.prevSP = null;
            s.dFiltered = 0;
        }

        this.pushGainsToSliders();
        this.modeBtns.forEach(b => b.classList.toggle('active', b.dataset.mode === mode));

        if (this.innerPanel) this.innerPanel.classList.toggle('is-bypassed', mode === 'single');
        if (this.outerSub) {
            this.outerSub.textContent = mode === 'single'
                ? 'product temperature \u2192 steam valve (direct)'
                : 'product temperature \u2192 jacket setpoint';
        }
        this.lastStepTime = this.simulationTime;
        this.steadyTj = null;
        this.updateSchematic();
        this.updatePlots();
        this.readoutsUpdateSafe();
    }

    readoutsUpdateSafe() {
        try { this.updateReadouts(); this.updateMetrics(); } catch (e) { /* pre-init */ }
    }

    pushGainsToSliders() {
        const o = this.gains.outer, i = this.gains.inner;
        const set = (el, out, v, fmt) => {
            if (el) el.value = v;
            if (out) out.textContent = fmt(v);
        };
        set(this.sliders.outKp, this.values.outKp, o.kp, v => v.toFixed(2));
        set(this.sliders.outKi, this.values.outKi, o.ki, v => v.toFixed(3));
        set(this.sliders.outKd, this.values.outKd, o.kd, v => v.toFixed(2));
        set(this.sliders.innKp, this.values.innKp, i.kp, v => v.toFixed(2));
        set(this.sliders.innKi, this.values.innKi, i.ki, v => v.toFixed(3));
        set(this.sliders.innKd, this.values.innKd, i.kd, v => v.toFixed(2));
    }

    setSpeed(speed) {
        this.speed = speed;
        this.speedBtns.forEach(btn => {
            btn.classList.toggle('active', parseFloat(btn.dataset.speed) === speed);
        });
    }


    // ------------------------------------------------------------- lifecycle
    reset() {
        this.stopRun();
        this.simulationTime = 0;
        this.T2 = this.params.Tin;
        this.Tj = this.params.Tin;
        this.u = 50;
        this.uApplied = 50;
        this.TjSP = this.params.setpoint;
        this.delayLine = new Array(this.delaySteps()).fill(50);
        this.outerPID.reset();
        this.innerPID.reset();
        this.singlePID.reset();
        this.outerPID.bias = this.params.setpoint;
        this.outerPID.min = this.params.Tin;
        this.outerPID.max = this.params.Tsteam;
        this.steadyTj = null;
        this.lastStepTime = 0;

        this.timeData = []; this.t2Data = []; this.tjData = [];
        this.spData = []; this.tjspData = []; this.uData = [];

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

    // Real-time loop: advances `speed` seconds of plant time per wall second,
    // capped so a stalled tab cannot run away with a huge batch of steps.
    animate() {
        if (!this.running) return;

        const now = performance.now();
        const delta = Math.min((now - this.lastFrameTime) / 1000, 0.25);
        this.lastFrameTime = now;

        let steps = Math.min(Math.max(1, Math.round(delta * this.speed / this.dt)), 500);
        const remaining = Math.round((this.tEnd - this.simulationTime) / this.dt);
        if (remaining <= 0) { this.updateAll(); this.stopRun(); return; }
        steps = Math.min(steps, remaining);

        for (let i = 0; i < steps; i++) this.stepSimulation();

        this.updateAll();

        if (this.simulationTime >= this.tEnd - 1e-9) { this.stopRun(); return; }
        this.animationId = requestAnimationFrame(() => this.animate());
    }

    updateAll() {
        this.updatePlots();
        this.updateSchematic();
        this.updateReadouts();
        this.updateMetrics();
    }

    // --------------------------------------------------------------- physics
    stepSimulation() {
        const p = this.params;

        // Mirror the live slider gains onto the controllers.
        this.outerPID.kp = this.gains.outer.kp;
        this.outerPID.ki = this.gains.outer.ki;
        this.outerPID.kd = this.gains.outer.kd;
        this.innerPID.kp = this.gains.inner.kp;
        this.innerPID.ki = this.gains.inner.ki;
        this.innerPID.kd = this.gains.inner.kd;

        // Derivative filter (one time constant shared by all three loops).
        const a = this.derivativeRetention();
        this.outerPID.alpha = a;
        this.innerPID.alpha = a;
        this.singlePID.alpha = a;

        // The outer loop biases at the product setpoint, and its window is the
        // physical jacket range. During a mode switch `steadyTj` holds the
        // ideal jacket temperature instead, so the transfer is bumpless.
        this.outerPID.bias = this.steadyTj !== null ? this.steadyTj : p.setpoint;
        this.outerPID.min = Math.min(p.Tin, p.Tsteam);
        this.outerPID.max = Math.max(p.Tin, p.Tsteam);

        // Measurements (optional Gaussian sensor noise, measurement only).
        const t2m = this.noiseStd > 0 ? this.T2 + this.noiseStd * gaussianNoise() : this.T2;
        const tjm = this.noiseStd > 0 ? this.Tj + this.noiseStd * gaussianNoise() : this.Tj;

        // ---- Both structures run every step -------------------------------
        // The inner loop keeps the jacket on its setpoint, so if the operator
        // switches to cascade the inner controller is already there.
        this.TjSP = this.outerPID.update(p.setpoint, t2m, this.dt);
        const uCascade = this.innerPID.update(this.TjSP, tjm, this.dt);
        const uSingle = this.singlePID.update(p.setpoint, t2m, this.dt);

        this.u = this.mode === 'single' ? uSingle : uCascade;

        // ---- Transport delay on the steam valve ---------------------------
        // A FIFO of exactly n = theta/dt steps: the steam line refills before
        // the jacket sees a new valve position, so this is a true dead time.
        const n = this.delaySteps();
        if (this.delayLine.length !== n) {
            this.delayLine = new Array(n).fill(this.uApplied);
        }
        if (n > 0) {
            this.delayLine.push(this.u);
            this.uApplied = this.delayLine.shift();
        } else {
            this.uApplied = this.u;
        }

        // ---- Two-capacity energy balance ----------------------------------
        // The coupling term (Tj - T2) enters with opposite signs in the two
        // equations, so the model conserves energy exactly. The jacket term
        // (Tsteam - Tj) can never drive Tj past Tsteam.
        const dT2 = ((this.Tj - this.T2) - p.K1 * (this.T2 - p.Tin)) / p.tau2;
        const dTj = ((this.T2 - this.Tj) + p.Ks * (this.uApplied / 100) * (p.Tsteam - this.Tj)) / p.tauj;
        this.T2 += dT2 * this.dt;
        this.Tj += dTj * this.dt;

        this.simulationTime += this.dt;

        // ---- Record (decimated to a fixed 0.1 s sample interval) ----------
        if (this.timeData.length === 0 ||
            this.simulationTime - this.timeData[this.timeData.length - 1] >= 0.1) {
            this.timeData.push(this.simulationTime);
            this.t2Data.push(this.T2);
            this.tjData.push(this.Tj);
            this.spData.push(p.setpoint);
            this.tjspData.push(this.TjSP);
            this.uData.push(this.u);
            if (this.timeData.length > this.bufferSize) {
                [this.timeData, this.t2Data, this.tjData, this.spData,
                 this.tjspData, this.uData].forEach(arr => arr.shift());
            }
        }
    }

    // ----------------------------------------------------------------- plots
    baseLayout(yTitle) {
        return {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { family: 'Inter' },
            margin: { l: 55, r: 18, t: 10, b: 40 },
            xaxis: { title: 'Time (s)' },
            yaxis: { title: yTitle },
            legend: { x: 0.99, y: 0.99 },
            hovermode: 'x unified',
            uirevision: 'true'
        };
    }

    initPlots() {
        // PV: product temperature (the outer loop).
        Plotly.newPlot('chart-t2', [
            { x: [], y: [], name: 'Product T2', line: { color: '#06b6d4', width: 3 } },
            { x: [], y: [], name: 'Setpoint', line: { color: '#fbbf24', width: 2.5, dash: 'dash' } }
        ], this.baseLayout('Temperature (\u00B0C)'), { responsive: true, displayModeBar: false });

        // Inner loop: jacket temperature against the setpoint the TIC hands down.
        Plotly.newPlot('chart-tj', [
            { x: [], y: [], name: 'Jacket Tj', line: { color: '#f87171', width: 3 } },
            { x: [], y: [], name: 'Tj setpoint (from TIC)', line: { color: '#a78bfa', width: 2, dash: 'dash' } }
        ], this.baseLayout('Temperature (\u00B0C)'), { responsive: true, displayModeBar: false });

        // MV: the steam valve, plus the steam temperature that disturbs it.
        Plotly.newPlot('chart-u', [
            { x: [], y: [], name: 'Steam valve u', line: { color: '#22c55e', width: 3 },
              fill: 'tozeroy', fillcolor: 'rgba(34,197,94,0.10)' },
            { x: [], y: [], name: 'Tsteam (disturbance)', line: { color: '#fbbf24', width: 1.5, dash: 'dot' }, yaxis: 'y2' }
        ], Object.assign(this.baseLayout('Valve (%)'), {
            yaxis2: { title: 'Tsteam (\u00B0C)', overlaying: 'y', side: 'right' }
        }), { responsive: true, displayModeBar: false });

        this.setTab('outer');
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
        if (this.lastStepTime > tStart && this.lastStepTime <= tEnd) {
            shapes.push({
                type: 'line', x0: this.lastStepTime, x1: this.lastStepTime,
                yref: 'paper', y0: 0, y1: 1,
                line: { color: '#fbbf24', width: 1.5, dash: 'dot' }
            });
        }
        const xaxis = { range: [tStart, Math.max(tEnd, windowSpan)] };

        // Explicit Y ranges. The temperature axes show the whole physical
        // window (feed .. steam) so the approach to setpoint is readable from
        // t = 0, and the valve axis is pinned to 0..100 %.
        const lo = Math.min(this.params.Tin, this.params.Tsteam);
        const hi = Math.max(this.params.Tin, this.params.Tsteam);
        const yLo = Math.min(lo, this.params.setpoint) - 5;
        const yHi = Math.max(hi, this.params.setpoint) + 5;

        Plotly.update('chart-t2',
            { x: [this.timeData, spX], y: [this.t2Data, spY] },
            { xaxis: xaxis, shapes: shapes, yaxis: { range: [yLo, yHi] } });
        Plotly.update('chart-tj',
            { x: [this.timeData, this.timeData], y: [this.tjData, this.tjspData] },
            { xaxis: xaxis, shapes: shapes, yaxis: { range: [yLo, yHi] } });
        Plotly.update('chart-u',
            { x: [this.timeData, this.timeData], y: [this.uData, this.uData.map(() => this.params.Tsteam)] },
            { xaxis: xaxis, shapes: shapes, yaxis: { range: [-5, 105] } });
    }


    // ----------------------------------------------------------- schematic
    // Every id here must exist in the <svg> in cascade-control.html. The
    // original page referenced 14 ids that were never in the markup, so the
    // drawing was completely static.
    initSchematic() {
        const $ = (id) => document.getElementById(id);
        this.svg = {
            product: $('casc-product-fill'),
            jacket: $('casc-jacket-fill'),
            t2Label: $('casc-t2-label'),
            tjLabel: $('casc-tj-label'),
            tjspLine: $('casc-tjsp-line'),
            tjspLabel: $('casc-tjsp-label'),
            valveStem: $('casc-valve-stem'),
            valveBody: $('casc-valve-body'),
            valveLabel: $('casc-valve-label'),
            steamFlow: $('casc-steam-flow'),
            productFlow: $('casc-product-flow'),
            feedFlow: $('casc-feed-flow'),
            pathCascade: $('casc-path-cascade'),
            pathSingle: $('casc-path-single'),
            modeLabel: $('casc-mode-label'),
            timeLabel: $('casc-time-label'),
            uLabel: $('casc-u-label'),
            outerOut: $('casc-outer-out'),
            innerOut: $('casc-inner-out')
        };
        // Geometry shared with the SVG. These MUST match the <rect> band and
        // valve coordinates in cascade-control.html.
        this.geom = {
            vesselTop: 214, vesselBot: 344,   // inner vessel fill band
            jacketTop: 186, jacketBot: 372,   // jacket fill band
            valveStemTop: 66, valveStemMax: 20
        };
    }

    tempFraction(T, lo, hi) {
        if (!(hi > lo)) return 0;
        return Math.max(0, Math.min(1, (T - lo) / (hi - lo)));
    }

    updateSchematic() {
        const s = this.svg, p = this.params, g = this.geom;
        if (!s || !s.product) return;
        const set = (el, txt) => { if (el) el.textContent = txt; };

        const lo = Math.min(p.Tin, p.Tsteam), hi = Math.max(p.Tin, p.Tsteam);

        // Vessel fill: height tracks the vessel temperature inside the
        // feed..steam window, so the picture visibly heats up as the loop runs.
        const f2 = this.tempFraction(this.T2, lo, hi);
        const hh2 = 10 + f2 * (g.vesselBot - g.vesselTop - 10);
        s.product.setAttribute('y', (g.vesselBot - hh2).toFixed(1));
        s.product.setAttribute('height', hh2.toFixed(1));

        // Jacket fill: same idea on the jacket band.
        if (s.jacket) {
            const fj = this.tempFraction(this.Tj, lo, hi);
            const hhj = 8 + fj * (g.jacketBot - g.jacketTop - 8);
            s.jacket.setAttribute('y', (g.jacketBot - hhj).toFixed(1));
            s.jacket.setAttribute('height', hhj.toFixed(1));
        }

        set(s.t2Label, this.T2.toFixed(1) + ' \u00B0C');
        set(s.tjLabel, this.Tj.toFixed(1) + ' \u00B0C');
        set(s.valveLabel, this.u.toFixed(0) + '%');
        set(s.modeLabel, this.mode === 'single' ? 'SINGLE LOOP' : 'CASCADE');
        set(s.timeLabel, this.simulationTime.toFixed(1) + ' s');
        set(s.uLabel, 'steam ' + this.u.toFixed(0) + ' %');
        set(s.outerOut, this.TjSP.toFixed(1) + ' \u00B0C');
        set(s.innerOut, this.u.toFixed(0) + ' %');

        // Jacket-setpoint marker: a horizontal line across the jacket at the
        // temperature the outer loop is currently asking for.
        if (s.tjspLine) {
            const y = g.jacketBot - this.tempFraction(this.TjSP, lo, hi) * (g.jacketBot - g.jacketTop);
            s.tjspLine.setAttribute('y1', y.toFixed(1));
            s.tjspLine.setAttribute('y2', y.toFixed(1));
            if (s.tjspLabel) {
                // The label sits just BELOW the line, left-aligned at the line
                // start, so it never collides with the Tj-measurement trace.
                s.tjspLabel.setAttribute('y', (y - 6).toFixed(1));
                s.tjspLabel.textContent = 'Tj,sp ' + this.TjSP.toFixed(1);
            }
        }

        // Steam valve: the stem is drawn as a plug that lifts off its seat as
        // the valve opens, so the stroke length is the opening.
        if (s.valveStem) {
            const stemH = 3 + (this.u / 100) * g.valveStemMax;
            s.valveStem.setAttribute('height', stemH.toFixed(1));
            s.valveStem.setAttribute('y', (g.valveStemTop - stemH).toFixed(1));
        }
        if (s.valveBody) {
            s.valveBody.setAttribute('opacity', (0.22 + 0.62 * (this.u / 100)).toFixed(2));
        }

        // Flow lines: opacity tracks the corresponding flow.
        const op = (el, v) => { if (el) el.setAttribute('opacity', v.toFixed(2)); };
        op(s.steamFlow, 0.18 + 0.82 * (this.u / 100));
        op(s.productFlow, 0.8);
        op(s.feedFlow, 0.8);

        // Which control path is live.
        if (s.pathCascade) s.pathCascade.setAttribute('opacity', this.mode === 'cascade' ? '1' : '0.10');
        if (s.pathSingle) s.pathSingle.setAttribute('opacity', this.mode === 'single' ? '1' : '0.10');
        if (s.modeLabel) s.modeLabel.setAttribute('fill', this.mode === 'single' ? '#f59e0b' : '#a78bfa');
    }

    updateReadouts() {
        const r = this.readouts;
        const set = (el, txt) => { if (el) el.textContent = txt; };
        set(r.t2, this.T2.toFixed(2) + ' \u00B0C');
        set(r.tj, this.Tj.toFixed(2) + ' \u00B0C');
        set(r.tjsp, this.TjSP.toFixed(2) + ' \u00B0C');
        set(r.u, this.u.toFixed(1) + ' %');
        set(r.mode, this.mode === 'single' ? 'single loop' : 'cascade');
        set(r.err, (this.params.setpoint - this.T2).toFixed(3) + ' \u00B0C');
    }

    updateMetrics(reset) {
        const m = this.metrics;
        const set = (el, txt) => { if (el) el.textContent = txt; };

        if (reset) {
            set(m.t2, this.params.Tin.toFixed(2) + ' \u00B0C');
            set(m.tj, this.params.Tin.toFixed(2) + ' \u00B0C');
            set(m.u, '50.0 %');
            set(m.ep, (this.params.setpoint - this.params.Tin).toFixed(2) + ' \u00B0C');
            set(m.ej, '\u2014');
            set(m.iae, '0.0');
            return;
        }

        set(m.t2, this.T2.toFixed(2) + ' \u00B0C');
        set(m.tj, this.Tj.toFixed(2) + ' \u00B0C');
        set(m.u, this.u.toFixed(1) + ' %');
        set(m.ep, (this.params.setpoint - this.T2).toFixed(3) + ' \u00B0C');
        set(m.ej, (this.TjSP - this.Tj).toFixed(3) + ' \u00B0C');

        // Integral absolute error since the last setpoint move / mode change.
        // The record is decimated to 0.1 s, so each sample weighs 0.1 s.
        let iae = 0;
        for (let i = 0; i < this.timeData.length; i++) {
            if (this.timeData[i] >= this.lastStepTime) {
                iae += Math.abs(this.spData[i] - this.t2Data[i]) * 0.1;
            }
        }
        set(m.iae, iae.toFixed(1) + ' \u00B0C\u00B7s');
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.cascadeControl = new CascadeControlSimulator();
});

