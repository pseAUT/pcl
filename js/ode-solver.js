/* ============================================================================
 * ODE Solver Library
 * ----------------------------------------------------------------------------
 * A small, dependency-free, production-quality library for numerically
 * integrating systems of first order ordinary differential equations
 *
 *      dy/dt = f(t, y),      y(t0) = y0,      y in R^n
 *
 * Two integration engines are provided:
 *
 *   - "rk45"   : adaptive explicit Dormand-Prince Runge-Kutta (4)5.
 *                Fast and accurate for NON-STIFF problems.
 *
 *   - "trbdf2" : adaptive, L-stable TR-BDF2 implicit method (trapezoidal rule
 *                followed by a 2nd order backward differentiation formula).
 *                Robust on STIFF problems (e.g. chemical reactors with fast
 *                Arrhenius kinetics, thermal runaway, electrical networks).
 *                The implicit stages are solved with a damped Newton method
 *                using a numerically computed Jacobian. Error is controlled
 *                with step doubling + Richardson extrapolation.
 *
 *   - "auto"   : tries the explicit solver first and automatically falls back
 *                to the stiff solver when the explicit step size collapses.
 *
 * The solver returns a trajectory sampled at the (adaptive) accepted step
 * points plus a statistics object (steps, function evaluations, method used,
 * wall-clock time, success flag, ...).
 *
 * Usage:
 *   const res = ODELib.solve((t, y) => [ ...dy... ], [0, 20], [y0, y1], {
 *       method: 'auto', rtol: 1e-6, atol: 1e-8
 *   });
 *   res.t        // number[]
 *   res.y        // number[][]  (row i is the state vector at res.t[i])
 *   res.stats    // { steps, accepted, rejected, fEvals, method, ... }
 * ==========================================================================*/
(function (global) {
    'use strict';

    const EPS = Number.EPSILON;
    const SQRT_EPS = Math.sqrt(EPS);
    const DEFAULT_MAX_STEPS = 200000;

    /* ---------------------------------------------------------------------
     * Small linear algebra / vector helpers
     * -------------------------------------------------------------------*/

    function vecNorm(v) {
        let m = 0;
        for (let i = 0; i < v.length; i++) {
            const a = Math.abs(v[i]);
            if (a > m) m = a;
        }
        return m;
    }

    function weightedNorm(v, y, atol, rtol) {
        let m = 0;
        for (let i = 0; i < v.length; i++) {
            const s = atol + rtol * Math.abs(y[i]);
            const a = Math.abs(v[i]) / s;
            if (a > m) m = a;
        }
        return m;
    }

    function isFiniteVec(v) {
        for (let i = 0; i < v.length; i++) {
            if (!isFinite(v[i])) return false;
        }
        return true;
    }

    /* Solve A x = b with Gaussian elimination and partial pivoting.
     * A is an array of rows (n x n), b is length n. Returns x or null. */
    function solveLinearSystem(A, b) {
        const n = b.length;
        const M = new Array(n);
        for (let i = 0; i < n; i++) {
            const row = new Array(n + 1);
            for (let j = 0; j < n; j++) row[j] = A[i][j];
            row[n] = b[i];
            M[i] = row;
        }

        for (let col = 0; col < n; col++) {
            let pivot = col;
            let max = Math.abs(M[col][col]);
            for (let r = col + 1; r < n; r++) {
                const v = Math.abs(M[r][col]);
                if (v > max) { max = v; pivot = r; }
            }
            if (max < 1e-300) return null;
            if (pivot !== col) {
                const tmp = M[pivot]; M[pivot] = M[col]; M[col] = tmp;
            }
            const d = M[col][col];
            for (let r = col + 1; r < n; r++) {
                const factor = M[r][col] / d;
                if (factor === 0) continue;
                for (let c = col; c <= n; c++) M[r][c] -= factor * M[col][c];
            }
        }

        const x = new Array(n);
        for (let i = n - 1; i >= 0; i--) {
            let s = M[i][n];
            for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
            x[i] = s / M[i][i];
        }
        return x;
    }

    /* Forward-difference Jacobian of fn at y. `f0` optionally supplies fn(y)
     * to avoid one extra evaluation. */
    function numericalJacobian(fn, y, f0) {
        const n = y.length;
        const base = f0 || fn(y);
        const J = new Array(n);
        for (let i = 0; i < n; i++) J[i] = new Array(n).fill(0);

        for (let j = 0; j < n; j++) {
            const yj = y[j];
            const h = SQRT_EPS * Math.max(Math.abs(yj), 1e-3);
            y[j] = yj + h;
            const fj = fn(y);
            y[j] = yj;
            const inv = 1 / h;
            for (let i = 0; i < n; i++) J[i][j] = (fj[i] - base[i]) * inv;
        }
        return J;
    }

    /* ---------------------------------------------------------------------
     * Damped Newton solver for G(y) = 0
     * -------------------------------------------------------------------*/
    function newtonSolve(G, y0, atol, rtol, tol, maxIter) {
        let y = y0.slice();
        let F = G(y);
        let iter = 0;
        let converged = weightedNorm(F, y, atol, rtol) <= tol;

        while (!converged && iter < maxIter) {
            const J = numericalJacobian(G, y, F);
            const negF = new Array(F.length);
            for (let i = 0; i < F.length; i++) negF[i] = -F[i];
            const delta = solveLinearSystem(J, negF);
            if (!delta) break;

            const normBefore = vecNorm(F);
            let alpha = 1.0;
            let yNew = null;
            let FNew = null;
            for (let ls = 0; ls < 8; ls++) {
                yNew = new Array(y.length);
                for (let i = 0; i < y.length; i++) yNew[i] = y[i] + alpha * delta[i];
                if (!isFiniteVec(yNew)) { alpha *= 0.5; continue; }
                FNew = G(yNew);
                if (vecNorm(FNew) < normBefore || alpha < 1e-3) break;
                alpha *= 0.5;
            }
            y = yNew;
            F = FNew;
            iter++;
            converged = weightedNorm(F, y, atol, rtol) <= tol;
        }
        return { y, converged, iterations: iter };
    }

    /* ---------------------------------------------------------------------
     * Explicit Dormand-Prince RK45
     * -------------------------------------------------------------------*/
    const DP = {
        c: [0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1, 1],
        a: [
            [],
            [1 / 5],
            [3 / 40, 9 / 40],
            [44 / 45, -56 / 15, 32 / 9],
            [19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729],
            [9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656],
            [35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84]
        ],
        b5: [35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84, 0],
        b4: [5179 / 57600, 0, 7571 / 16695, 393 / 640, -92097 / 339200, 187 / 2100, 1 / 40]
    };

    function rk45Step(t, y, h, f) {
        const n = y.length;
        const k = new Array(7);
        k[0] = f(t, y);

        for (let s = 1; s < 7; s++) {
            const ys = new Array(n);
            for (let i = 0; i < n; i++) {
                let sum = 0;
                const aRow = DP.a[s];
                for (let j = 0; j < s; j++) sum += aRow[j] * k[j][i];
                ys[i] = y[i] + h * sum;
            }
            k[s] = f(t + DP.c[s] * h, ys);
        }

        const y5 = new Array(n);
        const y4 = new Array(n);
        for (let i = 0; i < n; i++) {
            let s5 = 0, s4 = 0;
            for (let j = 0; j < 7; j++) {
                s5 += DP.b5[j] * k[j][i];
                s4 += DP.b4[j] * k[j][i];
            }
            y5[i] = y[i] + h * s5;
            y4[i] = y[i] + h * s4;
        }
        // FSAL: k[6] is f(t+h, y5) already, reuse as k[0] of next step if needed.
        return { y: y5, yLow: y4 };
    }

    /* ---------------------------------------------------------------------
     * L-stable TR-BDF2 implicit step
     * -------------------------------------------------------------------*/
    const TB_GAMMA = 2 - Math.SQRT2;        // gamma = 2 - sqrt(2)
    const TB_C1 = (1 - TB_GAMMA) / (2 - TB_GAMMA);            // weight on h*f(t+h)
    const TB_C2 = -Math.pow(1 - TB_GAMMA, 2) / (TB_GAMMA * (2 - TB_GAMMA));
    const TB_C3 = 1 / (TB_GAMMA * (2 - TB_GAMMA));

    function trbdf2Step(t, y, h, f, atol, rtol, newtonTol, newtonMaxIter) {
        const n = y.length;
        const f0 = f(t, y);

        // ---- Stage 1: trapezoidal over [t, t + gamma*h] ----
        const tg = t + TB_GAMMA * h;
        const halfGammaH = 0.5 * TB_GAMMA * h;
        const rhs1 = new Array(n);
        for (let i = 0; i < n; i++) rhs1[i] = y[i] + halfGammaH * f0[i];

        const G1 = function (yg) {
            const fg = f(tg, yg);
            const out = new Array(n);
            for (let i = 0; i < n; i++) out[i] = yg[i] - halfGammaH * fg[i] - rhs1[i];
            return out;
        };
        const s1 = newtonSolve(G1, y, atol, rtol, newtonTol, newtonMaxIter);
        if (!s1.converged || !isFiniteVec(s1.y)) return { ok: false, reason: 'stage1' };
        const yg = s1.y;

        // ---- Stage 2: BDF2 over [t, t + h] ----
        const t1 = t + h;
        const rhs2 = new Array(n);
        for (let i = 0; i < n; i++) rhs2[i] = TB_C2 * y[i] + TB_C3 * yg[i];

        const G2 = function (y1) {
            const f1 = f(t1, y1);
            const out = new Array(n);
            for (let i = 0; i < n; i++) out[i] = y1[i] - TB_C1 * h * f1[i] - rhs2[i];
            return out;
        };
        const guess = new Array(n);
        const ext = (1 - TB_GAMMA) / TB_GAMMA;
        for (let i = 0; i < n; i++) guess[i] = yg[i] + ext * (yg[i] - y[i]);

        const s2 = newtonSolve(G2, guess, atol, rtol, newtonTol, newtonMaxIter);
        if (!s2.converged || !isFiniteVec(s2.y)) return { ok: false, reason: 'stage2' };

        return { ok: true, y: s2.y, yg: yg };
    }

    /* ---------------------------------------------------------------------
     * ODESolver class
     * -------------------------------------------------------------------*/
    class ODESolver {
        constructor(rhs, options) {
            if (typeof rhs !== 'function') throw new Error('ODESolver: rhs must be a function f(t, y)');
            this.rhs = rhs;
            this.opts = Object.assign({
                method: 'auto',          // 'auto' | 'rk45' | 'trbdf2'
                rtol: 1e-6,
                atol: 1e-8,
                dtInitial: null,         // auto if null
                dtMin: 1e-12,
                dtMax: null,             // auto if null
                maxSteps: DEFAULT_MAX_STEPS,
                safety: 0.9,
                minShrink: 0.2,
                maxGrow: 5.0,
                newtonTol: 1e-10,
                newtonMaxIter: 15,
                minPoints: 400,          // lower bound on output sample count
                blowUp: 1e14,            // |y| threshold treated as divergence
                onEvent: null            // optional callback(info)
            }, options || {});
        }

        _resetStats(method) {
            this.stats = {
                method: method,
                steps: 0,          // accepted steps
                rejected: 0,
                fEvals: 0,
                jacEvals: 0,
                success: false,
                message: '',
                tStart: 0,
                tEnd: 0,
                wallTime: 0
            };
            const self = this;
            this._f = function (t, y) {
                self.stats.fEvals++;
                return self.rhs(t, y);
            };
        }

        /* Public entry point. */
        solve(tspan, y0) {
            const t0 = performance_now();
            const method = (this.opts.method || 'auto').toLowerCase();
            let result;
            if (method === 'auto') {
                result = this._solveAuto(tspan, y0);
            } else if (method === 'rk45') {
                result = this._integrate(tspan, y0, 'rk45', this.opts.maxSteps);
            } else if (method === 'trbdf2') {
                result = this._integrate(tspan, y0, 'trbdf2', this.opts.maxSteps);
            } else {
                throw new Error('ODESolver: unknown method "' + method + '"');
            }
            this.stats.wallTime = performance_now() - t0;
            result.stats = this.stats;
            return result;
        }

        _solveAuto(tspan, y0) {
            // Try the explicit method with a bounded budget. Non-stiff problems
            // finish quickly; stiff ones exhaust the budget or collapse the step
            // and we transparently restart with the L-stable solver.
            const budget = Math.min(this.opts.maxSteps, 4000);
            const attempt = this._integrate(tspan, y0, 'rk45', budget, true);
            if (attempt.success) return attempt;

            // Fall back to the stiff solver.
            this.stats = null;
            const stiff = this._integrate(tspan, y0, 'trbdf2', this.opts.maxSteps, false);
            stiff.stats.fallbackFrom = 'rk45';
            stiff.stats.fallbackReason = attempt.message;
            return stiff;
        }

        _integrate(tspan, y0, method, stepBudget, isProbe) {
            const opts = this.opts;
            const t0 = tspan[0];
            const tEnd = tspan[1];
            const span = tEnd - t0;
            if (!(span > 0)) throw new Error('ODESolver: tspan must be increasing');

            this._resetStats(method);

            let y = y0.slice();
            let t = t0;

            const tOut = [t];
            const yOut = [y.slice()];

            let h = opts.dtInitial;
            if (h == null || !(h > 0)) h = span / 100;
            const dtMax = opts.dtMax != null ? opts.dtMax : span / Math.max(1, opts.minPoints);
            const dtMin = opts.dtMin;
            if (h > dtMax) h = dtMax;
            if (h < dtMin) h = dtMin;

            const order = method === 'rk45' ? 5 : 2; // error exponent uses order+1
            const errExponent = 1 / (order + 1);

            let failed = false;
            let message = '';

            while (t < tEnd - 1e-12) {
                if (this.stats.steps + this.stats.rejected > stepBudget) {
                    failed = true;
                    message = 'step budget exhausted (' + stepBudget + ')';
                    break;
                }
                if (h < dtMin) {
                    failed = true;
                    message = 'step size below dtMin (' + dtMin + ')';
                    break;
                }
                if (t + h > tEnd) h = tEnd - t;

                let accepted = false;
                let yNew = null;
                let err = 0;
                let errVec = null;

                if (method === 'rk45') {
                    const step = rk45Step(t, y, h, this._f);
                    errVec = new Array(y.length);
                    for (let i = 0; i < y.length; i++) errVec[i] = step.y[i] - step.yLow[i];
                    err = weightedNorm(errVec, step.y, opts.atol, opts.rtol);
                    yNew = step.y;
                    accepted = err <= 1.0;
                } else {
                    // TR-BDF2 with step doubling for the error estimate.
                    const full = trbdf2Step(t, y, h, this._f, opts.atol, opts.rtol, opts.newtonTol, opts.newtonMaxIter);
                    if (!full.ok) {
                        // Newton failed: reject and shrink aggressively.
                        this.stats.rejected++;
                        h = Math.max(dtMin, h * 0.25);
                        continue;
                    }
                    const half1 = trbdf2Step(t, y, h / 2, this._f, opts.atol, opts.rtol, opts.newtonTol, opts.newtonMaxIter);
                    if (!half1.ok) {
                        this.stats.rejected++;
                        h = Math.max(dtMin, h * 0.25);
                        continue;
                    }
                    const half2 = trbdf2Step(t + h / 2, half1.y, h / 2, this._f, opts.atol, opts.rtol, opts.newtonTol, opts.newtonMaxIter);
                    if (!half2.ok) {
                        this.stats.rejected++;
                        h = Math.max(dtMin, h * 0.25);
                        continue;
                    }
                    errVec = new Array(y.length);
                    for (let i = 0; i < y.length; i++) errVec[i] = (half2.y[i] - full.y[i]) / 3; // 2^2 - 1
                    err = weightedNorm(errVec, half2.y, opts.atol, opts.rtol);
                    yNew = half2.y;
                    accepted = err <= 1.0;
                }

                if (!isFiniteVec(yNew) || vecNorm(yNew) > opts.blowUp) {
                    failed = true;
                    message = 'solution diverged (non-finite or exceeded blow-up limit)';
                    break;
                }

                if (accepted) {
                    t += h;
                    y = yNew;
                    this.stats.steps++;
                    tOut.push(t);
                    yOut.push(y.slice());

                    if (opts.onEvent) opts.onEvent({ t: t, y: y, h: h, step: this.stats.steps });

                    // Grow the step based on the local error.
                    let factor = opts.maxGrow;
                    if (err > 0) factor = opts.safety * Math.pow(err, -errExponent);
                    factor = Math.min(opts.maxGrow, Math.max(1.0, factor));
                    h = Math.min(dtMax, h * factor);
                } else {
                    this.stats.rejected++;
                    let factor = opts.safety * Math.pow(err > 0 ? err : 1e6, -errExponent);
                    factor = Math.max(opts.minShrink, Math.min(1.0, factor));
                    h = Math.max(dtMin, h * factor);
                }
            }

            this.stats.tStart = t0;
            this.stats.tEnd = t;
            this.stats.success = !failed && t >= tEnd - 1e-9;
            this.stats.message = this.stats.success
                ? 'completed in ' + this.stats.steps + ' accepted steps'
                : message;
            if (isProbe) {
                this.stats.probe = true;
            }

            return {
                t: tOut,
                y: yOut,
                stats: this.stats,
                final: y,
                success: this.stats.success
            };
        }
    }

    /* Cross-environment high resolution timer. */
    function performance_now() {
        if (typeof performance !== 'undefined' && performance.now) return performance.now();
        return Date.now();
    }

    const ODELib = {
        version: '1.0.0',
        ODESolver: ODESolver,
        methods: ['auto', 'rk45', 'trbdf2'],
        solve: function (rhs, tspan, y0, options) {
            return new ODESolver(rhs, options).solve(tspan, y0);
        }
    };

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = ODELib;
    }
    global.ODELib = ODELib;

})(typeof window !== 'undefined' ? window : globalThis);
