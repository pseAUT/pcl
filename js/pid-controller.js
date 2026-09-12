/* ============================================================================
 * PIDController
 * ----------------------------------------------------------------------------
 * A 2-degree-of-freedom PID controller with optional anti-reset windup,
 * in the style of the Aspen HYSYS PID block.
 *
 *   u = bias + Kp (beta*SP - PV)
 *            + Ki * integral(SP - PV)
 *            + Kd * (gamma*dSP/dt - dPV/dt)
 *
 *   beta  : proportional setpoint weight (2-DOF).  beta = 1 -> standard PID,
 *           beta < 1 -> softer proportional action on setpoint changes
 *           (reduces overshoot without slowing disturbance rejection).
 *   gamma : derivative setpoint weight (2-DOF).  gamma = 0 -> derivative on
 *           measurement (no derivative kick).  gamma = 1 -> derivative on error.
 *
 *   alpha : first-order derivative filter strength (0 = off, -> 1 = heavy
 *           smoothing).  The filter is the standard first-order low-pass
 *             Df[k] = a*Df[k-1] + (1-a)*D_raw[k],  a = exp(-dt/tau)
 *           where the time constant tau is taken from the alpha slider at a
 *           fixed reference step dtRef.  This makes the physical smoothing
 *           independent of the simulator's integration step (previously the
 *           same alpha gave a 5x different time constant in the 0.002 s CSTR
 *           and the 0.01 s tank loops).
 *
 *   antiWindup : when true, back-calculation ("anti-reset windup") bleeds the
 *           integral by the amount the actuator is over-driven. Unlike the old
 *           error-sign test this is robust to noisy measurements, and the
 *           integral is always kept inside [iMin, iMax].
 *
 * The last update exposes the individual contributions in this.terms
 * ({P, I, D, bias, out, saturated}) so the UI can show how each term of the
 * equation contributes to the manipulated variable.
 * ==========================================================================*/
class PIDController {
    constructor(opts) {
        opts = opts || {};
        this.kp = opts.kp != null ? opts.kp : 1;
        this.ki = opts.ki != null ? opts.ki : 0;
        this.kd = opts.kd != null ? opts.kd : 0;
        this.bias = opts.bias != null ? opts.bias : 0;
        this.min = opts.min != null ? opts.min : -Infinity;
        this.max = opts.max != null ? opts.max : Infinity;
        this.iMin = opts.iMin != null ? opts.iMin : -1e9;
        this.iMax = opts.iMax != null ? opts.iMax : 1e9;
        this.beta = opts.beta != null ? opts.beta : 1;
        this.gamma = opts.gamma != null ? opts.gamma : 0;
        this.alpha = opts.alpha != null ? opts.alpha : 0;   // derivative filter (0 = off)
        this.antiWindup = opts.antiWindup != null ? opts.antiWindup : true;
        // Reference step (s) at which alpha is defined, and the back-calculation
        // tracking time (s). trackingTime defaults to the integral time Kp/Ki.
        this.dtRef = opts.dtRef != null ? opts.dtRef : 0.01;
        this.trackingTime = opts.trackingTime != null ? opts.trackingTime : null;
        this.reset();
    }

    reset() {
        this.integral = 0;
        this.prevPv = null;
        this.prevSP = null;
        this.dFiltered = 0;
        this.terms = { P: 0, I: 0, D: 0, bias: this.bias, out: this.bias, saturated: false };
    }

    // Per-step retention of the first-order derivative filter. alpha is the
    // retention over the reference step dtRef; converting it to a continuous
    // time constant makes the smoothing independent of the integration step.
    filterCoefficient(dt) {
        if (!(this.alpha > 0)) return 0;          // filter off -> D = D_raw
        if (this.alpha >= 1) return 1;            // fully frozen
        const tau = -this.dtRef / Math.log(this.alpha);
        return tau > 0 ? Math.exp(-dt / tau) : 0;
    }

    update(sp, pv, dt) {
        if (!(dt > 0)) dt = 1e-9;

        const e = sp - pv;
        const dSP = this.prevSP === null ? 0 : (sp - this.prevSP) / dt;
        const dPV = this.prevPv === null ? 0 : (pv - this.prevPv) / dt;
        this.prevSP = sp;
        this.prevPv = pv;

        // 2-DOF proportional action (setpoint weighted) and derivative action
        // (measurement weighted when gamma = 0, so a setpoint step has no kick).
        const P = this.kp * (this.beta * sp - pv);
        const D_raw = this.kd * (this.gamma * dSP - dPV);

        // First-order derivative filter.
        const a = this.filterCoefficient(dt);
        this.dFiltered = a * this.dFiltered + (1 - a) * D_raw;
        const D = this.dFiltered;

        const I_try = this.integral + this.ki * e * dt;
        let out = this.bias + P + I_try + D;
        let saturated = false;

        if (this.antiWindup) {
            const uSat = Math.max(this.min, Math.min(this.max, out));
            saturated = uSat !== out;
            let I_new = I_try;
            if (saturated) {
                // Back-calculation: bleed the integral by the over-drive, so it
                // settles at the value that holds the actuator on its limit.
                const Tt = this.trackingTime != null
                    ? this.trackingTime
                    : (this.ki > 0 ? Math.max(this.kp / this.ki, dt) : Math.max(dt, 0.1));
                I_new = I_try + (dt / Tt) * (uSat - out);
            }
            this.integral = Math.max(this.iMin, Math.min(this.iMax, I_new));
            out = this.bias + P + this.integral + D;
        } else {
            this.integral = I_try;
        }

        const clamped = Math.max(this.min, Math.min(this.max, out));
        this.terms = {
            P, I: this.integral, D, bias: this.bias,
            out: clamped,
            saturated: saturated || clamped !== out
        };
        return clamped;
    }
}

/* Standard normal random number (Box-Muller). */
function gaussianNoise() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

if (typeof window !== 'undefined') {
    window.PIDController = PIDController;
    window.gaussianNoise = gaussianNoise;
}
