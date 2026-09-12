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
 *   alpha : first-order derivative filter (0 = off, -> 1 = heavy smoothing).
 *           D_f[k] = alpha*D_f[k-1] + (1-alpha)*D_raw[k]
 *
 *   antiWindup : when true, conditional-integration ("clamping") prevents the
 *           integral from winding up while the output is saturated. When false
 *           the integral is allowed to wind up (useful for demonstration).
 *
 * The derivative is computed directly from the raw measurements; callers that
 * need derivative filtering can low-pass the PV before feeding it in.
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
        this.reset();
    }

    reset() {
        this.integral = 0;
        this.prevPv = null;
        this.prevSP = null;
        this.dFiltered = 0;
    }

    update(sp, pv, dt) {
        if (!(dt > 0)) dt = 1e-9;

        const e = sp - pv;
        const dSP = this.prevSP === null ? 0 : (sp - this.prevSP) / dt;
        const dPV = this.prevPv === null ? 0 : (pv - this.prevPv) / dt;
        this.prevSP = sp;
        this.prevPv = pv;

        const P = this.kp * (this.beta * sp - pv);
        const D_raw = this.kd * (this.gamma * dSP - dPV);
        // First-order (exponential) derivative filter. alpha = 0 -> no filter,
        // alpha -> 1 -> heavy smoothing (useful when the PV is noisy).
        this.dFiltered = this.alpha * this.dFiltered + (1 - this.alpha) * D_raw;
        const D = this.dFiltered;
        const I_try = this.integral + this.ki * e * dt;

        let out = this.bias + P + I_try + D;

        if (this.antiWindup) {
            // Conditional integration: freeze the integral when the output is
            // saturated and the error would push it further into saturation.
            const satHigh = out > this.max && e > 0;
            const satLow = out < this.min && e < 0;
            if (!satHigh && !satLow) {
                this.integral = Math.max(this.iMin, Math.min(this.iMax, I_try));
            }
            out = this.bias + P + this.ki * this.integral + D;
        } else {
            this.integral = I_try;
        }

        return Math.max(this.min, Math.min(this.max, out));
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
