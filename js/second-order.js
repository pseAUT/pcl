// ============================================
// Second-Order System - Interactive Analysis
// ============================================

class SecondOrderSimulator {
    constructor() {
        this.params = { zeta: 0.3, wn: 2.0 };
        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.run();
    }

    initDOM() {
        this.sliders = {
            zeta: document.getElementById('zeta'),
            wn: document.getElementById('wn')
        };
        this.values = {
            zeta: document.getElementById('zeta-val'),
            wn: document.getElementById('wn-val')
        };
        this.chartTabs = document.querySelectorAll('.chart-tab');
    }

    bindEvents() {
        Object.keys(this.sliders).forEach(key => {
            this.sliders[key].addEventListener('input', (e) => {
                this.params[key] = parseFloat(e.target.value);
                this.values[key].textContent = parseFloat(e.target.value).toFixed(key === 'zeta' ? 2 : 1);
                this.run();
            });
        });

        this.chartTabs.forEach(tab => {
            tab.addEventListener('click', () => this.switchChart(tab.dataset.chart));
        });

        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', () => this.setPreset(btn.dataset.preset));
        });
    }

    setPreset(preset) {
        const presets = {
            'highly-under': { zeta: 0.1, wn: 3 },
            'under': { zeta: 0.3, wn: 2 },
            'near-optimal': { zeta: 0.7, wn: 2 },
            'critical': { zeta: 1.0, wn: 2 },
            'over': { zeta: 2.0, wn: 2 },
            'fast-osc': { zeta: 0.5, wn: 5 }
        };
        const p = presets[preset];
        if (!p) return;

        this.params.zeta = p.zeta; this.params.wn = p.wn;
        this.sliders.zeta.value = p.zeta; this.values.zeta.textContent = p.zeta.toFixed(2);
        this.sliders.wn.value = p.wn; this.values.wn.textContent = p.wn.toFixed(1);

        document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
        document.querySelector(`[data-preset="${preset}"]`).classList.add('active');
        this.run();
    }

    stepResponse(t, zeta, wn) {
        if (t < 0) return 0;
        if (zeta < 0.999) {
            const wd = wn * Math.sqrt(1 - zeta * zeta);
            const phi = Math.atan2(zeta, Math.sqrt(1 - zeta * zeta));
            return 1 - (Math.exp(-zeta * wn * t) / Math.sqrt(1 - zeta * zeta)) * Math.sin(wd * t + phi);
        } else if (zeta > 1.001) {
            const s1 = -wn * (zeta + Math.sqrt(zeta * zeta - 1));
            const s2 = -wn * (zeta - Math.sqrt(zeta * zeta - 1));
            return 1 + (s1 * Math.exp(s2 * t) - s2 * Math.exp(s1 * t)) / (s2 - s1);
        } else {
            return 1 - (1 + wn * t) * Math.exp(-wn * t);
        }
    }

    initPlots() {
        const layout = {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 50, r: 20, t: 10, b: 40 },
            xaxis: { title: 'Time (s)', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { title: 'Output', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.99, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified',
            uirevision: 'true'
        };

        Plotly.newPlot('chart-step', [], { ...layout }, { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-poles', [], { ...layout, xaxis: { ...layout.xaxis, title: 'Real Axis (σ)' }, yaxis: { ...layout.yaxis, title: 'Imaginary Axis (jω)', scaleanchor: 'x' } }, { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-damping', [], { ...layout }, { responsive: true, displayModeBar: false });

        this.currentChart = 'step';
    }

    switchChart(chart) {
        this.currentChart = chart;
        this.chartTabs.forEach(t => t.classList.toggle('active', t.dataset.chart === chart));
        document.getElementById('chart-step').style.display = chart === 'step' ? 'block' : 'none';
        document.getElementById('chart-poles').style.display = chart === 'poles' ? 'block' : 'none';
        document.getElementById('chart-damping').style.display = chart === 'damping' ? 'block' : 'none';
        Plotly.Plots.resize(document.getElementById('chart-' + chart));
    }

    run() {
        const { zeta, wn } = this.params;
        const T = Math.max(4 / (zeta * wn || 0.1), 3);
        const dt = 0.005;
        const time = [], response = [];

        for (let t = 0; t <= T; t += dt) {
            time.push(t);
            response.push(this.stepResponse(t, zeta, wn));
        }

        // System type
        const badge = document.getElementById('system-type');
        const info = document.getElementById('system-info');
        if (zeta < 0.999) {
            badge.className = 'status-badge underdamped';
            badge.innerHTML = '<i class="fas fa-wave-square"></i> Underdamped (ζ < 1)';
            info.innerHTML = '<strong>Underdamped</strong> — Oscillatory response. Common in lightly damped systems. Overshoot: ' + (Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) * 100).toFixed(1) + '%';
        } else if (zeta > 1.001) {
            badge.className = 'status-badge overdamped';
            badge.innerHTML = '<i class="fas fa-minus-circle"></i> Overdamped (ζ > 1)';
            info.innerHTML = '<strong>Overdamped</strong> — Slow, non-oscillatory approach. Safe but sluggish.';
        } else {
            badge.className = 'status-badge critical';
            badge.innerHTML = '<i class="fas fa-check-circle"></i> Critically Damped (ζ = 1)';
            info.innerHTML = '<strong>Critically Damped</strong> — Fastest non-oscillatory response. Optimal boundary.';
        }

        // Metrics
        const overshoot = zeta < 1 ? Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)) * 100 : 0;
        const peakTime = zeta < 1 ? Math.PI / (wn * Math.sqrt(1 - zeta * zeta)) : 0;
        const wd = zeta < 1 ? wn * Math.sqrt(1 - zeta * zeta) : 0;

        // Rise time (10% to 90%)
        let t10 = 0, t90 = 0;
        for (let i = 0; i < response.length; i++) {
            if (response[i] >= 0.1 && t10 === 0) t10 = time[i];
            if (response[i] >= 0.9 && t90 === 0) { t90 = time[i]; break; }
        }
        const riseTime = t90 - t10;

        // Settling time (2%)
        let settlingTime = time[time.length - 1];
        for (let i = response.length - 1; i >= 0; i--) {
            if (Math.abs(response[i] - 1) > 0.02) {
                settlingTime = time[Math.min(i + 1, time.length - 1)];
                break;
            }
        }

        // Poles
        let poles;
        if (zeta < 1) {
            poles = [
                { re: -zeta * wn, im: wd },
                { re: -zeta * wn, im: -wd }
            ];
        } else if (zeta > 1) {
            poles = [
                { re: -wn * (zeta + Math.sqrt(zeta * zeta - 1)), im: 0 },
                { re: -wn * (zeta - Math.sqrt(zeta * zeta - 1)), im: 0 }
            ];
        } else {
            poles = [ { re: -wn, im: 0 }, { re: -wn, im: 0 } ];
        }

        // Step Response Plot
        const stepTraces = [
            { x: [0, T], y: [1, 1], name: 'Setpoint', line: { color: '#fbbf24', width: 2, dash: 'dash' } },
            { x: time, y: response, name: 'Response', line: { color: '#06b6d4', width: 3 }, fill: 'tozeroy', fillcolor: 'rgba(6,182,212,0.1)' }
        ];
        if (zeta < 1 && peakTime > 0) {
            stepTraces.push({ x: [peakTime], y: [1 + overshoot / 100], mode: 'markers', name: 'Peak', marker: { color: '#ef4444', size: 13, symbol: 'star' } });
        }

        const stepLayout = {
            paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 50, r: 20, t: 10, b: 40 },
            xaxis: { title: 'Time (s)', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { title: 'Output', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.99, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified', uirevision: 'true'
        };

        Plotly.react('chart-step', stepTraces, stepLayout, { responsive: true, displayModeBar: false });

        // Pole Plot
        const poleRe = poles.map(p => p.re);
        const poleIm = poles.map(p => p.im);
        const maxIm = Math.max(...poleIm.map(Math.abs), 1);
        const minRe = Math.min(...poleRe);

        Plotly.react('chart-poles', [
            { x: poleRe, y: poleIm, mode: 'markers', name: 'Poles', marker: { color: '#ef4444', size: 16, symbol: 'x', line: { width: 3, color: '#ef4444' } } },
            { x: [minRe * 1.3, 1], y: [0, 0], mode: 'lines', line: { color: '#334155', width: 1, dash: 'dot' }, showlegend: false },
            { x: [0, 0], y: [-maxIm * 1.6, maxIm * 1.6], mode: 'lines', line: { color: '#ef4444', width: 1, dash: 'dash' }, showlegend: false }
        ], {
            ...stepLayout,
            xaxis: { title: 'Real Axis (σ)', gridcolor: '#1e293b', zerolinecolor: '#334155', range: [minRe * 1.5, 1] },
            yaxis: { title: 'Imaginary Axis (jω)', gridcolor: '#1e293b', zerolinecolor: '#334155', scaleanchor: 'x' },
            showlegend: false,
            annotations: [{ x: 0.5, y: 1.08, xref: 'paper', yref: 'paper', text: `Poles: ${poles.map(p => `${p.re.toFixed(2)}${p.im >= 0 ? '+' : ''}${p.im.toFixed(2)}j`).join(', ')}`, showarrow: false, font: { color: '#94a3b8', size: 11 } }]
        }, { responsive: true, displayModeBar: false });

        // Damping Comparison
        const zetas = [0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 2.0];
        const dampColors = ['#ef4444', '#f97316', '#f59e0b', '#84cc16', '#10b981', '#06b6d4', '#a78bfa'];
        const dampData = zetas.map((z, i) => ({
            x: time, y: time.map(t => this.stepResponse(t, z, wn)),
            name: `ζ=${z}`, line: { color: dampColors[i], width: 2 }
        }));
        dampData.push({ x: [0, T], y: [1, 1], name: 'Setpoint', line: { color: '#64748b', width: 1, dash: 'dash' } });
        Plotly.react('chart-damping', dampData, {
            ...stepLayout,
            annotations: [{ x: 0.5, y: 1.08, xref: 'paper', yref: 'paper', text: `ωn = ${wn.toFixed(1)} rad/s — Effect of varying ζ`, showarrow: false, font: { color: '#94a3b8', size: 12 } }]
        }, { responsive: true, displayModeBar: false });

        // Metrics
        document.getElementById('metric-rise').textContent = riseTime > 0 ? riseTime.toFixed(2) + ' s' : '— s';
        document.getElementById('metric-overshoot').textContent = overshoot.toFixed(1) + '%';
        document.getElementById('metric-peak-time').textContent = zeta < 1 ? peakTime.toFixed(2) + ' s' : '— s';
        document.getElementById('metric-settling').textContent = settlingTime.toFixed(2) + ' s';
        document.getElementById('metric-wd').textContent = zeta < 1 ? wd.toFixed(2) + ' rad/s' : '0 rad/s';
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.soSim = new SecondOrderSimulator();
});