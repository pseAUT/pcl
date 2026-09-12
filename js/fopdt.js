// ============================================
// FOPDT System - Interactive Analysis
// ============================================

class FOPDTSimulator {
    constructor() {
        this.params = { gain: 2.0, tau: 3.0, theta: 1.0, amplitude: 1.0 };
        this.initDOM();
        this.bindEvents();
        this.initPlots();
        this.run();
    }

    initDOM() {
        this.sliders = {
            gain: document.getElementById('gain'),
            tau: document.getElementById('tau'),
            theta: document.getElementById('theta'),
            amplitude: document.getElementById('amplitude')
        };
        this.values = {
            gain: document.getElementById('gain-val'),
            tau: document.getElementById('tau-val'),
            theta: document.getElementById('theta-val'),
            amplitude: document.getElementById('amplitude-val')
        };
        this.chartTabs = document.querySelectorAll('.chart-tab');
    }

    bindEvents() {
        Object.keys(this.sliders).forEach(key => {
            this.sliders[key].addEventListener('input', (e) => {
                this.params[key] = parseFloat(e.target.value);
                this.values[key].textContent = parseFloat(e.target.value).toFixed(1);
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
            'typical': { gain: 2, tau: 3, theta: 1 },
            'fast-gain': { gain: 5, tau: 1, theta: 0.2 },
            'slow-delay': { gain: 1, tau: 10, theta: 5 },
            'no-delay': { gain: 3, tau: 2, theta: 0 },
            'large-delay': { gain: 0.5, tau: 5, theta: 3 }
        };
        const p = presets[preset];
        if (!p) return;

        Object.keys(p).forEach(key => {
            this.params[key] = p[key];
            this.sliders[key].value = p[key];
            this.values[key].textContent = p[key].toFixed(1);
        });

        document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
        document.querySelector(`[data-preset="${preset}"]`).classList.add('active');
        this.run();
    }

    fopdtStep(t, K, tau, theta, A) {
        if (t < theta) return 0;
        return K * A * (1 - Math.exp(-(t - theta) / tau));
    }

    fopdtRamp(t, K, tau, theta, A) {
        if (t < theta) return 0;
        const td = t - theta;
        return K * A * (td - tau * (1 - Math.exp(-td / tau)));
    }

    initPlots() {
        const layout = {
            paper_bgcolor: 'transparent',
            plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 50, r: 20, t: 10, b: 40 },
            xaxis: { title: 'Time (s)', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { title: 'Output', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.01, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified',
            uirevision: 'true'
        };

        Plotly.newPlot('chart-response', [], { ...layout }, { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-comparison', [], { ...layout }, { responsive: true, displayModeBar: false });
        Plotly.newPlot('chart-ramp', [], { ...layout }, { responsive: true, displayModeBar: false });

        this.currentChart = 'response';
    }

    switchChart(chart) {
        this.currentChart = chart;
        this.chartTabs.forEach(t => t.classList.toggle('active', t.dataset.chart === chart));
        document.getElementById('chart-response').style.display = chart === 'response' ? 'block' : 'none';
        document.getElementById('chart-comparison').style.display = chart === 'comparison' ? 'block' : 'none';
        document.getElementById('chart-ramp').style.display = chart === 'ramp' ? 'block' : 'none';
        Plotly.Plots.resize(document.getElementById('chart-' + chart));
    }

    run() {
        const { gain: K, tau, theta, amplitude: A } = this.params;
        const T = Math.max(theta + tau * 5, 15);
        const dt = 0.02;
        const time = [], response = [], input = [];
        const timeRamp = [], rampResponse = [], rampInput = [];

        for (let t = 0; t <= T; t += dt) {
            time.push(t);
            response.push(this.fopdtStep(t, K, tau, theta, A));
            input.push(A);
        }

        for (let t = 0; t <= T; t += dt) {
            timeRamp.push(t);
            rampResponse.push(this.fopdtRamp(t, K, tau, theta, A));
            rampInput.push(t * A);
        }

        const finalVal = K * A;
        const t63 = theta + tau;
        const t95 = theta + 3 * tau;
        const t98 = theta + 4 * tau;
        const initialSlope = K * A / tau;

        // Step Response
        const stepData = [
            {
                x: time, y: input, name: 'Input (Step)',
                line: { color: '#64748b', width: 2, dash: 'dot' }
            },
            {
                x: time, y: response, name: 'Output',
                line: { color: '#f43f5e', width: 3 },
                fill: 'tozeroy', fillcolor: 'rgba(244,63,94,0.08)'
            },
            {
                x: [t63], y: [finalVal * 0.632], mode: 'markers', name: '63.2%',
                marker: { color: '#10b981', size: 11, symbol: 'circle' }
            },
            {
                x: [t95], y: [finalVal * 0.95], mode: 'markers', name: '95%',
                marker: { color: '#f59e0b', size: 11, symbol: 'diamond' }
            },
            {
                x: [0, T], y: [finalVal, finalVal], name: 'Final Value',
                line: { color: '#94a3b8', width: 1, dash: 'dash' }
            }
        ];

        const stepLayout = {
            paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
            font: { color: '#94a3b8', family: 'Inter' },
            margin: { l: 50, r: 20, t: 10, b: 40 },
            xaxis: { title: 'Time (s)', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            yaxis: { title: 'Output', gridcolor: '#1e293b', zerolinecolor: '#334155' },
            legend: { x: 0.01, y: 0.99, bgcolor: 'rgba(30,41,59,0.9)' },
            hovermode: 'x unified', uirevision: 'true'
        };

        Plotly.react('chart-response', stepData, stepLayout, { responsive: true, displayModeBar: false });

        // Comparison - vary tau
        const taus = [1, 2, 4, 8, 15];
        const colors = ['#06b6d4', '#f59e0b', '#10b981', '#ef4444', '#a78bfa'];
        const compData = taus.map((t, i) => ({
            x: time, y: time.map(tt => this.fopdtStep(tt, K, t, theta, A)),
            name: `τ=${t}`, line: { color: colors[i], width: 2 }
        }));
        compData.push({ x: [0, T], y: [finalVal, finalVal], name: 'Final', line: { color: '#64748b', width: 1, dash: 'dash' } });

        Plotly.react('chart-comparison', compData, {
            ...stepLayout,
            annotations: [{
                x: 0.5, y: 1.05, xref: 'paper', yref: 'paper',
                text: `Effect of varying τ (K=${K}, θ=${theta}s)`,
                showarrow: false, font: { color: '#94a3b8', size: 12 }
            }]
        }, { responsive: true, displayModeBar: false });

        // Ramp Response
        Plotly.react('chart-ramp', [
            {
                x: timeRamp, y: rampInput, name: 'Input (Ramp)',
                line: { color: '#64748b', width: 2, dash: 'dot' }
            },
            {
                x: timeRamp, y: rampResponse, name: 'Output',
                line: { color: '#06b6d4', width: 3 },
                fill: 'tozeroy', fillcolor: 'rgba(6,182,212,0.08)'
            }
        ], stepLayout, { responsive: true, displayModeBar: false });

        // Metrics
        document.getElementById('metric-final').textContent = finalVal.toFixed(2);
        document.getElementById('metric-63').textContent = t63.toFixed(1) + ' s';
        document.getElementById('metric-95').textContent = t95.toFixed(1) + ' s';
        document.getElementById('metric-98').textContent = t98.toFixed(1) + ' s';
        document.getElementById('metric-slope').textContent = initialSlope.toFixed(2);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.fopdtSim = new FOPDTSimulator();
});