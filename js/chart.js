/* ============================================================================
 * MiniChart - a tiny, dependency-free charting library that implements the
 * subset of the Plotly.js API used by the PDC Lab pages, rendering to inline
 * SVG.  It is intentionally self-contained (no CDN, no external fonts/images)
 * so the examples run completely offline.
 *
 * Supported API:
 *   Plotly.newPlot(div, data, layout, config)
 *   Plotly.react(div, data, layout, config)
 *   Plotly.update(div, dataUpdate, layoutUpdate)
 *   Plotly.restyle(div, update, indices)
 *   Plotly.relayout(div, layoutUpdate)
 *   Plotly.Plots.resize(div)
 *
 * Supported trace options:
 *   x, y, name, mode ('lines','markers','lines+markers'), line{color,width,dash},
 *   fill ('tozeroy'), fillcolor, marker{color,size,symbol,line{color,width}},
 *   yaxis ('y'|'y2'), showlegend
 *
 * Supported layout options:
 *   width, height, margin{l,r,t,b}, paper_bgcolor, plot_bgcolor,
 *   font{color,family,size}, xaxis/yaxis/yaxis2 {title,gridcolor,zerolinecolor,
 *   range,scaleanchor,side,overlaying}, legend{x,y,bgcolor}, hovermode,
 *   showlegend, shapes[], annotations[]
 * ==========================================================================*/
(function (global) {
    'use strict';

    const NS = 'http://www.w3.org/2000/svg';
    const charts = new Set();
    const registry = new WeakMap();

    function svgEl(tag, attrs) {
        const e = document.createElementNS(NS, tag);
        if (attrs) for (const k in attrs) e.setAttribute(k, attrs[k]);
        return e;
    }

    /* ---- theme-aware chart chrome ----
       Axis text, gridlines, legends and tooltips are chrome, not data, so they
       come from the active theme's --chart-* tokens. Reading them at render
       time (rather than caching) is what lets a repaint follow the theme
       switch. The fallbacks are the dark values this library originally
       hardcoded, used only if styles.css is missing. */
    const CHROME_FALLBACK = {
        'grid': '#1e293b',
        'zero': '#334155',
        'axis': '#94a3b8',
        'legend-bg': 'rgba(30,41,59,0.85)',
        'legend-border': '#334155',
        'tooltip-bg': 'rgba(15,23,42,0.95)',
        'tooltip-border': '#334155',
        'tooltip-text': '#e2e8f0',
        'hover-line': '#64748b',
        'btn-bg': 'rgba(15,23,42,0.85)',
        'btn-active-bg': 'rgba(56,189,248,0.25)'
    };

    function themeVar(name, fallback) {
        try {
            const v = getComputedStyle(document.documentElement)
                .getPropertyValue(name).trim();
            return v || fallback;
        } catch (e) {
            return fallback;
        }
    }

    function chrome(name) {
        return themeVar('--chart-' + name, CHROME_FALLBACK[name] || '');
    }
    function getEl(div) { return typeof div === 'string' ? document.getElementById(div) : div; }
    function clone(o) { return o == null ? o : JSON.parse(JSON.stringify(o)); }
    function isNum(v) { return typeof v === 'number' && isFinite(v); }
    function deepMerge(a, b) {
        const out = Object.assign({}, a);
        for (const k in b) {
            const bv = b[k];
            if (bv && typeof bv === 'object' && !Array.isArray(bv) && a[k] && typeof a[k] === 'object' && !Array.isArray(a[k])) {
                out[k] = deepMerge(a[k], bv);
            } else {
                out[k] = clone(bv);
            }
        }
        return out;
    }

    function niceTicks(min, max, count) {
        if (!isFinite(min) || !isFinite(max)) { min = 0; max = 1; }
        if (min === max) { min -= 0.5; max += 0.5; }
        const span = max - min;
        const raw = span / Math.max(1, count);
        const mag = Math.pow(10, Math.floor(Math.log10(raw)));
        const norm = raw / mag;
        let step = norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10;
        step *= mag;
        const start = Math.ceil(min / step) * step;
        const ticks = [];
        for (let v = start; v <= max + step * 1e-9; v += step) ticks.push(+v.toFixed(12));
        if (ticks.length > 40) return niceTicks(min, max, Math.ceil(count * 1.5));
        return ticks;
    }

    function fmtNum(v) {
        if (!isFinite(v)) return '';
        const a = Math.abs(v);
        if (a === 0) return '0';
        if (a >= 1e5 || a < 1e-3) return v.toExponential(1);
        if (a >= 100) return v.toFixed(0);
        if (a >= 10) return v.toFixed(a % 1 ? 1 : 0);
        if (a >= 1) return v.toFixed(a % 1 ? 2 : 0);
        return v.toFixed(2);
    }

    function scale(dmin, dmax, pmin, pmax) {
        if (!isFinite(dmin) || !isFinite(dmax)) { dmin = 0; dmax = 1; }
        if (dmin === dmax) { dmin -= 0.5; dmax += 0.5; }
        const s = (pmax - pmin) / (dmax - dmin);
        return { dmin, dmax, pmin, pmax, s, to: v => pmin + (v - dmin) * s };
    }

    function dashArray(d) {
        if (d === 'dash') return '6,4';
        if (d === 'dot') return '2,3';
        if (d === 'dashdot') return '6,3,2,3';
        return '';
    }

    function markerPath(symbol, cx, cy, r) {
        switch (symbol) {
            case 'diamond': return `M${cx},${cy - r} L${cx + r},${cy} L${cx},${cy + r} L${cx - r},${cy} Z`;
            case 'square': return `M${cx - r},${cy - r} h${2 * r} v${2 * r} h${-2 * r} Z`;
            case 'triangle-up': return `M${cx},${cy - r} L${cx + r},${cy + r} L${cx - r},${cy + r} Z`;
            case 'triangle-down': return `M${cx},${cy + r} L${cx + r},${cy - r} L${cx - r},${cy - r} Z`;
            case 'star': {
                let d = '';
                for (let i = 0; i < 10; i++) {
                    const ang = -Math.PI / 2 + i * Math.PI / 5;
                    const rr = i % 2 === 0 ? r * 1.35 : r * 0.6;
                    d += (i ? ' L' : 'M') + (cx + rr * Math.cos(ang)) + ',' + (cy + rr * Math.sin(ang));
                }
                return d + ' Z';
            }
            case 'x': return '';
            case 'circle':
            case 'circle-open':
            default:
                return `M${cx - r},${cy} a${r},${r} 0 1,0 ${2 * r},0 a${r},${r} 0 1,0 ${-2 * r},0 Z`;
        }
    }

    class MiniChart {
        constructor(container) {
            this.container = container;
            this.data = [];
            this.layout = {};
            this.config = {};
            this.hover = null;
            this.fit = false;   // when true, axes are fitted to the full data range
            this._lastRange = null;
            this._onMove = this._onMove.bind(this);
            this._onLeave = this._onLeave.bind(this);
        }

        set(data, layout, config) {
            this.data = (data || []).map(clone);
            this.layout = Object.assign({}, this.layout, clone(layout || {}));
            if (config) this.config = config;
            this.render();
        }

        update(dataUpdate, layoutUpdate) {
            if (dataUpdate) this._applyUpdate(dataUpdate, null);
            if (layoutUpdate) this.layout = deepMerge(this.layout, layoutUpdate);
            this.render();
        }

        restyle(update, indices) {
            this._applyUpdate(update, indices);
            this.render();
        }

        relayout(layoutUpdate) {
            this.layout = deepMerge(this.layout, layoutUpdate);
            this.render();
        }

        _applyUpdate(update, indices) {
            const idxs = (indices && indices.length) ? indices : this.data.map((_, i) => i);
            idxs.forEach((ti, j) => {
                const tr = this.data[ti];
                if (!tr) return;
                for (const k in update) {
                    const v = update[k];
                    if (Array.isArray(v) && v.length === idxs.length && Array.isArray(v[j])) {
                        tr[k] = clone(v[j]);
                    } else {
                        tr[k] = clone(v);
                    }
                }
            });
        }

        resize() { this.render(); }

        _rangeFromData(axis) {
            let min = Infinity, max = -Infinity;
            for (const tr of this.data) {
                const trAxis = tr.yaxis === 'y2' ? 'y2' : 'y';
                const vals = axis === 'x' ? tr.x : (trAxis === axis ? tr.y : null);
                if (!vals) continue;
                for (let i = 0; i < vals.length; i++) {
                    const v = vals[i];
                    if (isNum(v)) { if (v < min) min = v; if (v > max) max = v; }
                }
            }
            if (!isFinite(min)) { min = 0; max = 1; }
            if (min === max) { min -= 0.5; max += 0.5; }
            const pad = (max - min) * 0.06;
            return [min - pad, max + pad];
        }

        // Range that fits all data; the x-axis starts at 0 for non-negative data.
        _fitRange(axis) {
            let min = Infinity, max = -Infinity;
            for (const tr of this.data) {
                const trAxis = tr.yaxis === 'y2' ? 'y2' : 'y';
                const vals = axis === 'x' ? tr.x : (trAxis === axis ? tr.y : null);
                if (!vals) continue;
                for (let i = 0; i < vals.length; i++) {
                    const v = vals[i];
                    if (isNum(v)) { if (v < min) min = v; if (v > max) max = v; }
                }
            }
            if (!isFinite(min)) { min = 0; max = 1; }
            if (min === max) { max = min + 1; }
            if (axis === 'x' && min >= 0) {
                return [0, max + (max - min) * 0.02];
            }
            const pad = (max - min) * 0.05;
            return [min - pad, max + pad];
        }

        render() {
            const c = this.container;
            if (!c) return;
            // Never lay out a hidden chart. A display:none container reports
            // clientWidth 0; the old fallback to c.parentElement.clientWidth
            // includes the parent's padding, so it produced an over-wide SVG.
            // Once that chart was shown, the oversized SVG inflated its grid
            // track, making the next hidden render wider again - the plot boxes
            // grew on every tab switch. Wait until the chart is actually shown
            // (switchChart calls Plotly.Plots.resize) and size to its own box.
            const width = c.clientWidth;
            if (width <= 1) return;
            const height = this.layout.height || c.clientHeight || 350;

            const L = this.layout;
            const font = Object.assign({ color: chrome('axis'), family: 'inherit', size: 11 }, L.font);
            const margin = Object.assign({ l: 55, r: (this._hasY2() ? 55 : 20), t: 12, b: 42 }, L.margin);
            const W = width, H = height;
            const x0 = margin.l, x1 = W - margin.r, y0 = margin.t, y1 = H - margin.b;

            // ---- ranges ----
            const xa = L.xaxis || {}, ya = L.yaxis || {}, ya2 = L.yaxis2 || {};
            let xr, yr, yr2;
            if (this.fit) {
                // "Zoom to fit": show the whole record (x from 0 to the end).
                xr = this._fitRange('x');
                yr = this._fitRange('y');
                yr2 = this._hasY2() ? this._fitRange('y2') : yr;
            } else {
                xr = xa.range ? xa.range.slice() : this._rangeFromData('x');
                yr = ya.range ? ya.range.slice() : this._rangeFromData('y');
                yr2 = ya2.range ? ya2.range.slice() : (this._hasY2() ? this._rangeFromData('y2') : yr);
            }
            this._lastRange = { x: xr.slice(), y: yr.slice(), y2: yr2.slice(), fit: this.fit };

            // equal-aspect support (scaleanchor)
            let xs = scale(xr[0], xr[1], x0, x1);
            let ys = scale(yr[0], yr[1], y1, y0);
            if (ya.scaleanchor === 'x') {
                const s = Math.min((x1 - x0) / (xr[1] - xr[0]), (y1 - y0) / (yr[1] - yr[0]));
                const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
                const hw = (xr[1] - xr[0]) * s / 2, hh = (yr[1] - yr[0]) * s / 2;
                xs = scale(xr[0], xr[1], cx - hw, cx + hw);
                ys = scale(yr[0], yr[1], cy + hh, cy - hh);
            }
            let ys2 = this._hasY2() ? scale(yr2[0], yr2[1], y1, y0) : ys;

            // ---- clear ----
            while (c.firstChild) c.removeChild(c.firstChild);
            const svg = svgEl('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, class: 'mini-chart-svg' });
            svg.style.display = 'block';
            c.appendChild(svg);

            if (L.paper_bgcolor && L.paper_bgcolor !== 'transparent') {
                svg.appendChild(svgEl('rect', { x: 0, y: 0, width: W, height: H, fill: L.paper_bgcolor }));
            }
            if (L.plot_bgcolor && L.plot_bgcolor !== 'transparent') {
                svg.appendChild(svgEl('rect', { x: x0, y: y0, width: x1 - x0, height: y1 - y0, fill: L.plot_bgcolor }));
            }

            const gGrid = svgEl('g');
            const gData = svgEl('g');
            const gShape = svgEl('g');
            const gAxis = svgEl('g');
            const gLegend = svgEl('g');
            const gAnno = svgEl('g');
            svg.appendChild(gGrid); svg.appendChild(gShape); svg.appendChild(gData);
            svg.appendChild(gAxis); svg.appendChild(gAnno); svg.appendChild(gLegend);

            const gridColor = xa.gridcolor || chrome('grid');
            const zeroColor = xa.zerolinecolor || chrome('zero');

            // ---- x grid + ticks ----
            const xTicks = niceTicks(xs.dmin, xs.dmax, Math.max(3, Math.floor((x1 - x0) / 90)));
            for (const tv of xTicks) {
                const px = xs.to(tv);
                if (px < x0 - 1 || px > x1 + 1) continue;
                gGrid.appendChild(svgEl('line', { x1: px, y1: y0, x2: px, y2: y1, stroke: gridColor, 'stroke-width': 1 }));
                const t = svgEl('text', { x: px, y: y1 + 16, fill: font.color, 'font-size': 10, 'text-anchor': 'middle', 'font-family': font.family });
                t.textContent = fmtNum(tv);
                gAxis.appendChild(t);
            }
            // ---- y grid + ticks (left) ----
            const yTicks = niceTicks(ys.dmin, ys.dmax, Math.max(3, Math.floor((y1 - y0) / 45)));
            for (const tv of yTicks) {
                const py = ys.to(tv);
                if (py < y0 - 1 || py > y1 + 1) continue;
                gGrid.appendChild(svgEl('line', { x1: x0, y1: py, x2: x1, y2: py, stroke: gridColor, 'stroke-width': 1 }));
                const t = svgEl('text', { x: x0 - 8, y: py + 3, fill: font.color, 'font-size': 10, 'text-anchor': 'end', 'font-family': font.family });
                t.textContent = fmtNum(tv);
                gAxis.appendChild(t);
            }
            // ---- y2 ticks (right) ----
            if (this._hasY2()) {
                const y2Ticks = niceTicks(ys2.dmin, ys2.dmax, Math.max(3, Math.floor((y1 - y0) / 45)));
                for (const tv of y2Ticks) {
                    const py = ys2.to(tv);
                    if (py < y0 - 1 || py > y1 + 1) continue;
                    const t = svgEl('text', { x: x1 + 8, y: py + 3, fill: (ya2.tickfont && ya2.tickfont.color) || '#a78bfa', 'font-size': 10, 'text-anchor': 'start', 'font-family': font.family });
                    t.textContent = fmtNum(tv);
                    gAxis.appendChild(t);
                }
            }

            // zero lines
            if (xs.dmin < 0 && xs.dmax > 0) gAxis.appendChild(svgEl('line', { x1: xs.to(0), y1: y0, x2: xs.to(0), y2: y1, stroke: zeroColor, 'stroke-width': 1 }));
            if (ys.dmin < 0 && ys.dmax > 0) gAxis.appendChild(svgEl('line', { x1: x0, y1: ys.to(0), x2: x1, y2: ys.to(0), stroke: zeroColor, 'stroke-width': 1 }));

            // ---- axis titles ----
            if (xa.title) {
                const t = svgEl('text', { x: (x0 + x1) / 2, y: H - 6, fill: font.color, 'font-size': 11, 'text-anchor': 'middle', 'font-family': font.family });
                t.textContent = xa.title;
                gAxis.appendChild(t);
            }
            if (ya.title) {
                const t = svgEl('text', { x: 14, y: (y0 + y1) / 2, fill: font.color, 'font-size': 11, 'text-anchor': 'middle', 'font-family': font.family, transform: `rotate(-90 14 ${(y0 + y1) / 2})` });
                t.textContent = ya.title;
                gAxis.appendChild(t);
            }
            if (ya2.title && this._hasY2()) {
                const t = svgEl('text', { x: W - 6, y: (y0 + y1) / 2, fill: '#a78bfa', 'font-size': 11, 'text-anchor': 'middle', 'font-family': font.family, transform: `rotate(90 ${W - 6} ${(y0 + y1) / 2})` });
                t.textContent = ya2.title;
                gAxis.appendChild(t);
            }

            // ---- shapes ----
            const shapes = L.shapes || [];
            for (const sh of shapes) {
                if (sh.type === 'line') {
                    const col = (sh.line && sh.line.color) || '#fbbf24';
                    const wd = (sh.line && sh.line.width) || 1;
                    const da = dashArray(sh.line && sh.line.dash);
                    let xA, xB, yA, yB;
                    if (sh.xref === 'paper' || sh.yref === 'paper') {
                        xA = sh.xref === 'paper' ? x0 + (x1 - x0) * sh.x0 : xs.to(sh.x0);
                        xB = sh.xref === 'paper' ? x0 + (x1 - x0) * sh.x1 : xs.to(sh.x1);
                        yA = sh.yref === 'paper' ? y0 + (y1 - y0) * sh.y0 : ys.to(sh.y0);
                        yB = sh.yref === 'paper' ? y0 + (y1 - y0) * sh.y1 : ys.to(sh.y1);
                    } else {
                        xA = xs.to(sh.x0); xB = xs.to(sh.x1); yA = ys.to(sh.y0); yB = ys.to(sh.y1);
                    }
                    gShape.appendChild(svgEl('line', { x1: xA, y1: yA, x2: xB, y2: yB, stroke: col, 'stroke-width': wd, 'stroke-dasharray': da }));
                }
            }

            // ---- traces ----
            const clip = svgEl('clipPath', { id: 'clip-' + Math.random().toString(36).slice(2) });
            clip.appendChild(svgEl('rect', { x: x0, y: y0, width: Math.max(0, x1 - x0), height: Math.max(0, y1 - y0) }));
            svg.appendChild(clip);
            const clipId = clip.getAttribute('id');

            let legendItems = [];
            this.data.forEach((tr, ti) => {
                if (tr.visible === false) return;
                const onY2 = tr.yaxis === 'y2';
                const sy = onY2 ? ys2 : ys;
                const mode = tr.mode || 'lines';
                const line = Object.assign({ color: '#38bdf8', width: 2 }, tr.line);
                const marker = Object.assign({ color: line.color, size: 7, symbol: 'circle' }, tr.marker);
                const xsArr = tr.x || [];
                const ysArr = tr.y || [];
                const n = Math.min(xsArr.length, ysArr.length);
                const showLegend = tr.showlegend !== false && tr.name;
                if (showLegend) legendItems.push({ name: tr.name, color: line.color, dash: line.dash, symbol: mode.indexOf('markers') >= 0 ? marker.symbol : null });

                // fill to zero
                if (tr.fill === 'tozeroy' && n > 1) {
                    let d = '';
                    for (let i = 0; i < n; i++) {
                        if (!isNum(xsArr[i]) || !isNum(ysArr[i])) continue;
                        d += (d ? ' L' : 'M') + xs.to(xsArr[i]) + ',' + sy.to(ysArr[i]);
                    }
                    if (d) {
                        d += ` L${xs.to(xsArr[n - 1])},${sy.to(0)} L${xs.to(xsArr[0])},${sy.to(0)} Z`;
                        gData.appendChild(svgEl('path', { d, fill: tr.fillcolor || 'rgba(56,189,248,0.12)', stroke: 'none', 'clip-path': `url(#${clipId})` }));
                    }
                }

                // line
                if (mode.indexOf('lines') >= 0 && n > 1) {
                    let d = '';
                    let started = false;
                    for (let i = 0; i < n; i++) {
                        const xv = xsArr[i], yv = ysArr[i];
                        if (!isNum(xv) || !isNum(yv)) { started = false; continue; }
                        d += (started ? ' L' : ' M') + xs.to(xv) + ',' + sy.to(yv);
                        started = true;
                    }
                    if (d) gData.appendChild(svgEl('path', { d, fill: 'none', stroke: line.color, 'stroke-width': line.width, 'stroke-dasharray': dashArray(line.dash), 'stroke-linejoin': 'round', 'stroke-linecap': 'round', 'clip-path': `url(#${clipId})` }));
                }

                // markers
                if (mode.indexOf('markers') >= 0) {
                    for (let i = 0; i < n; i++) {
                        const xv = xsArr[i], yv = ysArr[i];
                        if (!isNum(xv) || !isNum(yv)) continue;
                        const cx = xs.to(xv), cy = sy.to(yv);
                        const ml = marker.line || {};
                        if (marker.symbol === 'x') {
                            const r = marker.size * 0.6;
                            gData.appendChild(svgEl('path', { d: `M${cx - r},${cy - r} L${cx + r},${cy + r} M${cx + r},${cy - r} L${cx - r},${cy + r}`, stroke: marker.color, 'stroke-width': ml.width || 3, fill: 'none', 'clip-path': `url(#${clipId})` }));
                        } else {
                            const d = markerPath(marker.symbol, cx, cy, marker.size);
                            if (d) gData.appendChild(svgEl('path', { d, fill: marker.color, stroke: ml.color || 'none', 'stroke-width': ml.width || 0, 'clip-path': `url(#${clipId})` }));
                        }
                    }
                }
            });

            // ---- annotations ----
            (L.annotations || []).forEach(a => {
                const ax = a.xref === 'paper' ? x0 + (x1 - x0) * a.x : xs.to(a.x);
                const ay = a.yref === 'paper' ? y0 + (y1 - y0) * a.y : ys.to(a.y);
                const f = Object.assign({ color: chrome('axis'), size: 11 }, a.font);
                const t = svgEl('text', { x: ax, y: ay, fill: f.color, 'font-size': f.size, 'text-anchor': 'middle', 'font-family': font.family });
                t.textContent = a.text || '';
                gAnno.appendChild(t);
            });

            // ---- legend ----
            if (L.showlegend !== false && legendItems.length) {
                const lg = L.legend || {};
                const rightSide = lg.x != null && lg.x > 0.5;
                // On the right the legend sits below the Fit button so the two
                // never overlap; on the left it stays at the top-left.
                const yShift = rightSide ? 18 : 0;
                let ly = y0 + 12 + yShift;
                const boxW = 150;
                const bx = rightSide ? x1 - boxW - 6 : x0 + 6;
                const bg = svgEl('rect', { x: bx, y: y0 + 4 + yShift, width: boxW, height: 16 * legendItems.length + 8, rx: 6, fill: lg.bgcolor || chrome('legend-bg'), stroke: chrome('legend-border'), 'stroke-width': 1 });
                gLegend.appendChild(bg);
                legendItems.forEach((it, i) => {
                    const yy = ly + 4 + i * 16;
                    gLegend.appendChild(svgEl('line', { x1: bx + 8, y1: yy, x2: bx + 28, y2: yy, stroke: it.color, 'stroke-width': 3, 'stroke-dasharray': dashArray(it.dash) }));
                    const t = svgEl('text', { x: bx + 34, y: yy + 3, fill: font.color, 'font-size': 10, 'font-family': font.family });
                    t.textContent = it.name;
                    gLegend.appendChild(t);
                });
            }

            // ---- hover ----
            if (L.hovermode) {
                const hoverG = svgEl('g', { class: 'mini-hover', style: 'pointer-events:none' });
                svg.appendChild(hoverG);
                const rect = svgEl('rect', { x: x0, y: y0, width: Math.max(0, x1 - x0), height: Math.max(0, y1 - y0), fill: 'transparent', style: 'cursor:crosshair' });
                svg.appendChild(rect);
                this._hoverInfo = { x0, x1, y0, y1, xs, ys, ys2, hoverG, svg, font };
                rect.addEventListener('mousemove', this._onMove);
                rect.addEventListener('mouseleave', this._onLeave);
            }

            // ---- "Zoom to fit" toggle (top-right of the plot) ----
            const fitBtn = svgEl('g', { style: 'cursor:pointer' });
            const fbx = x1 - 46, fby = y0 + 2;
            fitBtn.appendChild(svgEl('rect', { x: fbx, y: fby, width: 46, height: 17, rx: 5, fill: this.fit ? chrome('btn-active-bg') : chrome('btn-bg'), stroke: this.fit ? chrome('axis') : chrome('legend-border'), 'stroke-width': 1 }));
            const ftx = svgEl('text', { x: fbx + 23, y: fby + 12.5, fill: this.fit ? themeVar('--primary', '#38bdf8') : chrome('axis'), 'font-size': 9.5, 'text-anchor': 'middle', 'font-family': font.family });
            ftx.textContent = this.fit ? 'Auto' : 'Fit';
            fitBtn.appendChild(ftx);
            // Use pointerdown (fires on press) rather than click: the chart is
            // re-rendered every frame while a simulation runs, so the button is
            // recreated between mousedown and mouseup and a 'click' would never
            // complete. pointerdown fires on the element that exists at press.
            fitBtn.addEventListener('pointerdown', (e) => {
                if (e && e.stopPropagation) e.stopPropagation();
                if (e && e.preventDefault) e.preventDefault();
                this.fit = !this.fit;
                this.render();
            });
            svg.appendChild(fitBtn);
        }

        _hasY2() { return this.data.some(t => t.yaxis === 'y2'); }

        _onMove(evt) {
            const hi = this._hoverInfo;
            if (!hi) return;
            const r = hi.svg.getBoundingClientRect();
            const mx = evt.clientX - r.left;
            const my = evt.clientY - r.top;
            if (mx < hi.x0 || mx > hi.x1) { this._onLeave(); return; }
            const xData = hi.xs.dmin + (mx - hi.xs.pmin) / hi.xs.s;

            // nearest point on the first trace that has x
            let best = null, bestD = Infinity;
            this.data.forEach(tr => {
                const xsA = tr.x || [], ysA = tr.y || [];
                const sy = tr.yaxis === 'y2' ? hi.ys2 : hi.ys;
                for (let i = 0; i < xsA.length; i++) {
                    const d = Math.abs(xsA[i] - xData);
                    if (d < bestD) { bestD = d; best = { i, x: xsA[i], y: ysA[i], tr, sy }; }
                }
            });
            if (!best) return;
            hi.hoverG.textContent = '';
            const px = hi.xs.to(best.x);
            hi.hoverG.appendChild(svgEl('line', { x1: px, y1: hi.y0, x2: px, y2: hi.y1, stroke: chrome('hover-line'), 'stroke-width': 1, 'stroke-dasharray': '3,3' }));
            // tooltip listing each trace at that x (if same length) or nearest
            const lines = [];
            this.data.forEach(tr => {
                const xsA = tr.x || [], ysA = tr.y || [];
                if (!tr.name) return;
                let v = null;
                if (xsA.length === (this.data[0].x || []).length) v = ysA[best.i];
                else {
                    let bd = Infinity;
                    for (let i = 0; i < xsA.length; i++) { const d = Math.abs(xsA[i] - xData); if (d < bd) { bd = d; v = ysA[i]; } }
                }
                if (isNum(v)) lines.push((tr.name + ': ' + fmtNum(v)));
            });
            const bw = 150, bh = 15 * lines.length + 22;
            let bx = px + 10; if (bx + bw > hi.x1) bx = px - bw - 10;
            const by = Math.max(hi.y0 + 4, Math.min(my - 10, hi.y1 - bh - 4));
            hi.hoverG.appendChild(svgEl('rect', { x: bx, y: by, width: bw, height: bh, rx: 6, fill: chrome('tooltip-bg'), stroke: chrome('tooltip-border'), 'stroke-width': 1 }));
            const xt = svgEl('text', { x: bx + 8, y: by + 14, fill: chrome('tooltip-text'), 'font-size': 10, 'font-family': hi.font.family });
            xt.textContent = 'x = ' + fmtNum(best.x);
            hi.hoverG.appendChild(xt);
            lines.forEach((ln, i) => {
                const t = svgEl('text', { x: bx + 8, y: by + 28 + i * 14, fill: hi.font.color, 'font-size': 10, 'font-family': hi.font.family });
                t.textContent = ln;
                hi.hoverG.appendChild(t);
            });
        }

        _onLeave() {
            if (this._hoverInfo) this._hoverInfo.hoverG.textContent = '';
        }
    }

    function getChart(div) {
        const e = getEl(div);
        if (!e) return null;
        let ch = registry.get(e);
        if (!ch) { ch = new MiniChart(e); registry.set(e, ch); charts.add(ch); }
        return ch;
    }

    const Plotly = {
        newPlot(div, data, layout, config) { const c = getChart(div); if (c) c.set(data, layout, config); return Promise.resolve(); },
        react(div, data, layout, config) { const c = getChart(div); if (c) c.set(data, layout, config); return Promise.resolve(); },
        update(div, dataUpdate, layoutUpdate) { const c = getChart(div); if (c) c.update(dataUpdate, layoutUpdate); return Promise.resolve(); },
        restyle(div, update, indices) { const c = getChart(div); if (c) c.restyle(update, indices); return Promise.resolve(); },
        relayout(div, layoutUpdate) { const c = getChart(div); if (c) c.relayout(layoutUpdate); return Promise.resolve(); },
        Plots: { resize(div) { const ch = registry.get(getEl(div)); if (ch) ch.resize(); } }
    };

    if (typeof window !== 'undefined') {
        window.Plotly = Plotly;
        window.MiniChart = MiniChart;
        window.__miniCharts = charts;
        let rt = null;
        window.addEventListener('resize', () => {
            clearTimeout(rt);
            rt = setTimeout(() => charts.forEach(ch => ch.render()), 120);
        });

        // Repaint every chart with the new palette when the theme flips.
        // theme.js fires this after it has written the data-theme attribute,
        // so chrome() reads the tokens that are actually in effect.
        document.addEventListener('themechange', () => {
            charts.forEach(ch => {
                if (ch.container && ch.container.clientWidth > 1) ch.render();
            });
        });
    }
    global.Plotly = Plotly;
})(typeof window !== 'undefined' ? window : globalThis);
