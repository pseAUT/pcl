/* ============================================================
   Theme controller

   - Light is the default. A stored choice wins on later visits.
   - The `data-theme` attribute is set on <html> as early as possible
     (this script is loaded synchronously in <head>) so there is no
     flash of the wrong theme.
   - The toggle button lives in the navbar, which is parsed after this
     script runs, so binding waits for DOMContentLoaded.
   - Charts listen for the "themechange" event to repaint with the new
     palette.
   ============================================================ */
(function () {
  'use strict';

  var KEY = 'pdc-theme';
  var root = document.documentElement;
  var LIGHT = 'light';
  var DARK = 'dark';

  /* ---------- read / write ---------- */

  function stored() {
    try {
      var v = localStorage.getItem(KEY);
      return (v === DARK || v === LIGHT) ? v : null;
    } catch (e) {
      return null; // storage blocked (private mode, file://, etc.)
    }
  }

  function persist(theme) {
    try { localStorage.setItem(KEY, theme); } catch (e) { /* ignore */ }
  }

  function current() {
    return root.getAttribute('data-theme') === DARK ? DARK : LIGHT;
  }

  /* ---------- apply ---------- */

  function apply(theme, notify) {
    var changed = current() !== theme;
    root.setAttribute('data-theme', theme);
    syncButton(theme);

    // Let the browser paint form controls / scrollbars to match
    root.style.colorScheme = theme;

    if (notify && changed) {
      document.dispatchEvent(new CustomEvent('themechange', {
        detail: { theme: theme }
      }));
    }
  }

  function syncButton(theme) {
    var btn = document.querySelector('.theme-toggle');
    if (!btn) return;
    var dark = theme === DARK;
    // The button is role="switch", so state is exposed via aria-checked
    btn.setAttribute('aria-checked', dark ? 'true' : 'false');
    btn.setAttribute('aria-label', dark ? 'Switch to light theme' : 'Switch to dark theme');
    btn.setAttribute('title', dark ? 'Switch to light theme' : 'Switch to dark theme');
  }

  /* ---------- init ----------
     Runs immediately (still in <head>) so the first paint is correct.
     The HTML ships with data-theme="light", so an unset key keeps light. */

  apply(stored() || LIGHT, false);

  /* ---------- wiring ---------- */

  function bind() {
    var btn = document.querySelector('.theme-toggle');
    if (!btn) return;

    syncButton(current());

    btn.addEventListener('click', function () {
      var next = current() === DARK ? LIGHT : DARK;
      persist(next);
      apply(next, true);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bind);
  } else {
    bind();
  }

  /* ---------- public API ----------
     Charts and page scripts read palette values through this so a
     repaint always picks up the active theme's tokens. */

  window.PDCTheme = {
    current: current,

    isDark: function () { return current() === DARK; },

    /** Read a CSS custom property from :root, with a fallback. */
    token: function (name, fallback) {
      var v = getComputedStyle(root).getPropertyValue(name);
      v = (v || '').trim();
      return v || fallback || '';
    },

    /** Resolve a chart token, e.g. color('grid') -> --chart-grid. */
    color: function (name, fallback) {
      return window.PDCTheme.token('--chart-' + name, fallback);
    },

    set: function (theme) {
      persist(theme);
      apply(theme, true);
    },

    toggle: function () {
      window.PDCTheme.set(current() === DARK ? LIGHT : DARK);
    }
  };
})();
