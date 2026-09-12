/* ============================================================================
 * Self-contained icon set.
 * Replaces Font Awesome with inline SVG data-URIs (no external requests, no
 * third-party font licensing).  Keeps the original `<i class="fas fa-...">`
 * markup: each icon is applied as a CSS mask so it inherits `currentColor`.
 * ==========================================================================*/
(function () {
    'use strict';
    var ICONS = {
        'fa-wave-square': '<path d="M2 16h5V8h5v8h5V8h5" fill="none" stroke="black" stroke-width="2.4" stroke-linejoin="round"/>',
        'fa-water': '<path d="M12 3C12 3 5.5 11 5.5 15a6.5 6.5 0 0 0 13 0C18.5 11 12 3 12 3Z"/>',
        'fa-fire': '<path d="M12 2c1.5 4 6 6 6 11a6 6 0 0 1-12 0c0-3.5 3-5.5 4-8.5.8 1.8 2 2.5 2 4.5 1-2 .5-5 0-7Z"/>',
        'fa-flask': '<path d="M9 3h6v5l4 10a2 2 0 0 1-2 3H7a2 2 0 0 1-2-3L9 8Z"/>',
        'fa-university': '<path d="M12 2 22 8H2Z"/><rect x="4" y="10" width="2.4" height="8"/><rect x="9" y="10" width="2.4" height="8"/><rect x="14" y="10" width="2.4" height="8"/><rect x="19" y="10" width="2.4" height="8"/><rect x="2" y="19" width="20" height="2.4"/>',
        'fa-chart-area': '<path d="M3 20V15l5-4 5 3 8-8v14Z"/>',
        'fa-sliders-h': '<rect x="3" y="6" width="18" height="2"/><circle cx="9" cy="7" r="3"/><rect x="3" y="16" width="18" height="2"/><circle cx="15" cy="17" r="3"/>',
        'fa-microchip': '<path fill-rule="evenodd" d="M6 6h12v12H6Zm3 3h6v6H9Z"/><rect x="3" y="8" width="3" height="1.6"/><rect x="3" y="11.2" width="3" height="1.6"/><rect x="3" y="14.4" width="3" height="1.6"/><rect x="18" y="8" width="3" height="1.6"/><rect x="18" y="11.2" width="3" height="1.6"/><rect x="18" y="14.4" width="3" height="1.6"/>',
        'fa-thermometer-half': '<rect x="10" y="2.5" width="4" height="13" rx="2"/><circle cx="12" cy="18" r="3.4"/>',
        'fa-balance-scale': '<rect x="11" y="2" width="2" height="17"/><rect x="5" y="20" width="14" height="2"/><path d="M12 4 4 9h16Z"/><path d="M2.5 9h7l-3.5 5Z"/><path d="M14.5 9h7l-3.5 5Z"/>',
        'fa-clock': '<circle cx="12" cy="12" r="8.5" fill="none" stroke="black" stroke-width="2"/><rect x="11" y="6" width="2" height="7"/><rect x="12" y="11" width="5.5" height="2"/>',
        'fa-bolt': '<path d="M13 2 4 14h7l-2 8 11-13h-7Z"/>',
        'fa-tachometer-alt': '<path d="M4 13a8 8 0 0 1 16 0Z"/><rect x="11.2" y="6" width="1.6" height="8" transform="rotate(35 12 13)"/><circle cx="12" cy="13" r="1.6"/>',
        'fa-play': '<path d="M6 4 20 12 6 20Z"/>',
        'fa-pause': '<rect x="6" y="4" width="4.5" height="16"/><rect x="13.5" y="4" width="4.5" height="16"/>',
        'fa-redo-alt': '<path d="M19 12a7 7 0 1 1-2-5" fill="none" stroke="black" stroke-width="2.4"/><path d="M17 3v5h-5Z"/>',
        'fa-arrow-up': '<path d="M12 4 20 14h-5v6H9v-6H4Z"/>',
        'fa-arrow-down': '<path d="M12 20 4 10h5V4h6v6h5Z"/>',
        'fa-bars': '<rect x="3" y="5" width="18" height="2.6"/><rect x="3" y="10.7" width="18" height="2.6"/><rect x="3" y="16.4" width="18" height="2.6"/>',
        'fa-times': '<path d="M6 4l6 6 6-6 2 2-6 6 6 6-2 2-6-6-6 6-2-2 6-6-6-6Z"/>',
        'fa-check-circle': '<path fill-rule="evenodd" d="M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20Zm-1.5 14.5-5-5L7 10l3.5 3.5L17 7l1.5 1.5Z"/>',
        'fa-minus-circle': '<path fill-rule="evenodd" d="M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20ZM7 11h10v2H7Z"/>',
        'fa-book': '<path d="M4 3h7a2 2 0 0 1 2 2v15a3 3 0 0 0-2.5-2H4Z"/><path d="M20 3h-7a2 2 0 0 0-2 2v15a3 3 0 0 1 2.5-2H20Z"/>',
        'fa-external-link-alt': '<path d="M14 3h7v7h-2V6.4l-8.3 8.3-1.4-1.4L17.6 5H14Z"/><path d="M19 14v5a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V7a2 2 0 0 1 2-2h5v2H5v12h12v-5Z"/>'
    };

    var css = '';
    for (var name in ICONS) {
        if (!Object.prototype.hasOwnProperty.call(ICONS, name)) continue;
        var svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">' + ICONS[name] + '</svg>';
        var uri = 'data:image/svg+xml,' + encodeURIComponent(svg);
        css += '.' + name + '::before{-webkit-mask-image:url("' + uri + '");mask-image:url("' + uri + '");}';
    }
    css +=
        'i.fas,i.far,i.fab,i.fa{display:inline-block;width:1em;height:1em;vertical-align:-0.125em;line-height:1;}' +
        'i.fas::before,i.far::before,i.fab::before,i.fa::before{content:"";display:block;width:100%;height:100%;background-color:currentColor;' +
        '-webkit-mask-repeat:no-repeat;mask-repeat:no-repeat;-webkit-mask-position:center;mask-position:center;-webkit-mask-size:contain;mask-size:contain;}';

    var style = document.createElement('style');
    style.setAttribute('data-icons', 'local');
    style.textContent = css;
    (document.head || document.documentElement).appendChild(style);
})();
