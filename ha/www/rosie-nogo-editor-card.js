/**
 * ROSie No-Go Line Editor — Custom Lovelace Card
 *
 * Displays the ROSie map and allows drawing no-go lines by clicking
 * two points on the map. Lines are sent to the Pi driver via MQTT.
 *
 * Config:
 *   type: custom:rosie-nogo-editor-card
 *   camera_entity: camera.rosie_rosie_map    # optional, default shown
 *   meta_topic: rosie/map_meta               # optional
 *   nogo_set_topic: rosie/nogo_lines/set     # optional
 *   nogo_topic: rosie/nogo_lines             # optional
 */

const NOGO_COLOUR = "#dc2828";
const NOGO_COLOUR_PREVIEW = "rgba(220,40,40,0.5)";
const NOGO_COLOUR_SELECTED = "#2196f3";
const ENDPOINT_RADIUS = 6;

class RosieNogoEditorCard extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this._hass = null;
    this._config = {};
    this._meta = null;
    this._lines = [];          // committed lines [{p1:[x,y], p2:[x,y]}, ...]
    this._pendingPoint = null; // first click in drawing mode [mx, my]
    this._drawing = false;
    this._built = false;
    this._dragging = null;     // {lineIdx, endpoint, startX, startY} during drag
    this._dragMoved = false;   // true if mouseup followed a real drag (suppress click)
    this._cursorPos = null;    // current cursor display coords [x,y] for live preview
    this._editing = false;     // edit mode — allows drag + delete
    this._selectedLine = null;  // index of selected line in edit mode
    this._zoom = 1;             // current zoom level
    this._panX = 0;             // pan offset X (px, layout space)
    this._panY = 0;             // pan offset Y (px, layout space)
    this._pinchStartDist = null;
    this._pinchStartZoom = null;
    this._pinchStartMid = null;
    this._pinchStartPan = null;
    this._lastPanTouch = null;  // for one-finger pan when zoomed
    this._mousePanning = false;  // true when middle/left-click panning with mouse
    this._mousePanStart = null;  // {clientX, clientY, panX, panY} at drag start
    this._doubleTapTime = 0;    // last tap timestamp for double-tap reset
    this._pendingSaveSnapshot = null;   // JSON snapshot of lines we just saved
    this._pendingSaveTimer = null;      // fallback timeout to clear snapshot
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._built) this._build();
    // Only use _updateImage for the very first load; once the periodic
    // timer is running _forceRefreshImage handles everything so we avoid
    // a race where the entity-based URL loads a browser-cached stale image.
    if (!this._refreshTimer) {
      this._updateImage();
    }
    this._loadMetaFromSensor();
    this._loadLinesFromMeta();

    // Periodic image refresh (every 2s)
    if (!this._refreshTimer) {
      this._refreshTimer = setInterval(() => this._forceRefreshImage(), 2000);
      this._countdownTimer = setInterval(() => {
        if (this._refreshCountdown > 0) this._refreshCountdown--;
        if (this._countdownEl) this._countdownEl.textContent = this._refreshCountdown;
      }, 1000);
    }
  }

  setConfig(config) {
    this._config = {
      camera_entity: config.camera_entity || "camera.rosie_rosie_map",
      meta_topic: config.meta_topic || "rosie/map_meta",
      nogo_set_topic: config.nogo_set_topic || "rosie/nogo_lines/set",
      nogo_topic: config.nogo_topic || "rosie/nogo_lines",
      nogo_entity: config.nogo_entity || "sensor.rosie_nogo_line_count",
      ...config,
    };
  }

  static getStubConfig() {
    return { camera_entity: "camera.rosie_rosie_map" };
  }

  getCardSize() {
    return 6;
  }

  // ── Build DOM ──────────────────────────────────────────────────

  _build() {
    if (this._built) return;
    this._built = true;

    const style = document.createElement("style");
    style.textContent = `
      :host { display: block; }
      .card { background: var(--ha-card-background, var(--card-background-color, white));
              border-radius: var(--ha-card-border-radius, 12px);
              box-shadow: var(--ha-card-box-shadow, none);
              overflow: hidden; padding: 16px;
              transition: all 0.2s ease; }
      .card.fullscreen { position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
                          z-index: 9999; border-radius: 0; padding: 12px;
                          display: flex; flex-direction: column;
                          background: var(--ha-card-background, var(--card-background-color, white)); }
      .card.fullscreen .map-wrap { flex: 1; min-height: 0; overflow: hidden; }
      .card.fullscreen .line-list { max-height: 120px; overflow-y: auto; }
      .header { font-size: 1.1em; font-weight: 500; margin-bottom: 12px;
                 display: flex; align-items: center; gap: 8px; }
      .header ha-icon { --mdc-icon-size: 24px; }
      .map-wrap { position: relative; width: 100%;
                  border-radius: 8px; overflow: hidden; background: #f0f5fa;
                  line-height: 0; touch-action: none; }
      .map-wrap.drawing { cursor: crosshair; }
      .map-wrap:not(.drawing) { cursor: default; }
      .map-inner { position: relative; width: 100%; line-height: 0;
                   transform-origin: 0 0; will-change: transform; }
      .map-img { width: 100%; display: block; }
      .map-canvas { position: absolute; top: 0; left: 0;
                    width: 100%; height: 100%; background: transparent; }
      .map-canvas.passthrough { pointer-events: none; }
      .toolbar { display: flex; gap: 8px; margin-top: 12px; flex-wrap: wrap; }
      .toolbar button {
        border: none; border-radius: 8px; padding: 8px 16px;
        font-size: 0.9em; cursor: pointer; font-weight: 500;
        transition: background 0.15s;
      }
      .btn-draw { background: #2196f3; color: white; }
      .btn-draw:hover { background: #1976d2; }
      .btn-draw.active { background: #dc2828; color: white; }
      .btn-save { background: #4caf50; color: white; }
      .btn-save:hover { background: #388e3c; }
      .btn-save:disabled { background: #ccc; color: #888; cursor: default; }
      .btn-clear { background: #ff9800; color: white; }
      .btn-clear:hover { background: #f57c00; }
      .btn-edit { background: #7c4dff; color: white; }
      .btn-edit:hover { background: #651fff; }
      .btn-edit.active { background: #dc2828; color: white; }
      .btn-edit:disabled { background: #ccc; color: #888; cursor: default; }
      .btn-cancel { background: #9e9e9e; color: white; }
      .btn-cancel:hover { background: #757575; }
      .line-list { margin-top: 12px; }
      .line-item { display: flex; align-items: center; gap: 8px;
                   padding: 6px 8px; border-radius: 6px;
                   background: var(--secondary-background-color, #f5f5f5);
                   margin-bottom: 4px; font-size: 0.85em;
                   transition: background 0.15s, box-shadow 0.15s; }
      .line-item.selected { background: #e3f2fd; box-shadow: inset 3px 0 0 #2196f3; }
      .line-item.selectable { cursor: pointer; }
      .line-item.selectable:hover { background: #f0f4ff; }
      .line-item .idx { font-weight: 600; color: #dc2828;
                        min-width: 20px; }
      .line-item .coords { flex: 1; font-family: monospace; }
      .line-item .del-btn { background: none; border: none;
                            cursor: pointer; color: #999; font-size: 1.2em;
                            padding: 2px 6px; border-radius: 4px; }
      .line-item .del-btn:hover { background: #ffebee; color: #dc2828; }
      .status { font-size: 0.8em; color: var(--secondary-text-color, #888);
                margin-top: 8px; }
      .hint { font-size: 0.85em; color: #2196f3; margin-top: 8px;
              font-style: italic; }
      .countdown { position: absolute; top: 6px; right: 6px;
                   background: rgba(0,0,0,0.55); color: white;
                   font-size: 0.7em; font-weight: 600;
                   padding: 2px 6px; border-radius: 10px;
                   pointer-events: none; font-variant-numeric: tabular-nums;
                   min-width: 18px; text-align: center; }
      .btn-fullscreen { background: none; border: none; cursor: pointer;
                        color: var(--secondary-text-color, #888); font-size: 1.2em;
                        padding: 4px; margin-left: auto; display: flex;
                        align-items: center; }
      .btn-fullscreen:hover { color: var(--primary-text-color, #333); }
      .card.fullscreen .btn-fullscreen { position: fixed;
                                          top: clamp(8px, 1.5vh, 20px);
                                          right: clamp(8px, 2vw, 32px);
                                          background: var(--card-background-color, white);
                                          border-radius: 50%; margin-left: 0;
                                          width: clamp(36px, 5vw, 48px);
                                          height: clamp(36px, 5vw, 48px);
                                          font-size: clamp(1.1em, 2.5vw, 1.6em);
                                          justify-content: center;
                                          box-shadow: 0 2px 8px rgba(0,0,0,0.2);
                                          z-index: 10;
                                          color: var(--primary-text-color, #333); }
      .card.fullscreen .btn-fullscreen:hover { background: var(--secondary-background-color, #eee); }
      .card.fullscreen .countdown { display: none; }
    `;

    const card = document.createElement("div");
    card.className = "card";
    card.innerHTML = `
      <div class="header">
        <ha-icon icon="mdi:vector-line"></ha-icon>
        No-Go Lines
        <button class="btn-fullscreen" id="btnFullscreen" title="Toggle fullscreen">⛶</button>
      </div>
      <div class="map-wrap" id="mapWrap">
        <div class="map-inner" id="mapInner">
          <img class="map-img" id="mapImg" />
          <canvas class="map-canvas" id="mapCanvas"></canvas>
        </div>
        <span class="countdown" id="countdown">5</span>
      </div>
      <div class="hint" id="hint" style="display:none"></div>
      <div class="toolbar">
        <button class="btn-draw" id="btnDraw">Draw Line</button>
        <button class="btn-edit" id="btnEdit" disabled>Edit Lines</button>
        <button class="btn-save" id="btnSave" disabled>Save</button>
        <button class="btn-clear" id="btnClear">Clear All</button>
      </div>
      <div class="line-list" id="lineList"></div>
      <div class="status" id="status"></div>
    `;

    this.shadowRoot.appendChild(style);
    this.shadowRoot.appendChild(card);

    // References
    this._card = card;
    this._mapWrap = card.querySelector("#mapWrap");
    this._mapInner = card.querySelector("#mapInner");
    this._mapImg = card.querySelector("#mapImg");
    this._mapCanvas = card.querySelector("#mapCanvas");
    this._hint = card.querySelector("#hint");
    this._btnDraw = card.querySelector("#btnDraw");
    this._btnEdit = card.querySelector("#btnEdit");
    this._btnSave = card.querySelector("#btnSave");
    this._btnClear = card.querySelector("#btnClear");
    this._btnFullscreen = card.querySelector("#btnFullscreen");
    this._lineList = card.querySelector("#lineList");
    this._statusEl = card.querySelector("#status");
    this._countdownEl = card.querySelector("#countdown");
    this._fullscreen = false;
    this._refreshCountdown = 2;

    // Events
    this._mapWrap.addEventListener("click", (e) => this._onMapClick(e));
    this._mapWrap.addEventListener("contextmenu", (e) => {
      if (this._drawing && this._pendingPoint) {
        e.preventDefault();
        this._pendingPoint = null;
        this._cursorPos = null;
        this._hint.textContent = "Click the first point on the map";
        this._redrawCanvas();
      }
    });
    this._btnDraw.addEventListener("click", () => this._toggleDrawMode());
    this._btnEdit.addEventListener("click", () => this._toggleEditMode());
    this._btnSave.addEventListener("click", () => this._saveLines());
    this._btnClear.addEventListener("click", () => this._clearAll());
    this._btnFullscreen.addEventListener("click", () => this._toggleFullscreen());

    // Escape key to exit fullscreen
    this._keyHandler = (e) => {
      if (e.key === "Escape" && this._fullscreen) {
        this._toggleFullscreen();
      }
    };
    document.addEventListener("keydown", this._keyHandler);

    // Mouse wheel zoom
    this._mapWrap.addEventListener("wheel", (e) => this._onWheel(e), { passive: false });

    // Drag editing — canvas intercepts mouse when not in draw mode
    this._mapCanvas.addEventListener("mousedown", (e) => this._onCanvasMouseDown(e));
    this._mapWrap.addEventListener("mousemove", (e) => this._onMouseMove(e));
    this._mapWrap.addEventListener("mouseup", (e) => this._onMouseUp(e));
    this._mapWrap.addEventListener("mouseleave", () => {
      this._cursorPos = null;
      if (this._mousePanning) { this._mousePanning = false; this._mousePanStart = null; }
      if (!this._dragging) this._redrawCanvas();
    });

    // Touch events for mobile
    this._mapCanvas.addEventListener("touchstart", (e) => this._onTouchStart(e), { passive: false });
    this._mapWrap.addEventListener("touchmove", (e) => this._onTouchMove(e), { passive: false });
    this._mapWrap.addEventListener("touchend", (e) => this._onTouchEnd(e));

    // Resize observer for canvas
    this._resizeObs = new ResizeObserver(() => this._resizeCanvas());
    this._resizeObs.observe(this._mapWrap);
  }

  // ── Image & Metadata ──────────────────────────────────────────

  _updateImage() {
    if (!this._hass || !this._mapImg) return;
    const entity = this._hass.states[this._config.camera_entity];
    if (!entity) return;

    // Use proxy URL — updates each time entity changes
    const url = `/api/camera_proxy/${this._config.camera_entity}?token=${entity.attributes.access_token}&t=${entity.last_updated}`;
    if (this._mapImg.src !== url && this._mapImg.getAttribute("data-url") !== url) {
      this._mapImg.setAttribute("data-url", url);
      this._mapImg.src = url;
      this._mapImg.onload = () => {
        this._resizeCanvas();
        this._redrawCanvas();
      };
    }
  }

  _forceRefreshImage() {
    this._refreshCountdown = 2;
    if (this._countdownEl) this._countdownEl.textContent = "2";
    if (!this._hass || !this._mapImg) return;
    const entity = this._hass.states[this._config.camera_entity];
    if (!entity) return;
    const url = `/api/camera_proxy/${this._config.camera_entity}?token=${entity.attributes.access_token}&t=${Date.now()}`;
    this._mapImg.setAttribute("data-url", url);
    this._mapImg.src = url;
    this._mapImg.onload = () => {
      this._resizeCanvas();
      this._redrawCanvas();
    };
  }

  _toggleFullscreen() {
    this._fullscreen = !this._fullscreen;
    this._card.classList.toggle("fullscreen", this._fullscreen);
    this._btnFullscreen.textContent = this._fullscreen ? "\u2715" : "\u26F6";
    this._btnFullscreen.title = this._fullscreen ? "Exit fullscreen" : "Toggle fullscreen";
    if (!this._fullscreen) this._resetZoom();
    // Let layout settle, then resize canvas
    requestAnimationFrame(() => {
      this._resizeCanvas();
      this._redrawCanvas();
    });
  }

  // ── Zoom / Pan ────────────────────────────────────────────────

  _applyTransform() {
    if (!this._mapInner) return;
    this._mapInner.style.transform =
      `translate(${this._panX}px, ${this._panY}px) scale(${this._zoom})`;
  }

  _resetZoom() {
    this._zoom = 1;
    this._panX = 0;
    this._panY = 0;
    this._applyTransform();
  }

  _clampPan() {
    const wrap = this._mapWrap;
    const img = this._mapImg;
    if (!wrap || !img) return;
    const wW = wrap.clientWidth;
    const wH = wrap.clientHeight;
    const iW = img.clientWidth * this._zoom;
    const iH = img.clientHeight * this._zoom;
    if (iW <= wW) {
      this._panX = (wW - iW) / 2;
    } else {
      this._panX = Math.min(0, Math.max(wW - iW, this._panX));
    }
    if (iH <= wH) {
      this._panY = (wH - iH) / 2;
    } else {
      this._panY = Math.min(0, Math.max(wH - iH, this._panY));
    }
  }

  _onWheel(e) {
    e.preventDefault();
    const rect = this._mapWrap.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const factor = e.deltaY < 0 ? 1.15 : 0.87;
    const newZoom = Math.max(1, Math.min(6, this._zoom * factor));
    // Zoom toward cursor
    this._panX = cx - (cx - this._panX) * (newZoom / this._zoom);
    this._panY = cy - (cy - this._panY) * (newZoom / this._zoom);
    this._zoom = newZoom;
    this._clampPan();
    this._applyTransform();
  }

  _loadMetaFromSensor() {
    // Try to load from the MQTT sensor that echoes map_meta
    if (!this._hass) return;

    // HA entity ID depends on device/name slugification — try common patterns
    const candidates = [
      "sensor.rosie_map_metadata",
      "sensor.rosie_map_meta",
      "sensor.rosie_rosie_map_meta",
      "sensor.rosie_rosie_map_metadata",
    ];
    for (const id of candidates) {
      const entity = this._hass.states[id];
      if (entity && entity.attributes && entity.attributes.origin) {
        this._meta = entity.attributes;
        this._statusEl.textContent = `Map metadata loaded from ${id}`;
        return;
      }
    }
    // Fallback: scan all sensors for one with origin attribute
    for (const [id, entity] of Object.entries(this._hass.states)) {
      if (id.startsWith("sensor.") && entity.attributes && entity.attributes.origin
          && entity.attributes.resolution && entity.attributes.scale) {
        this._meta = entity.attributes;
        this._statusEl.textContent = `Map metadata loaded from ${id}`;
        return;
      }
    }
    // Debug: list all rosie sensor entities to help find the right one
    const rosieEntities = Object.keys(this._hass.states)
      .filter((id) => id.includes("rosie") && id.includes("map"))
      .join(", ");
    this._statusEl.textContent = `No metadata found. Map entities: ${rosieEntities || "none"}`;
  }

  _loadLinesFromMeta() {
    const dbg = [];
    if (!this._meta) { dbg.push("no meta"); this._setDebug(dbg); return; }
    if (this._dirty) { dbg.push("dirty=true, skipping"); this._setDebug(dbg); return; }

    const incoming = this._meta.nogo_lines;
    dbg.push(`meta.nogo_lines type=${typeof incoming}, isArray=${Array.isArray(incoming)}`);
    if (incoming !== undefined) dbg.push(`len=${Array.isArray(incoming) ? incoming.length : 'N/A'}`);

    if (!Array.isArray(incoming)) {
      dbg.push("not an array, bail");
      // List all meta keys for debugging
      dbg.push(`meta keys: ${Object.keys(this._meta).join(', ')}`);
      this._setDebug(dbg);
      return;
    }

    const incomingJson = JSON.stringify(incoming);
    dbg.push(`incoming: ${incomingJson.substring(0, 120)}`);

    if (this._pendingSaveSnapshot !== null) {
      dbg.push(`pendingSave: ${this._pendingSaveSnapshot.substring(0, 80)}`);
      if (incomingJson === this._pendingSaveSnapshot) {
        this._pendingSaveSnapshot = null;
        if (this._pendingSaveTimer) { clearTimeout(this._pendingSaveTimer); this._pendingSaveTimer = null; }
        dbg.push("snapshot matched, cleared");
      } else {
        dbg.push("snapshot mismatch, waiting");
        this._setDebug(dbg);
        return;
      }
    }

    const currentJson = JSON.stringify(this._lines);
    dbg.push(`current: ${currentJson.substring(0, 120)}`);

    if (incomingJson !== currentJson) {
      this._lines = incoming.map((l) => {
        if (l.p1) return { p1: l.p1, p2: l.p2 };
        return { p1: l[0], p2: l[1] };
      });
      this._dirty = false;
      this._updateEditButton();
      this._renderLineList();
      this._redrawCanvas();
      dbg.push(`UPDATED to ${this._lines.length} line(s)`);
    } else {
      dbg.push("no change");
    }
    this._setDebug(dbg);
  }

  _setDebug(lines) {
    console.log('[rosie-nogo]', lines.join(' | '));
  }

  disconnectedCallback() {
    if (this._refreshTimer) { clearInterval(this._refreshTimer); this._refreshTimer = null; }
    if (this._countdownTimer) { clearInterval(this._countdownTimer); this._countdownTimer = null; }
    if (this._keyHandler) { document.removeEventListener("keydown", this._keyHandler); }
  }

  // ── Coordinate Transforms ─────────────────────────────────────

  /**
   * Convert a display-pixel click on the <img> to map-frame metres.
   * Inverse of Python map_to_pixel() from clean_map.py.
   */
  _pixelToMap(displayX, displayY) {
    if (!this._meta) return null;
    const m = this._meta;
    const img = this._mapImg;
    if (!img.naturalWidth) return null;

    // Scale from CSS display size → natural image pixels
    const sx = img.naturalWidth / img.clientWidth;
    const sy = img.naturalHeight / img.clientHeight;
    let px = displayX * sx;
    let py = displayY * sy;

    // Undo border + crop
    px = px - m.border + m.crop_c;
    py = py - m.border + m.crop_r;

    // Undo scale
    px = px / m.scale;
    py = py / m.scale;

    // Undo rotation (rotate by -angle around post-rotation centre)
    const angle = m.angle || 0;
    if (Math.abs(angle) > 0.1) {
      const rad = angle * Math.PI / 180;
      const cos_a = Math.cos(rad);
      const sin_a = Math.sin(rad);
      const cx = m.post_rot_w / 2;
      const cy = m.post_rot_h / 2;
      const dx = px - cx;
      const dy = py - cy;
      // Inverse rotation: rotate by -angle
      px = cos_a * dx + sin_a * dy + m.pre_rot_w / 2;
      py = -sin_a * dx + cos_a * dy + m.pre_rot_h / 2;
    }

    // PGM pixel → map metres
    const [ox, oy] = m.origin;
    const mx = px * m.resolution + ox;
    const my = (m.map_h - 1 - py) * m.resolution + oy;

    return [Math.round(mx * 1000) / 1000, Math.round(my * 1000) / 1000];
  }

  /**
   * Convert map-frame metres to display pixels (forward transform).
   * Mirrors Python map_to_pixel().
   */
  _mapToPixel(mx, my) {
    if (!this._meta) return null;
    const m = this._meta;
    const img = this._mapImg;
    if (!img.naturalWidth) return null;

    const [ox, oy] = m.origin;
    let px = (mx - ox) / m.resolution;
    let py = (m.map_h - 1) - (my - oy) / m.resolution;

    const angle = m.angle || 0;
    if (Math.abs(angle) > 0.1) {
      const rad = angle * Math.PI / 180;
      const cos_a = Math.cos(rad);
      const sin_a = Math.sin(rad);
      const cx = m.pre_rot_w / 2;
      const cy = m.pre_rot_h / 2;
      const dx = px - cx;
      const dy = py - cy;
      px = cos_a * dx - sin_a * dy + m.post_rot_w / 2;
      py = sin_a * dx + cos_a * dy + m.post_rot_h / 2;
    }

    px = px * m.scale;
    py = py * m.scale;

    px = px - m.crop_c + m.border;
    py = py - m.crop_r + m.border;

    // Scale from natural image pixels → CSS display pixels
    const sx = img.clientWidth / img.naturalWidth;
    const sy = img.clientHeight / img.naturalHeight;
    return [px * sx, py * sy];
  }

  // ── Drawing ───────────────────────────────────────────────────

  _toggleDrawMode() {
    this._drawing = !this._drawing;
    this._pendingPoint = null;
    this._cursorPos = null;
    this._btnDraw.textContent = this._drawing ? "Cancel Drawing" : "Draw Line";
    this._btnDraw.classList.toggle("active", this._drawing);
    this._mapWrap.classList.toggle("drawing", this._drawing);
    // In draw mode, canvas is passthrough so clicks reach the map image;
    // when not drawing, canvas captures mouse for endpoint dragging.
    if (this._drawing && this._editing) this._toggleEditMode();
    this._updateCanvasPassthrough();
    this._hint.style.display = this._drawing ? "" : "none";
    this._hint.textContent = this._drawing
      ? "Click the first point on the map"
      : "";
    this._redrawCanvas();
  }

  _toggleEditMode() {
    this._editing = !this._editing;
    if (this._editing && this._drawing) this._toggleDrawMode();
    if (!this._editing) this._selectedLine = null;
    this._btnEdit.textContent = this._editing ? "Done Editing" : "Edit Lines";
    this._btnEdit.classList.toggle("active", this._editing);
    this._updateCanvasPassthrough();
    this._renderLineList();
    this._redrawCanvas();
  }

  _updateCanvasPassthrough() {
    // Canvas captures mouse only in edit mode (for endpoint dragging).
    // In draw mode or normal view it's passthrough.
    this._mapCanvas.classList.toggle("passthrough", !this._editing || this._drawing);
  }

  _updateEditButton() {
    if (this._btnEdit) {
      this._btnEdit.disabled = this._lines.length === 0 && !this._editing;
    }
  }

  _onMapClick(e) {
    if (!this._drawing) return;
    // Suppress click that ended a drag
    if (this._dragMoved) { this._dragMoved = false; return; }
    if (!this._meta) {
      this._statusEl.textContent =
        "Map metadata not available — cannot place points. Is the map loaded?";
      return;
    }

    const rect = this._mapImg.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    const mapPt = this._pixelToMap(x, y);
    if (!mapPt) return;

    if (!this._pendingPoint) {
      // First click
      this._pendingPoint = mapPt;
      this._hint.textContent = `Point 1: (${mapPt[0]}, ${mapPt[1]}) — click second point`;
      this._redrawCanvas();
    } else {
      // Second click — create line
      this._lines.push({ p1: this._pendingPoint, p2: mapPt });
      this._pendingPoint = null;
      this._dirty = true;
      this._btnSave.disabled = false;
      this._hint.textContent = "Line added! Click to draw another, or save.";
      this._renderLineList();
      this._redrawCanvas();
    }
  }

  _deleteLine(idx) {
    this._lines.splice(idx, 1);
    this._dirty = true;
    this._btnSave.disabled = false;
    this._updateEditButton();
    this._renderLineList();
    this._redrawCanvas();
  }

  _clearAll() {
    this._lines = [];
    this._pendingPoint = null;
    this._dirty = true;
    this._btnSave.disabled = false;
    if (this._editing) this._toggleEditMode();
    this._updateEditButton();
    this._renderLineList();
    this._redrawCanvas();
    this._statusEl.textContent = "All lines cleared (save to apply)";
  }

  // ── Save to MQTT ──────────────────────────────────────────────

  async _saveLines() {
    if (!this._hass) return;
    const payload = JSON.stringify({ lines: this._lines });
    try {
      await this._hass.callService("mqtt", "publish", {
        topic: this._config.nogo_set_topic,
        payload: payload,
        qos: 1,
        retain: false,
      });
      // Record the saved snapshot so _loadLinesFromSensor won't overwrite
      // _lines with stale sensor data before the Pi echoes back the update
      this._pendingSaveSnapshot = JSON.stringify(this._lines);
      if (this._pendingSaveTimer) clearTimeout(this._pendingSaveTimer);
      this._pendingSaveTimer = setTimeout(() => {
        this._pendingSaveSnapshot = null;
        this._pendingSaveTimer = null;
      }, 8000); // give up waiting after 8s
      this._dirty = false;
      this._btnSave.disabled = true;
      this._statusEl.textContent = `Saved ${this._lines.length} line(s)`;
    } catch (err) {
      this._statusEl.textContent = `Save failed: ${err.message || err}`;
    }
  }

  // ── Load meta via MQTT ────────────────────────────────────────

  async _fetchMeta() {
    // One-shot: subscribe, get retained message, unsubscribe
    // This is a fallback if sensor entity is not available
    if (!this._hass || this._meta) return;
    try {
      // HA doesn't expose raw MQTT subscribe to cards easily.
      // We publish a dummy to trigger and rely on the sensor instead.
      this._statusEl.textContent =
        "Waiting for map metadata (sensor.rosie_map_meta)...";
    } catch (_) {
      // ignore
    }
  }

  // ── Canvas Rendering ──────────────────────────────────────────

  _resizeCanvas() {
    const canvas = this._mapCanvas;
    const img = this._mapImg;
    if (!canvas || !img) return;
    canvas.width = img.clientWidth;
    canvas.height = img.clientHeight;
    this._redrawCanvas();
  }

  _redrawCanvas() {
    const canvas = this._mapCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw committed lines
    for (let i = 0; i < this._lines.length; i++) {
      const line = this._lines[i];
      const p1 = this._mapToPixel(line.p1[0], line.p1[1]);
      const p2 = this._mapToPixel(line.p2[0], line.p2[1]);
      if (!p1 || !p2) continue;
      const isSelected = this._editing && this._selectedLine === i;
      this._drawLine(ctx, p1, p2, isSelected ? NOGO_COLOUR_SELECTED : NOGO_COLOUR, isSelected ? 4 : 3);
    }

    // Draw pending point (first click of a new line) + live preview line
    if (this._pendingPoint) {
      const pt = this._mapToPixel(this._pendingPoint[0], this._pendingPoint[1]);
      if (pt) {
        // Live preview line toward cursor
        if (this._cursorPos) {
          ctx.beginPath();
          ctx.moveTo(pt[0], pt[1]);
          ctx.lineTo(this._cursorPos[0], this._cursorPos[1]);
          ctx.strokeStyle = NOGO_COLOUR_PREVIEW;
          ctx.lineWidth = 2;
          ctx.setLineDash([6, 4]);
          ctx.stroke();
          ctx.setLineDash([]);
        }
        ctx.beginPath();
        ctx.arc(pt[0], pt[1], ENDPOINT_RADIUS, 0, Math.PI * 2);
        ctx.fillStyle = NOGO_COLOUR_PREVIEW;
        ctx.fill();
        ctx.strokeStyle = NOGO_COLOUR;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
    }
  }

  _drawLine(ctx, p1, p2, colour, width) {
    ctx.beginPath();
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.strokeStyle = colour;
    ctx.lineWidth = width;
    ctx.lineCap = "round";
    ctx.stroke();
    // Endpoints
    for (const pt of [p1, p2]) {
      ctx.beginPath();
      ctx.arc(pt[0], pt[1], ENDPOINT_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = colour;
      ctx.fill();
    }
  }

  // ── Line List ─────────────────────────────────────────────────

  // ── Drag Editing ──────────────────────────────────────────────

  _getEventPos(e) {
    const rect = this._mapImg.getBoundingClientRect();
    // getBoundingClientRect reflects CSS transform; convert back to layout coords
    const scaleX = this._mapImg.clientWidth / rect.width;
    const scaleY = this._mapImg.clientHeight / rect.height;
    return [(e.clientX - rect.left) * scaleX, (e.clientY - rect.top) * scaleY];
  }

  _onCanvasMouseDown(e) {
    if (e.button !== 0) return;

    // ── Middle or left click to pan when zoomed ──────────────────
    // Handled after edit-mode checks below; flag set if no endpoint hit.

    if (this._drawing || !this._editing) {
      // Not in edit mode - panning is the only mouse action
      if (this._zoom > 1.01) {
        this._mousePanning = true;
        this._mousePanStart = { clientX: e.clientX, clientY: e.clientY, panX: this._panX, panY: this._panY };
        this._mapCanvas.style.cursor = "grabbing";
        e.preventDefault();
      }
      return;
    }
    if (!this._meta || this._lines.length === 0) {
      // In edit mode but no lines — still allow pan
      if (this._zoom > 1.01) {
        this._mousePanning = true;
        this._mousePanStart = { clientX: e.clientX, clientY: e.clientY, panX: this._panX, panY: this._panY };
        this._mapCanvas.style.cursor = "grabbing";
        e.preventDefault();
      }
      return;
    }
    const [ex, ey] = this._getEventPos(e);
    const HIT = ENDPOINT_RADIUS + 6;
    // Check endpoint hit for dragging
    for (let i = 0; i < this._lines.length; i++) {
      for (const ep of ["p1", "p2"]) {
        const pt = this._mapToPixel(this._lines[i][ep][0], this._lines[i][ep][1]);
        if (!pt) continue;
        const dist = Math.hypot(ex - pt[0], ey - pt[1]);
        if (dist <= HIT) {
          this._dragging = { lineIdx: i, endpoint: ep };
          this._dragMoved = false;
          this._selectedLine = i;
          this._mapCanvas.style.cursor = "grabbing";
          this._renderLineList();
          this._redrawCanvas();
          e.preventDefault();
          e.stopPropagation();
          return;
        }
      }
    }
    // Check line-body hit for selection (within ~10px)
    const LINE_HIT = 10;
    for (let i = 0; i < this._lines.length; i++) {
      const a = this._mapToPixel(this._lines[i].p1[0], this._lines[i].p1[1]);
      const b = this._mapToPixel(this._lines[i].p2[0], this._lines[i].p2[1]);
      if (!a || !b) continue;
      if (this._pointToSegmentDist(ex, ey, a, b) <= LINE_HIT) {
        this._selectedLine = i;
        this._renderLineList();
        this._redrawCanvas();
        e.preventDefault();
        e.stopPropagation();
        return;
      }
    }
    // Clicked empty space — deselect, and start pan if zoomed
    if (this._selectedLine !== null) {
      this._selectedLine = null;
      this._renderLineList();
      this._redrawCanvas();
    }
    if (this._zoom > 1.01) {
      this._mousePanning = true;
      this._mousePanStart = { clientX: e.clientX, clientY: e.clientY, panX: this._panX, panY: this._panY };
      this._mapCanvas.style.cursor = "grabbing";
      e.preventDefault();
    }
  }

  _pointToSegmentDist(px, py, a, b) {
    const dx = b[0] - a[0], dy = b[1] - a[1];
    const lenSq = dx * dx + dy * dy;
    if (lenSq === 0) return Math.hypot(px - a[0], py - a[1]);
    let t = ((px - a[0]) * dx + (py - a[1]) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy));
  }

  _onMouseMove(e) {
    const [ex, ey] = this._getEventPos(e);

    if (this._mousePanning && this._mousePanStart) {
      this._panX = this._mousePanStart.panX + (e.clientX - this._mousePanStart.clientX);
      this._panY = this._mousePanStart.panY + (e.clientY - this._mousePanStart.clientY);
      this._clampPan();
      this._applyTransform();
      this._dragMoved = true;
      e.preventDefault();
      return;
    }

    if (this._dragging) {
      const mapPt = this._pixelToMap(ex, ey);
      if (mapPt) {
        const { lineIdx, endpoint } = this._dragging;
        this._lines[lineIdx][endpoint] = mapPt;
        this._dragMoved = true;
        this._dirty = true;
        this._btnSave.disabled = false;
        this._renderLineList();
        this._redrawCanvas();
      }
      return;
    }

    // Cursor feedback: grab when over endpoint (edit mode) or when zoomed in
    if (this._editing && !this._drawing && this._meta && this._lines.length > 0) {
      const HIT = ENDPOINT_RADIUS + 6;
      let overEndpoint = false;
      for (const line of this._lines) {
        for (const ep of ["p1", "p2"]) {
          const pt = this._mapToPixel(line[ep][0], line[ep][1]);
          if (pt && Math.hypot(ex - pt[0], ey - pt[1]) <= HIT) {
            overEndpoint = true;
            break;
          }
        }
        if (overEndpoint) break;
      }
      this._mapCanvas.style.cursor = overEndpoint ? "grab" : (this._zoom > 1.01 ? "grab" : "default");
    } else if (!this._drawing && this._zoom > 1.01) {
      this._mapCanvas.style.cursor = "grab";
    }

    // Live preview while waiting for second point
    if (this._drawing && this._pendingPoint) {
      this._cursorPos = [ex, ey];
      this._redrawCanvas();
    }
  }

  _onMouseUp(e) {
    if (this._mousePanning) {
      this._mousePanning = false;
      this._mousePanStart = null;
      this._mapCanvas.style.cursor = this._zoom > 1.01 ? "grab" : "default";
      return;
    }
    if (this._dragging) {
      this._mapCanvas.style.cursor = "grab";
      this._dragging = null;
      this._renderLineList();
    }
  }

  // ── Touch Events (mobile) ────────────────────────────────────

  _getTouchPos(e) {
    const touch = e.touches[0] || e.changedTouches[0];
    const rect = this._mapImg.getBoundingClientRect();
    const scaleX = this._mapImg.clientWidth / rect.width;
    const scaleY = this._mapImg.clientHeight / rect.height;
    return [(touch.clientX - rect.left) * scaleX, (touch.clientY - rect.top) * scaleY];
  }

  _getTouchDist(e) {
    const [a, b] = [e.touches[0], e.touches[1]];
    return Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
  }

  _getTouchMid(e) {
    const [a, b] = [e.touches[0], e.touches[1]];
    const rect = this._mapWrap.getBoundingClientRect();
    return [(a.clientX + b.clientX) / 2 - rect.left,
            (a.clientY + b.clientY) / 2 - rect.top];
  }

  _onTouchStart(e) {
    // Two fingers — start pinch-to-zoom
    if (e.touches.length === 2) {
      e.preventDefault();
      this._dragging = null;
      this._touchTapPos = null;
      this._pinchStartDist = this._getTouchDist(e);
      this._pinchStartZoom = this._zoom;
      this._pinchStartMid = this._getTouchMid(e);
      this._pinchStartPan = { x: this._panX, y: this._panY };
      return;
    }

    if (e.touches.length !== 1) return;
    const [ex, ey] = this._getTouchPos(e);

    // Double-tap to reset zoom
    const now = Date.now();
    if (now - this._doubleTapTime < 300) {
      this._doubleTapTime = 0;
      this._resetZoom();
      e.preventDefault();
      return;
    }
    this._doubleTapTime = now;

    // If zoomed in and not drawing/editing, start pan
    if (this._zoom > 1.05 && !this._drawing && !this._editing) {
      this._lastPanTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
      e.preventDefault();
      return;
    }

    // In edit mode — check endpoint hit for dragging
    if (this._editing && !this._drawing && this._meta && this._lines.length > 0) {
      const HIT = ENDPOINT_RADIUS + 14; // larger hit zone for fingers
      for (let i = 0; i < this._lines.length; i++) {
        for (const ep of ["p1", "p2"]) {
          const pt = this._mapToPixel(this._lines[i][ep][0], this._lines[i][ep][1]);
          if (!pt) continue;
          if (Math.hypot(ex - pt[0], ey - pt[1]) <= HIT) {
            this._dragging = { lineIdx: i, endpoint: ep };
            this._dragMoved = false;
            this._selectedLine = i;
            this._renderLineList();
            this._redrawCanvas();
            e.preventDefault();
            return;
          }
        }
      }
      // Check line-body hit for selection
      const LINE_HIT = 20; // larger for touch
      for (let i = 0; i < this._lines.length; i++) {
        const a = this._mapToPixel(this._lines[i].p1[0], this._lines[i].p1[1]);
        const b = this._mapToPixel(this._lines[i].p2[0], this._lines[i].p2[1]);
        if (!a || !b) continue;
        if (this._pointToSegmentDist(ex, ey, a, b) <= LINE_HIT) {
          this._selectedLine = i;
          this._renderLineList();
          this._redrawCanvas();
          e.preventDefault();
          return;
        }
      }
    }

    // In draw mode — place point on tap
    if (this._drawing) {
      this._touchTapPos = [ex, ey];
      e.preventDefault();
    }
  }

  _onTouchMove(e) {
    // Pinch zoom
    if (e.touches.length === 2 && this._pinchStartDist !== null) {
      e.preventDefault();
      const dist = this._getTouchDist(e);
      const mid = this._getTouchMid(e);
      const scale = dist / this._pinchStartDist;
      const newZoom = Math.max(1, Math.min(6, this._pinchStartZoom * scale));
      // Zoom toward pinch midpoint + follow midpoint pan
      this._panX = mid[0] - (this._pinchStartMid[0] - this._pinchStartPan.x) * (newZoom / this._pinchStartZoom);
      this._panY = mid[1] - (this._pinchStartMid[1] - this._pinchStartPan.y) * (newZoom / this._pinchStartZoom);
      this._zoom = newZoom;
      this._clampPan();
      this._applyTransform();
      return;
    }

    if (e.touches.length !== 1) return;

    // One-finger pan when zoomed
    if (this._lastPanTouch) {
      e.preventDefault();
      const dx = e.touches[0].clientX - this._lastPanTouch.x;
      const dy = e.touches[0].clientY - this._lastPanTouch.y;
      this._panX += dx;
      this._panY += dy;
      this._lastPanTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
      this._clampPan();
      this._applyTransform();
      return;
    }

    const [ex, ey] = this._getTouchPos(e);

    if (this._dragging) {
      e.preventDefault();
      const mapPt = this._pixelToMap(ex, ey);
      if (mapPt) {
        const { lineIdx, endpoint } = this._dragging;
        this._lines[lineIdx][endpoint] = mapPt;
        this._dragMoved = true;
        this._dirty = true;
        this._btnSave.disabled = false;
        this._renderLineList();
        this._redrawCanvas();
      }
      return;
    }

    // Live preview for drawing
    if (this._drawing && this._pendingPoint) {
      e.preventDefault();
      this._cursorPos = [ex, ey];
      this._redrawCanvas();
    }

    // Mark that touch moved (not a tap)
    this._touchTapPos = null;
  }

  _onTouchEnd(e) {
    // End pinch
    if (this._pinchStartDist !== null && e.touches.length < 2) {
      this._pinchStartDist = null;
      this._pinchStartZoom = null;
      this._pinchStartMid = null;
      this._pinchStartPan = null;
      return;
    }

    // End pan
    if (this._lastPanTouch) {
      this._lastPanTouch = null;
      return;
    }

    if (this._dragging) {
      this._dragging = null;
      this._renderLineList();
      return;
    }

    // Handle tap for drawing
    if (this._drawing && this._touchTapPos) {
      const [ex, ey] = this._touchTapPos;
      this._touchTapPos = null;
      this._cursorPos = null;
      if (!this._meta) return;
      const mapPt = this._pixelToMap(ex, ey);
      if (!mapPt) return;

      if (!this._pendingPoint) {
        this._pendingPoint = mapPt;
        this._hint.textContent = `Point 1: (${mapPt[0]}, ${mapPt[1]}) — tap second point`;
        this._redrawCanvas();
      } else {
        this._lines.push({ p1: this._pendingPoint, p2: mapPt });
        this._pendingPoint = null;
        this._dirty = true;
        this._btnSave.disabled = false;
        this._hint.textContent = "Line added! Tap to draw another, or save.";
        this._renderLineList();
        this._redrawCanvas();
      }
    }
  }

  // ── Line List ────────────────────────────────────────────────

  _renderLineList() {
    if (!this._lineList) return;
    if (this._lines.length === 0) {
      this._lineList.innerHTML =
        '<div style="font-size:0.85em;color:#888;padding:4px">No lines defined</div>';
      return;
    }
    this._lineList.innerHTML = this._lines
      .map(
        (l, i) => `
      <div class="line-item${this._editing && this._selectedLine === i ? ' selected' : ''}${this._editing ? ' selectable' : ''}" data-line-idx="${i}">
        <span class="idx">${i + 1}</span>
        <span class="coords">(${l.p1[0]}, ${l.p1[1]}) → (${l.p2[0]}, ${l.p2[1]})</span>
        <button class="del-btn" data-idx="${i}" title="Delete line" style="${this._editing ? '' : 'display:none'}">✕</button>
      </div>`
      )
      .join("");

    // Attach delete handlers
    this._lineList.querySelectorAll(".del-btn").forEach((btn) => {
      btn.addEventListener("click", (e) => {
        e.stopPropagation();
        this._deleteLine(parseInt(btn.dataset.idx, 10));
      });
    });
    // Attach line-item click handlers for selection
    this._lineList.querySelectorAll(".line-item").forEach((el) => {
      el.addEventListener("click", (e) => {
        if (!this._editing) return;
        const idx = parseInt(el.dataset.lineIdx, 10);
        this._selectedLine = this._selectedLine === idx ? null : idx;
        this._renderLineList();
        this._redrawCanvas();
      });
    });
  }
}

customElements.define("rosie-nogo-editor-card", RosieNogoEditorCard);

window.customCards = window.customCards || [];
window.customCards.push({
  type: "rosie-nogo-editor-card",
  name: "ROSie No-Go Line Editor",
  description: "Draw and manage no-go lines on the ROSie vacuum map",
});
