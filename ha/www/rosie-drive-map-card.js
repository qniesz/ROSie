/**
 * ROSie Drive Map Card — Custom Lovelace Card
 *
 * Displays the ROSie map and allows click-to-navigate during a Manual
 * Mapping session.  Click anywhere on the map to send the robot to that
 * world-frame position.  A pulsing reticle marks the active target.
 *
 * During non-mapping states the card is read-only (click has no effect).
 *
 * Config:
 *   type: custom:rosie-drive-map-card
 *   camera_entity: camera.rosie_rosie_map    # optional
 *   navigate_topic: rosie/navigate_to        # optional
 *   pipeline_entity: sensor.rosie_map_pipeline  # optional
 */

class RosieDriveMapCard extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this._hass = null;
    this._config = {};
    this._meta = null;
    this._targetWorld = null;   // [wx, wy] last sent destination
    this._targetDisplay = null; // [dx, dy] canvas coords for the reticle
    this._navStatus = "idle";   // from rosie/navigate_to/status
    this._pipelineStatus = "";  // from pipeline entity state
    this._built = false;
    this._zoom = 1;
    this._panX = 0;
    this._panY = 0;
    this._pinchStartDist = null;
    this._pinchStartZoom = null;
    this._pinchStartMid = null;
    this._pinchStartPan = null;
    this._lastPanTouch = null;
    this._mousePanning = false;
    this._mousePanStart = null;
    this._doubleTapTime = 0;
    this._reticleAnim = null;   // requestAnimationFrame handle
    this._reticlePhase = 0;     // 0–2π pulse phase
    this._fullscreen = false;
    this._refreshCountdown = 2;
  }

  set hass(hass) {
    this._hass = hass;
    if (!this._built) this._build();
    if (!this._refreshTimer) {
      this._updateImage();
    }
    this._loadMetaFromSensor();
    this._updatePipelineStatus();

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
      camera_entity:   config.camera_entity   || "camera.rosie_rosie_map",
      navigate_topic:  config.navigate_topic  || "rosie/navigate_to",
      pipeline_entity: config.pipeline_entity || "sensor.rosie_map_pipeline",
      ...config,
    };
  }

  static getStubConfig() {
    return { camera_entity: "camera.rosie_rosie_map" };
  }

  getCardSize() { return 6; }

  // ── Build DOM ───────────────────────────────────────────────────────────

  _build() {
    if (this._built) return;
    this._built = true;

    const style = document.createElement("style");
    style.textContent = `
      :host { display: block; }
      .card {
        background: var(--ha-card-background, var(--card-background-color, white));
        border-radius: var(--ha-card-border-radius, 12px);
        box-shadow: var(--ha-card-box-shadow, none);
        overflow: hidden; padding: 16px;
      }
      .card.fullscreen {
        position: fixed; top: 0; left: 0; width: 100vw; height: 100vh;
        z-index: 9999; border-radius: 0; padding: 12px;
        display: flex; flex-direction: column;
        background: var(--ha-card-background, var(--card-background-color, white));
      }
      .card.fullscreen .map-wrap { flex: 1; min-height: 0; overflow: hidden; }
      .header {
        font-size: 1.1em; font-weight: 500; margin-bottom: 10px;
        display: flex; align-items: center; gap: 8px;
      }
      .header ha-icon { --mdc-icon-size: 24px; }
      .map-wrap {
        position: relative; width: 100%;
        border-radius: 8px; overflow: hidden; background: #f0f5fa;
        line-height: 0; touch-action: none; cursor: crosshair;
      }
      .map-inner {
        position: relative; width: 100%; line-height: 0;
        transform-origin: 0 0; will-change: transform;
      }
      .map-img { width: 100%; display: block; }
      .map-canvas {
        position: absolute; top: 0; left: 0;
        width: 100%; height: 100%; background: transparent;
        pointer-events: none;
      }
      .status-bar {
        margin-top: 8px; padding: 6px 10px;
        background: var(--secondary-background-color, #f5f5f5);
        border-radius: 6px; font-size: 0.82em;
        color: var(--secondary-text-color, #666);
        min-height: 1.6em;
      }
      .status-bar.active { color: #1976d2; font-weight: 500; }
      .status-bar.arrived { color: #388e3c; font-weight: 500; }
      .status-bar.idle-map { color: var(--secondary-text-color, #666); }
      .countdown {
        position: absolute; top: 6px; right: 6px;
        background: rgba(0,0,0,0.55); color: white;
        font-size: 0.7em; font-weight: 600;
        padding: 2px 6px; border-radius: 10px;
        pointer-events: none; font-variant-numeric: tabular-nums;
        min-width: 18px; text-align: center;
      }
      .btn-fullscreen {
        background: none; border: none; cursor: pointer;
        color: var(--secondary-text-color, #888); font-size: 1.2em;
        padding: 4px; margin-left: auto; display: flex; align-items: center;
      }
      .btn-fullscreen:hover { color: var(--primary-text-color, #333); }
      .card.fullscreen .btn-fullscreen {
        position: fixed;
        top: clamp(8px, 1.5vh, 20px); right: clamp(8px, 2vw, 32px);
        background: var(--card-background-color, white);
        border-radius: 50%; margin-left: 0;
        width: clamp(36px,5vw,48px); height: clamp(36px,5vw,48px);
        font-size: clamp(1.1em, 2.5vw, 1.6em); justify-content: center;
        box-shadow: 0 2px 8px rgba(0,0,0,0.2); z-index: 10;
        color: var(--primary-text-color, #333);
      }
      .card.fullscreen .countdown { display: none; }
    `;

    const card = document.createElement("div");
    card.className = "card";
    card.innerHTML = `
      <div class="header">
        <ha-icon icon="mdi:map-marker-radius"></ha-icon>
        Drive Map
        <button class="btn-fullscreen" id="btnFullscreen" title="Toggle fullscreen">⛶</button>
      </div>
      <div class="map-wrap" id="mapWrap">
        <div class="map-inner" id="mapInner">
          <img class="map-img" id="mapImg" />
          <canvas class="map-canvas" id="mapCanvas"></canvas>
        </div>
        <span class="countdown" id="countdown">2</span>
      </div>
      <div class="status-bar idle-map" id="statusBar">Tap the map to navigate</div>
    `;

    this.shadowRoot.appendChild(style);
    this.shadowRoot.appendChild(card);

    this._card       = card;
    this._mapWrap    = card.querySelector("#mapWrap");
    this._mapInner   = card.querySelector("#mapInner");
    this._mapImg     = card.querySelector("#mapImg");
    this._mapCanvas  = card.querySelector("#mapCanvas");
    this._statusBar  = card.querySelector("#statusBar");
    this._countdownEl = card.querySelector("#countdown");
    this._btnFullscreen = card.querySelector("#btnFullscreen");
    this._refreshCountdown = 2;

    // Click-to-navigate
    this._mapWrap.addEventListener("click", (e) => this._onMapClick(e));

    // Zoom
    this._mapWrap.addEventListener("wheel", (e) => this._onWheel(e), { passive: false });

    // Mouse pan
    this._mapWrap.addEventListener("mousedown",  (e) => this._onMouseDown(e));
    this._mapWrap.addEventListener("mousemove",  (e) => this._onMouseMove(e));
    this._mapWrap.addEventListener("mouseup",    (e) => this._onMouseUp(e));
    this._mapWrap.addEventListener("mouseleave", ()  => {
      if (this._mousePanning) { this._mousePanning = false; this._mousePanStart = null; }
    });

    // Touch
    this._mapWrap.addEventListener("touchstart", (e) => this._onTouchStart(e), { passive: false });
    this._mapWrap.addEventListener("touchmove",  (e) => this._onTouchMove(e),  { passive: false });
    this._mapWrap.addEventListener("touchend",   (e) => this._onTouchEnd(e));

    // Fullscreen
    this._btnFullscreen.addEventListener("click", () => this._toggleFullscreen());
    this._keyHandler = (e) => {
      if (e.key === "Escape" && this._fullscreen) this._toggleFullscreen();
    };
    document.addEventListener("keydown", this._keyHandler);

    // Canvas resize
    this._resizeObs = new ResizeObserver(() => this._resizeCanvas());
    this._resizeObs.observe(this._mapWrap);

    // Animated reticle
    this._startReticleAnim();
  }

  disconnectedCallback() {
    if (this._refreshTimer)  { clearInterval(this._refreshTimer);  this._refreshTimer = null; }
    if (this._countdownTimer){ clearInterval(this._countdownTimer); this._countdownTimer = null; }
    if (this._keyHandler)    { document.removeEventListener("keydown", this._keyHandler); }
    if (this._reticleAnim)   { cancelAnimationFrame(this._reticleAnim); this._reticleAnim = null; }
  }

  // ── Image loading ───────────────────────────────────────────────────────

  _updateImage() {
    if (!this._hass || !this._mapImg) return;
    const entity = this._hass.states[this._config.camera_entity];
    if (!entity) return;
    const url = `/api/camera_proxy/${this._config.camera_entity}?token=${entity.attributes.access_token}&t=${entity.last_updated}`;
    if (this._mapImg.getAttribute("data-url") !== url) {
      this._mapImg.setAttribute("data-url", url);
      this._mapImg.src = url;
      this._mapImg.onload = () => { this._resizeCanvas(); this._redrawCanvas(); };
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
    this._mapImg.onload = () => { this._resizeCanvas(); this._redrawCanvas(); };
  }

  // ── Metadata ────────────────────────────────────────────────────────────

  _loadMetaFromSensor() {
    if (!this._hass) return;
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
        return;
      }
    }
    for (const [id, entity] of Object.entries(this._hass.states)) {
      if (id.startsWith("sensor.") && entity.attributes &&
          entity.attributes.origin && entity.attributes.resolution &&
          entity.attributes.scale) {
        this._meta = entity.attributes;
        return;
      }
    }
  }

  _updatePipelineStatus() {
    if (!this._hass) return;
    const entity = this._hass.states[this._config.pipeline_entity];
    if (!entity) return;
    const st = (entity.state || "").toLowerCase();
    if (st !== this._pipelineStatus) {
      this._pipelineStatus = st;
      this._refreshStatusBar();
    }
  }

  _refreshStatusBar() {
    if (!this._statusBar) return;
    const isManualReady = this._pipelineStatus === "manual_ready";
    const navSt = this._navStatus;

    this._statusBar.className = "status-bar";
    if (isManualReady) {
      if (navSt === "rotating" || navSt === "driving") {
        this._statusBar.classList.add("active");
        const verb = navSt === "rotating" ? "Rotating…" : "Driving…";
        const t = this._targetWorld;
        this._statusBar.textContent = t
          ? `${verb} → (${t[0].toFixed(2)}, ${t[1].toFixed(2)}) m`
          : verb;
      } else if (navSt === "arrived") {
        this._statusBar.classList.add("arrived");
        this._statusBar.textContent = "Arrived ✓  Tap map for next destination";
      } else {
        this._statusBar.textContent = "Tap the map to navigate";
      }
    } else if (this._pipelineStatus === "manual_mapping") {
      this._statusBar.textContent = "SLAM starting — please wait…";
    } else {
      this._statusBar.classList.add("idle-map");
      this._statusBar.textContent = this._targetWorld
        ? `Last target: (${this._targetWorld[0].toFixed(2)}, ${this._targetWorld[1].toFixed(2)}) m`
        : "Tap the map to navigate during Manual Map";
    }
  }

  // ── Click-to-navigate ───────────────────────────────────────────────────

  _onMapClick(e) {
    if (this._mousePanning) return;  // suppress click that ended a pan drag
    if (!this._meta) {
      this._statusBar.textContent = "No map metadata — start a mapping session first";
      return;
    }
    if (this._pipelineStatus !== "manual_ready") {
      this._statusBar.textContent = "Navigation only active during Manual Map session";
      return;
    }

    const rect = this._mapImg.getBoundingClientRect();
    // Convert click to layout coordinates (undo CSS transform zoom/pan)
    const layoutX = (e.clientX - rect.left - this._panX) / this._zoom;
    const layoutY = (e.clientY - rect.top  - this._panY) / this._zoom;

    const world = this._pixelToMap(layoutX, layoutY);
    if (!world) return;

    const [wx, wy] = world;
    this._targetWorld   = [wx, wy];
    this._targetDisplay = [layoutX, layoutY];
    this._navStatus     = "rotating";
    this._refreshStatusBar();
    this._redrawCanvas();

    this._hass.callService("mqtt", "publish", {
      topic:   this._config.navigate_topic,
      payload: JSON.stringify({ x: wx, y: wy }),
      qos: 0,
      retain: false,
    });
  }

  // ── Coordinate Transforms (identical to rosie-nogo-editor-card.js) ──────

  _pixelToMap(displayX, displayY) {
    if (!this._meta) return null;
    const m   = this._meta;
    const img = this._mapImg;
    if (!img.naturalWidth) return null;

    const sx = img.naturalWidth  / img.clientWidth;
    const sy = img.naturalHeight / img.clientHeight;
    let px = displayX * sx;
    let py = displayY * sy;

    px = px - m.border + m.crop_c;
    py = py - m.border + m.crop_r;

    px = px / m.scale;
    py = py / m.scale;

    const angle = m.angle || 0;
    if (Math.abs(angle) > 0.1) {
      const rad   = angle * Math.PI / 180;
      const cos_a = Math.cos(rad);
      const sin_a = Math.sin(rad);
      const cx = m.post_rot_w / 2;
      const cy = m.post_rot_h / 2;
      const dx = px - cx;
      const dy = py - cy;
      px = cos_a * dx + sin_a * dy + m.pre_rot_w / 2;
      py = -sin_a * dx + cos_a * dy + m.pre_rot_h / 2;
    }

    const [ox, oy] = m.origin;
    const mx = px * m.resolution + ox;
    const my = (m.map_h - 1 - py) * m.resolution + oy;
    return [Math.round(mx * 1000) / 1000, Math.round(my * 1000) / 1000];
  }

  _mapToPixel(mx, my) {
    if (!this._meta) return null;
    const m   = this._meta;
    const img = this._mapImg;
    if (!img.naturalWidth) return null;

    const [ox, oy] = m.origin;
    let px = (mx - ox) / m.resolution;
    let py = (m.map_h - 1) - (my - oy) / m.resolution;

    const angle = m.angle || 0;
    if (Math.abs(angle) > 0.1) {
      const rad   = angle * Math.PI / 180;
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

    const sx = img.clientWidth  / img.naturalWidth;
    const sy = img.clientHeight / img.naturalHeight;
    return [px * sx, py * sy];
  }

  // ── Canvas rendering ────────────────────────────────────────────────────

  _resizeCanvas() {
    const canvas = this._mapCanvas;
    const img    = this._mapImg;
    if (!canvas || !img) return;
    canvas.width  = img.clientWidth;
    canvas.height = img.clientHeight;
    this._redrawCanvas();
  }

  _redrawCanvas() {
    const canvas = this._mapCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (!this._targetDisplay) return;

    const [dx, dy] = this._targetDisplay;
    const pulse = 0.7 + 0.3 * Math.sin(this._reticlePhase);
    const isActive = this._navStatus === "rotating" || this._navStatus === "driving";
    const isArrived = this._navStatus === "arrived";

    const colour = isArrived ? "rgba(56, 142, 60, 0.9)"   // green
                 : isActive  ? "rgba(25, 118, 210, 0.9)"   // blue
                              : "rgba(220, 40, 40, 0.75)"; // red/grey

    const outerR = isActive ? 18 * pulse : 18;
    const innerR = 5;

    // Outer pulsing ring
    ctx.beginPath();
    ctx.arc(dx, dy, outerR, 0, Math.PI * 2);
    ctx.strokeStyle = colour;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Inner filled dot
    ctx.beginPath();
    ctx.arc(dx, dy, innerR, 0, Math.PI * 2);
    ctx.fillStyle = colour;
    ctx.fill();

    // Crosshair lines
    const arm = outerR + 6;
    ctx.strokeStyle = colour;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(dx - arm, dy); ctx.lineTo(dx - outerR - 2, dy);
    ctx.moveTo(dx + outerR + 2, dy); ctx.lineTo(dx + arm, dy);
    ctx.moveTo(dx, dy - arm); ctx.lineTo(dx, dy - outerR - 2);
    ctx.moveTo(dx, dy + outerR + 2); ctx.lineTo(dx, dy + arm);
    ctx.stroke();
  }

  _startReticleAnim() {
    const tick = () => {
      this._reticlePhase = (this._reticlePhase + 0.06) % (Math.PI * 2);
      if (this._targetDisplay &&
          (this._navStatus === "rotating" || this._navStatus === "driving")) {
        this._redrawCanvas();
      }
      this._reticleAnim = requestAnimationFrame(tick);
    };
    this._reticleAnim = requestAnimationFrame(tick);
  }

  // ── Zoom / Pan ──────────────────────────────────────────────────────────

  _applyTransform() {
    if (!this._mapInner) return;
    this._mapInner.style.transform =
      `translate(${this._panX}px, ${this._panY}px) scale(${this._zoom})`;
  }

  _resetZoom() {
    this._zoom = 1; this._panX = 0; this._panY = 0;
    this._applyTransform();
  }

  _clampPan() {
    const wrap = this._mapWrap;
    const img  = this._mapImg;
    if (!wrap || !img) return;
    const wW = wrap.clientWidth,  wH = wrap.clientHeight;
    const iW = img.clientWidth  * this._zoom;
    const iH = img.clientHeight * this._zoom;
    if (iW <= wW) this._panX = (wW - iW) / 2;
    else this._panX = Math.min(0, Math.max(wW - iW, this._panX));
    if (iH <= wH) this._panY = (wH - iH) / 2;
    else this._panY = Math.min(0, Math.max(wH - iH, this._panY));
  }

  _onWheel(e) {
    e.preventDefault();
    const rect = this._mapWrap.getBoundingClientRect();
    const cx = e.clientX - rect.left, cy = e.clientY - rect.top;
    const factor  = e.deltaY < 0 ? 1.15 : 0.87;
    const newZoom = Math.max(1, Math.min(6, this._zoom * factor));
    this._panX = cx - (cx - this._panX) * (newZoom / this._zoom);
    this._panY = cy - (cy - this._panY) * (newZoom / this._zoom);
    this._zoom = newZoom;
    this._clampPan();
    this._applyTransform();
  }

  _onMouseDown(e) {
    if (e.button !== 0 || this._zoom <= 1.01) return;
    this._mousePanning  = false;  // reset; set on first move
    this._mousePanStart = { clientX: e.clientX, clientY: e.clientY, panX: this._panX, panY: this._panY };
    e.preventDefault();
  }

  _onMouseMove(e) {
    if (!this._mousePanStart) return;
    const dx = e.clientX - this._mousePanStart.clientX;
    const dy = e.clientY - this._mousePanStart.clientY;
    if (!this._mousePanning && (Math.abs(dx) > 4 || Math.abs(dy) > 4)) {
      this._mousePanning = true;
    }
    if (!this._mousePanning) return;
    this._panX = this._mousePanStart.panX + dx;
    this._panY = this._mousePanStart.panY + dy;
    this._clampPan();
    this._applyTransform();
    e.preventDefault();
  }

  _onMouseUp(e) {
    this._mousePanStart = null;
    // don't clear _mousePanning here — _onMapClick checks it and clears after
    requestAnimationFrame(() => { this._mousePanning = false; });
  }

  _onTouchStart(e) {
    if (e.touches.length === 1) {
      const t = e.touches[0];
      const now = Date.now();
      if (now - this._doubleTapTime < 300) { this._resetZoom(); e.preventDefault(); return; }
      this._doubleTapTime = now;
      this._lastPanTouch = { clientX: t.clientX, clientY: t.clientY, panX: this._panX, panY: this._panY };
    } else if (e.touches.length === 2) {
      e.preventDefault();
      const a = e.touches[0], b = e.touches[1];
      this._pinchStartDist = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
      this._pinchStartZoom = this._zoom;
      const mid = [(a.clientX + b.clientX) / 2, (a.clientY + b.clientY) / 2];
      const rect = this._mapWrap.getBoundingClientRect();
      this._pinchStartMid = [mid[0] - rect.left, mid[1] - rect.top];
      this._pinchStartPan = [this._panX, this._panY];
      this._lastPanTouch = null;
    }
  }

  _onTouchMove(e) {
    e.preventDefault();
    if (e.touches.length === 2 && this._pinchStartDist) {
      const a = e.touches[0], b = e.touches[1];
      const dist    = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
      const newZoom = Math.max(1, Math.min(6, this._pinchStartZoom * (dist / this._pinchStartDist)));
      const [mx, my] = this._pinchStartMid;
      const [px0, py0] = this._pinchStartPan;
      this._zoom = newZoom;
      this._panX = mx - (mx - px0) * (newZoom / this._pinchStartZoom);
      this._panY = my - (my - py0) * (newZoom / this._pinchStartZoom);
      this._clampPan();
      this._applyTransform();
    } else if (e.touches.length === 1 && this._lastPanTouch && this._zoom > 1.01) {
      const t = e.touches[0];
      this._panX = this._lastPanTouch.panX + (t.clientX - this._lastPanTouch.clientX);
      this._panY = this._lastPanTouch.panY + (t.clientY - this._lastPanTouch.clientY);
      this._clampPan();
      this._applyTransform();
    }
  }

  _onTouchEnd(e) {
    if (e.touches.length < 2) {
      this._pinchStartDist = null;
      this._pinchStartZoom = null;
    }
    if (e.touches.length === 0) this._lastPanTouch = null;
  }

  // ── Fullscreen ──────────────────────────────────────────────────────────

  _toggleFullscreen() {
    this._fullscreen = !this._fullscreen;
    this._card.classList.toggle("fullscreen", this._fullscreen);
    this._btnFullscreen.textContent = this._fullscreen ? "\u2715" : "\u26F6";
    this._btnFullscreen.title = this._fullscreen ? "Exit fullscreen" : "Toggle fullscreen";
    if (!this._fullscreen) this._resetZoom();
    requestAnimationFrame(() => { this._resizeCanvas(); this._redrawCanvas(); });
  }
}

customElements.define("rosie-drive-map-card", RosieDriveMapCard);
window.customCards = window.customCards || [];
window.customCards.push({
  type: "rosie-drive-map-card",
  name: "ROSie Drive Map",
  description: "Click-to-navigate map for ROSie manual mapping sessions",
});
