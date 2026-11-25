import { Qfn48Widget } from "./qfn48.js";
import { LogicAnalyzerWidget } from "./logic_analyzer.js";

const decoder = new TextDecoder();
const encoder = new TextEncoder();

const state = {
  wasm: null,
  memory: null,
  elfLoaded: false,
  running: false,
  rafId: 0,
  selectedPin: null,
  activeUart: 0,
  uartBuffers: ["", "", "", ""],
  latestSnap: null,
};

const dom = {
  elfFile: document.getElementById("elf-file"),
  btnRun: document.getElementById("btn-run"),
  btnStepFrame: document.getElementById("btn-step-frame"),
  btnStepInsn: document.getElementById("btn-step-insn"),
  btnReset: document.getElementById("btn-reset"),
  speedSelect: document.getElementById("speed-select"),

  statFw: document.getElementById("stat-fw"),
  statPc: document.getElementById("stat-pc"),
  statMode: document.getElementById("stat-mode"),
  statClk: document.getElementById("stat-clk"),
  statTime: document.getElementById("stat-time"),
  statCycles: document.getElementById("stat-cycles"),
  statIrq: document.getElementById("stat-irq"),
  statMmio: document.getElementById("stat-mmio"),

  qfn48Host: document.getElementById("qfn48-host"),
  ledGrid: document.getElementById("led-grid"),
  btnPb9Toggle: document.getElementById("btn-pb9-toggle"),
  btnPb4Toggle: document.getElementById("btn-pb4-toggle"),
  btnPa0Toggle: document.getElementById("btn-pa0-toggle"),

  logicCanvas: document.getElementById("logic-canvas"),
  logicZoom: document.getElementById("logic-zoom"),
  logicZoomVal: document.getElementById("logic-zoom-val"),
  btnLogicClear: document.getElementById("btn-logic-clear"),
  logicChannelRow: document.getElementById("logic-channel-row"),

  pinInspectTitle: document.getElementById("pin-inspect-title"),
  pinInspectDetails: document.getElementById("pin-inspect-details"),
  btnPinFloat: document.getElementById("btn-pin-float"),
  btnPinLow: document.getElementById("btn-pin-low"),
  btnPinHigh: document.getElementById("btn-pin-high"),

  adcCh: document.getElementById("adc-ch"),
  adcMode: document.getElementById("adc-mode"),
  adcLiveMv: document.getElementById("adc-live-mv"),
  adcMvSlider: document.getElementById("adc-mv-slider"),
  adcMvNum: document.getElementById("adc-mv-num"),
  adcP1: document.getElementById("adc-p1"),
  adcP2: document.getElementById("adc-p2"),
  btnAdcApply: document.getElementById("btn-adc-apply"),
  btnAdcRestAll: document.getElementById("btn-adc-rest-all"),
  btnAdcKeyPress: document.getElementById("btn-adc-key-press"),

  uartTabs: Array.from(document.querySelectorAll('.tab-btn[role="tab"]')),
  uartLog: document.getElementById("uart-log"),
  uartRxInput: document.getElementById("uart-rx-input"),
  btnUartSend: document.getElementById("btn-uart-send"),
  btnUartClear: document.getElementById("btn-uart-clear"),

  usbStatusBadge: document.getElementById("usb-status-badge"),
  btnUsbEnum: document.getElementById("btn-usb-enum"),
  btnUsbClear: document.getElementById("btn-usb-clear"),
  usbDescInfo: document.getElementById("usb-desc-info"),
  usbPktLog: document.getElementById("usb-pkt-log"),
  usbEp: document.getElementById("usb-ep"),
  usbOutHex: document.getElementById("usb-out-hex"),
  btnUsbSend: document.getElementById("btn-usb-send"),

  bleAdvBadge: document.getElementById("ble-adv-badge"),
  bleMeta: document.getElementById("ble-meta"),
  bleEventLog: document.getElementById("ble-event-log"),
};

const qfn48 = new Qfn48Widget(dom.qfn48Host, {
  onPinClick(pinInfo) {
    selectPin(pinInfo);
    if (!state.wasm) return;
    if (pinInfo.port === "A" || pinInfo.port === "B") {
      const nextMode =
        pinInfo.ext === "float" ? 1 : pinInfo.ext === "low" ? 2 : 0;
      state.wasm.ch585_set_pin(pinInfo.port.charCodeAt(0), pinInfo.pidx, nextMode, 0);
      refreshSnapshot();
    }
  },
});

const logic = new LogicAnalyzerWidget(dom.logicCanvas);

function withWasmBytes(bytes, fn) {
  const ptr = state.wasm.wasm_alloc(bytes.length);
  try {
    const view = new Uint8Array(state.memory.buffer, ptr, bytes.length);
    view.set(bytes);
    return fn(ptr, bytes.length);
  } finally {
    state.wasm.wasm_free(ptr, bytes.length);
  }
}

function setControlsEnabled(enabled) {
  for (const btn of [
    dom.btnRun,
    dom.btnStepFrame,
    dom.btnStepInsn,
    dom.btnReset,
    dom.btnPb9Toggle,
    dom.btnPb4Toggle,
    dom.btnPa0Toggle,
    dom.btnLogicClear,
    dom.btnAdcApply,
    dom.btnAdcRestAll,
    dom.btnAdcKeyPress,
    dom.btnUartSend,
    dom.btnUsbEnum,
    dom.btnUsbClear,
    dom.btnUsbSend,
  ]) {
    btn.disabled = !enabled;
  }
}

function setRunning(flag) {
  state.running = flag && state.elfLoaded;
  dom.btnRun.textContent = state.running ? "Pause" : "Run";
  dom.btnRun.classList.toggle("active-run", state.running);
  if (state.running) {
    scheduleFrame();
  } else if (state.rafId) {
    cancelAnimationFrame(state.rafId);
    state.rafId = 0;
  }
}

function scheduleFrame() {
  if (!state.running) return;
  state.rafId = requestAnimationFrame(() => {
    if (!state.running || !state.wasm) return;
    const stepUs = Number(dom.speedSelect.value) || 1000;
    state.wasm.ch585_step_us(stepUs);
    refreshSnapshot();
    scheduleFrame();
  });
}

function selectPin(pinInfo) {
  state.selectedPin = pinInfo.pin;
  qfn48.setSelectedPin(pinInfo.pin);
  renderPinInspector(pinInfo);
  if (pinInfo.adc !== null && pinInfo.adc < 14) {
    dom.adcCh.value = String(pinInfo.adc);
  }
}

function renderPinInspector(p) {
  if (!p) {
    dom.pinInspectTitle.textContent = "No pin selected";
    dom.btnPinFloat.disabled = true;
    dom.btnPinLow.disabled = true;
    dom.btnPinHigh.disabled = true;
    return;
  }
  const gpio = p.port === "A" || p.port === "B";
  dom.pinInspectTitle.textContent = `Pin ${p.pin}: ${p.name}`;
  dom.pinInspectDetails.textContent =
    `State=${p.state.toUpperCase()} Dir=${p.out ? "OUT" : "IN"} PU=${p.pu ? 1 : 0} PD=${p.pd ? 1 : 0} Ext=${p.ext}` +
    (p.adc !== null ? ` AIN${p.adc}=${p.mv ?? "?"}mV` : "") +
    `\nAlt: ${p.alt || "-"}`;
  dom.btnPinFloat.disabled = !gpio || !state.elfLoaded;
  dom.btnPinLow.disabled = !gpio || !state.elfLoaded;
  dom.btnPinHigh.disabled = !gpio || !state.elfLoaded;
}

function renderLedGrid(snap) {
  const watchList = [
    { name: "PB9", tag: "VBUS" },
    { name: "PB4", tag: "LED/Key" },
    { name: "PB20", tag: "LED" },
    { name: "PB22", tag: "U2_TX" },
    { name: "PB23", tag: "U2_TX*" },
    { name: "PA8", tag: "U1_TX" },
    { name: "PA14", tag: "MOSI" },
    { name: "PA15", tag: "MISO" },
  ];
  const byName = new Map(snap.pins.map((p) => [p.name, p]));
  const items = [];
  const seen = new Set();

  for (const w of watchList) {
    const p = byName.get(w.name);
    if (!p) continue;
    seen.add(p.name);
    items.push({ pin: p, note: w.tag });
  }
  // Also surface any extra actively driven output pin
  for (const p of snap.pins) {
    if (!seen.has(p.name) && p.out) {
      items.push({ pin: p, note: "OUT" });
    }
  }

  const cards = items.slice(0, 14).map(({ pin, note }) => {
    const card = document.createElement("div");
    card.className = "led-card";
    const bulb = document.createElement("span");
    bulb.className =
      "led-bulb" +
      (pin.state === "high" ? " on" : pin.state === "low" ? " low-drive" : "");
    const txt = document.createElement("span");
    txt.textContent = `${pin.name} (${note})`;
    card.appendChild(bulb);
    card.appendChild(txt);
    return card;
  });
  dom.ledGrid.replaceChildren(...cards);
}

function renderUart() {
  const text = state.uartBuffers[state.activeUart] || "";
  if (dom.uartLog.textContent !== text) {
    const atBottom =
      dom.uartLog.scrollTop + dom.uartLog.clientHeight >=
      dom.uartLog.scrollHeight - 24;
    dom.uartLog.textContent = text;
    if (atBottom) {
      dom.uartLog.scrollTop = dom.uartLog.scrollHeight;
    }
  }
}

function refreshSnapshot() {
  if (!state.wasm) return;
  const len = state.wasm.ch585_build_snapshot(512);
  const ptr = state.wasm.ch585_get_snapshot_ptr();
  const jsonText = decoder.decode(new Uint8Array(state.memory.buffer, ptr, len));
  const snap = JSON.parse(jsonText);
  state.latestSnap = snap;

  // Top telemetry
  dom.statPc.textContent = `0x${snap.pc.toString(16).toUpperCase().padStart(8, "0")}`;
  dom.statMode.textContent = snap.wfi ? `${snap.mode} (WFI)` : snap.mode;
  dom.statClk.textContent = `${(snap.sysClkHz / 1_000_000).toFixed(2)} MHz`;
  dom.statTime.textContent = `${(snap.timeUs / 1000).toFixed(2)} ms`;
  dom.statCycles.textContent = snap.cycles.toLocaleString("en-US");
  dom.statIrq.textContent =
    `0x${snap.pfic.en[0].toString(16)} / 0x${snap.pfic.pend[0].toString(16)}`;
  dom.statMmio.textContent = String(snap.unknownMmio);

  // QFN48 diagram + Inspector + LEDs
  qfn48.update(
    snap.pins,
    `${(snap.sysClkHz / 1e6).toFixed(1)}MHz · PC=${dom.statPc.textContent}`,
  );
  if (state.selectedPin !== null) {
    renderPinInspector(snap.pins[state.selectedPin - 1]);
  }
  renderLedGrid(snap);

  // ADC live readout
  const chIdx = Number(dom.adcCh.value) || 0;
  dom.adcLiveMv.textContent = `${snap.adcMv[chIdx] ?? 0} mV`;

  // UART append deltas
  for (let i = 0; i < 4; i++) {
    if (snap.uartDeltas[i]) {
      state.uartBuffers[i] += snap.uartDeltas[i];
      if (state.uartBuffers[i].length > 32768) {
        state.uartBuffers[i] = state.uartBuffers[i].slice(-24576);
      }
    }
  }
  renderUart();

  // USB HS view
  const u = snap.usb;
  dom.usbStatusBadge.textContent = u.enumerated ? "Enumerated (HS)" : "Detached / Idle";
  dom.usbStatusBadge.classList.toggle("ok", u.enumerated);
  dom.usbDescInfo.textContent =
    `VID=${u.vid.toString(16).toUpperCase().padStart(4, "0")} PID=${u.pid
      .toString(16)
      .toUpperCase()
      .padStart(4, "0")} DevAddr=${u.devAd} CfgDesc=${u.cfgLen}B HIDReports=${u.hidDescs} TotalPkts=${u.totalPkts}`;
  dom.usbPktLog.textContent = u.packets
    .map((pkt) => `EP${pkt.ep} ${pkt.in ? "IN " : "OUT"} (${pkt.len}B): ${pkt.hex}`)
    .join("\n");

  // BLE view
  const b = snap.ble;
  dom.bleAdvBadge.textContent = b.advertising ? "Advertising" : "Disabled";
  dom.bleAdvBadge.classList.toggle("ok", b.advertising);
  dom.bleMeta.textContent =
    `Hooks=${b.hooks} | Adv=${b.advHex || "(empty)"} | ScanRsp=${b.scanHex || "(empty)"}`;
  const bleLines = [...b.events];
  if (b.hidReports.length > 0) {
    bleLines.push("--- BLE HID Reports ---", ...b.hidReports);
  }
  dom.bleEventLog.textContent = bleLines.join("\n");

  // Logic Analyzer
  logic.updateSamples(snap.logic, snap.logicTotal, snap.timeUs);
}

function initLogicChannelSelectors() {
  const opts = [];
  for (let i = 0; i < 16; i++) opts.push({ port: "A", idx: i, text: `PA${i}` });
  for (let i = 0; i < 24; i++) opts.push({ port: "B", idx: i, text: `PB${i}` });

  const selects = logic.getChannels().map((ch, slot) => {
    const sel = document.createElement("select");
    sel.setAttribute("aria-label", `Logic channel ${slot + 1}`);
    for (const o of opts) {
      const optEl = document.createElement("option");
      optEl.value = `${o.port}:${o.idx}`;
      optEl.textContent = `Ch${slot + 1}: ${o.text}`;
      if (o.port === ch.port && o.idx === ch.idx) {
        optEl.selected = true;
      }
      sel.appendChild(optEl);
    }
    sel.addEventListener("change", () => {
      const [p, idxStr] = sel.value.split(":");
      logic.setChannel(slot, p, Number(idxStr));
    });
    return sel;
  });
  dom.logicChannelRow.replaceChildren(...selects);
}

function togglePinDrive(port, idx) {
  if (!state.wasm || !state.latestSnap) return;
  const target = state.latestSnap.pins.find((p) => p.port === port && p.pidx === idx);
  const nextMode = target && target.ext === "high" ? 1 : 2;
  state.wasm.ch585_set_pin(port.charCodeAt(0), idx, nextMode, 0);
  refreshSnapshot();
}

function applyAdcChannel(ch, kind, p0, p1, p2, p3) {
  if (!state.wasm) return;
  state.wasm.ch585_set_adc_wave(ch, kind, p0, p1, p2, p3);
  refreshSnapshot();
}

function wireEvents() {
  dom.elfFile.addEventListener("change", async (e) => {
    const file = e.target.files?.[0];
    if (!file || !state.wasm) return;
    setRunning(false);
    const bytes = new Uint8Array(await file.arrayBuffer());
    const rc = withWasmBytes(bytes, (ptr, len) => state.wasm.ch585_load_elf(ptr, len));
    if (rc !== 0) {
      dom.statFw.textContent = `ELF Error (${rc})`;
      setControlsEnabled(false);
      state.elfLoaded = false;
      return;
    }
    state.elfLoaded = true;
    state.uartBuffers = ["", "", "", ""];
    dom.statFw.textContent = file.name;
    setControlsEnabled(true);
    // Pre-seed ADC rest mV cleanly if desired or reflect reset state
    refreshSnapshot();
  });

  dom.btnRun.addEventListener("click", () => setRunning(!state.running));
  dom.btnStepFrame.addEventListener("click", () => {
    if (!state.wasm) return;
    setRunning(false);
    state.wasm.ch585_step_us(1000);
    refreshSnapshot();
  });
  dom.btnStepInsn.addEventListener("click", () => {
    if (!state.wasm) return;
    setRunning(false);
    state.wasm.ch585_step_cycles(1);
    refreshSnapshot();
  });
  dom.btnReset.addEventListener("click", () => {
    if (!state.wasm) return;
    setRunning(false);
    state.wasm.ch585_reset();
    state.uartBuffers = ["", "", "", ""];
    refreshSnapshot();
  });

  // Pin actions
  const setSelectedDrive = (mode) => {
    if (!state.wasm || !state.latestSnap || state.selectedPin === null) return;
    const p = state.latestSnap.pins[state.selectedPin - 1];
    if (!p || (p.port !== "A" && p.port !== "B")) return;
    state.wasm.ch585_set_pin(p.port.charCodeAt(0), p.pidx, mode, 0);
    refreshSnapshot();
  };
  dom.btnPinFloat.addEventListener("click", () => setSelectedDrive(0));
  dom.btnPinLow.addEventListener("click", () => setSelectedDrive(1));
  dom.btnPinHigh.addEventListener("click", () => setSelectedDrive(2));

  dom.btnPb9Toggle.addEventListener("click", () => togglePinDrive("B", 9));
  dom.btnPb4Toggle.addEventListener("click", () => togglePinDrive("B", 4));
  dom.btnPa0Toggle.addEventListener("click", () => togglePinDrive("A", 0));

  // Logic analyzer
  dom.logicZoom.addEventListener("input", () => {
    const z = Number(dom.logicZoom.value) || 1;
    dom.logicZoomVal.textContent = `${z.toFixed(1)}x`;
    logic.setZoom(z);
  });
  dom.btnLogicClear.addEventListener("click", () => {
    state.wasm?.ch585_logic_clear();
    refreshSnapshot();
  });
  window.addEventListener("resize", () => logic.render(true));

  // ADC sliders & presets
  dom.adcMvSlider.addEventListener("input", () => {
    dom.adcMvNum.value = dom.adcMvSlider.value;
    if (Number(dom.adcMode.value) === 0 && state.elfLoaded) {
      applyAdcChannel(Number(dom.adcCh.value), 0, Number(dom.adcMvSlider.value), 0, 1, 0.5);
    }
  });
  dom.adcMvNum.addEventListener("input", () => {
    dom.adcMvSlider.value = dom.adcMvNum.value;
  });
  dom.btnAdcApply.addEventListener("click", () => {
    const ch = Number(dom.adcCh.value) || 0;
    const kind = Number(dom.adcMode.value) || 0;
    const p0 = Number(dom.adcMvNum.value) || 0;
    const p1 = Number(dom.adcP1.value) || 0;
    const p2 = Number(dom.adcP2.value) || 10;
    applyAdcChannel(ch, kind, p0, p1, p2, 0.5);
  });
  dom.btnAdcRestAll.addEventListener("click", () => {
    if (!state.wasm) return;
    for (let ch = 0; ch < 14; ch++) {
      state.wasm.ch585_set_adc_wave(ch, 0, 1650, 0, 1, 0.5);
    }
    dom.adcMvSlider.value = "1650";
    dom.adcMvNum.value = "1650";
    refreshSnapshot();
  });
  dom.btnAdcKeyPress.addEventListener("click", () => {
    if (!state.wasm) return;
    state.wasm.ch585_set_adc_wave(0, 0, 2800, 0, 1, 0.5);
    state.wasm.ch585_set_adc_wave(1, 0, 2800, 0, 1, 0.5);
    refreshSnapshot();
  });

  // UART tabs & RX
  const activateUartTab = (idx) => {
    state.activeUart = idx;
    dom.uartTabs.forEach((btn, i) => {
      btn.setAttribute("aria-selected", i === idx ? "true" : "false");
    });
    dom.uartLog.setAttribute("aria-labelledby", `tab-uart-${idx}`);
    renderUart();
  };
  dom.uartTabs.forEach((btn, idx) => {
    btn.addEventListener("click", () => activateUartTab(idx));
    btn.addEventListener("keydown", (e) => {
      let next = null;
      if (e.key === "ArrowRight") next = (idx + 1) % dom.uartTabs.length;
      if (e.key === "ArrowLeft") next = (idx - 1 + dom.uartTabs.length) % dom.uartTabs.length;
      if (e.key === "Home") next = 0;
      if (e.key === "End") next = dom.uartTabs.length - 1;
      if (next !== null) {
        e.preventDefault();
        dom.uartTabs[next].focus();
        activateUartTab(next);
      }
    });
  });
  dom.btnUartSend.addEventListener("click", () => {
    if (!state.wasm || !dom.uartRxInput.value) return;
    const bytes = encoder.encode(dom.uartRxInput.value);
    withWasmBytes(bytes, (ptr, len) =>
      state.wasm.ch585_uart_inject_rx(state.activeUart, ptr, len),
    );
    dom.uartRxInput.value = "";
    refreshSnapshot();
  });
  dom.btnUartClear.addEventListener("click", () => {
    state.uartBuffers[state.activeUart] = "";
    renderUart();
  });

  // USB HS actions
  dom.btnUsbEnum.addEventListener("click", () => {
    state.wasm?.ch585_usb_enumerate();
    refreshSnapshot();
  });
  dom.btnUsbClear.addEventListener("click", () => {
    state.wasm?.ch585_usb_clear_packets();
    refreshSnapshot();
  });
  dom.btnUsbSend.addEventListener("click", () => {
    if (!state.wasm) return;
    const tokens = dom.usbOutHex.value.trim().split(/[\s,]+/).filter(Boolean);
    const buf = new Uint8Array(64);
    for (let i = 0; i < Math.min(64, tokens.length); i++) {
      buf[i] = Number.parseInt(tokens[i], 16) & 0xff;
    }
    const ep = Number(dom.usbEp.value) || 4;
    withWasmBytes(buf, (ptr, len) => state.wasm.ch585_usb_send_out(ep, ptr, len));
    refreshSnapshot();
  });
}

async function boot() {
  initLogicChannelSelectors();
  wireEvents();
  try {
    let result;
    try {
      result = await WebAssembly.instantiateStreaming(fetch("./ch585_wasm.wasm"), {});
    } catch {
      const resp = await fetch("./ch585_wasm.wasm");
      if (!resp.ok) throw new Error(`HTTP ${resp.status} loading ch585_wasm.wasm`);
      const bytes = await resp.arrayBuffer();
      result = await WebAssembly.instantiate(bytes, {});
    }
    state.wasm = result.instance.exports;
    state.memory = state.wasm.memory;
    state.wasm.ch585_init();
    refreshSnapshot();
    dom.statFw.textContent = `Ready (ABI v${state.wasm.ch585_wasm_abi_version()}) — select .elf`;
  } catch (err) {
    dom.statFw.textContent = `WASM Error: ${err.message}`;
  }
}

boot();
