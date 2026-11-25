const DEFAULT_CHANNELS = [
  { port: "A", idx: 8, label: "PA8 (UART1_TX)" },
  { port: "A", idx: 9, label: "PA9 (UART1_RX)" },
  { port: "B", idx: 9, label: "PB9 (VBUS)" },
  { port: "B", idx: 20, label: "PB20" },
  { port: "B", idx: 22, label: "PB22 (UART2_TX)" },
  { port: "B", idx: 23, label: "PB23 (UART2_TX)" },
  { port: "A", idx: 14, label: "PA14 (SPI0_MOSI)" },
  { port: "B", idx: 4, label: "PB4" },
];

const TRACE_COLORS = [
  "#38bdf8",
  "#22c55e",
  "#f59e0b",
  "#a78bfa",
  "#f472b6",
  "#2dd4bf",
  "#fb7185",
  "#facc15",
];

function sampleBit(sample, ch) {
  if (ch.port === "A") {
    return (sample[1] >>> ch.idx) & 1;
  }
  return (sample[2] >>> ch.idx) & 1;
}

export class LogicAnalyzerWidget {
  /**
   * @param {HTMLCanvasElement} canvas
   */
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.channels = DEFAULT_CHANNELS.map((c) => ({ ...c }));
    this.samples = [];
    this.zoom = 1;
    this.lastRenderKey = "";
  }

  getChannels() {
    return this.channels;
  }

  setChannel(slot, port, idx) {
    if (slot < 0 || slot >= this.channels.length) return;
    this.channels[slot] = { port, idx, label: `P${port}${idx}` };
    this.lastRenderKey = "";
    this.render(true);
  }

  setZoom(zoom) {
    this.zoom = Math.max(0.25, Math.min(16, zoom));
    this.lastRenderKey = "";
    this.render(true);
  }

  updateSamples(samples, totalCount, latestTimeUs) {
    const key = `${totalCount}:${samples.length}:${samples.at(-1)?.[0] ?? 0}:${this.zoom}`;
    this.samples = samples;
    this.latestTimeNs = latestTimeUs * 1000;
    if (key !== this.lastRenderKey) {
      this.lastRenderKey = key;
      this.render();
    }
  }

  render(force = false) {
    const canvas = this.canvas;
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(320, Math.floor(rect.width));
    const height = Math.max(200, Math.floor(rect.height));

    if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
      canvas.width = Math.floor(width * dpr);
      canvas.height = Math.floor(height * dpr);
      force = true;
    }
    if (!force && !this.ctx) return;

    const ctx = this.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    // Background
    ctx.fillStyle = "#0b1120";
    ctx.fillRect(0, 0, width, height);

    const labelW = 128;
    const topPad = 22;
    const rowH = Math.floor((height - topPad - 8) / this.channels.length);
    const plotW = Math.max(80, width - labelW - 12);

    // Vertical grid & header
    ctx.strokeStyle = "rgba(148, 163, 184, 0.14)";
    ctx.lineWidth = 1;
    for (let g = 0; g <= 8; g++) {
      const gx = labelW + (plotW * g) / 8;
      ctx.beginPath();
      ctx.moveTo(gx, topPad);
      ctx.lineTo(gx, height - 6);
      ctx.stroke();
    }

    const samples = this.samples;
    if (!samples || samples.length === 0) {
      ctx.fillStyle = "#94a3b8";
      ctx.font = "12px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillText("No logic transitions captured yet", labelW + 14, height / 2);
      return;
    }

    const tEnd = Math.max(samples.at(-1)[0], this.latestTimeNs || 0, samples[0][0] + 1000);
    const fullSpan = Math.max(1000, tEnd - samples[0][0]);
    const viewSpan = Math.max(1000, fullSpan / this.zoom);
    const tStart = Math.max(0, tEnd - viewSpan);

    // Time axis labels
    ctx.fillStyle = "#94a3b8";
    ctx.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    for (let g = 0; g <= 4; g++) {
      const frac = g / 4;
      const tNs = tStart + viewSpan * frac;
      const label =
        tNs >= 1_000_000
          ? `${(tNs / 1_000_000).toFixed(2)}ms`
          : `${(tNs / 1000).toFixed(1)}us`;
      const x = labelW + plotW * frac;
      ctx.fillText(label, Math.min(width - 48, x + 2), 14);
    }

    const timeToX = (tNs) => labelW + ((tNs - tStart) / viewSpan) * plotW;

    // Draw each channel trace
    this.channels.forEach((ch, rowIdx) => {
      const yTop = topPad + rowIdx * rowH;
      const yHigh = yTop + 5;
      const yLow = yTop + rowH - 6;
      const color = TRACE_COLORS[rowIdx % TRACE_COLORS.length];

      // Row divider & label
      ctx.fillStyle = rowIdx % 2 === 0 ? "rgba(15, 23, 42, 0.55)" : "rgba(30, 41, 59, 0.35)";
      ctx.fillRect(0, yTop, width, rowH);

      ctx.fillStyle = "#e2e8f0";
      ctx.font = "11px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillText(ch.label, 8, yTop + rowH / 2 + 4);

      ctx.strokeStyle = color;
      ctx.lineWidth = 1.6;
      ctx.beginPath();

      // Determine starting value at tStart
      let prevBit = sampleBit(samples[0], ch);
      for (let i = 0; i < samples.length; i++) {
        if (samples[i][0] <= tStart) {
          prevBit = sampleBit(samples[i], ch);
        } else {
          break;
        }
      }

      let curY = prevBit ? yHigh : yLow;
      ctx.moveTo(labelW, curY);

      for (let i = 0; i < samples.length; i++) {
        const s = samples[i];
        if (s[0] <= tStart) continue;
        if (s[0] > tEnd) break;
        const x = timeToX(s[0]);
        const nextBit = sampleBit(s, ch);
        ctx.lineTo(x, curY);
        if (nextBit !== prevBit) {
          curY = nextBit ? yHigh : yLow;
          ctx.lineTo(x, curY);
          prevBit = nextBit;
        }
      }
      ctx.lineTo(labelW + plotW, curY);
      ctx.stroke();
    });
  }
}
