const SVG_NS = "http://www.w3.org/2000/svg";

function el(tag, attrs = {}, textContent = null) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    node.setAttribute(k, String(v));
  }
  if (textContent !== null) {
    node.textContent = textContent;
  }
  return node;
}

export class Qfn48Widget {
  /**
   * @param {HTMLElement} container
   * @param {{ onPinClick: (pinInfo: object) => void }} callbacks
   */
  constructor(container, callbacks) {
    this.container = container;
    this.callbacks = callbacks;
    this.pinNodes = new Map();
    this.selectedPinNum = null;
    this.lastPins = [];
    this.#buildSvg();
  }

  #buildSvg() {
    const svg = el("svg", {
      viewBox: "0 0 640 640",
      class: "qfn48-svg",
      role: "img",
      "aria-label": "CH585M QFN48 48-pin interactive chip view",
    });
    const title = el("title", {}, "CH585M QFN48 48-pin interactive package");
    svg.appendChild(title);

    // Subtle PCB / substrate backdrop
    svg.appendChild(
      el("rect", {
        x: 6,
        y: 6,
        width: 628,
        height: 628,
        rx: 18,
        class: "qfn-board-bg",
      }),
    );

    // Molded IC package body (centered at 320, 320)
    const bodyX = 160;
    const bodyY = 160;
    const bodyW = 320;
    const bodyH = 320;
    svg.appendChild(
      el("rect", {
        x: bodyX,
        y: bodyY,
        width: bodyW,
        height: bodyH,
        rx: 10,
        class: "qfn-package-body",
      }),
    );

    // Pin 1 orientation dot
    svg.appendChild(
      el("circle", {
        cx: bodyX + 24,
        cy: bodyY + 24,
        r: 7,
        class: "qfn-pin1-dot",
      }),
    );

    // Exposed thermal pad
    svg.appendChild(
      el("rect", {
        x: 245,
        y: 245,
        width: 150,
        height: 150,
        rx: 6,
        class: "qfn-thermal-pad",
      }),
    );
    svg.appendChild(el("text", { x: 320, y: 215, class: "qfn-chip-title" }, "WCH CH585M"));
    svg.appendChild(el("text", { x: 320, y: 233, class: "qfn-chip-sub" }, "QingKe V3C · QFN48"));
    svg.appendChild(el("text", { x: 320, y: 316, class: "qfn-gnd-label" }, "EPAD / GND"));
    this.statusLine = el("text", { x: 320, y: 338, class: "qfn-live-label" }, "No ELF loaded");
    svg.appendChild(this.statusLine);

    // 48 pins arranged CCW around body [bodyX..bodyX+bodyW, bodyY..bodyY+bodyH]
    const pitch = 22.5;
    const firstOffset = 36;
    const padLen = 28;
    const padThick = 15;

    for (let pin = 1; pin <= 48; pin++) {
      let px = 0;
      let py = 0;
      let pw = padLen;
      let ph = padThick;
      let numX = 0;
      let numY = 0;
      let lblX = 0;
      let lblY = 0;
      let lblAnchor = "end";
      let textRot = null;

      if (pin <= 12) {
        // Left edge: top -> bottom (1..12)
        const idx = pin - 1;
        py = bodyY + firstOffset + idx * pitch - padThick / 2;
        px = bodyX - padLen + 4;
        numX = bodyX + 12;
        numY = py + padThick / 2 + 3.5;
        lblX = px - 7;
        lblY = py + padThick / 2 + 4;
        lblAnchor = "end";
      } else if (pin <= 24) {
        // Bottom edge: left -> right (13..24)
        const idx = pin - 13;
        pw = padThick;
        ph = padLen;
        px = bodyX + firstOffset + idx * pitch - padThick / 2;
        py = bodyY + bodyH - 4;
        numX = px + padThick / 2;
        numY = bodyY + bodyH - 12;
        lblX = px + padThick / 2 + 3.5;
        lblY = py + padLen + 8;
        lblAnchor = "start";
        textRot = `rotate(90 ${lblX} ${lblY})`;
      } else if (pin <= 36) {
        // Right edge: bottom -> top (25..36)
        const idx = pin - 25;
        py = bodyY + bodyH - firstOffset - idx * pitch - padThick / 2;
        px = bodyX + bodyW - 4;
        numX = bodyX + bodyW - 12;
        numY = py + padThick / 2 + 3.5;
        lblX = px + padLen + 7;
        lblY = py + padThick / 2 + 4;
        lblAnchor = "start";
      } else {
        // Top edge: right -> left (37..48)
        const idx = pin - 37;
        pw = padThick;
        ph = padLen;
        px = bodyX + bodyW - firstOffset - idx * pitch - padThick / 2;
        py = bodyY - padLen + 4;
        numX = px + padThick / 2;
        numY = bodyY + 18;
        lblX = px + padThick / 2 + 3.5;
        lblY = py - 7;
        lblAnchor = "start";
        textRot = `rotate(-90 ${lblX} ${lblY})`;
      }

      const g = el("g", {
        class: "qfn-pin-group",
        role: "button",
        tabindex: "0",
        "data-pin": pin,
        "aria-label": `Pin ${pin}`,
      });

      const pad = el("rect", {
        x: px,
        y: py,
        width: pw,
        height: ph,
        rx: 3,
        class: "qfn-pin-pad state-hiz",
      });

      const numTxt = el("text", { x: numX, y: numY, class: "qfn-pin-num" }, String(pin));
      const lblAttrs = {
        x: lblX,
        y: lblY,
        "text-anchor": lblAnchor,
        class: "qfn-pin-label",
      };
      if (textRot) {
        lblAttrs.transform = textRot;
      }
      const lblTxt = el("text", lblAttrs, `P${pin}`);

      const extBadge = el("circle", {
        cx: px + pw / 2,
        cy: py + ph / 2,
        r: 3.5,
        class: "qfn-ext-dot hidden",
      });

      g.appendChild(pad);
      g.appendChild(numTxt);
      g.appendChild(lblTxt);
      g.appendChild(extBadge);

      const activate = () => {
        const info = this.lastPins[pin - 1];
        if (info) {
          this.callbacks.onPinClick?.(info);
        }
      };
      g.addEventListener("click", activate);
      g.addEventListener("keydown", (e) => {
        if (e.key === "Enter") {
          activate();
        } else if (e.key === " ") {
          e.preventDefault();
        }
      });
      g.addEventListener("keyup", (e) => {
        if (e.key === " ") {
          activate();
        }
      });

      svg.appendChild(g);
      this.pinNodes.set(pin, { g, pad, lblTxt, extBadge });
    }

    this.container.replaceChildren(svg);
  }

  setSelectedPin(pinNum) {
    this.selectedPinNum = pinNum;
    for (const [p, node] of this.pinNodes.entries()) {
      node.g.classList.toggle("selected", p === pinNum);
    }
  }

  /**
   * @param {Array<object>} pins
   * @param {string} statusText
   */
  update(pins, statusText) {
    this.lastPins = pins;
    if (statusText) {
      this.statusLine.textContent = statusText;
    }
    for (const p of pins) {
      const node = this.pinNodes.get(p.pin);
      if (!node) continue;
      node.lblTxt.textContent = p.name;
      node.pad.setAttribute(
        "class",
        `qfn-pin-pad state-${p.state}${p.out ? " is-out" : ""}`,
      );
      const forced = p.ext !== "float" && p.ext !== "power" && p.ext !== "analog";
      node.extBadge.classList.toggle("hidden", !forced);
      node.extBadge.setAttribute(
        "class",
        `qfn-ext-dot${forced ? "" : " hidden"} ext-${p.ext}`,
      );
      node.g.setAttribute(
        "aria-label",
        `Pin ${p.pin} ${p.name} (${p.state.toUpperCase()}${p.out ? ", OUT" : ""}${p.adc !== null ? `, AIN${p.adc}` : ""})`,
      );
    }
  }
}
