// Section 1: tilt from level over time for both vehicles; the dashed line is the time the top
// views show (state.t, s since the clip starts; the fault is at T_FAULT). Tilt = the angle between body z and vertical.
const rad = Math.PI / 180;
const T_FAULT = 5, T_GONE = T_FAULT + 1.3, TMAX = 50;

export default (rows, Plot, { t }) => {
  const tilt = rows
    .filter((d) => d.vehicle === "tilt" || d.t < T_GONE)
    .map((d) => ({ t: d.t, vehicle: d.vehicle === "tilt" ? "tilted" : "flat",
      tilt: Math.min(TMAX, Math.acos(Math.cos(d.roll * rad) * Math.cos(d.pitch * rad)) / rad) }));
  return {
    width: 560, height: 330, marginLeft: 40, marginRight: 12, marginTop: 22, marginBottom: 24,
    style: { fontSize: "11px" },
    x: { domain: [0, 25], ticks: [0, 5, 10, 15, 20, 25], tickFormat: (s) => `${s} s`, label: null, tickSize: 0 },
    y: { domain: [0, TMAX], ticks: [0, 25, 50], tickFormat: (v) => `${v}°`, grid: true, label: "Tilt from level", labelArrow: "none", tickSize: 0 },
    color: { domain: ["flat", "tilted"], range: ["var(--c-std)", "var(--c-tilt)"], label: "vehicle" },
    marks: [
      Plot.rect([0], { x1: () => T_FAULT, x2: () => T_FAULT + 15, fill: "var(--fault-tint)" }),
      Plot.ruleY([0], { stroke: "var(--muted)" }),
      Plot.line(tilt, { x: "t", y: "tilt", stroke: "vehicle", strokeWidth: 2, strokeLinejoin: "round" }),
      Plot.text(["motor 2 capped at 40 % PWM"], { x: T_FAULT + 2, y: TMAX, dy: 12, text: (d) => d, textAnchor: "start", fill: "var(--c-fault)" }),
      Plot.text(["← flat: rolled over 1.3 s after the fault"], { x: T_GONE, y: 34, dx: 8, text: (d) => d, textAnchor: "start", fill: "var(--fg-2)" }),
      Plot.text(["tilted: ≤ 0.7°"], { x: 25, y: 0, dy: -6, text: (d) => d, textAnchor: "end", fill: "var(--c-tilt)" }),
      Plot.ruleX([t], { stroke: "var(--fg)", strokeDasharray: "2 2" }),
    ],
  };
};
