// Section 6: ICRIS18 Fig. 11, mean PWM per motor in hover, healthy vs motor 2 off.
export default (rows, Plot) => ({
  width: 560, height: 230, marginLeft: 40, marginRight: 12, marginTop: 30, marginBottom: 24,
  style: { fontSize: "11px" },
  fx: { tickFormat: (m) => `motor ${m}`, label: null, padding: 0.25, tickSize: 0 },
  x: { axis: null, domain: ["healthy", "motor 2 off"], padding: 0.1 },
  y: { domain: [0, 100], ticks: [0, 25, 50, 75, 100], tickFormat: (v) => `${v}%`, grid: true, label: "Mean PWM in hover", labelArrow: "none", tickSize: 0 },
  color: { domain: ["healthy", "motor 2 off"], range: ["var(--c-std)", "var(--c-tilt)"], legend: true },
  marks: [
    Plot.barY(rows, { fx: "motor", x: "state", y: "pwm", fill: "state", rx: 2 }),
    Plot.ruleY([14], { stroke: "var(--c-fault)", strokeDasharray: "4 3" }),
    Plot.text(["14 % floor"], { x: () => "motor 2 off", fx: () => 2, y: 14, dy: -6, dx: 2, text: (d) => d, textAnchor: "start", fill: "var(--c-fault)" }),
    Plot.text(rows.filter((d) => d.state !== "healthy"), { fx: "motor", x: "state", y: "pwm", dy: -6,
      text: (d) => (d.pwm < 1 ? "off" : Math.round(d.pwm)), fontFamily: "var(--font-mono)",
      fill: (d) => (d.motor === 2 ? "var(--c-fault)" : "var(--fg)") }),
    Plot.ruleY([0], { stroke: "var(--muted)" }),
  ],
});
