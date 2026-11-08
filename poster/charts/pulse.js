// Section 5: ICRIS18 Fig. 6, pitch during a −1 N·m pitch pulse (5–15 s) with motor 2 off.
const SERIES = [
  { key: "nofault", name: "No fault, aggressive", color: "var(--c-std)", dash: "4 3", width: 1.6 },
  { key: "aggr", name: "Motor 2 off, aggressive", color: "var(--c-aggr)", width: 1.6 },
  { key: "mild", name: "Motor 2 off, mild", color: "var(--c-tilt)", width: 2.2 },
];

export default (rows, Plot) => {
  const peak = rows.reduce((a, b) => (b.mild > a.mild ? b : a));
  return {
    width: 560, height: 250, marginLeft: 40, marginRight: 12, marginTop: 34, marginBottom: 24,
    style: { fontSize: "11px" },
    x: { domain: [0, 25], ticks: [0, 5, 10, 15, 20, 25], tickFormat: (s) => `${s} s`, label: null, tickSize: 0 },
    y: { domain: [-30, 30], ticks: [-30, -15, 0, 15, 30], tickFormat: (v) => `${v}°`, grid: true, label: "Pitch angle", labelArrow: "none", tickSize: 0 },
    color: { domain: SERIES.map((s) => s.name), range: SERIES.map((s) => s.color), legend: true },
    marks: [
      Plot.rect([0], { x1: () => 5, x2: () => 15, fill: "var(--fault-tint)" }),
      Plot.ruleY([0], { stroke: "var(--muted)" }),
      ...SERIES.map((s) => Plot.line(rows, { x: "t", y: s.key, stroke: () => s.name, strokeWidth: s.width, strokeDasharray: s.dash, strokeLinejoin: "round" })),
      Plot.text(["−1 N·m pitch torque"], { x: 10, y: 30, dy: 12, text: (d) => d, fill: "var(--c-fault)" }),
      Plot.text([peak], { x: "t", y: "mild", dx: 8, text: (d) => `${d.mild.toFixed(0)}°`, textAnchor: "start", fontFamily: "var(--font-mono)", fill: "var(--fg)" }),
    ],
  };
};
