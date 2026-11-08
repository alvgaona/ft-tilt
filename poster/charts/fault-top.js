// Section 1: top views of the flat and the tilted hexacopter during RPIC17's in-flight fault, at
// time state.t (s since the clip starts; motor 2 is capped at T_FAULT). Ring = PWM per motor; the drawing foreshortens with
// roll and pitch. Data units: arm length 1 = R px, both panels side by side.
const R = 74, W = 300, H = 250, TOP = 44; // px: one panel, and the title band above it
const T_FAULT = 5, T_GONE = T_FAULT + 1.3; // the flat vehicle has rolled over 1.3 s after the fault
const rad = Math.PI / 180;
const ring = 15 / R;
const VEHICLES = [
  { key: "std", title: "Flat, γ = 90° · pinv + clamp", color: "var(--c-std)", titleColor: "currentColor" },
  { key: "tilt", title: "Tilted, γ = 107° · null space", color: "var(--c-tilt)", titleColor: "var(--accent)" },
];

// Body point (x forward, y left) → page, x to the right and y up, foreshortened by roll and pitch.
const proj = (x, y, r, p) => [-y * Math.cos(r * rad), x * Math.cos(p * rad) + y * Math.sin(r * rad) * Math.sin(p * rad)];
// The data is 10 Hz; blend the two rows around t so the animation moves smoothly between them.
const KEYS = ["roll", "pitch", "pwm1", "pwm2", "pwm3", "pwm4", "pwm5", "pwm6"];
const at = (rows, t) => {
  const j = Math.max(1, Math.min(rows.length - 1, rows.findIndex((d) => d.t >= t)));
  const [a, b] = [rows[j - 1], rows[j]], k = Math.max(0, Math.min(1, (t - a.t) / (b.t - a.t)));
  return Object.fromEntries(KEYS.map((key) => [key, a[key] + k * (b[key] - a[key])]));
};

export default (rows, Plot, { t }) => {
  const arms = [], hubs = [], arcs = [], labels = [], bodies = [], titles = [];
  VEHICLES.forEach((v, k) => {
    const cx = (k * W) / R;
    const mine = rows.filter((d) => d.vehicle === v.key);
    const gone = v.key === "std" && t >= T_GONE;
    const d = at(mine, gone ? T_GONE - 0.1 : t);
    const faulty = t >= T_FAULT && t < T_FAULT + 15;
    const alpha = gone ? 0.35 : 1;
    const pos = [0, 1, 2, 3, 4, 5].map((m) => proj(Math.cos((m * Math.PI) / 3), Math.sin((m * Math.PI) / 3), d.roll, d.pitch));
    bodies.push({ pts: [...pos, pos[0]].map(([x, y]) => [cx + x / 3, y / 3]), color: v.color, alpha });
    pos.forEach(([x, y], m) => {
      const pwm = d[`pwm${m + 1}`];
      const color = m === 1 && faulty ? "var(--c-fault)" : v.color;
      arms.push({ x1: cx + 0.2 * x, y1: 0.2 * y, x2: cx + x, y2: y, color: v.color, alpha });
      hubs.push({ x: cx + x, y, pwm, n: m + 1, nx: cx + 1.38 * x, ny: 1.38 * y });
      const steps = Math.ceil(pwm / 2);
      for (let s = 0; s <= steps && pwm > 0; s++) {
        const a = ((2 * Math.PI * pwm) / 100) * (s / steps); // clockwise from the top
        arcs.push({ id: `${v.key}${m}`, x: cx + x + ring * Math.sin(a), y: y + ring * Math.cos(a), color, alpha });
      }
    });
    titles.push({ x: cx - W / 2 / R + 0.1, y: (122 + TOP - 13) / R, text: v.title, color: v.titleColor });
    titles.push({ x: cx - W / 2 / R + 0.1, y: (122 + TOP - 31) / R, text: gone ? "rolled over" : `roll ${Math.round(d.roll)}° · pitch ${Math.round(d.pitch)}°`,
      color: "var(--muted)", mono: true });
    labels.push({ x: cx, y: (122 - 12) / R, text: "front ↑" });
    if (gone) labels.push({ x: cx, y: 0, gone: true, text: "rolled over" });
  });
  return {
    width: 2 * W, height: H + TOP, margin: 0,
    x: { domain: [-W / 2 / R, (1.5 * W) / R], axis: null },
    y: { domain: [-(H - 122) / R, (122 + TOP) / R], axis: null },
    marks: [
      ...bodies.map((b) => Plot.line(b.pts, { fill: b.color, fillOpacity: 0.08 * b.alpha, stroke: b.color, strokeWidth: 1.2, strokeOpacity: b.alpha })),
      Plot.link(arms, { x1: "x1", y1: "y1", x2: "x2", y2: "y2", stroke: "color", strokeOpacity: "alpha", strokeWidth: 3, strokeLinecap: "round" }),
      Plot.dot(hubs, { x: "x", y: "y", r: 15, fill: "var(--bg)", stroke: "var(--border)", strokeWidth: 4 }),
      Plot.line(arcs, { x: "x", y: "y", z: "id", stroke: "color", strokeOpacity: "alpha", strokeWidth: 4, curve: "linear" }),
      Plot.text(hubs, { x: "x", y: "y", text: (d) => Math.round(d.pwm), fontSize: 10, fontFamily: "var(--font-mono)", fill: "var(--fg)" }),
      Plot.text(hubs, { x: "nx", y: "ny", text: "n", fontSize: 10, fill: "var(--muted)" }),
      Plot.text(labels.filter((l) => !l.gone), { x: "x", y: "y", text: "text", fontSize: 10, fill: "var(--muted)" }),
      Plot.text(labels.filter((l) => l.gone), { x: "x", y: "y", text: "text", fontSize: 20, fontWeight: 600, fontFamily: "var(--font-display)",
        fill: "var(--c-fault)", stroke: "var(--bg)", strokeWidth: 5, paintOrder: "stroke" }),
      Plot.text(titles.filter((l) => !l.mono), { x: "x", y: "y", text: "text", fill: "color", textAnchor: "start", fontSize: 14, fontWeight: 500 }),
      Plot.text(titles.filter((l) => l.mono), { x: "x", y: "y", text: "text", fill: "color", textAnchor: "start", fontSize: 13, fontFamily: "var(--font-mono)" }),
    ],
  };
};
