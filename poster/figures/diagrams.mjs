// Bake the $…$ labels of figures/src/*.svg into figures/*.svg with KaTeX (posuta's tex tool), so
// the diagrams need no JS and print as drawn. Edit the sources, then: node figures/diagrams.mjs
import { readFileSync, readdirSync, writeFileSync } from "node:fs";
import { tex } from "@posuta/posuta/tex";

const dir = new URL("./", import.meta.url);
for (const f of readdirSync(new URL("src/", dir)).filter((f) => f.endsWith(".svg"))) {
  const svg = readFileSync(new URL(`src/${f}`, dir), "utf8")
    .replace(/(<div [^>]*class="fo">)([^<]*\$[^<]*)(<\/div>)/g, (_, a, s, b) => a + tex(s) + b);
  writeFileSync(new URL(f, dir), svg);
  console.log(`figures/${f}`);
}
