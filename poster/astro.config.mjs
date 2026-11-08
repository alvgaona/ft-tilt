import { defineConfig } from "astro/config";
import posuta from "@posuta/posuta";

// SITE and BASE_PATH come from GitHub Pages in the deploy workflow; set them for other hosts.
// `site` also gives the poster its QR code and link previews.
export default defineConfig({
  site: process.env.SITE,
  base: process.env.BASE_PATH,
  integrations: [posuta()],
});
