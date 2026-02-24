# Deployment gallery images

Photos for the overview page carousel. Images are displayed in a 4:3 frame and scaled by CSS.

**Current images** were generated from `rdzsonde-web/img/`: center‑cropped to 4:3, resized to 640×480, JPG quality 85. Sources: img1.png … img3.png, and img4.jpg for the fourth slide. To regenerate:

```bash
cd rdzsonde-web
magick img/img1.png -gravity center -crop 4:3+0+0 +repage -resize 640x480 -quality 85 assets/img/gallery/01-esp32-cyd-sx1278.jpg
magick img/img2.png -gravity center -crop 4:3+0+0 +repage -resize 640x480 -quality 85 assets/img/gallery/02-m5stack.jpg
magick img/img3.png -gravity center -crop 4:3+0+0 +repage -resize 640x480 -quality 85 assets/img/gallery/03-esp32-tft.jpg
magick img/img4.jpg -gravity center -crop 4:3+0+0 +repage -resize 640x480 -quality 85 assets/img/gallery/04-esp32-oled.jpg
```

**Adding more slides:** Add the image to this folder, then in `index.html` copy one slide block and set `src` and caption. Use 4:3 or similar; 640×480 is a good target size.
