// Minimal JS for navigation, theme toggle, and copy buttons

(function () {
  const doc = document;

  // Mobile nav
  const navToggle = doc.querySelector("[data-nav-toggle]");
  const nav = doc.querySelector("[data-site-nav]");
  if (navToggle && nav) {
    navToggle.addEventListener("click", () => {
      const isOpen = nav.classList.toggle("nav-open");
      navToggle.setAttribute("aria-expanded", isOpen ? "true" : "false");
    });
  }

  // Optional theme toggle (dark/light)
  const themeToggle = doc.querySelector("[data-theme-toggle]");
  if (themeToggle) {
    const root = doc.documentElement;
    const stored = window.localStorage.getItem("rdz-theme");
    if (stored) {
      root.setAttribute("data-theme", stored);
    }
    themeToggle.addEventListener("click", () => {
      const current = root.getAttribute("data-theme") || "dark";
      const next = current === "dark" ? "light" : "dark";
      root.setAttribute("data-theme", next);
      window.localStorage.setItem("rdz-theme", next);
    });
  }

  // Self-rotating deployment gallery carousel
  const carouselTrack = doc.querySelector("[data-carousel]");
  if (carouselTrack) {
    const slides = carouselTrack.querySelectorAll(".hero-carousel-slide");
    if (slides.length > 1) {
      let current = 0;
      const interval = 4500;
      function advance() {
        current = (current + 1) % slides.length;
        const target = slides[current];
        const trackRect = carouselTrack.getBoundingClientRect();
        const targetRect = target.getBoundingClientRect();
        carouselTrack.scrollLeft += targetRect.left - trackRect.left;
      }
      let tid = setInterval(advance, interval);
      carouselTrack.addEventListener("mouseenter", () => clearInterval(tid));
      carouselTrack.addEventListener("mouseleave", () => { tid = setInterval(advance, interval); });
    }
  }

  // Copy buttons for code blocks
  const codeBlocks = doc.querySelectorAll("[data-code-block]");
  codeBlocks.forEach((block) => {
    const btn = block.querySelector("[data-copy]");
    const pre = block.querySelector("pre, code");
    if (!btn || !pre) return;

    btn.addEventListener("click", async () => {
      const text = pre.innerText;
      try {
        await navigator.clipboard.writeText(text);
        btn.textContent = "Copied";
        setTimeout(() => {
          btn.textContent = "Copy";
        }, 1500);
      } catch {
        btn.textContent = "Error";
        setTimeout(() => {
          btn.textContent = "Copy";
        }, 1500);
      }
    });
  });
})();
