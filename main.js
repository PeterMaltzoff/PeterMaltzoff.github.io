/* ============================================================
   Main JS — Page logic, animation mounting, interactions
   ============================================================ */

document.addEventListener("DOMContentLoaded", () => {
  const E = window.ASCIIEngine;

  /* ───────── Navigation ───────── */
  const navbar = document.getElementById("navbar");
  const navToggle = document.getElementById("navToggle");
  const navLinks = document.getElementById("navLinks");

  window.addEventListener("scroll", () => {
    navbar.classList.toggle("scrolled", window.scrollY > 10);
  });

  if (navToggle) {
    navToggle.addEventListener("click", () => {
      navLinks.classList.toggle("open");
    });
  }

  /* close mobile nav on link click */
  if (navLinks) {
    navLinks.querySelectorAll("a").forEach((a) =>
      a.addEventListener("click", () => navLinks.classList.remove("open"))
    );
  }

  /* ───────── ASCII Animations ───────── */

  /* Hero — Neural Network */
  const heroEl = document.getElementById("asciiHero");
  if (heroEl) {
    const ctrl = new E.Controller(heroEl, E.NeuralNetwork, { fps: 18 });
    ctrl.start();
  }

  /* Data Stream divider */
  const dsEl = document.getElementById("asciiDataStream");
  if (dsEl) {
    const ctrl = new E.Controller(dsEl, E.DataStream, { fps: 16 });
    ctrl.start();
  }

  /* Waveform divider */
  const wfEl = document.getElementById("asciiWaveform");
  if (wfEl) {
    const ctrl = new E.Controller(wfEl, E.Waveform, { fps: 24 });
    ctrl.start();
  }

  /* CTA background */
  const ctaEl = document.getElementById("asciiCTA");
  if (ctaEl) {
    const ctrl = new E.Controller(ctaEl, E.DataStream, { fps: 12 });
    ctrl.start();
  }

  /* Booking page — Graph animation */
  const bookEl = document.getElementById("asciiBooking");
  if (bookEl) {
    const ctrl = new E.Controller(bookEl, E.Graph, { fps: 16 });
    ctrl.start();
  }

  /* Case study mini-animations */
  const caseAnimations = [E.Brain, E.Waveform, E.Graph];
  for (let i = 1; i <= 3; i++) {
    const el = document.getElementById("asciiCase" + i);
    if (el) {
      const ctrl = new E.Controller(el, caseAnimations[i - 1], { fps: 14 });
      ctrl.start();
    }
  }

  /* ───────── Fade-in on scroll ───────── */
  const fadeEls = document.querySelectorAll(".fade-in");
  if (fadeEls.length) {
    const fadeObserver = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            entry.target.classList.add("visible");
            fadeObserver.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.12, rootMargin: "0px 0px -40px 0px" }
    );
    fadeEls.forEach((el) => fadeObserver.observe(el));
  }

  /* ───────── Stat counter animation ───────── */
  const statNumbers = document.querySelectorAll("[data-count]");
  if (statNumbers.length) {
    const statObserver = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            animateCounter(entry.target);
            statObserver.unobserve(entry.target);
          }
        });
      },
      { threshold: 0.5 }
    );
    statNumbers.forEach((el) => statObserver.observe(el));
  }

  function animateCounter(el) {
    const target = parseInt(el.dataset.count, 10);
    const suffix = el.dataset.suffix || "";
    const duration = 1600;
    const start = performance.now();

    function tick(now) {
      const elapsed = now - start;
      const progress = Math.min(elapsed / duration, 1);
      const eased = 1 - Math.pow(1 - progress, 3); // ease-out cubic
      const current = Math.round(eased * target);
      el.textContent = current + suffix;
      if (progress < 1) requestAnimationFrame(tick);
    }
    requestAnimationFrame(tick);
  }

  /* ───────── Booking form ───────── */
  const form = document.getElementById("bookingForm");
  const formSuccess = document.getElementById("formSuccess");
  if (form && formSuccess) {
    form.addEventListener("submit", (e) => {
      e.preventDefault();
      /* In production, send to an API (Formspree, Netlify Forms, etc.) */
      form.style.display = "none";
      formSuccess.classList.add("active");
      window.scrollTo({ top: 0, behavior: "smooth" });
    });
  }
});
