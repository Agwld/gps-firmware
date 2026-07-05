# Documentation Map

Quick reference for finding the right document.

## For different audiences

### I'm a driver — how do I use the timing system?
→ **[DRIVER_GUIDE.md](DRIVER_GUIDE.md)** covers gate setup, button presses, troubleshooting, pre-event checklist

### I want to get the firmware building in 5 minutes
→ **[QUICK_START.md](QUICK_START.md)** has toolchain install, CMake presets, build commands

### I'm a developer and want to work on the firmware
→ **[DEVELOPER.md](DEVELOPER.md)** covers environment setup, build system, testing, debugging, CI, code style

### I need to understand how the system works
→ **[ARCHITECTURE.md](ARCHITECTURE.md)** explains sensor fusion, lap timing, CAN broadcast, task architecture, persistence

### I need hardware specs, pinouts, CAN signals, memory layout
→ **[DATASHEET.md](DATASHEET.md)** has all component specs, electrical characteristics, CAN matrix, signal scaling

### I want to know why certain design choices were made
→ **[NOTES.md](NOTES.md)** covers design rationale, known limitations, future enhancements, performance profiling

## Document structure

```
README.md ─────────────────────────────────────┐
           (product overview, features, status) │
           ↓ links to all guides below          │
           ├─→ DRIVER_GUIDE.md         (users)  │
           ├─→ QUICK_START.md          (5 min)  │
           ├─→ DEVELOPER.md            (eng)    │ These form a
           ├─→ ARCHITECTURE.md         (design) │ complete suite
           ├─→ DATASHEET.md            (specs)  │
           └─→ NOTES.md                (future) │
                                                 │
This file (DOCUMENTATION.md) ←──────────────────┘
   (navigation guide)
```

## Quick lookup table

| Question | Document | Section |
|----------|----------|---------|
| "How do I set a gate?" | DRIVER_GUIDE | Setting up gates |
| "What's the button FSM?" | DRIVER_GUIDE | Creating gates: the button FSM |
| "How do I build the firmware?" | QUICK_START | — |
| "What's the CPU load?" | DEVELOPER | Performance |
| "How does sensor fusion work?" | ARCHITECTURE | Sensor fusion |
| "What's the CAN matrix?" | DATASHEET | CAN message matrix |
| "What are the MCU specs?" | DATASHEET | Microcontroller |
| "Why absolute lat/lon for gates?" | NOTES | Absolute lat/lon for gate storage |
| "What about RTK support?" | NOTES | Limited RTK support |
| "How do I debug on hardware?" | DEVELOPER | Debugging |
| "What are the test commands?" | DEVELOPER | Testing |
| "How do gates persist?" | ARCHITECTURE | Gate persistence |

## Documentation philosophy

- **README** — Product-focused, minimal jargon, links to everything else
- **QUICK_START** — Fastest path to success; assumes minimal context
- **DEVELOPER** — Comprehensive reference for hands-on work (build, test, debug)
- **DRIVER_GUIDE** — User-friendly; no firmware internals, just "what to do"
- **ARCHITECTURE** — Deep technical dive; explains *why* systems are designed certain ways
- **DATASHEET** — Authoritative specs; numbers, memory maps, electrical characteristics
- **NOTES** — Forward-looking; design rationale, limitations, future work (for maintainers)

## File formats

All documents are **markdown** (`.md`), checked into git, and viewable on GitHub or locally. No PDFs are auto-generated; if a PDF is needed, markdown can be converted via Pandoc or similar:

```bash
pandoc README.md -o README.pdf --pdf-engine=xhtml2pdf
# or use online converters
```

## Updates and maintenance

- **README**: Update when product status changes (e.g., "pending testing" → "production-ready")
- **QUICK_START**: Update when build presets or toolchain requirements change
- **DEVELOPER**: Update when adding new tests, build features, or development workflows
- **DRIVER_GUIDE**: Update when button FSM changes or new features affect drivers
- **ARCHITECTURE**: Stable; rarely changes (core design frozen)
- **DATASHEET**: Update when hardware specs change (firmware memory, CAN rates, electrical characteristics)
- **NOTES**: Update as new decisions are made and future enhancements are planned

---

**Lost? Start with [README.md](README.md).**

*Last updated: 2026-07-05*
