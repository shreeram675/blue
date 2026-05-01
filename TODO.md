# Blueprint Navigation - Canvas/Overlay Size Mismatch Fix
Current Working Directory: c:/Users/shreeram/OneDrive/Desktop/rmsblue/Blueprint-nav

## Plan Summary
Fix canvas-overlay size mismatch by setting HTML <img> width/height attributes to match canvas buffer size in JS.

**Files to edit**: templates/index.html (JS/CSS)

## Implementation Steps
- [x] Step 1: Added `syncOverlaySizes()` JS function.
- [x] Step 2: CSS `.overlay-img { width/height: auto; z-index:2; }`.
- [x] Step 3: Canvas `{ position:relative; z-index:1; }`.
- [x] Step 4: Calls in load/toggle/resize handler.
- [ ] Step 5: Test: `python app.py`, upload blueprint, verify overlay alignment.
- [x] Step 6: Ready for completion.

**Progress**: Code changes applied successfully. Test recommended before finalizing.


