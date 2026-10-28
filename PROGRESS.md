# Progress of the 3.3 update – 28 October 2026

Work is on the git branch `update-3.3` (not pushed). Everything from the task list is done except the
items under "Still open".

## Done

1. **Setup.** Git history from GitHub (3.1.0) + a baseline commit of the 3.2.1 release as delivered, then
   the 3.3 work on top. Baseline test run before any change: 13 passed, 4 failed (bugs #33–#36).
2. **Environment.** Installed (user space): playwright + Chromium, apktool 2.9.3, uber-apk-signer 1.3.0,
   MediaPipe tasks-vision 1.1.1 (bundled in the app), TeX Live 2026 (`~/texlive`), pandoc 3.5 and
   LanguageTool 6.3 (`~/tools`). ROS 2 Humble, Gazebo Harmonic, MuJoCo, OpenCV, Node and Java were already there.
3. **Gesture control in the app / website**: MediaPipe Hand Landmarker offline, two hands, pinch, fallback
   colour tracker, reasons on screen, skeleton; APK 3.3.0 rebuilt with the same signing key.
4. **New features**: draw a path, programs export/import, belt tracking (MuJoCo and the app; switched off in
   Gazebo, see below), demo for the review, hardware bring-up check and docs/HARDWARE_GUIDE.md.
5. **Bugs fixed** #33–#38 (docs/FIXED_BUGS.md), each with a test.
6. **Docs** for both versions; the plain version `../SCARA_SIMULATION` has every change and no name
   (`grep -r SUBHANI` is empty there).
7. **Full test suite**: named version 21/21 (the twin test re-run after a timing fix), plain version 21/21,
   0 skipped, Gazebo included.
8. **Thesis (104 pages) and report (45 pages)** updated with the 3.3 work in the same student voice, built
   with pdflatex: 0 errors, 0 undefined references, 0 overfull lines, 0 warnings. Flowcharts checked; the
   homing chart's loop ran through two boxes and the thesis cycle chart had a slanted arrow – both fixed, and
   the thesis now uses the hardware-free versions of the diagrams. No AI statement was added (as asked).
9. **Paper draft** in `../research_paper_q1/` (see its README): checked references, gap, journal shortlist,
   experimental plan, results validation, figures from archived data.

## Belt tracking in Gazebo – result

Two attempts as agreed: (1) the camera-measured belt speed was right (0.079 m/s), the misses remained;
(2) planning 0.2 s ahead for the controller lag: 2 of 6 picks came down around the puck, 1 held. Gazebo's
controller lag changes from pick to pick, so tracking is switched off in Gazebo (the app says so) and the
README explains it. `tools/test_gazebo.py` checks the refusal and passes.

## Still open (needs you)

- Push the branch (you asked to be asked first).
- The APK on a real phone, and anything on the real robot (no phone or robot here).
- Paper: author list, funding / conflicts, the AI-use declaration the journal requires, the target journal,
  and the hardware experiments in `research_paper_q1/experimental_plan.md`.
- The Android package id is `com.iamsubhani.scaradigitaltwin` in both versions (it was so before). It contains
  the name in lower case; changing it would make the app a different app that cannot update the old one.
  Your choice.
- This file is a work log; delete it before pushing if you do not want it on GitHub.
