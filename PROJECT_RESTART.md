# PROJECT_RESTART — structured notes

## 0) Short intro (why this exists)
This project is my bachelor’s final project. I’m still learning robotics/embedded/CAN, so I’m documenting everything to reduce anxiety and keep the work manageable. I’m building a prototype, not a perfect industry-grade system. The goal is a working result; “good enough” is enough for a bachelor’s project.

## 1) One‑paragraph summary
**Purpose:** Build a simple embedded control system for a 6‑axis robot arm that can reliably move joints to target angles with feedback over CAN, suitable for a short exhibition demo. Думать будешь в аспирантуре;)

## 2) Scope (what we build vs what we skip)
### In scope (must have)
- Reliable motion to target angles
- Basic feedback handling (confirm reached / retry / detect error)
- Emergency stop
- Simple host GUI controls
- Demo works repeatedly

### Out of scope (for now)
- Full industrial reliability (24/7 production)
- Advanced kinematics (optional)
- Perfection/optimization beyond demo needs

## 3) Demo scenario (exhibition requirements)
**Target date:** late April prototype, mid‑May exhibition

Robot task at demo:
- Conveyor stops
- Robot recognizes white cubes
- Robot picks white cubes one by one and places them into a separate bin

Notes:
- Gripper likely vacuum with discrete output (ON/OFF)
- Kinematic solver may be skipped

## 4) System behavior / user features (GUI)
- Directional jog buttons (XD/XU/YD/YU/ZD/ZU)
- Status display: moving / reached target / error
- Emergency stop
- Speed slider (1–100% for longest‑distance motor; others scaled)
- Set joint target angles, validate limits, start movement

## 5) Hardware checklist
- MCU: STM32F103C8T6 (Blue Pill) — consider memory upgrade if needed
- Motors (AVATAR M Series):
    - 1J: M8025E25B_50_L
    - 2J: M8025E25B_50_L
    - 3J: M8010E17B_50_L
    - 4J: M4215E14B_50_L
    - 5J: M4215E14B_50_L
    - 6J: TBD
- CAN transceiver: TJA1050
- Motor info: https://atarrobot.com/harmonic-robot/m-series/M-robot-joint.html

## 6) Current status
### Works
- MAJ/MRJ commands: set target positions, compute speeds, send sync

### Doesn’t work / missing
- Readback of motor state (no CAN RX handling yet)
- Confirmation that motors reached targets
- Error handling and recovery

## 7) How to run (quick start)
1. Connect CANH/CANL to TJA1050
2. Power motors (36V)
3. Connect USB‑UART
4. Select port and reset MCU

## 8) Core goals (only these matter)
- Code runs reliably every time
- You can command movement and verify result
- You can explain the system to your supervisor

## 9) Design principles (for me)
- Clarity over cleverness
- Simple, readable, testable code
- Beginner‑friendly steps
> Идеальное — враг хорошего

## 10) Immediate next tasks
### Next 2 weeks (likely)
- Meet supervisor to align goals
- Clean code + documentation; remove unused files
- Track changes in a changelog / Notion
- Make move command reliable (angle + speed)
- Read motor data and show in serial

### Next 90 minutes (example micro‑plan)
- Move hardware init out of global scope into `setup()`
- Remove unused files / commented code
- Make project compile
- Add 5–10 lines to a CHANGELOG / tracker

## 11) CANopen cheat sheet (TODO)
- How to set target position
- How to set speed/accel
- How to sync
- How to read status/position
- Error codes and recovery actions

## 12) Questions for supervisor
Ask when you can say:
- “Here’s what works now.”
- “Here’s what I’m unsure about.”
- “Here’s what I plan to do next — does this make sense?”

## 13) Notes / log (raw thoughts)
- Keep dated notes below (no need to be pretty)
- Old notes can be moved here and summarized later

