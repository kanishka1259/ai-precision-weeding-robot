# Real-world use case: ROBOTIS OP2 keeping its balance in a RoboCup soccer match

## Academic acknowledgement

- **Student Name(s) and Registration Number(s):**
  - [Add Name] — CB.SC.U4CSE23430
  - [Add Name] — CB.SC.U4CSE23318
  - [Add Name] — CB.SC.U4CSE23336
  - [Add Name] — CB.SC.U4CSE23328
  - [Add Name] — CB.SC.U4CSE23609
  - [Add Name] — CB.SC.U4CSE23611
- **Mentor:** Dr. A. Baskar
- **Course:** 23CSE472 – Artificial Intelligence and Robotics
- **Department:** Department of Computer Science and Engineering
- **Institution:** School of Computing, Amrita Vishwa Vidyapeetham, Coimbatore, India.

## The use case

The **ROBOTIS OP2** (DARwIn-OP2) is a real competition robot in the **RoboCup Humanoid
KidSize league**, where small humanoids play football on their own. In a match the robot
has to:

- **walk** to the ball without falling,
- **kick** while standing on one leg,
- **stay on its feet** when opponents bump into it from the side or charge into it from
  behind.

Falling costs the robot time, can damage it, and removes it from play. Balance control
is therefore one of the most important skills a soccer humanoid has. This demo plays one
short match situation and uses all six subtopics of the case study to keep the robot up.

The same balance skills apply to other real humanoids: service robots in crowded
hospitals, malls or warehouses (people bump into them), exoskeletons and prosthetics
(fall prevention), and rescue robots on rough ground.

## How to run

1. In MATLAB, set **Current Folder** to `OP2_Robot_Casestudy`.
2. Run **`main_robocup.m`**.

It takes about **50 s**. Everything appears in one window:

- **Left:** the robot on a football pitch, with a ball and a goal.
- **Top right:** a live top view showing footprints, the CoM path, the support polygon
  and the ball.
- **Bottom right:** the live stability margin over the whole match, with event markers
  E0–E4.
- **Big title:** which event is running and which subtopics it uses.

Two settings at the top of `main_robocup.m` control the speed:

| Setting | Default | Effect |
|---|---|---|
| `holdTime` | 0.5 | pause between events in seconds; `0` means no pauses |
| `drawEvery` | 2 | draws every 2nd frame; use `1` for the smoothest animation, `3` for the fastest |

## The match: five events

| Event | Action | What happens | Subtopics |
|---|---|---|---|
| 0 | Kick-off | IK crouch into a ready stance, CoM centred | 4 |
| 1 | **Walk to the ball** | 4 statically stable steps (12 cm). Each step: weight shift over the stance foot, lift, swing, land. The CoM never leaves the support polygon. | 2, 3, 4 |
| 2 | **Kick the ball** | Balance on the left leg, wind up, swing the right foot through the ball. The ball rolls into the goal: **GOAL!** | 2, 3, 4 |
| 3 | **Side bump from an opponent** | The same inverted pendulum, solved sideways. The capture point stays inside the feet, so the ankle strategy (moving the ZMP) brings the CoM back. | 5, 6, 4 |
| 4 | Hard charge from behind | The capture point lands 3.6 cm past the toes, so the robot takes a 9.5 cm recovery step, then brings the other foot forward. | 5, 6, 4 |

## Results from the test run

```
===== Match report =====
Event                  Lowest margin (mm)
Kick-off                  +51.3
Walk to the ball          +22.3
Kick                      +22.2
Side bump                 +52.5
Charge from behind         -8.8
Goal scored: true.  Fell over: no.
```

- **Walking and kicking:** the margin never drops below +22 mm. The robot is statically
  stable at every instant and could freeze mid-step without falling.
- **Side bump:** the capture point is at y = −0.036 m, well inside the feet (±0.080 m),
  so the ankles alone recover.
- **Charge:** the capture point (x = 0.203 m) is beyond the toes (0.167 m), so standing
  still would mean a fall. The margin dips to −8.8 mm only during the 0.12 s swing, then
  the stepped foot widens the polygon and the margin climbs back above +50 mm.

## How the subtopics work together in the match

```
opponent push ──▶ inverted pendulum (5) ──▶ capture point vs support polygon (3)
                                                     │
                     ankle / hip / step (6) ◀────────┘
                               │
walking, kicking, recovery ──▶ IK with a CoM target (4) ──▶ joint angles
                               │
                     CoM (2) + robot model (1) ──▶ animation, top view, margin graph
```

## Files

| File | What it does |
|---|---|
| `main_robocup.m` | **The use case.** Pitch, ball, goal and the five events, plus the live graphs |
| `op2_setup.m` | Subtopic 1: robot model and initial pose |
| `computeCoM.m` | Subtopic 2: centre of mass |
| `supportPolygon.m` | Subtopic 3: support polygon from the foot contacts |
| `solvePosture.m`, `planMove.m` | Subtopic 4: inverse kinematics with a CoM target, smooth multi-frame moves |
| `lipmSimulate.m`, `capturePoint.m` | Subtopic 5: linear inverted pendulum model and capture point |
| `chooseStrategy.m`, `recoveryPlan.m` | Subtopic 6: ankle/hip/step balance recovery strategies |
| `stabilityMargin.m` | Signed distance from the CoM projection to the support polygon edge |
| `animateFrame.m` | Draws one frame: robot, ZMP/capture point markers, push arrow |
