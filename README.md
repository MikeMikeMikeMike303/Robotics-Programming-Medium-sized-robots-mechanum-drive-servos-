# FinalRebuilds

FTC TeleOp code for a four-motor mecanum robot with two positional servos driving the same arm. The drivetrain supports field-centric control, full-power driving and 35% precision mode. Each facing arm servo has its own calibrated endpoints; normal arm controls move both through one shared progress value.

**Current Java file and Driver Station OpMode: `FinalRebuilds`. All robot controls use Controller 1.** Normal arm movement requires both servo calibrations to be saved. Driving remains available when arm setup is incomplete. Pickup, Carry and Place presets are optional.

## Features

| Area | Features |
| --- | --- |
| Drivetrain | Field-centric and robot-centric modes; heading reset; heading hold during translation; automatic IMU fallback |
| Drive power | Full-power normal commands; hold-to-use 35% precision mode; stick deadband; wheel-power normalization; BRAKE at zero power |
| Arm | Separate endpoint mappings for facing servos; gradual D-pad movement; holding the last target; direct endpoint commands with triggers |
| Calibration | Individual LEFT/RIGHT setup; synchronized paired setup; validated endpoints and parking position; shared command bounds |
| Presets | Optional Pickup, Carry and Place positions; R3 teaching; inactive unset shortcuts |
| Settings | Persistent calibration and presets; confirmed saves; cache rollback and attempted disk recovery on failed writes |
| Telemetry | Calibration mode and selected side; drive mode and power; controller inputs; saved endpoints; commanded arm progress; preset and storage status |

No drive acceleration ramp is applied. Encoder speed control is present as an option but remains disabled until wiring and motor configuration are verified.

## Files

- [FinalRebuilds.java](FinalRebuilds.java): current robot OpMode.
- [Detailed controls guide](Rebuildv6-controls.md): full calibration and control behavior.
- [Feature table](Rebuildv6%20Feature%20Table.docx): complete feature reference, updated for FinalRebuilds.
- [GitHub description](FinalRebuilds-GitHub-description.md): repository summary and version notes.
- [Checks](checks/check_game_specific.py): entry point for simulated Java/logic verification.

The controls guide and feature table retain their earlier filenames. Older Java files and controller images in this workspace describe previous versions; use the current file and controls below.

## Hardware configuration

Use these exact names in the Robot Controller configuration:

| Name | Device |
| --- | --- |
| `front_left` | Front-left drive motor |
| `front_right` | Front-right drive motor |
| `back_left` | Back-left drive motor |
| `back_right` | Back-right drive motor |
| `front_left_servo` | Left positional arm servo |
| `front_right_servo` | Right positional arm servo |
| `imu` | REV Hub IMU |

Both right drive motors use `REVERSE`. The left servo uses `FORWARD` and the right uses `REVERSE` to account for opposing shafts. Independent calibration accounts for each horn's alignment. Do not add another `1 - target` reversal to the right servo.

The code assumes the Hub logo faces **UP** and its USB ports face **FORWARD**. Change those orientation settings if the actual mounting differs. An unavailable or invalid IMU causes robot-centric fallback; the configured drive motors and both servos are still required.

This code is for **standard positional servos**, not continuous-rotation servos.

## Install and start

1. Add `FinalRebuilds.java` to your FTC `org.firstinspires.ftc.teamcode` package. In an Android Studio FTC project, this is normally `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
2. Build with your FTC SDK project and deploy to the Robot Controller. This folder alone is not a complete FTC Android project.
3. Select **FinalRebuilds** on the Driver Station and assign the controller as **Controller 1**.
4. Support the arm and keep its travel clear before pressing INIT. A valid saved pair commands its saved parking position during INIT, before PLAY. An incomplete pair receives no new INIT arm target.
5. Face the intended field-forward direction and press PLAY. Verify the telemetry code label reads **FinalRebuilds - individual + synchronized calibration**.

Calibration buttons operate after PLAY. Presets never activate automatically during INIT or PLAY.

## Controller 1 controls

### Driving

| Control | Action |
| --- | --- |
| Left stick | Move forward, backward and sideways |
| Right stick left/right | Turn; overrides heading hold |
| Hold L1 or R1 | Limit drive power commands to 35%; release both for normal full power |
| Tap Circle | Toggle field-centric/robot-centric once per press |
| Tap Triangle | Reset field-forward to the current facing direction |
| PLAY | Set the initial field-forward heading |

Field-centric movement follows the heading reference even after the robot turns. Robot-centric movement follows the robot's current facing direction. Heading hold operates during field-centric translation with valid yaw and no manual turn; stopping or turning releases its target.

Calibration stops the chassis. Holding Options also stops the chassis and pauses arm adjustment while the code distinguishes a short press from a long press. BRAKE reduces coasting at zero power; it does not lock wheel position. Full power and 35% precision describe commands, not measured travel speed.

### Normal arm movement

Both saved calibrations must be valid and calibration must be closed.

| Control | Action |
| --- | --- |
| Hold D-pad Up / Down | Gradually raise / lower the paired arm |
| Release Up/Down | Hold the last target when no other arm movement control is active |
| L2 more than halfway | Command the taught lower endpoint directly |
| R2 more than halfway | Command the taught upper endpoint directly |
| Tap D-pad Left | Recall Pickup |
| Click left stick L3 | Recall Carry |
| Tap D-pad Right | Recall Place |
| Hold R3, then freshly press a preset shortcut | Teach that preset |

D-pad movement changes shared progress by **0.10 per second**. Trigger endpoint commands bypass this rate limit; physical servo travel still takes time. Both triggers together hold the target. Both Up/Down together cancel D-pad adjustment, and triggers take priority over the D-pad. Holding Cross or Square pauses arm-target changes in normal operation.

Right stick up/down, touchpad and PS/Home have no explicit robot-code action. Controller 2 inputs shown on screen are diagnostic only.

## Individual servo calibration

**Support the arm and disconnect the unselected servo's rigid mechanical coupling before individual calibration. Stop the OpMode and power off before disconnecting, switching or reconnecting couplings.** An unselected servo can keep holding its previous target and fight the selected servo if both remain rigidly connected.

Teach both sides at the **same physical lowered pose and the same physical raised pose**. Their numeric commands can differ. A physically raised pose may have a smaller logical command than the lowered pose.

| Control | Individual setup action |
| --- | --- |
| Tap Share | Start LEFT setup; tap again to save LEFT and exit |
| Short Options press and release | Start RIGHT setup; repeat to save RIGHT and exit |
| Hold D-pad Up / Down | Increase / decrease only the selected servo's logical command |
| Tap Cross / X | Select the physical lowered endpoint |
| Tap Square | Select the physical raised endpoint |

For each side:

1. Prepare the couplings with power off, then power on, INIT and PLAY. Center drive sticks and release all arm controls.
2. Select LEFT with Share or RIGHT with a short Options press. Confirm the selected side in telemetry.
3. Use Up/Down to reach the agreed lowered pose. Release movement controls and tap Cross / X.
4. Reach the agreed raised pose. Release movement controls and tap Square.
5. Move to a parking pose between the endpoints. Center sticks and release arm controls.
6. Press the same setup button to save and exit. Verify a successful save before changing couplings or restarting.
7. Teach the other side, then verify alignment through the intended range while disconnected. Park the pair, stop and power off before reconnecting without forcing either horn out of position.

Entering individual setup sends no new servo target. The first adjustment uses the last known command, or 0.50 if unavailable; this is not a measured angle. Individual adjustment is bounded to commands 0–1. Up/Down here do not necessarily mean physical raising/lowering.

Each side needs endpoints within 0–1, a span of at least **0.01**, and a parking command between them. Existing endpoints are preloaded when revising a saved side. Finish the active side before selecting another setup mode. L2/R2 and presets are disabled during calibration; simultaneous Cross/Square is rejected.

A successful individual save preserves the other side's endpoints, updates shared parking progress and clears all presets. If the common physical endpoints change, recalibrate both sides. When both sides are ready, paired commands resume on the loop after a successful exit, including the mapped parking target.

## Synchronized calibration

Save both individual calibrations and verify mechanical alignment first. Synchronized setup adjusts the arm as a pair while retaining different starting and ending commands for each servo.

1. Center drive sticks and release arm controls. Hold **Options for 1 second** to enter **SYNC BOTH**.
2. Use Up/Down to move both servos through their own endpoint mappings.
3. At the common lowered pose, release movement controls and tap Cross. At the common raised pose, release controls and tap Square.
4. Park between those poses, center sticks and release arm controls.
5. **Short Options:** save both endpoint pairs and parking together, then exit to normal operation while retaining SYNC style.
6. **Long Options:** save valid active SYNC edits and return to INDIVIDUAL style. From normal operation with SYNC selected, a long press returns to individual style without another save.

A short Options press in SYNC style reopens paired setup. Share cannot start left-only setup until individual style is restored. A long press acts once per hold and does not also produce a short action on release. Setup style resets to INDIVIDUAL on a new OpMode run.

SYNC uses shared reference progress bounded so neither servo command exceeds 0–1. It can extend beyond the old limits; negative reference percentages or values above 100% describe travel beyond those limits. Observe clearance because these bounds cannot detect mechanical stops. The common lower reference must precede the upper, each resulting servo span must be at least 0.01, and the parking pose must be between them.

Invalid or failed saves keep setup active and retain the draft. Conflicting input during an Options hold invalidates that press even if released later; release Options and make a fresh press. Options already held at PLAY is ignored until released.

## Teach optional presets

Presets store shared progress, which maps to each servo's own endpoints. **UNSET shortcuts do nothing**, so the robot can operate before any presets are taught.

1. In normal operation with both sides calibrated, position the arm and wait for it to physically settle.
2. Center drive sticks and release Up/Down, L2/R2 and endpoint buttons.
3. Hold **R3** and freshly press **D-pad Left for Pickup**, **L3 for Carry** or **D-pad Right for Place**.
4. Check telemetry for a successful save, then release both buttons. Recall using the shortcut without R3.

Use one shortcut at a time. Manual controls retain priority. Pressing R3 after a shortcut is already held does not teach it; release and press the shortcut again. Share is calibration, not preset teaching. Recalibrating either side or saving SYNC clears all presets; teach them again afterward.

## Saved settings and telemetry

Calibration and presets persist in Robot Controller preferences under `Rebuildv6.arm.independent.v1`. The FinalRebuilds rename preserves existing independent settings. Earlier shared-limit calibration and presets are ignored. Presets load only when their saved endpoint mappings match both sides.

Saves stop all drive motors before writing. Failed writes preserve confirmed state, restore the preference cache and attempt disk recovery. If **Settings storage** reports unconfirmed recovery, retry before restarting the Robot Controller app; disk contents cannot be guaranteed.

The calibration indicator reports LEFT ONLY, RIGHT ONLY or SYNC BOTH with CHASSIS STOPPED during setup. Outside setup, it reports BOTH SERVOS READY or BOTH CALIBRATIONS REQUIRED. Telemetry also shows setup style, saved endpoints, parking progress, presets, controller inputs and both servo commands.

**Servo telemetry reports targets, not measured shaft angles or arm heights.** Commands are issued consecutively; the code cannot verify physical synchronization, settling or a stall. STOP commands zero drive power without deliberately retargeting the arm or disabling its holding torque; normal SDK/controller STOP behavior still applies.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| D-pad or triggers do nothing | Confirm both side calibrations are saved, calibration is closed, Options is released and inputs are on Controller 1 |
| Precision mode does not engage | Hold L1/R1; look for drive scale 0.35 and Controller 1 bumper input |
| Circle appears ineffective | Check selected/effective mode and Circle switch count; modes behave alike at the original heading, and invalid IMU data forces robot-centric fallback |
| Options does not change setup | Center sticks, release other arm controls and retry with a fresh press; finish individual setup before switching to SYNC |
| Servo alignment differs | Check right-servo reversal and mechanical alignment; teach matching physical poses separately before reconnecting |
| A preset does nothing | Check whether it is UNSET; recalibration clears presets |
| Setup does not exit | Check endpoints, minimum span, parking between endpoints, released controls and storage error telemetry |

## Validation and limitations

The latest simulated checks passed **31,073 assertions** for driving, individual/SYNC calibration, paired mapping, button timing, presets and settings persistence. They compile the actual Java source against minimal API doubles.

With Python 3 and a JDK providing `javac` and `java`, run from this folder:

```sh
python3 checks/check_game_specific.py FinalRebuilds
```

These checks do not replace a real FTC SDK build or robot testing. Verify controller mapping, Hub orientation, motor/wheel directions, servo alignment and mechanical clearance on the robot.

There is no measured servo feedback, stall detection, automatic homing, added battery-warning code, field-position tracking, autonomous routine or claw control. Battery status remains available on the Driver Station.
