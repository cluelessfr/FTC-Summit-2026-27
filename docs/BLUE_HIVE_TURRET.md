# Parallel blue hive turrets (position mode)

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/turret/BlueHiveTurret.java`
is a reusable subsystem, not a separate drive OpMode. The three existing Pedro
autonomous test OpModes now use it. No drive TeleOp or robot constants were changed.

## What is implemented

- One goBILDA SPEED servo drives two separate turret axes, 7 inches apart laterally.
  Both gear trains must have the same external ratio and turn the same direction.
- Keep both turrets parallel and aim their midpoint horizontally at the fixed blue
  hive pivot, using Pedro robot pose. A single servo cannot enforce parallelism in
  software if the mechanical linkage produces different angles.
- Assume the midpoint of the two axes is 5 inches behind the localization reference,
  centered laterally. `pairCenterForwardInches`, `pairCenterLeftInches` and
  `turretAxisSpacingInches` expose these dimensions for later measurement.
- Map a reachable turret angle directly to an FTC `Servo` position. The servo uses
  its own internal feedback to move to and hold that shaft position.
- Restrict targets to the intersection of safe servo travel and cable limits.
  If an equivalent aiming angle is unavailable, retain the previous command and
  report that the hive is outside the position range. Do not clamp to an endpoint
  and pretend that endpoint aims at the hive.
- Save robot pose and the last commanded turret angle during autonomous and on stop.
- Resume Pinpoint localization in TeleOp without constructing a drivetrain or
  recalibrating its IMU. The new localizer copies the existing robot configuration
  with reset mode `NONE`; it seeds the saved pose because the robot stays stationary
  during the transition. Only one component should update/own this Pinpoint instance.

Parallel turrets do not both individually intersect the exact hive pivot. This
implementation aims the centerline between them at it, as requested. With axes
7 inches apart, their individual aiming lines remain parallel on either side of
that centerline; target-relative spacing changes with the robot's orientation.

The app receives **no measured turret angle**. The servo's internal position
feedback controls its output shaft, but cannot measure external gear backlash,
flex, or separate turret misalignment. Telemetry labels the angle as COMMANDED,
not as the current actual angle. The old timed movement estimator, speed
calibration, update-gap fault and continuous-rotation unwind state are removed.

## Coordinate convention

Use the attached top-down field diagram: origin at the lower-left inside corner,
+X right towards the blue alliance wall, +Y upwards, inches, heading zero along +X,
and counterclockwise headings positive. All integrated autos must use this same
field frame. Existing test paths were left unchanged; they must be positioned in
that frame when tested on a field. A robot-relative test-path origin is insufficient.

FIRST's nominal field is 144 by 144 inches, the hive structure is centered, and
Figure 9-10 gives 25.5 inches between the two hive centerlines. In this convention
the nominal blue pivot is therefore **(84.75, 72)**. These are derived coordinates,
not a separately specified FIRST coordinate system. Check your actual field and
adjust `hiveX` and `hiveY` if needed. Aiming is at the pivot, not a tipping CELL;
this code does not control shooter elevation, projectile speed or launch timing.

Source: [FIRST BIOBUZZ arena manual, Section 9, Figures 9-2 and 9-10](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09).

## Required setup before movement

Edit `defaultSettings()` to create a `Settings`, set the calibrated fields, and
return it. Defaults intentionally leave hardware movement disabled. The class
does not map or command the servo until all required settings are present, so the
existing autonomous tests can still run without a configured turret servo.

1. Confirm the servo is **goBILDA 2000-0025-0003, 2000 Series SPEED**, in its original
   POSITION mode. If it was physically programmed for continuous rotation, change
   it back with the hardware programmer. FTC Java cannot switch its stored mode.
   Configure the servo port as `servo`. The class uses `Servo` and requires hardware
   implementing `PwmControl` to request the full 500-2500 microsecond position range.
2. Calibrate `servoForwardPosition`: the position value from 0 to 1 that makes BOTH
   turrets face robot-forward with untwisted wires. This is a fixed relationship
   between the shaft and turret angles; do not recalibrate it at the Auto-to-TeleOp
   transition. Check that both remain parallel throughout their motion.
3. Enter `servoTurnsPerTurretTurn`, the EXTERNAL ratio. For one external gear pair,
   this is turret gear teeth divided by servo gear teeth. Do not enter the servo's
   internal 135:1 reduction. For multiple stages, multiply the external ratios.
4. Check which direction increasing the servo position turns BOTH assembled turrets.
   Set `increasingPositionCcwSign` to +1 for counterclockwise from above or -1 for
   clockwise. The SDK servo direction is explicitly FORWARD; this calibrated sign
   handles the gearing's reversal.
5. Set safe `servoMinimumPosition` and `servoMaximumPosition` endpoints. Defaults
   are 0 and 1, but they must be checked against the physical mechanism before
   enabling movement. `servoTravelDegrees` is the output shaft travel across 0..1,
   nominally 300 degrees with the configured signal range. Calibrate it if needed.
6. Verify that BOTH turrets allow the assumed cable range of -360 to +360 degrees
   from untwisted forward; use the narrower range if their limits differ.
   `cableMarginDegrees = 15` restricts planned targets to -345 through +345 degrees.
   The margin is only a buffer against modest error. Check for mechanical stops.
7. Set `motionEnabled = true` after setup and verify targeting while stationary
   before driving. No loaded-speed measurements or timed-angle calibration are needed.

The mapping is:

`position = forwardPosition + directionSign * turretAngle * externalRatio / servoTravelDegrees`

Usable turret range is derived by mapping the two safe position endpoints back to
angles, then intersecting that range with the cable limits. For example, a 1:1
external ratio, forward at 0.5, and safe positions 0..1 produce +/-150 degrees.
A 3:1 ratio gives +/-50 degrees. Reducing safe endpoints reduces available travel.
The accepted limited range does not guarantee that the hive is reachable at every
robot position and heading.

Servo source: [goBILDA Speed servo specifications and programmer](https://www.gobilda.com/2000-series-dual-mode-servo-25-3-speed/).

## Add to any Pedro autonomous

After constructing the follower and setting its starting pose:

```java
BlueHiveTurret turret = BlueHiveTurret.forAutonomous(hardwareMap);
```

In `start()`, call `turret.start()`. Every loop, call `follower.update()` once and
then `turret.update(follower.pose())`. Add `turret.addTelemetry(telemetry)` before
the OpMode's existing telemetry update. In `stop()` (or a LinearOpMode `finally`
block), call `turret.stop()` and stop the follower. `stop()` stops sending new
targets; it does not command position zero or return forward. The last position
command remains in place, with subsequent PWM behavior managed by the SDK.
The existing three autos provide
complete examples. This cannot capture an autonomous that has not added these hooks.

Starting an integrated autonomous clears the previous handoff. The handoff is an
in-memory snapshot, consumed once by TeleOp, expires after five minutes, and is
lost if the Robot Controller app restarts. The last commanded turret angle is
saved for equivalent-angle selection; it is not a measured current angle. Init
does not issue a new target or return the turret forward.

## Add to a future drive TeleOp

Imports:

```java
import org.firstinspires.ftc.teamcode.turret.BlueHiveTurret;
import com.pedropathing.math.Pose;
```

During init, after your existing drive setup (which must not reset Pinpoint):

```java
// Normal match: require a fresh autonomous handoff.
BlueHiveTurret turret = BlueHiveTurret.forTeleOp(hardwareMap, null);
```

For a standalone test, pass `BlueHiveTurret.blueWallStartPose(measuredDistance)`
instead of `null`. `measuredDistance` is inches from the INSIDE blue wall to the
point tracked by Pedro, with the robot centered along that wall and facing the
hive. This generates `(144 - measuredDistance, 72, pi)`. It uses the standalone
pose only when no fresh handoff exists. Call `BlueHiveTurret.clearHandoff()` first
if you intentionally want to discard a recent autonomous and run standalone.
No robot length or wall clearance is silently assumed.

After Start, and inside your normal drive loop:

```java
turret.start(); // once, after waitForStart()
try {
    while (opModeIsActive()) {
        Pose pose = turret.updateLocalizationAndAim();
        // Your existing driving code goes here.
        // pose.heading() is the preserved FIELD heading, in radians.
        turret.addTelemetry(telemetry);
        telemetry.update();
    }
} finally {
    turret.stop();
    // Stop your drive motors here too.
}
```

Do not also create/reset another Pinpoint localizer or update another Pedro
follower using the same hardware in this loop. The turret's TeleOp factory owns
localization only; motor power/velocity, drive speed and joystick behavior remain
the responsibility of the drive OpMode. Resetting a drive IMU's field-forward
reference must not reset Pinpoint's global field pose.

## Verification boundary

The hardware-independent checks cover the pair midpoint and 7-inch axis separation,
rear offset, wrapped headings, equivalent-angle choice, forward offset and reversed
gearing, position mapping, safe endpoints, cable limits, unreachable targets,
standalone pose construction and the handoff. Compilation checks the installed
Pedro 3.0.1 and FTC SDK APIs. These checks cannot verify actual aim, parallelism,
wire clearance, field placement or the transition on the robot; those require
physical tests after calibration.

Run the checks from the repository root after compiling:

```powershell
.\gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline
.\scripts\check-blue-hive-turret.ps1
```

The check script uses the installed production SDK/library jars, compiles a plain
Java harness under `TeamCode/build/turret-checks`, and requires no new dependencies.
