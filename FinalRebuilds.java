package org.firstinspires.ftc.teamcode;
import android.content.Context;
import android.content.SharedPreferences;
// Describes how the REV Hub is physically mounted on the robot.
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Gives access to the Hub's orientation sensor (Inertial Measurement Unit).
import com.qualcomm.robotcore.hardware.IMU;
// Lets us request angles in radians for Java's sine and cosine functions.
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Map;

/**
 * Field-centric mecanum drive: left stick translates, right stick turns.
 * Face field-forward before pressing PLAY. Press Triangle while facing field-forward
 * to reset the heading reference during operation.
 *
 * Field-forward means the direction the robot faces when its heading is reset.
 * After the robot turns, pushing the left stick forward still requests movement
 * in that original direction. The IMU measures rotation; it does not track the
 * robot's position or automatically know which direction on the field is forward.
 */
@TeleOp(name="FinalRebuilds", group="Iterative Opmode")
public class FinalRebuilds extends OpMode {
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    // Store the sensor here so init(), start(), and loop() can all use it.
    private IMU imu;
    private Servo servoMotorLeft;
    private Servo servoMotorRight;

    // Drive settings: normal driving can reach full power. No acceleration ramp.
    private static final double STICK_DEADBAND = 0.05;
    private static final double PRECISION_POWER = 0.35;
    private static final double HEADING_GAIN = 1.0; // Turn command per radian of error.
    private static final double MAX_HEADING_CORRECTION = 0.25;
    // This preserves your original motor/joystick signs. Change to -1 ONLY if
    // heading hold turns farther away from its target during a wheels-raised test.
    private static final double HEADING_CORRECTION_SIGN = 1.0;
    // Requires all four encoder cables and correct motor types in configuration.
    // Leave false until the encoder wiring is verified. No new motors required.
    private static final boolean USE_DRIVE_ENCODERS = false;
    private boolean fieldCentric = true;
    private boolean imuAvailable;
    private boolean wasTrianglePressed;
    private boolean wasCirclePressed;
    private int driveModeChanges;
    private boolean holdHeadingActive;
    private double holdHeading;
    private double headingCorrection;
    private String imuMessage = "";

    // ONE target and ONE pair of limits control BOTH facing positional servos.
    // Servo positions are commands, not measured arm angles.
    private static final double ARM_START = 0.50; // First calibration's starting command.
    private static final double ARM_RATE_PER_SECOND = 0.10; // D-pad only.
    private double armPosition = ARM_START;
    private double armLowerLimit = 0.0;
    private double armUpperLimit = 1.0;
    private double armStartupPosition = ARM_START;
    private boolean calibrationMode;
    private boolean calibrationLoaded;
    private boolean lowerLimitSet;
    private boolean upperLimitSet;
    private boolean wasOptionsPressed;
    // PlayStation Cross = a; Square = x; Triangle = y; Circle = b; Options = start.
    private boolean wasCrossPressed;
    private boolean wasSquarePressed;
    private String armMessage = "";
    private SharedPreferences armSettings;
    private String settingsMessage = ""; // Only shown when storage needs attention.
    private final ElapsedTime armTimer = new ElapsedTime();

    // Optional presets are command targets, not measured physical positions.
    // NaN means UNSET: no default pickup/carry/place movement is ever guessed.
    private final double[] armPresets = {Double.NaN, Double.NaN, Double.NaN};
    private final String[] presetNames = {"Pickup", "Carry", "Place"};
    private final String[] presetKeys = {"presetPickup", "presetCarry", "presetPlace"};
    private final boolean[] wasPresetPressed = {false, false, false};
    private String presetMessage = "Presets optional: unset buttons do not move the arm.";


    @Override
    public void init() {
        // Names must match the Robot Controller configuration.
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        servoMotorLeft = hardwareMap.get(Servo.class, "front_left_servo");
        servoMotorRight = hardwareMap.get(Servo.class, "front_right_servo");

        // Opposing shafts require mirrored physical rotation. REVERSE makes the
        // right servo mirror the shared target, so do NOT also send it 1 - target.
        // This assumes matching positional servos with mechanically aligned horns.
        servoMotorLeft.setDirection(Servo.Direction.FORWARD);
        servoMotorRight.setDirection(Servo.Direction.REVERSE);
        servoMotorLeft.scaleRange(0.0, 1.0);
        servoMotorRight.scaleRange(0.0, 1.0);

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE reduces coasting at zero power; it is not a position lock.
        for (DcMotor motor : driveMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(USE_DRIVE_ENCODERS ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }

        // Match these directions to the actual Hub mounting.
        // If IMU setup fails, robot-centric driving remains available.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imuAvailable = imu.initialize(new IMU.Parameters(orientation));
            imuMessage = imuAvailable ? "Ready" : "Initialization failed; robot-centric fallback";
        } catch (RuntimeException error) {
            imuAvailable = false;
            imuMessage = "Unavailable; robot-centric fallback: " + error.getMessage();
        }

        // Android preferences stay on the Robot Controller after OpMode restarts.
        // Only a complete, valid calibration is used at startup.
        loadArmCalibration();
        loadArmPresets(); // Loading never commands movement.
        if (calibrationLoaded) {
            armPosition = armStartupPosition;
            applyArmPositionToBothServos(); // INIT can move the arm to the saved target.
        }
        // Without calibration, INIT sends no new arm position. Enter calibration
        // after PLAY; its first command is ARM_START (0.50), so support the arm.

        // Keep the calibration indicator first on the Driver Station display.
        showCalibrationIndicator();
        showStartupStatus(); // Informational only; does not enable/block controls.

        // Show a setup reminder on the Driver Station; update() sends it.
        telemetry.addLine("Face field-forward before pressing PLAY.");
        telemetry.addLine("Arm: D-pad Up raises; Down lowers; release holds target.");
        telemetry.addLine("Circle: drive mode; L1/R1: precision; Triangle: reset heading.");
        telemetry.addLine("Options: calibration; Cross: minimum; Square: maximum.");
        showArmTelemetry();
        telemetry.addData("Left servo connection", servoMotorLeft.getConnectionInfo());
        telemetry.addData("Right servo connection", servoMotorRight.getConnectionInfo());
        telemetry.update();
    }

    @Override
    public void start() {
        // FTC calls start() once when PLAY is pressed.
        // Define the robot's current facing direction as zero degrees of yaw
        // (heading). Align the robot with your desired field-forward direction
        // first. Resetting yaw changes the reference; it does not move the robot.
        resetHeading();
        // Start timing at PLAY so time spent waiting in INIT cannot cause a jump.
        armTimer.reset();
        wasCrossPressed = gamepad1.a;
        wasSquarePressed = gamepad1.x;
        wasOptionsPressed = gamepad1.start;
        wasTrianglePressed = gamepad1.y;
        wasCirclePressed = gamepad1.b;
        boolean[] presetButtons = presetButtons();
        for (int i = 0; i < wasPresetPressed.length; i++) {
            wasPresetPressed[i] = presetButtons[i];
        }
    }

    @Override
    public void init_loop() {
        // Only display buttons during INIT. Arm movement starts after PLAY.
        showCalibrationIndicator();
        showStartupStatus();
        showArmTelemetry();
        telemetry.addLine("INIT: press PLAY to move arm.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // A new press toggles or resets ONCE; holding the button does not repeat.
        if (gamepad1.y && !wasTrianglePressed) {
            resetHeading(); // Face field-forward before pressing Triangle.
        }
        if (gamepad1.b && !wasCirclePressed) {
            fieldCentric = !fieldCentric;
            driveModeChanges++;
            holdHeadingActive = false;
        }
        wasTrianglePressed = gamepad1.y;
        wasCirclePressed = gamepad1.b;
        updateArm(); // Calibration mode is also used to stop the chassis below.
        showCalibrationIndicator(); // Display the mode AFTER processing Options.

        // Deadband removes center drift and rescales the rest of the stick range
        // so a full stick STILL commands 1.0. Preserve your original drive signs.
        double left_y = applyDeadband(gamepad1.left_stick_y);
        double left_x = -applyDeadband(gamepad1.left_stick_x);
        double right_x = -applyDeadband(gamepad1.right_stick_x);
        double heading = readHeading();
        boolean headingValid = !Double.isNaN(heading) && !Double.isInfinite(heading);
        boolean translating = left_x != 0.0 || left_y != 0.0;
        headingCorrection = 0.0;

        // Heading hold operates only in field-centric mode, while translating
        // without a turn request. Circle gives a manual robot-centric fallback.
        // Capture a fresh target after manual turning, stopping, or mode changes.
        if (fieldCentric && headingValid && translating && right_x == 0.0
                && !calibrationMode) {
            if (!holdHeadingActive) {
                holdHeading = heading;
                holdHeadingActive = true;
            }
            // Wrapping avoids a full-circle correction when yaw crosses +/-180°.
            double error = wrapRadians(holdHeading - heading);
            headingCorrection = Range.clip(HEADING_CORRECTION_SIGN * HEADING_GAIN * error,
                    -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
            right_x = headingCorrection;
        } else {
            holdHeadingActive = false;
        }

        // Rotate translation into robot coordinates only when heading is usable.
        // Robot-centric mode ignores the IMU for both translation and heading hold.
        double robotX = left_x;
        double robotY = left_y;
        if (fieldCentric && headingValid) {
            robotX = left_x * Math.cos(-heading) - left_y * Math.sin(-heading);
            robotY = left_x * Math.sin(-heading) + left_y * Math.cos(-heading);
        }
        // Full power normally; precision only while L1/R1 is held.
        // Arm calibration stops all drive motors so stick bumps cannot move chassis.
        double driveScale = calibrationMode ? 0.0
                : (gamepad1.left_bumper || gamepad1.right_bumper ? PRECISION_POWER : 1.0);

        // Keep motor powers within [-1, 1], preserving their proportions.
        // Combining forward, sideways, and turning commands can exceed 1.
        // Sum their magnitudes to get a shared scale factor, at least 1.0.
        // If the sum is 2, for example, all four outputs are divided by 2.
        // If the sum is below 1, dividing by 1 leaves the requested power alone.
        // Scaling together avoids clipping individual wheels and distorting
        // the requested balance between movement and turning.
        double denominator = Math.max(
                Math.abs(robotY) + Math.abs(robotX) + Math.abs(right_x),
                1.0
        );

        // Use your original mecanum wheel pattern with the corrected X/Y values.
        // robotY contributes equally to all wheels for forward/backward travel.
        // robotX uses opposite signs on the diagonal pairs for sideways travel.
        // right_x uses opposite signs on the left/right sides for turning.
        // Divide every wheel by the SAME denominator to preserve these ratios.
        frontLeftMotor.setPower((robotY + robotX + right_x) / denominator * driveScale);
        backLeftMotor.setPower((robotY - robotX + right_x) / denominator * driveScale);
        frontRightMotor.setPower((robotY - robotX - right_x) / denominator * driveScale);
        backRightMotor.setPower((robotY + robotX - right_x) / denominator * driveScale);

        // Display heading in easier-to-read degrees on the Driver Station.
        // This conversion is only for display; the driving math uses radians.
        telemetry.addData("Heading (degrees)", Math.toDegrees(heading));
        telemetry.addData("Drive mode", fieldCentric && headingValid ? "Field-centric" : "Robot-centric");
        telemetry.addData("Drive scale", "%.2f", driveScale);
        telemetry.addData("Heading hold correction", "%.3f", headingCorrection);
        telemetry.addData("IMU", imuMessage);
        telemetry.addData("Drive encoder mode", USE_DRIVE_ENCODERS ? "Enabled" : "Disabled - verify wiring first");
        showArmTelemetry();
        telemetry.update();
    }

    private DcMotor[] driveMotors() {
        return new DcMotor[] {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
    }

    private double applyDeadband(double input) {
        if (Math.abs(input) <= STICK_DEADBAND) return 0.0;
        return Math.signum(input) * (Math.abs(input) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
    }

    private double wrapRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private void resetHeading() {
        holdHeadingActive = false;
        if (!imuAvailable) return;
        try {
            imu.resetYaw();
        } catch (RuntimeException error) {
            imuAvailable = false;
            imuMessage = "Reset failed; robot-centric fallback";
        }
    }

    private double readHeading() {
        if (!imuAvailable) return Double.NaN;
        try {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if (Double.isNaN(heading) || Double.isInfinite(heading)) {
                imuMessage = "Invalid heading; robot-centric fallback";
            } else {
                imuMessage = "Ready";
            }
            return heading;
        } catch (RuntimeException error) {
            imuAvailable = false;
            imuMessage = "Read failed; robot-centric fallback";
            return Double.NaN;
        }
    }

    private boolean validArmCalibration(double lower, double upper, double startup) {
        // Explicit finite checks also reject corrupt stored data.
        return !Double.isNaN(lower) && !Double.isNaN(upper) && !Double.isNaN(startup)
                && lower >= 0.0 && upper <= 1.0 && lower < upper
                && startup >= lower && startup <= upper;
    }

    private void loadArmCalibration() {
        armSettings = hardwareMap.appContext.getSharedPreferences(
                "GameSpecificRebuild.arm.v1", Context.MODE_PRIVATE);
        double lower = armSettings.getFloat("lower", Float.NaN);
        double upper = armSettings.getFloat("upper", Float.NaN);
        double startup = armSettings.getFloat("startup", Float.NaN);
        calibrationLoaded = armSettings.getBoolean("valid", false)
                && validArmCalibration(lower, upper, startup);
        if (calibrationLoaded) {
            armLowerLimit = lower;
            armUpperLimit = upper;
            armStartupPosition = startup;
            lowerLimitSet = true;
            upperLimitSet = true;
            armMessage = "Saved calibration loaded. INIT commands saved startup target.";
        } else {
            armMessage = "No valid calibration. Options enters setup; first command is 0.50.";
        }
    }

    private boolean saveArmCalibration() {
        // Save both limits and the current target together. This is the startup
        // target: exit calibration with the arm in your chosen parking position.
        // commit() reports whether disk writing succeeded; never claim a failed save.
        SharedPreferences.Editor editor = armSettings.edit().putFloat("lower", (float) armLowerLimit)
                .putFloat("upper", (float) armUpperLimit)
                .putFloat("startup", (float) armPosition)
                .putBoolean("valid", true)
                // Recalibrating changes what arm positions mean. Remove presets
                // in the SAME successful write. The helper restores a failed write.
                .putFloat("presetPickup", Float.NaN)
                .putFloat("presetCarry", Float.NaN)
                .putFloat("presetPlace", Float.NaN);
        return commitArmSettings(editor);
    }

    private boolean commitArmSettings(SharedPreferences.Editor editor) {
        // Disk writes can pause this loop. Stop BEFORE writing so the previous
        // loop's drive power cannot remain active while the sticks are centered.
        for (DcMotor motor : driveMotors()) motor.setPower(0.0);

        // Android changes its preferences cache even when commit() reports a
        // disk failure. Keep a snapshot so a fresh OpMode cannot load that failure.
        Map<String, ?> previous = armSettings.getAll();
        String[] floatKeys = {"lower", "upper", "startup", "presetPickup", "presetCarry",
                "presetPlace", "presetLimitsLower", "presetLimitsUpper"};
        settingsMessage = "";
        // Do not overwrite an unexpected stored type that we cannot restore.
        for (String key : floatKeys) {
            if (previous.containsKey(key) && !(previous.get(key) instanceof Float)) {
                settingsMessage = "Unexpected stored setting type: " + key;
                return false;
            }
        }
        Object valid = previous.get("valid");
        if (previous.containsKey("valid") && !(valid instanceof Boolean)) {
            settingsMessage = "Unexpected stored setting type: valid";
            return false;
        }
        if (editor.commit()) return true;

        SharedPreferences.Editor restore = armSettings.edit();
        for (String key : floatKeys) {
            Object value = previous.get(key);
            if (value instanceof Float) restore.putFloat(key, (Float) value);
            else restore.remove(key); // Missing settings must stay missing.
        }
        if (valid instanceof Boolean) restore.putBoolean("valid", (Boolean) valid);
        else restore.remove("valid");
        // Even if disk writing fails again, this restores the in-process cache.
        // Return false: the requested new settings were NOT confirmed saved.
        if (!restore.commit()) {
            settingsMessage = "Cache restored; disk recovery unconfirmed. Retry the save.";
        }
        return false;
    }

    private void updateArm() {
        double seconds = Math.min(armTimer.seconds(), 0.05);
        armTimer.reset();
        boolean l2Pressed = gamepad1.left_trigger > 0.5;
        boolean r2Pressed = gamepad1.right_trigger > 0.5;
        boolean movementReleased = !gamepad1.dpad_up && !gamepad1.dpad_down
                && !l2Pressed && !r2Pressed;
        boolean toggledCalibration = false;

        // Options toggles calibration only with all arm movement/save buttons released.
        if (gamepad1.start && !wasOptionsPressed) {
            if (!movementReleased || gamepad1.a || gamepad1.x) {
                armMessage = "Release arm controls, then press Options again.";
            } else if (!calibrationMode) {
                calibrationMode = true;
                armMessage = "CALIBRATION: D-pad adjusts; Cross minimum; Square maximum; Options saves.";
                toggledCalibration = true;
            } else if (Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND
                    || Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
                    || Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND) {
                // Leaving calibration restores drive power; require centered sticks.
                armMessage = "Center drive sticks before leaving calibration.";
            } else if (!lowerLimitSet || !upperLimitSet
                    || !validArmCalibration(armLowerLimit, armUpperLimit, armPosition)) {
                armMessage = "Set BOTH ordered limits; park BETWEEN them before saving.";
            } else {
                if (saveArmCalibration()) {
                    armStartupPosition = armPosition; // Update only after a confirmed save.
                    armPosition = armStartupPosition;
                    calibrationLoaded = true;
                    calibrationMode = false;
                    for (int i = 0; i < armPresets.length; i++) armPresets[i] = Double.NaN;
                    presetMessage = "Arm recalibrated: presets cleared; manual controls still work.";
                    armMessage = "Limits and startup target saved on Robot Controller.";
                    toggledCalibration = true;
                } else {
                    armMessage = "Save failed; still in calibration. Press Options to retry.";
                }
            }
        }
        wasOptionsPressed = gamepad1.start;

        // Calibration permits manual travel through the servo's command range so
        // existing limits can be extended. Verify mechanical clearance as you move.
        // L2/R2 are disabled during calibration and before a valid calibration exists.
        boolean armEnabled = calibrationMode || calibrationLoaded;
        if (armEnabled && !toggledCalibration && !gamepad1.a && !gamepad1.x) {
            if (!calibrationMode && l2Pressed && !r2Pressed) {
                armPosition = armLowerLimit; // Direct endpoint: no D-pad speed limit.
            } else if (!calibrationMode && r2Pressed && !l2Pressed) {
                armPosition = armUpperLimit;
            } else if (!l2Pressed && !r2Pressed) {
                double direction = 0.0;
                if (gamepad1.dpad_up && !gamepad1.dpad_down) direction = 1.0;
                else if (gamepad1.dpad_down && !gamepad1.dpad_up) direction = -1.0;
                // Increasing the target must raise the arm. If opposite, reverse
                // BOTH servo directions in init(), then recalibrate.
                armPosition += direction * ARM_RATE_PER_SECOND * seconds;
            }
        }
        armPosition = Range.clip(armPosition, calibrationMode ? 0.0 : armLowerLimit,
                calibrationMode ? 1.0 : armUpperLimit);

        boolean crossPressed = gamepad1.a && !wasCrossPressed;
        boolean squarePressed = gamepad1.x && !wasSquarePressed;
        if (crossPressed || squarePressed) {
            if (!calibrationMode) {
                armMessage = "Limits locked. Press Options to enter calibration.";
            } else if (!movementReleased || (gamepad1.a && gamepad1.x)) {
                armMessage = "Release D-pad/L2/R2, then press ONE save button again.";
            } else if (crossPressed) {
                armLowerLimit = armPosition;
                lowerLimitSet = true;
                armMessage = "Minimum selected. Set maximum, choose parking target; Options saves.";
            } else {
                armUpperLimit = armPosition;
                upperLimitSet = true;
                armMessage = "Maximum selected. Choose parking target; Options saves.";
            }
        }
        wasCrossPressed = gamepad1.a;
        wasSquarePressed = gamepad1.x;
        updateArmPresets(movementReleased);
        if (armEnabled) applyArmPositionToBothServos();
    }

    private boolean[] presetButtons() {
        // Free buttons: D-pad Left = pickup; L3 = carry; D-pad Right = place.
        return new boolean[] {gamepad1.dpad_left, gamepad1.left_stick_button, gamepad1.dpad_right};
    }

    private void loadArmPresets() {
        // Require the same limits used when these targets were taught. Older
        // OpModes that alter limits also cannot accidentally revive old presets.
        // Compare after float conversion to match Android's stored precision.
        boolean sameLimits = calibrationLoaded
                && armSettings.getFloat("presetLimitsLower", Float.NaN) == (float) armLowerLimit
                && armSettings.getFloat("presetLimitsUpper", Float.NaN) == (float) armUpperLimit;
        if (!sameLimits) return;
        for (int i = 0; i < armPresets.length; i++) {
            double target = armSettings.getFloat(presetKeys[i], Float.NaN);
            if (target >= armLowerLimit && target <= armUpperLimit) armPresets[i] = target;
        }
    }

    private void updateArmPresets(boolean movementReleased) {
        boolean[] buttons = presetButtons();
        int held = 0;
        for (boolean button : buttons) if (button) held++;
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] && !wasPresetPressed[i]) {
                // Existing manual controls take priority. Only one shortcut at
                // a time; never recall or teach while changing arm calibration.
                if (held != 1 || !movementReleased || gamepad1.a || gamepad1.x
                        || gamepad1.start || calibrationMode) {
                    presetMessage = "Release arm controls; use ONE preset button outside calibration.";
                } else if (!calibrationLoaded) {
                    presetMessage = "Presets inactive until arm limits are calibrated.";
                } else if (gamepad1.back && (Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND
                        || Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
                        || Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND)) {
                    presetMessage = "Center drive sticks before teaching a preset.";
                } else if (gamepad1.back) { // PlayStation Share is FTC's back button.
                    // Hold Share + shortcut to teach the CURRENT target. Wait
                    // for the mechanism to settle first; no sensor measures it.
                    SharedPreferences.Editor editor = armSettings.edit();
                    // Write the entire validated in-memory set. This also removes
                    // stale stored presets from a different calibration snapshot.
                    for (int j = 0; j < armPresets.length; j++) {
                        editor.putFloat(presetKeys[j], (float) (j == i ? armPosition : armPresets[j]));
                    }
                    editor.putFloat("presetLimitsLower", (float) armLowerLimit)
                            .putFloat("presetLimitsUpper", (float) armUpperLimit);
                    boolean saved = commitArmSettings(editor);
                    if (saved) {
                        armPresets[i] = armPosition;
                        presetMessage = presetNames[i] + " target saved. Release Share to recall.";
                    } else {
                        presetMessage = "Preset save failed; previous target kept. Release and retry.";
                    }
                } else if (Double.isNaN(armPresets[i])) {
                    // Unset presets DO NOTHING: the robot can be used before
                    // the front mechanism is fitted and taught.
                    presetMessage = presetNames[i] + " is UNSET; arm target unchanged.";
                } else {
                    armPosition = Range.clip(armPresets[i], armLowerLimit, armUpperLimit);
                    presetMessage = presetNames[i] + " target commanded to BOTH servos.";
                }
            }
            wasPresetPressed[i] = buttons[i];
        }
    }

    private void showStartupStatus() {
        // One compact status line during INIT. No new startup movement, tests,
        // acknowledgments, or restrictions are introduced by this display.
        int taught = 0;
        for (double target : armPresets) if (!Double.isNaN(target)) taught++;
        telemetry.addData("Startup status", "IMU: %s | arm limits: %s | presets: %d/3 (optional)",
                imuAvailable ? "initialized" : "robot-centric fallback",
                calibrationLoaded ? "saved" : "not saved", taught);
    }

    private void showCalibrationIndicator() {
        // Read the SAME flag that stops the chassis and enables endpoint buttons.
        // This appears first during INIT and PLAY, including a failed save/exit.
        telemetry.addData("*** ARM CALIBRATION ***", calibrationMode
                ? "ON - CHASSIS STOPPED"
                : (calibrationLoaded ? "OFF - NORMAL OPERATION" : "OFF - SETUP REQUIRED"));
        telemetry.addData("Calibration controls", calibrationMode
                ? "D-pad: move | Cross: min | Square: max | Options: save/exit"
                : "Press Options after PLAY to enter calibration");
    }

    private void showDriveControlsTelemetry() {
        // Watch these while pressing buttons. Input on controller 2 does not
        // control this OpMode; assign the driving controller as controller 1.
        telemetry.addData("Controller 1 L1/R1/Circle", "%b / %b / %b",
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.b);
        telemetry.addData("Controller 2 L1/R1/Circle", "%b / %b / %b",
                gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.b);
        telemetry.addData("Precision button held", gamepad1.left_bumper || gamepad1.right_bumper);
        telemetry.addData("Selected drive mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Circle switches detected", driveModeChanges);
        // These are electrical power commands, not measured wheel speed.
        telemetry.addData("Drive powers FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                frontLeftMotor.getPower(), backLeftMotor.getPower(),
                frontRightMotor.getPower(), backRightMotor.getPower());
    }

    private void showArmTelemetry() {
        telemetry.addData("Code", "FinalRebuilds - optional presets");
        showDriveControlsTelemetry();
        telemetry.addData("Arm mode", calibrationMode ? "CALIBRATION - drive stopped"
                : (calibrationLoaded ? "Normal - limits locked" : "Needs calibration - arm commands disabled"));
        telemetry.addLine("Options: calibration; Cross: minimum; Square: maximum.");
        telemetry.addLine("Normal: L2 minimum / R2 maximum; D-pad adjusts; release holds.");
        telemetry.addData("Arm COMMAND target", "%.3f", armPosition);
        telemetry.addData("Minimum / maximum", "%.3f / %.3f", armLowerLimit, armUpperLimit);
        telemetry.addData("Saved startup target", "%.3f", armStartupPosition);
        telemetry.addData("Arm status", armMessage);
        if (!settingsMessage.isEmpty()) telemetry.addData("Settings storage", settingsMessage);
        telemetry.addData("Presets pickup / carry / place", "%s / %s / %s",
                presetLabel(0), presetLabel(1), presetLabel(2));
        telemetry.addData("Preset status", presetMessage);
        telemetry.addData("Controller 1 Up/Down/Cross/Square", "%b / %b / %b / %b",
                gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a, gamepad1.x);
        telemetry.addData("Controller 1 L2 / R2", "%.2f / %.2f",
                gamepad1.left_trigger, gamepad1.right_trigger);
        // getPosition() reports the last target, not shaft feedback.
        telemetry.addData("Servo COMMANDS left / right", "%.3f / %.3f",
                servoMotorLeft.getPosition(), servoMotorRight.getPosition());
    }

    private String presetLabel(int index) {
        return Double.isNaN(armPresets[index]) ? "UNSET" : String.format(java.util.Locale.US, "%.3f", armPresets[index]);
    }

    private void applyArmPositionToBothServos() {
        // Every arm button controls ONE target. The right direction is REVERSE,
        // so both receive the SAME logical target; do not also use 1 - target.
        // Commands are consecutive, not guaranteed physical synchronization.
        servoMotorLeft.setPosition(armPosition);
        servoMotorRight.setPosition(armPosition);
    }

    @Override
    public void stop() {
        for (DcMotor motor : driveMotors()) motor.setPower(0.0);
        // Do not command a different arm target or deliberately remove holding
        // torque here; the servo/controller's normal STOP behavior still applies.
    }
}
