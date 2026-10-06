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

    // Normal driving uses ONE arm progress value: 0 = lowered, 1 = raised.
    // Each servo maps that progress through its OWN taught endpoint commands.
    // These are commands, not measured angles or guaranteed physical synchronization.
    private static final double ARM_START = 0.50;
    private static final double ARM_RATE_PER_SECOND = 0.10; // D-pad only.
    private static final double MIN_CALIBRATED_SPAN = 0.01;
    private double armPosition = ARM_START;
    private double armStartupPosition = ARM_START;
    // Index 0 = left, index 1 = right. NaN means an endpoint has not been taught.
    private final double[] servoLowerLimits = {Double.NaN, Double.NaN};
    private final double[] servoUpperLimits = {Double.NaN, Double.NaN};
    private final boolean[] servoCalibrated = {false, false};
    private final String[] servoNames = {"LEFT", "RIGHT"};
    private boolean calibrationMode;
    private boolean synchronizedCalibrationSelected;
    private boolean calibrationLoaded; // Normal arm controls require BOTH sides.
    private int calibrationServo = -1; // -1 = normal, 0 = left, 1 = right, 2 = synchronized.
    // Working calibration is separate from the confirmed saved endpoints.
    // Individual setup uses raw servo commands; SYNC uses shared reference progress.
    private double calibrationPosition;
    private double calibrationLower;
    private double calibrationUpper;
    private boolean lowerLimitSet;
    private boolean upperLimitSet;
    private boolean wasSharePressed;
    private boolean wasOptionsPressed;
    // Defer a short Options action until release, so a hold cannot also act as a tap.
    private static final double OPTIONS_LONG_PRESS_SECONDS = 1.0;
    private final ElapsedTime optionsHoldTimer = new ElapsedTime();
    private boolean optionsPressPending;
    private boolean optionsLongPressHandled;
    private boolean optionsPressValid;
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
        // right servo mirror its mapped target, so do NOT also send it 1 - target.
        // Independent endpoint calibration accounts for each horn's alignment.
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
        // With an incomplete pair, INIT sends no new arm position. Individual
        // calibration only commands the selected servo after a D-pad movement.

        // Keep the calibration indicator first on the Driver Station display.
        showCalibrationIndicator();
        showStartupStatus(); // Informational only; does not enable/block controls.

        // Show a setup reminder on the Driver Station; update() sends it.
        telemetry.addLine("Face field-forward before pressing PLAY.");
        telemetry.addLine("Normal arm: D-pad Up raises; Down lowers; release holds target.");
        telemetry.addLine("Circle: drive mode; L1/R1: precision; Triangle: reset heading.");
        telemetry.addLine("Share: LEFT; tap Options: RIGHT; hold Options 1s: SYNC; Cross: min; Square: max.");
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
        wasSharePressed = gamepad1.back;
        wasOptionsPressed = gamepad1.start;
        optionsPressPending = false;
        optionsLongPressHandled = gamepad1.start; // Ignore a button already held at PLAY.
        optionsPressValid = false;
        optionsHoldTimer.reset();
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
        telemetry.addLine("INIT: setup buttons work after PLAY; valid saved pair may already park the arm.");
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
        showCalibrationIndicator(); // Display the mode AFTER processing setup buttons.

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
        double driveScale = calibrationMode || optionsPressPending ? 0.0
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

    private boolean unitPosition(double position) {
        // Range comparisons also reject NaN and infinity.
        return position >= 0.0 && position <= 1.0;
    }

    private boolean validServoCalibration(double lower, double upper) {
        // These labels mean physical lowered/raised poses. Either logical command
        // direction is allowed. The small span prevents division by (nearly) zero;
        // tolerance permits endpoints saved as Android floats to reload correctly.
        return unitPosition(lower) && unitPosition(upper)
                && Math.abs(upper - lower) >= MIN_CALIBRATED_SPAN - 0.000001;
    }

    private double readArmFloat(Map<String, ?> settings, String key) {
        Object value = settings.get(key);
        if (value instanceof Float) return (Float) value;
        if (settings.containsKey(key)) settingsMessage = "Unexpected stored setting type: " + key;
        return Double.NaN;
    }

    private boolean readArmBoolean(Map<String, ?> settings, String key) {
        Object value = settings.get(key);
        if (value instanceof Boolean) return (Boolean) value;
        if (settings.containsKey(key)) settingsMessage = "Unexpected stored setting type: " + key;
        return false;
    }

    private void loadArmCalibration() {
        // Old shared-command limits cannot safely represent two independent sides.
        // Use separate storage; this version requires each side to be taught once.
        armSettings = hardwareMap.appContext.getSharedPreferences(
                "Rebuildv6.arm.independent.v1", Context.MODE_PRIVATE);
        Map<String, ?> saved = armSettings.getAll();
        for (int i = 0; i < servoCalibrated.length; i++) {
            String side = i == 0 ? "left" : "right";
            servoLowerLimits[i] = readArmFloat(saved, side + "Lower");
            servoUpperLimits[i] = readArmFloat(saved, side + "Upper");
            servoCalibrated[i] = readArmBoolean(saved, side + "Valid")
                    && validServoCalibration(servoLowerLimits[i], servoUpperLimits[i]);
        }
        double startup = readArmFloat(saved, "startup");
        calibrationLoaded = servoCalibrated[0] && servoCalibrated[1] && unitPosition(startup);
        if (calibrationLoaded) {
            armStartupPosition = startup;
            armMessage = "BOTH calibrations loaded. INIT commands mapped saved parking position.";
        } else {
            armMessage = "Teach LEFT with Share and RIGHT with Options before normal arm movement.";
        }
    }

    private boolean saveArmCalibration(double parkingProgress) {
        String side = calibrationServo == 0 ? "left" : "right";
        // Save ONLY this side, preserving the other side's confirmed endpoints.
        // Recalibration changes the mapping, so clear all presets in the same write.
        SharedPreferences.Editor editor = armSettings.edit()
                .putFloat(side + "Lower", (float) calibrationLower)
                .putFloat(side + "Upper", (float) calibrationUpper)
                .putBoolean(side + "Valid", true)
                .putFloat("startup", (float) parkingProgress)
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
        String[] floatKeys = {"leftLower", "leftUpper", "rightLower", "rightUpper", "startup",
                "presetPickup", "presetCarry", "presetPlace", "presetLeftLower", "presetLeftUpper",
                "presetRightLower", "presetRightUpper"};
        String[] booleanKeys = {"leftValid", "rightValid"};
        settingsMessage = "";
        // Do not overwrite an unexpected stored type that we cannot restore.
        for (String key : floatKeys) {
            if (previous.containsKey(key) && !(previous.get(key) instanceof Float)) {
                settingsMessage = "Unexpected stored setting type: " + key;
                return false;
            }
        }
        for (String key : booleanKeys) {
            if (previous.containsKey(key) && !(previous.get(key) instanceof Boolean)) {
                settingsMessage = "Unexpected stored setting type: " + key;
                return false;
            }
        }
        if (editor.commit()) return true;

        SharedPreferences.Editor restore = armSettings.edit();
        for (String key : floatKeys) {
            Object value = previous.get(key);
            if (value instanceof Float) restore.putFloat(key, (Float) value);
            else restore.remove(key); // Missing settings must stay missing.
        }
        for (String key : booleanKeys) {
            Object value = previous.get(key);
            if (value instanceof Boolean) restore.putBoolean(key, (Boolean) value);
            else restore.remove(key);
        }
        // Even if disk writing fails again, this restores the in-process cache.
        // Return false: the requested new settings were NOT confirmed saved.
        if (!restore.commit()) {
            settingsMessage = "Cache restored; disk recovery unconfirmed. Retry the save.";
        }
        return false;
    }

    private boolean driveSticksCentered() {
        return Math.abs(gamepad1.left_stick_x) <= STICK_DEADBAND
                && Math.abs(gamepad1.left_stick_y) <= STICK_DEADBAND
                && Math.abs(gamepad1.right_stick_x) <= STICK_DEADBAND;
    }

    private boolean anyPresetButton() {
        return gamepad1.dpad_left || gamepad1.left_stick_button || gamepad1.dpad_right;
    }

    private Servo selectedCalibrationServo() {
        return calibrationServo == 0 ? servoMotorLeft : servoMotorRight;
    }

    private void beginServoCalibration(int side) {
        calibrationServo = side;
        calibrationMode = true;
        calibrationLower = servoLowerLimits[side];
        calibrationUpper = servoUpperLimits[side];
        lowerLimitSet = servoCalibrated[side];
        upperLimitSet = servoCalibrated[side];
        // getPosition() is the last command, not a measurement. If unavailable,
        // the first manual command starts near 0.50. Entry itself sends NO command.
        double lastCommand = selectedCalibrationServo().getPosition();
        calibrationPosition = unitPosition(lastCommand) ? lastCommand : ARM_START;
        armMessage = servoNames[side] + " setup: support arm; disconnect OTHER servo linkage."
                + " D-pad changes this servo; Cross min; Square max; same setup button saves.";
    }

    private void beginSynchronizedCalibration() {
        calibrationMode = true;
        calibrationServo = 2;
        // Keep the separate saved servo endpoints as our reference mapping.
        // 0 and 1 initially represent the existing common lower and upper poses.
        calibrationPosition = armPosition;
        calibrationLower = 0.0;
        calibrationUpper = 1.0;
        lowerLimitSet = true;
        upperLimitSet = true;
        armMessage = "SYNC setup: D-pad moves BOTH through their own ranges."
                + " Cross sets common min; Square common max; tap Options saves."
                + " Hold Options 1 second to save and return to individual setup.";
    }

    private double[] synchronizedProgressBounds() {
        // Find ONE shared travel range that keeps BOTH raw servo commands in 0..1.
        // This can extend beyond the previous common limits while preserving each
        // servo's alignment, without letting one hit its command endpoint early.
        double lower = Double.NEGATIVE_INFINITY;
        double upper = Double.POSITIVE_INFINITY;
        for (int i = 0; i < servoCalibrated.length; i++) {
            double span = servoUpperLimits[i] - servoLowerLimits[i];
            double atZero = -servoLowerLimits[i] / span;
            double atOne = (1.0 - servoLowerLimits[i]) / span;
            lower = Math.max(lower, Math.min(atZero, atOne));
            upper = Math.min(upper, Math.max(atZero, atOne));
        }
        return new double[] {lower, upper};
    }

    private boolean finishSynchronizedCalibration() {
        double[] bounds = synchronizedProgressBounds();
        if (!lowerLimitSet || !upperLimitSet || !(calibrationLower < calibrationUpper)
                || calibrationLower < bounds[0] || calibrationUpper > bounds[1]
                || Double.isNaN(calibrationPosition) || Double.isInfinite(calibrationPosition)
                || calibrationPosition < calibrationLower || calibrationPosition > calibrationUpper) {
            armMessage = "SYNC: set ordered common endpoints; park BETWEEN them before saving.";
            return false;
        }
        double[] lowered = new double[2];
        double[] raised = new double[2];
        for (int i = 0; i < servoCalibrated.length; i++) {
            lowered[i] = mappedServoCommand(i, calibrationLower);
            raised[i] = mappedServoCommand(i, calibrationUpper);
            if (!validServoCalibration(lowered[i], raised[i])) {
                armMessage = "SYNC range too small: each servo needs at least 0.01 command span.";
                return false;
            }
        }
        double parkingProgress = (calibrationPosition - calibrationLower)
                / (calibrationUpper - calibrationLower);
        // Both DIFFERENT endpoint pairs and the parking fraction save atomically.
        SharedPreferences.Editor editor = armSettings.edit()
                .putFloat("leftLower", (float) lowered[0]).putFloat("leftUpper", (float) raised[0])
                .putFloat("rightLower", (float) lowered[1]).putFloat("rightUpper", (float) raised[1])
                .putBoolean("leftValid", true).putBoolean("rightValid", true)
                .putFloat("startup", (float) parkingProgress)
                .putFloat("presetPickup", Float.NaN).putFloat("presetCarry", Float.NaN)
                .putFloat("presetPlace", Float.NaN);
        if (!commitArmSettings(editor)) {
            armMessage = "SYNC save failed; setup remains active. Tap Options to retry.";
            return false;
        }
        for (int i = 0; i < servoCalibrated.length; i++) {
            servoLowerLimits[i] = lowered[i];
            servoUpperLimits[i] = raised[i];
            servoCalibrated[i] = true;
        }
        armPosition = parkingProgress;
        armStartupPosition = parkingProgress;
        calibrationLoaded = true;
        calibrationMode = false;
        calibrationServo = -1;
        for (int i = 0; i < armPresets.length; i++) armPresets[i] = Double.NaN;
        presetMessage = "Synchronized calibration saved: all presets cleared.";
        armMessage = "BOTH endpoint pairs and parking position saved together.";
        return true;
    }

    private void updateArm() {
        double seconds = Math.min(armTimer.seconds(), 0.05);
        armTimer.reset();
        boolean l2Pressed = gamepad1.left_trigger > 0.5;
        boolean r2Pressed = gamepad1.right_trigger > 0.5;
        boolean movementReleased = !gamepad1.dpad_up && !gamepad1.dpad_down
                && !l2Pressed && !r2Pressed;
        boolean toggledCalibration = false;

        boolean setupControlsReleased = movementReleased && !gamepad1.a && !gamepad1.x
                && !anyPresetButton() && !gamepad1.right_stick_button;
        boolean sharePressed = gamepad1.back && !wasSharePressed;
        boolean optionsPressed = false;
        boolean optionsLongPressed = false;
        if (gamepad1.start && !wasOptionsPressed) {
            optionsPressPending = true;
            optionsLongPressHandled = false;
            optionsPressValid = setupControlsReleased && driveSticksCentered() && !gamepad1.back;
            optionsHoldTimer.reset();
        }
        if (optionsPressPending && (!setupControlsReleased || !driveSticksCentered() || gamepad1.back)) {
            // An invalid hold stays invalid even if the conflicting control is
            // released later. Release Options and make a fresh press to retry.
            optionsPressValid = false;
        }
        if (gamepad1.start && optionsPressPending && !optionsLongPressHandled
                && optionsHoldTimer.seconds() >= OPTIONS_LONG_PRESS_SECONDS) {
            optionsLongPressHandled = true; // One toggle per hold, never repeated.
            optionsLongPressed = true;
        }
        if (!gamepad1.start && wasOptionsPressed && optionsPressPending) {
            optionsPressed = !optionsLongPressHandled;
            optionsPressPending = false;
        }
        if (sharePressed || optionsPressed || optionsLongPressed) {
            int side = sharePressed ? 0 : 1;
            if (gamepad1.back && gamepad1.start) {
                armMessage = "Press ONE setup button: Share LEFT, Options RIGHT.";
            } else if (!setupControlsReleased || ((optionsPressed || optionsLongPressed) && !optionsPressValid)) {
                armMessage = "Center drive sticks, release arm controls, then make a fresh setup press.";
            } else if (!driveSticksCentered()) {
                armMessage = "Center drive sticks before entering or leaving servo setup.";
            } else if (optionsLongPressed) {
                if (!synchronizedCalibrationSelected) {
                    if (calibrationMode) {
                        armMessage = "Save this individual calibration before switching to SYNC.";
                    } else if (!calibrationLoaded) {
                        armMessage = "Save BOTH individual servo calibrations before entering SYNC.";
                    } else {
                        synchronizedCalibrationSelected = true;
                        beginSynchronizedCalibration();
                        toggledCalibration = true;
                    }
                } else if (!calibrationMode || finishSynchronizedCalibration()) {
                    // Leaving active SYNC saves valid edits first. A rejected or
                    // failed save stays in SYNC, preserving the working setup.
                    synchronizedCalibrationSelected = false;
                    armMessage = "INDIVIDUAL setup selected: Share LEFT; tap Options RIGHT.";
                    toggledCalibration = true;
                }
            } else if (synchronizedCalibrationSelected) {
                if (sharePressed) {
                    armMessage = "SYNC selected: tap Options for setup; hold Options to return to INDIVIDUAL.";
                } else if (!calibrationMode) {
                    beginSynchronizedCalibration();
                    toggledCalibration = true;
                } else {
                    toggledCalibration = finishSynchronizedCalibration();
                }
            } else if (calibrationMode && side != calibrationServo) {
                armMessage = "Finish " + servoNames[calibrationServo]
                        + " setup with its own button before selecting the other servo.";
            } else if (!calibrationMode) {
                beginServoCalibration(side);
                toggledCalibration = true;
            } else if (!lowerLimitSet || !upperLimitSet
                    || !validServoCalibration(calibrationLower, calibrationUpper)
                    || !unitPosition(calibrationPosition)
                    || calibrationPosition < Math.min(calibrationLower, calibrationUpper)
                    || calibrationPosition > Math.max(calibrationLower, calibrationUpper)) {
                armMessage = "Teach BOTH physical endpoints (span at least 0.01); park BETWEEN them.";
            } else {
                // Derive the shared parking fraction from this servo's present
                // command, preserving its pose instead of jumping to a midpoint.
                double parkingProgress = (calibrationPosition - calibrationLower)
                        / (calibrationUpper - calibrationLower);
                if (saveArmCalibration(parkingProgress)) {
                    servoLowerLimits[side] = calibrationLower;
                    servoUpperLimits[side] = calibrationUpper;
                    servoCalibrated[side] = true;
                    armStartupPosition = parkingProgress;
                    armPosition = parkingProgress;
                    calibrationLoaded = servoCalibrated[0] && servoCalibrated[1];
                    calibrationMode = false;
                    calibrationServo = -1;
                    for (int i = 0; i < armPresets.length; i++) armPresets[i] = Double.NaN;
                    presetMessage = "Servo calibration saved: all presets cleared.";
                    armMessage = servoNames[side] + " saved. " + (calibrationLoaded
                            ? "BOTH ready: normal controls use both mappings."
                            : "Teach the other servo before normal arm movement.");
                    toggledCalibration = true;
                } else {
                    armMessage = "Save failed; still in " + servoNames[side]
                            + " setup. Press its setup button to retry.";
                }
            }
        }
        wasSharePressed = gamepad1.back;
        wasOptionsPressed = gamepad1.start;

        double direction = 0.0;
        if (gamepad1.dpad_up && !gamepad1.dpad_down) direction = 1.0;
        else if (gamepad1.dpad_down && !gamepad1.dpad_up) direction = -1.0;
        if (!toggledCalibration && !optionsPressPending && !gamepad1.a && !gamepad1.x) {
            if (calibrationMode && !l2Pressed && !r2Pressed && direction != 0.0) {
                if (synchronizedCalibrationSelected) {
                    double[] bounds = synchronizedProgressBounds();
                    calibrationPosition = Range.clip(calibrationPosition
                            + direction * ARM_RATE_PER_SECOND * seconds, bounds[0], bounds[1]);
                    applyArmProgressToBothServos(calibrationPosition);
                } else {
                    // Individual setup adjusts ONLY the selected servo. The other
                    // linkage must be disconnected before isolated movement.
                    calibrationPosition = Range.clip(calibrationPosition
                            + direction * ARM_RATE_PER_SECOND * seconds, 0.0, 1.0);
                    selectedCalibrationServo().setPosition(calibrationPosition);
                }
            } else if (!calibrationMode && calibrationLoaded) {
                if (l2Pressed && !r2Pressed) armPosition = 0.0;
                else if (r2Pressed && !l2Pressed) armPosition = 1.0;
                else if (!l2Pressed && !r2Pressed) armPosition += direction * ARM_RATE_PER_SECOND * seconds;
            }
        }
        armPosition = Range.clip(armPosition, 0.0, 1.0);

        boolean crossPressed = gamepad1.a && !wasCrossPressed;
        boolean squarePressed = gamepad1.x && !wasSquarePressed;
        if (crossPressed || squarePressed) {
            if (!calibrationMode) {
                armMessage = "Limits locked. Share enters LEFT setup; Options enters RIGHT setup.";
            } else if (!movementReleased || optionsPressPending || (gamepad1.a && gamepad1.x)) {
                armMessage = "Release movement/setup controls, then press ONE endpoint button again.";
            } else if (crossPressed) {
                calibrationLower = calibrationPosition;
                lowerLimitSet = true;
                armMessage = "LOWERED pose selected. Teach raised pose; same setup button saves.";
            } else {
                calibrationUpper = calibrationPosition;
                upperLimitSet = true;
                armMessage = "RAISED pose selected. Park between endpoints; same setup button saves.";
            }
        }
        wasCrossPressed = gamepad1.a;
        wasSquarePressed = gamepad1.x;
        updateArmPresets(movementReleased);
        // No paired command on a mode-transition loop or until both sides are ready.
        if (!calibrationMode && calibrationLoaded && !toggledCalibration && !optionsPressPending) {
            applyArmPositionToBothServos();
        }
    }

    private boolean[] presetButtons() {
        // Free buttons: D-pad Left = pickup; L3 = carry; D-pad Right = place.
        return new boolean[] {gamepad1.dpad_left, gamepad1.left_stick_button, gamepad1.dpad_right};
    }

    private void loadArmPresets() {
        // A progress preset is valid only for BOTH endpoint mappings it was taught
        // with. Compare as floats to match Android's saved precision.
        Map<String, ?> saved = armSettings.getAll();
        if (!calibrationLoaded
                || readArmFloat(saved, "presetLeftLower") != (float) servoLowerLimits[0]
                || readArmFloat(saved, "presetLeftUpper") != (float) servoUpperLimits[0]
                || readArmFloat(saved, "presetRightLower") != (float) servoLowerLimits[1]
                || readArmFloat(saved, "presetRightUpper") != (float) servoUpperLimits[1]) return;
        for (int i = 0; i < armPresets.length; i++) {
            double target = readArmFloat(saved, presetKeys[i]);
            if (unitPosition(target)) armPresets[i] = target;
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
                        || gamepad1.back || gamepad1.start || calibrationMode) {
                    presetMessage = "Release arm controls; use ONE preset button outside calibration.";
                } else if (!calibrationLoaded) {
                    presetMessage = "Presets inactive until BOTH servos are calibrated.";
                } else if (gamepad1.right_stick_button && !driveSticksCentered()) {
                    presetMessage = "Center drive sticks before teaching a preset.";
                } else if (gamepad1.right_stick_button) {
                    // Share now selects LEFT calibration. Hold R3 + shortcut to
                    // teach the CURRENT shared progress instead. Wait
                    // for the mechanism to settle first; no sensor measures it.
                    SharedPreferences.Editor editor = armSettings.edit();
                    // Write the entire validated in-memory set. This also removes
                    // stale stored presets from a different calibration snapshot.
                    for (int j = 0; j < armPresets.length; j++) {
                        editor.putFloat(presetKeys[j], (float) (j == i ? armPosition : armPresets[j]));
                    }
                    editor.putFloat("presetLeftLower", (float) servoLowerLimits[0])
                            .putFloat("presetLeftUpper", (float) servoUpperLimits[0])
                            .putFloat("presetRightLower", (float) servoLowerLimits[1])
                            .putFloat("presetRightUpper", (float) servoUpperLimits[1]);
                    boolean saved = commitArmSettings(editor);
                    if (saved) {
                        armPresets[i] = armPosition;
                        presetMessage = presetNames[i] + " target saved. Release R3 to recall.";
                    } else {
                        presetMessage = "Preset save failed; previous target kept. Release and retry.";
                    }
                } else if (Double.isNaN(armPresets[i])) {
                    // Unset presets DO NOTHING: the robot can be used before
                    // the front mechanism is fitted and taught.
                    presetMessage = presetNames[i] + " is UNSET; arm target unchanged.";
                } else {
                    armPosition = Range.clip(armPresets[i], 0.0, 1.0);
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
        telemetry.addData("Startup status", "IMU: %s | left: %s | right: %s | presets: %d/3 (optional)",
                imuAvailable ? "initialized" : "robot-centric fallback",
                servoCalibrated[0] ? "saved" : "not saved",
                servoCalibrated[1] ? "saved" : "not saved", taught);
    }

    private void showCalibrationIndicator() {
        // Read the SAME flag that stops the chassis and enables endpoint buttons.
        // This appears first during INIT and PLAY, including a failed save/exit.
        telemetry.addData("*** ARM CALIBRATION ***", calibrationMode
                ? (synchronizedCalibrationSelected ? "ON - SYNC BOTH - CHASSIS STOPPED"
                        : "ON - " + servoNames[calibrationServo] + " ONLY - CHASSIS STOPPED")
                : (calibrationLoaded ? "OFF - BOTH SERVOS READY" : "OFF - BOTH CALIBRATIONS REQUIRED"));
        telemetry.addData("Calibration style", synchronizedCalibrationSelected ? "SYNCHRONIZED" : "INDIVIDUAL");
        telemetry.addData("Calibration controls", calibrationMode
                ? (synchronizedCalibrationSelected
                        ? "D-pad: BOTH | Cross: min | Square: max | tap Options: save | hold Options 1s: individual"
                        : "D-pad: selected servo | Cross: lowered | Square: raised | "
                                + (calibrationServo == 0 ? "Share" : "tap Options") + ": save/exit")
                : (synchronizedCalibrationSelected ? "Tap Options: SYNC setup | hold Options 1s: individual"
                        : "Share: LEFT | tap Options: RIGHT | hold Options 1s: SYNC"));
        if (calibrationMode && !synchronizedCalibrationSelected) {
            telemetry.addLine("Support arm; disconnect OTHER servo linkage before individual movement.");
        }
        if (optionsPressPending) telemetry.addData("Options hold (seconds)", "%.1f", optionsHoldTimer.seconds());
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
        telemetry.addData("Code", "FinalRebuilds - individual + synchronized calibration");
        showDriveControlsTelemetry();
        telemetry.addData("Arm mode", calibrationMode
                ? (synchronizedCalibrationSelected ? "SYNC BOTH CALIBRATION - drive stopped"
                        : servoNames[calibrationServo] + " CALIBRATION - drive stopped")
                : (calibrationLoaded ? "Normal - paired mappings" : "Needs BOTH calibrations - arm commands disabled"));
        telemetry.addLine("Individual: Share LEFT / tap Options RIGHT. Hold Options 1s: switch setup style.");
        telemetry.addLine("Normal: L2 minimum / R2 maximum; D-pad adjusts; release holds.");
        telemetry.addData("Arm COMMAND progress (%)", "%.1f", armPosition * 100.0);
        telemetry.addData("Saved parking progress (%)", "%.1f", armStartupPosition * 100.0);
        for (int i = 0; i < servoCalibrated.length; i++) {
            telemetry.addData(servoNames[i] + " calibration", servoCalibrated[i] ? "SAVED" : "NOT SET");
            telemetry.addData(servoNames[i] + " saved lowered / raised commands", "%.3f / %.3f",
                    servoLowerLimits[i], servoUpperLimits[i]);
        }
        if (calibrationMode) {
            if (synchronizedCalibrationSelected) {
                telemetry.addData("SYNC reference progress (%)", "%.1f", calibrationPosition * 100.0);
                telemetry.addData("Working common min / max (%)", "%.1f / %.1f",
                        calibrationLower * 100.0, calibrationUpper * 100.0);
            } else {
                telemetry.addData("Selected servo COMMAND", "%.3f", calibrationPosition);
                telemetry.addData("Working lowered / raised commands", "%s / %s",
                        lowerLimitSet ? String.format(java.util.Locale.US, "%.3f", calibrationLower) : "UNSET",
                        upperLimitSet ? String.format(java.util.Locale.US, "%.3f", calibrationUpper) : "UNSET");
            }
        }
        telemetry.addData("Arm status", armMessage);
        if (!settingsMessage.isEmpty()) telemetry.addData("Settings storage", settingsMessage);
        telemetry.addData("Presets pickup / carry / place", "%s / %s / %s",
                presetLabel(0), presetLabel(1), presetLabel(2));
        telemetry.addData("Preset status", presetMessage);
        telemetry.addData("Controller 1 Up/Down/Cross/Square", "%b / %b / %b / %b",
                gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.a, gamepad1.x);
        telemetry.addData("Controller 1 L2 / R2", "%.2f / %.2f",
                gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Controller 1 Share / Options / R3", "%b / %b / %b",
                gamepad1.back, gamepad1.start, gamepad1.right_stick_button);
        // getPosition() reports the last target, not shaft feedback.
        telemetry.addData("Servo COMMANDS left / right", "%.3f / %.3f",
                servoMotorLeft.getPosition(), servoMotorRight.getPosition());
    }

    private String presetLabel(int index) {
        return Double.isNaN(armPresets[index]) ? "UNSET" : String.format(java.util.Locale.US, "%.3f", armPresets[index]);
    }

    private void applyArmPositionToBothServos() {
        applyArmProgressToBothServos(armPosition);
    }

    private double mappedServoCommand(int side, double progress) {
        // Callers constrain ONE shared progress value first. Clipping here only
        // removes floating-point roundoff at a computed 0/1 command boundary.
        return Range.clip(servoLowerLimits[side]
                + progress * (servoUpperLimits[side] - servoLowerLimits[side]), 0.0, 1.0);
    }

    private void applyArmProgressToBothServos(double progress) {
        // Example: progress 0.5 commands each servo halfway between its OWN
        // taught lowered and raised poses. Different commands can therefore mean
        // the SAME arm pose. Teach both sides at identical physical endpoint poses.
        // The right direction remains REVERSE: do not also use 1 - target here.
        // Commands are consecutive, not guaranteed physical synchronization.
        double leftTarget = mappedServoCommand(0, progress);
        double rightTarget = mappedServoCommand(1, progress);
        servoMotorLeft.setPosition(leftTarget);
        servoMotorRight.setPosition(rightTarget);
    }

    @Override
    public void stop() {
        for (DcMotor motor : driveMotors()) motor.setPower(0.0);
        // Do not command a different arm target or deliberately remove holding
        // torque here; the servo/controller's normal STOP behavior still applies.
    }
}
