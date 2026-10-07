package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;
// Describes how the REV Hub is physically mounted on the robot.
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// REV Hubs have built-in current sensing; no extra sensor is required.
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Gives access to the Hub's orientation sensor (Inertial Measurement Unit).
import com.qualcomm.robotcore.hardware.IMU;
// Lets us request angles in radians for Java's sine and cosine functions.
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
    // Ignore tiny heading errors while translating. This does not limit manual turns.
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(1.0);
    // This preserves your original motor/joystick signs. Change to -1 ONLY if
    // heading hold turns farther away from its target during a wheels-raised test.
    private static final double HEADING_CORRECTION_SIGN = 1.0;
    // Requires all four encoder cables and correct motor types in configuration.
    // Leave false until the encoder wiring is verified. No new motors required.
    private static final boolean USE_DRIVE_ENCODERS = false;
    private boolean fieldCentric = true;
    private boolean imuAvailable;
    private int driveModeChanges;
    private boolean holdHeadingActive;
    private double holdHeading;
    private double headingCorrection;
    private String imuMessage = "";

    // Read Hub current only twice per second, then reuse the readings for telemetry.
    // These measure whole Hubs, not individual servos or every battery-powered device.
    private static final double HUB_CURRENT_INTERVAL_SECONDS = 0.5;
    private LynxModule[] currentHubs = new LynxModule[0];
    private double[] hubCurrentAmps = new double[0];
    private final ElapsedTime hubCurrentTimer = new ElapsedTime();
    private boolean hubCurrentSampled;
    private String hubCurrentMessage = "Unavailable - no REV Hubs found";

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
    // Defer a short Options action until release, so a hold cannot also act as a tap.
    private static final double OPTIONS_LONG_PRESS_SECONDS = 1.0;
    private final ElapsedTime optionsHoldTimer = new ElapsedTime();
    private boolean optionsPressPending;
    private boolean optionsLongPressHandled;
    private boolean optionsPressValid;
    // PlayStation Cross = a; Square = x; Triangle = y; Circle = b; Options = start.
    private String armMessage = "";
    private SharedPreferences armSettings;
    private String settingsMessage = ""; // Only shown when storage needs attention.
    private final ElapsedTime armTimer = new ElapsedTime();

    // Optional presets are command targets, not measured physical positions.
    // NaN means UNSET: no default pickup/carry/place movement is ever guessed.
    private final double[] armPresets = {Double.NaN, Double.NaN, Double.NaN};
    private final String[] presetNames = {"Pickup", "Carry", "Place"};
    private final String[] presetKeys = {"presetPickup", "presetCarry", "presetPlace"};
    private String presetMessage = "Presets optional: unset buttons do not move the arm.";


    // DUAL-DRIVER TUTORIAL, STEP 1: read both controllers once per robot loop.
    // Each controller keeps its own button history. A fresh press on Controller 2
    // still counts while Controller 1 holds the same button; simultaneous presses
    // are combined into one robot action. Never call loop()/updateArm() twice.
    private final ControllerInput[] controllers = {new ControllerInput(), new ControllerInput()};
    private final ControllerInput controls = new ControllerInput();
    private int driveController; // 0 = Controller 1, 1 = Controller 2.
    private int calibrationController = -1; // The controller that entered owns setup.
    private int optionsController = -1; // Latch a whole Options press to ONE controller.

    private static class ControllerInput {
        double left_stick_x, left_stick_y, right_stick_x, left_trigger, right_trigger;
        boolean a, b, x, y, back, start, dpad_up, dpad_down, dpad_left, dpad_right;
        boolean left_stick_button, right_stick_button, left_bumper, right_bumper;
        boolean crossPressed, squarePressed, trianglePressed, circlePressed;
        boolean sharePressed, optionsPressed;
        final boolean[] presetPressed = new boolean[3];

        void read(Gamepad pad) {
            crossPressed = pad.a && !a;
            squarePressed = pad.x && !x;
            trianglePressed = pad.y && !y;
            circlePressed = pad.b && !b;
            sharePressed = pad.back && !back;
            optionsPressed = pad.start && !start;
            presetPressed[0] = pad.dpad_left && !dpad_left;
            presetPressed[1] = pad.left_stick_button && !left_stick_button;
            presetPressed[2] = pad.dpad_right && !dpad_right;
            left_stick_x = pad.left_stick_x;
            left_stick_y = pad.left_stick_y;
            right_stick_x = pad.right_stick_x;
            left_trigger = pad.left_trigger;
            right_trigger = pad.right_trigger;
            a = pad.a; b = pad.b; x = pad.x; y = pad.y;
            back = pad.back; start = pad.start;
            dpad_up = pad.dpad_up; dpad_down = pad.dpad_down;
            dpad_left = pad.dpad_left; dpad_right = pad.dpad_right;
            left_stick_button = pad.left_stick_button;
            right_stick_button = pad.right_stick_button;
            left_bumper = pad.left_bumper; right_bumper = pad.right_bumper;
        }

        void seed(Gamepad pad) {
            read(pad);
            crossPressed = squarePressed = trianglePressed = circlePressed = false;
            sharePressed = optionsPressed = false;
            for (int i = 0; i < presetPressed.length; i++) presetPressed[i] = false;
        }

        boolean presetHeld(int index) {
            return index == 0 ? dpad_left : (index == 1 ? left_stick_button : dpad_right);
        }
    }

    private void readSharedControls() {
        controllers[0].read(gamepad1);
        controllers[1].read(gamepad2);
        ControllerInput one = controllers[0];
        ControllerInput two = controllers[1];
        // DUAL-DRIVER TUTORIAL, STEP 2: combine held buttons, not analog drive axes.
        // Either driver can use precision or a drive toggle. Arm conflicts are
        // checked separately below, so opposite requests do not fight each other.
        controls.a = one.a || two.a;
        controls.x = one.x || two.x;
        controls.back = one.back || two.back;
        controls.start = one.start || two.start;
        controls.dpad_up = one.dpad_up || two.dpad_up;
        controls.dpad_down = one.dpad_down || two.dpad_down;
        controls.dpad_left = one.dpad_left || two.dpad_left;
        controls.dpad_right = one.dpad_right || two.dpad_right;
        controls.left_stick_button = one.left_stick_button || two.left_stick_button;
        controls.right_stick_button = one.right_stick_button || two.right_stick_button;
        controls.left_bumper = one.left_bumper || two.left_bumper;
        controls.right_bumper = one.right_bumper || two.right_bumper;
        controls.left_trigger = Math.max(one.left_trigger, two.left_trigger);
        controls.right_trigger = Math.max(one.right_trigger, two.right_trigger);
        controls.trianglePressed = one.trianglePressed || two.trianglePressed;
        controls.circlePressed = one.circlePressed || two.circlePressed;
        // During calibration, only its owner can teach endpoints. Held buttons
        // on either controller still take part in existing conflict checks.
        controls.crossPressed = calibrationController >= 0
                ? controllers[calibrationController].crossPressed : one.crossPressed || two.crossPressed;
        controls.squarePressed = calibrationController >= 0
                ? controllers[calibrationController].squarePressed : one.squarePressed || two.squarePressed;

        // DUAL-DRIVER TUTORIAL, STEP 3: choose ONE complete drive-stick group.
        // Controller 1 wins when its translation or turn stick is outside .05.
        // Controller 2 drives when Controller 1 is centered. This allows one
        // person to drive while the other operates the arm, without adding speeds.
        int selected = !sticksCentered(one) ? 0 : (!sticksCentered(two) ? 1 : 0);
        if (selected != driveController) holdHeadingActive = false;
        driveController = selected;
    }

    private boolean sticksCentered(ControllerInput input) {
        return Math.abs(input.left_stick_x) <= STICK_DEADBAND
                && Math.abs(input.left_stick_y) <= STICK_DEADBAND
                && Math.abs(input.right_stick_x) <= STICK_DEADBAND;
    }

    private int armRequest(ControllerInput input) {
        boolean lower = input.left_trigger > 0.5;
        boolean raise = input.right_trigger > 0.5;
        // Preserve trigger precedence over the SAME controller's D-pad.
        if (lower && raise) return 0;
        if (lower) return -1;
        if (raise) return 1;
        if (input.dpad_up == input.dpad_down) return 0;
        return input.dpad_up ? 1 : -1;
    }

    private boolean opposingArmRequests() {
        return armRequest(controllers[0]) * armRequest(controllers[1]) < 0;
    }

    private int firstEligiblePress(boolean options) {
        // A controller cannot take over someone else's calibration session.
        if (calibrationController >= 0) {
            ControllerInput owner = controllers[calibrationController];
            return (options ? owner.optionsPressed : owner.sharePressed) ? calibrationController : -1;
        }
        for (int i = 0; i < controllers.length; i++) {
            if (options ? controllers[i].optionsPressed : controllers[i].sharePressed) return i;
        }
        return -1;
    }


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

        // Find configured Hubs automatically. Current monitoring is informational;
        // discovery or read failures must not disable driving or arm controls.
        try {
            currentHubs = hardwareMap.getAll(LynxModule.class).toArray(new LynxModule[0]);
            hubCurrentAmps = new double[currentHubs.length];
        } catch (RuntimeException error) {
            currentHubs = new LynxModule[0];
            hubCurrentAmps = new double[0];
            hubCurrentMessage = "Unavailable - Hub discovery failed";
        }
        hubCurrentSampled = false;
        hubCurrentTimer.reset();

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
        telemetry.addLine("Both controllers: same controls. Controller 1 drive sticks take priority.");
        telemetry.addLine("Circle: drive mode; L1/R1 on either: precision; Triangle: reset heading.");
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
        // Seed BOTH histories so a button already held at PLAY is not a new press.
        controllers[0].seed(gamepad1);
        controllers[1].seed(gamepad2);
        driveController = 0;
        calibrationController = -1;
        optionsController = -1;
        optionsPressPending = false;
        optionsLongPressHandled = false;
        optionsPressValid = false;
        optionsHoldTimer.reset();
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
        readSharedControls();
        // Per-controller edges coalesce into one action when both press together.
        if (controls.trianglePressed) {
            resetHeading(); // Face field-forward before pressing Triangle.
        }
        if (controls.circlePressed) {
            fieldCentric = !fieldCentric;
            driveModeChanges++;
            holdHeadingActive = false;
        }
        updateArm(); // Calibration mode is also used to stop the chassis below.
        showCalibrationIndicator(); // Display the mode AFTER processing setup buttons.

        // Deadband removes center drift and rescales the rest of the stick range
        // so a full stick STILL commands 1.0. Preserve your original drive signs.
        ControllerInput driveInput = controllers[driveController];
        double left_y = applyDeadband(driveInput.left_stick_y);
        double left_x = -applyDeadband(driveInput.left_stick_x);
        double right_x = -applyDeadband(driveInput.right_stick_x);
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
            // Keep the original held heading so small drift can accumulate and
            // be corrected once it exceeds 1 degree. Inside that band, request no turn.
            if (Math.abs(error) > HEADING_TOLERANCE_RADIANS) {
                headingCorrection = Range.clip(HEADING_CORRECTION_SIGN * HEADING_GAIN * error,
                        -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
            }
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
                : (controls.left_bumper || controls.right_bumper ? PRECISION_POWER : 1.0);

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
        // Setup and preset teaching require BOTH drivers to stop requesting motion.
        return sticksCentered(controllers[0]) && sticksCentered(controllers[1]);
    }

    private boolean anyPresetButton() {
        return controls.dpad_left || controls.left_stick_button || controls.dpad_right;
    }

    private Servo selectedCalibrationServo() {
        return calibrationServo == 0 ? servoMotorLeft : servoMotorRight;
    }

    private void beginServoCalibration(int side, int controller) {
        calibrationController = controller;
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

    private void beginSynchronizedCalibration(int controller) {
        calibrationController = controller;
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
        calibrationController = -1;
        calibrationServo = -1;
        for (int i = 0; i < armPresets.length; i++) armPresets[i] = Double.NaN;
        presetMessage = "Synchronized calibration saved: all presets cleared.";
        armMessage = "BOTH endpoint pairs and parking position saved together.";
        return true;
    }

    private void updateArm() {
        double seconds = Math.min(armTimer.seconds(), 0.05);
        armTimer.reset();
        boolean l2Pressed = controls.left_trigger > 0.5;
        boolean r2Pressed = controls.right_trigger > 0.5;
        boolean movementReleased = !controls.dpad_up && !controls.dpad_down
                && !l2Pressed && !r2Pressed;
        boolean toggledCalibration = false;

        boolean setupControlsReleased = movementReleased && !controls.a && !controls.x
                && !anyPresetButton() && !controls.right_stick_button;
        int shareController = firstEligiblePress(false);
        boolean sharePressed = shareController >= 0;
        boolean optionsPressed = false;
        boolean optionsLongPressed = false;
        int setupController = shareController;
        // DUAL-DRIVER TUTORIAL, STEP 4: keep a whole Options press on its source.
        // Another controller cannot prolong this hold or turn its release into
        // a tap. A second overlapping Options press is ignored until a fresh press.
        if (!optionsPressPending) {
            int pressedController = firstEligiblePress(true);
            if (pressedController >= 0) {
                optionsController = pressedController;
                optionsPressPending = true;
                optionsLongPressHandled = false;
                optionsPressValid = setupControlsReleased && driveSticksCentered() && !controls.back;
                optionsHoldTimer.reset();
            }
        }
        boolean ownerOptionsHeld = optionsController >= 0 && controllers[optionsController].start;
        if (optionsPressPending && (!setupControlsReleased || !driveSticksCentered() || controls.back)) {
            // An invalid hold stays invalid even if the conflicting control is
            // released later. Release Options and make a fresh press to retry.
            optionsPressValid = false;
        }
        if (ownerOptionsHeld && optionsPressPending && !optionsLongPressHandled
                && optionsHoldTimer.seconds() >= OPTIONS_LONG_PRESS_SECONDS) {
            optionsLongPressHandled = true; // One toggle per hold, never repeated.
            optionsLongPressed = true;
            setupController = optionsController;
        }
        if (!ownerOptionsHeld && optionsPressPending) {
            optionsPressed = !optionsLongPressHandled;
            setupController = optionsController;
            optionsPressPending = false;
            optionsController = -1;
        }
        if (sharePressed || optionsPressed || optionsLongPressed) {
            int side = sharePressed ? 0 : 1;
            // Options acts on RELEASE as well as a long hold. Reject conflicting
            // action events even when Options is no longer physically held.
            if ((controls.back && controls.start)
                    || (sharePressed && (optionsPressed || optionsLongPressed))) {
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
                        beginSynchronizedCalibration(setupController);
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
                    beginSynchronizedCalibration(setupController);
                    toggledCalibration = true;
                } else {
                    toggledCalibration = finishSynchronizedCalibration();
                }
            } else if (calibrationMode && side != calibrationServo) {
                armMessage = "Finish " + servoNames[calibrationServo]
                        + " setup with its own button before selecting the other servo.";
            } else if (!calibrationMode) {
                beginServoCalibration(side, setupController);
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
                    calibrationController = -1;
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

        ControllerInput armInput = calibrationController >= 0 ? controllers[calibrationController] : controls;
        double direction = 0.0;
        if (armInput.dpad_up && !armInput.dpad_down) direction = 1.0;
        else if (armInput.dpad_down && !armInput.dpad_up) direction = -1.0;
        if (!toggledCalibration && !optionsPressPending && !controls.a && !controls.x) {
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
            } else if (!calibrationMode && calibrationLoaded && !opposingArmRequests()) {
                if (l2Pressed && !r2Pressed) armPosition = 0.0;
                else if (r2Pressed && !l2Pressed) armPosition = 1.0;
                else if (!l2Pressed && !r2Pressed) armPosition += direction * ARM_RATE_PER_SECOND * seconds;
            }
        }
        armPosition = Range.clip(armPosition, 0.0, 1.0);

        boolean crossPressed = controls.crossPressed;
        boolean squarePressed = controls.squarePressed;
        if (crossPressed || squarePressed) {
            if (!calibrationMode) {
                armMessage = "Limits locked. Share enters LEFT setup; Options enters RIGHT setup.";
            } else if (!movementReleased || optionsPressPending || (controls.a && controls.x)) {
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
        updateArmPresets(movementReleased);
        // No paired command on a mode-transition loop or until both sides are ready.
        if (!calibrationMode && calibrationLoaded && !toggledCalibration && !optionsPressPending) {
            applyArmPositionToBothServos();
        }
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
        int held = 0;
        for (int i = 0; i < armPresets.length; i++) {
            if (controllers[0].presetHeld(i) || controllers[1].presetHeld(i)) held++;
        }
        for (int i = 0; i < armPresets.length; i++) {
            // DUAL-DRIVER TUTORIAL, STEP 5: a shortcut keeps its source controller.
            // R3 must be on THAT controller. Controller 1 wins only if both make
            // the same fresh shortcut press together; never perform two saves.
            int source = controllers[0].presetPressed[i] ? 0 : (controllers[1].presetPressed[i] ? 1 : -1);
            if (source >= 0) {
                boolean teach = controllers[source].right_stick_button;
                // Existing manual controls take priority. Only one shortcut at
                // a time; never recall or teach while changing arm calibration.
                if (held != 1 || !movementReleased || controls.a || controls.x
                        || controls.back || controls.start || calibrationMode) {
                    presetMessage = "Release arm controls; use ONE preset button outside calibration.";
                } else if (!calibrationLoaded) {
                    presetMessage = "Presets inactive until BOTH servos are calibrated.";
                } else if (teach && !driveSticksCentered()) {
                    presetMessage = "Center drive sticks before teaching a preset.";
                } else if (teach) {
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

    private void showHubCurrentTelemetry() {
        if (currentHubs.length == 0) {
            telemetry.addData("Hub current (A)", hubCurrentMessage);
            return;
        }
        if (!hubCurrentSampled || hubCurrentTimer.seconds() >= HUB_CURRENT_INTERVAL_SECONDS) {
            hubCurrentSampled = true;
            hubCurrentTimer.reset();
            for (int i = 0; i < currentHubs.length; i++) {
                try {
                    double amps = currentHubs[i].getCurrent(CurrentUnit.AMPS);
                    // Reject invalid readings; never reuse an old value after a failure.
                    hubCurrentAmps[i] = !Double.isNaN(amps) && !Double.isInfinite(amps)
                            && amps >= 0.0 ? amps : Double.NaN;
                } catch (RuntimeException error) {
                    hubCurrentAmps[i] = Double.NaN;
                }
            }
        }
        double total = 0.0;
        boolean allAvailable = true;
        for (int i = 0; i < currentHubs.length; i++) {
            String label = "Hub " + (i + 1) + " current (A)";
            if (Double.isNaN(hubCurrentAmps[i])) {
                telemetry.addData(label, "Unavailable");
                allAvailable = false;
            } else {
                telemetry.addData(label, "%.2f", hubCurrentAmps[i]);
                total += hubCurrentAmps[i];
            }
        }
        // A partial sum could look misleadingly low. Show a total only when all read.
        if (allAvailable) telemetry.addData("Hub current sum (A)", "%.2f", total);
        else telemetry.addData("Hub current sum (A)", "Unavailable - incomplete readings");
    }

    private void showDriveControlsTelemetry() {
        // Both assigned controllers have the same controls. These raw readings
        // and source labels help explain priority while both drivers are active.
        telemetry.addData("Controller 1 L1/R1/Circle", "%b / %b / %b",
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.b);
        telemetry.addData("Controller 2 L1/R1/Circle", "%b / %b / %b",
                gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.b);
        telemetry.addData("Precision button held", gamepad1.left_bumper || gamepad1.right_bumper
                || gamepad2.left_bumper || gamepad2.right_bumper);
        telemetry.addData("Driving controller", "Controller %d (1 wins when its sticks move)", driveController + 1);
        telemetry.addData("Calibration controller", calibrationController >= 0
                ? "Controller " + (calibrationController + 1) : "None - either may enter");
        telemetry.addData("Selected drive mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Circle switches detected", driveModeChanges);
        telemetry.addData("Heading hold tolerance (degrees)", "%.1f",
                Math.toDegrees(HEADING_TOLERANCE_RADIANS));
        showHubCurrentTelemetry(); // Also called during INIT through arm telemetry.
        // These are electrical power commands, not measured wheel speed.
        telemetry.addData("Drive powers FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                frontLeftMotor.getPower(), backLeftMotor.getPower(),
                frontRightMotor.getPower(), backRightMotor.getPower());
    }

    private void showArmTelemetry() {
        telemetry.addData("Code", "FinalRebuilds - dual drivers + individual/synchronized calibration");
        showDriveControlsTelemetry();
        telemetry.addData("Arm mode", calibrationMode
                ? (synchronizedCalibrationSelected ? "SYNC BOTH CALIBRATION - drive stopped"
                        : servoNames[calibrationServo] + " CALIBRATION - drive stopped")
                : (calibrationLoaded ? "Normal - paired mappings" : "Needs BOTH calibrations - arm commands disabled"));
        telemetry.addLine("Individual: Share LEFT / tap Options RIGHT. Hold Options 1s: switch setup style.");
        telemetry.addLine("Both controllers: L2 minimum / R2 maximum; D-pad adjusts; opposite requests hold.");
        telemetry.addLine("R3 + preset on the SAME controller teaches. Setup belongs to the controller that entered.");
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
        telemetry.addData("Controller 2 Up/Down/Cross/Square", "%b / %b / %b / %b",
                gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.a, gamepad2.x);
        telemetry.addData("Controller 2 L2 / R2", "%.2f / %.2f",
                gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("Controller 2 Share / Options / R3", "%b / %b / %b",
                gamepad2.back, gamepad2.start, gamepad2.right_stick_button);
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
