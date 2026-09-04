package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * auto_ticks
 *
 * Routine:
 *   0. raise the arm to its carry position and hold it there
 *   1. forward 250 mm
 *   2. turn left, on the spot
 *   3. forward, the long leg
 *   4. fire the intake several times, in bursts
 *
 * The driving runs off the drive motors' own encoders.
 *
 * THE ARM
 * The arm is put in RUN_TO_POSITION and left there. The hub's own motor
 * controller holds the position in hardware, which matters here: a
 * LinearOpMode blocks inside each drive move, so a TeleOp-style hold loop
 * would never get to run and the arm would sag.
 *
 * This is safe only because the arm's direction is known correct. If the
 * arm ever runs away when this OpMode starts, stop immediately - that
 * means the encoder and motor directions disagree, and RUN_TO_POSITION
 * cannot recover from it.
 *
 * WIRING REQUIRED
 * Each drive motor's 4-pin encoder cable must run into the encoder socket
 * of that motor's own port on the Control Hub. A port's encoder socket
 * serves an odometry pod or the motor, never both. A wheel that does not
 * turn at all is almost always an unplugged encoder: RUN_TO_POSITION with
 * no feedback means the hub sees zero movement and the motor sits still.
 *
 * ACCURACY, WORST TO BEST
 * Turning is the least reliable move here. Rotating a mecanum drive makes
 * every roller scrub sideways, so the wheels turn further than the
 * geometry predicts. TURN_CORRECTION exists to absorb that, but it drifts
 * with floor surface and battery charge. If the turn needs to be right
 * every time, use the Control Hub's built-in IMU for heading instead of
 * counting ticks.
 */
@Autonomous(name = "auto_ticks", group = "Auto")
public class auto_ticks extends LinearOpMode {

    // =============================================================
    // ROBOT CONSTANTS
    // =============================================================

    /** goBILDA 5203 Yellow Jacket, 435 RPM, 13.7:1 gearbox. */
    private static final double TICKS_PER_REV = 383.6;

    /** Mecanum wheel diameter in mm. goBILDA sells 96mm and 104mm -
     *  measure yours, a wrong value scales every distance. */
    private static final double WHEEL_DIAMETER_MM = 96.0;

    /** MEASURE THESE. Track width is left wheel centre to right wheel
     *  centre. Wheelbase is front axle to rear axle. Both in mm. They set
     *  how far a wheel must travel to swing the robot through an angle. */
    private static final double TRACK_WIDTH_MM = 350.0;
    private static final double WHEEL_BASE_MM = 300.0;

    /** Derived: ticks per millimetre of wheel travel. */
    private static final double TICKS_PER_MM =
            TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    /** Derived: the radius the wheels sweep when the robot spins in place. */
    private static final double TURN_RADIUS_MM =
            (TRACK_WIDTH_MM + WHEEL_BASE_MM) / 2.0;

    /**
     * Strafing and turning both scrub the rollers, so the wheels travel
     * further than the geometry says. 1.0 means no compensation.
     *
     * Measure each one: ask for the move, see what you actually get, and
     * set the factor to (asked / actual). Ask for 90 degrees, get 78, set
     * TURN_CORRECTION to 90/78 = 1.15. Most robots land near 1.1 to 1.2.
     */
    private static final double STRAFE_CORRECTION = 1.0;
    private static final double TURN_CORRECTION = 1.0;
    //ceva
    // =============================================================
    // ARM
    // =============================================================

    /** Where the arm goes at the start, in ticks from its resting spot.
     *  Matches ARM_POS_RAISED in the TeleOp. */
    private static final int ARM_AUTO_TICKS = 125;

    /** Power given to the arm's position controller. Enough to lift and
     *  hold against gravity without straining. */
    private static final double ARM_AUTO_POWER = 0.5;

    /** How long to wait for the arm to reach position before carrying on
     *  with the driving, in seconds. */
    private static final double ARM_TIMEOUT_S = 3.0;

    // =============================================================
    // INTAKE
    // =============================================================

    /** Servo power during a burst. Negative runs the pair OUTWARD, so
     *  each burst ejects rather than collects. This is the same as intake
     *  state 2 in the TeleOp, the one bound to Y. */
    private static final double INTAKE_POWER = -1.0;

    /** How long each burst runs. */
    private static final long INTAKE_FIRE_MS = 500;

    /** How long the intake sits still between bursts. */
    private static final long INTAKE_WAIT_MS = 500;

    /** Total number of bursts, counting the first one. */
    private static final int INTAKE_FIRE_COUNT = 6;

    // =============================================================
    // THE ROUTINE
    // =============================================================

    private static final double LEG_1_MM = 250;      // forward, a quarter metre
    private static final double TURN_1_DEG = 50;     // positive = LEFT
    private static final double LEG_2_MM = 2825;     // forward, metre and a half

    /** Pause between moves so the robot settles before the next one. */
    private static final long SETTLE_MS = 180;

    /** Drive power. Lower is more accurate - less overshoot, less slip. */
    private static final double POWER = 0.8;

    /** Turning slower is worth it: the scrub is more consistent. */
    private static final double TURN_POWER = 0.8;

    /** Strafing needs a bit more push to overcome roller scrub. */
    private static final double STRAFE_POWER = 0.8;

    /** Give up after this long, in seconds, in case a wheel is stuck. */
    private static final double TIMEOUT_S = 8.0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx armMotor;
    private CRServo intakeLeft, intakeRight;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        armMotor    = hardwareMap.get(DcMotorEx.class, "AM");
        intakeLeft  = hardwareMap.get(CRServo.class, "IL");
        intakeRight = hardwareMap.get(CRServo.class, "IR");

        // Reversing one side means the same power value spins the pair in
        // physically opposite directions. Same as the TeleOp.
        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.REVERSE);
        setIntake(0);

        // Arm zeroes here, so it must be resting at the bottom at init.
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);

        // Same directions as TeleOp: positive power drives all four forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        telemetry.addLine("auto_ticks ready");
        telemetry.addData("1", "forward %.0f mm", LEG_1_MM);
        telemetry.addData("2", "turn %.0f deg left", TURN_1_DEG);
        telemetry.addData("3", "forward %.0f mm", LEG_2_MM);
        telemetry.addData("Ticks per mm", "%.4f", TICKS_PER_MM);
        telemetry.addData("Turn radius", "%.0f mm", TURN_RADIUS_MM);
        telemetry.addLine("Arm must be resting at the BOTTOM right now.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        raiseArm();

        driveStraight(LEG_1_MM);
        sleep(SETTLE_MS);

        if (!opModeIsActive()) return;
        turnLeft(TURN_1_DEG);
        sleep(SETTLE_MS);

        if (!opModeIsActive()) return;
        driveStraight(LEG_2_MM);

        if (!opModeIsActive()) return;
        fireIntake();

        telemetry.addLine("Done.");
        telemetry.addData("Arm", armMotor.getCurrentPosition());
        reportPositions();
        telemetry.update();
        sleep(3000);
    }

    // ---------- Arm ----------

    /**
     * Sends the arm up and hands it to the hub's position controller,
     * which keeps holding it for the rest of the OpMode. Nothing here has
     * to be serviced in a loop.
     *
     * setTargetPosition MUST come before RUN_TO_POSITION, or the Driver
     * Station throws "target position not set".
     */
    private void raiseArm() {
        armMotor.setTargetPosition(ARM_AUTO_TICKS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_AUTO_POWER);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && armMotor.isBusy()
                && timer.seconds() < ARM_TIMEOUT_S) {
            telemetry.addData("Phase", "ARM UP");
            telemetry.addData("Arm", "%d -> %d",
                    armMotor.getCurrentPosition(), ARM_AUTO_TICKS);
            telemetry.update();
        }

        // Power stays on deliberately. Cutting it here would let the arm
        // fall, since it is not self-supporting.
    }

    // ---------- Intake ----------

    /**
     * Fires the intake in bursts: run, pause, run, pause, and so on.
     * INTAKE_FIRE_COUNT bursts in total, counting the first.
     *
     * The pause after the final burst is skipped - the routine ends on a
     * burst, not on dead time.
     */
    private void fireIntake() {
        for (int i = 1; i <= INTAKE_FIRE_COUNT && opModeIsActive(); i++) {
            telemetry.addData("Phase", "INTAKE");
            telemetry.addData("Burst", "%d of %d", i, INTAKE_FIRE_COUNT);
            telemetry.update();

            setIntake(INTAKE_POWER);
            sleep(INTAKE_FIRE_MS);
            setIntake(0);

            if (i < INTAKE_FIRE_COUNT) {
                sleep(INTAKE_WAIT_MS);
            }
        }
        setIntake(0);
    }

    private void setIntake(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    // ---------- Moves ----------

    /** Positive drives forward, negative backward. All four wheels equal. */
    private void driveStraight(double mm) {
        int t = (int) Math.round(mm * TICKS_PER_MM);
        runToTargets(t, t, t, t, POWER, "STRAIGHT");
    }

    /**
     * Positive turns LEFT (counter-clockwise seen from above), negative
     * turns right. The robot pivots on the spot: the left wheels roll
     * backward while the right wheels roll forward.
     *
     * Each wheel sweeps an arc of radius TURN_RADIUS_MM, so the distance
     * it must cover is that radius times the angle in radians.
     */
    private void turnLeft(double degrees) {
        double arcMm = Math.toRadians(degrees) * TURN_RADIUS_MM;
        int t = (int) Math.round(arcMm * TICKS_PER_MM * TURN_CORRECTION);
        runToTargets(-t, t, -t, t, TURN_POWER, "TURN");
    }

    /**
     * Positive strafes right, negative left. Kept for later - the current
     * routine does not use it.
     *
     * Strafing right means the diagonal pairs turn opposite ways:
     * front-left and back-right forward, front-right and back-left back.
     */
    private void strafe(double mm) {
        int t = (int) Math.round(mm * TICKS_PER_MM * STRAFE_CORRECTION);
        runToTargets(t, -t, -t, t, STRAFE_POWER, "STRAFE");
    }

    // ---------- Machinery ----------

    /**
     * Sets a per-wheel tick target and waits for the wheels to get there.
     * Targets are relative to wherever the encoders are now, so moves
     * chain without needing a reset between them.
     *
     * setTargetPosition MUST be called before RUN_TO_POSITION, or the
     * Driver Station throws "target position not set".
     */
    private void runToTargets(int dFL, int dFR, int dBL, int dBR,
                              double power, String label) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + dFL);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + dFR);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + dBL);
        backRight.setTargetPosition(backRight.getCurrentPosition() + dBR);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        ElapsedTime timer = new ElapsedTime();

        // Waiting on ALL four means one slow wheel cannot end the move early.
        while (opModeIsActive()
                && timer.seconds() < TIMEOUT_S
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("Phase", label);
            reportPositions();
            telemetry.addData("Time", "%.1f", timer.seconds());
            telemetry.update();
        }

        setPower(0);

        // Back to a normal mode so the next OpMode does not inherit
        // RUN_TO_POSITION and refuse to drive.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void reportPositions() {
        telemetry.addData("FL", "%d -> %d",
                frontLeft.getCurrentPosition(), frontLeft.getTargetPosition());
        telemetry.addData("FR", "%d -> %d",
                frontRight.getCurrentPosition(), frontRight.getTargetPosition());
        telemetry.addData("BL", "%d -> %d",
                backLeft.getCurrentPosition(), backLeft.getTargetPosition());
        telemetry.addData("BR", "%d -> %d",
                backRight.getCurrentPosition(), backRight.getTargetPosition());
    }

    private void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}