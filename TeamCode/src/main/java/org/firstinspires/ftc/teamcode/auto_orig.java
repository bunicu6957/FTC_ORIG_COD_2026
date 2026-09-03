package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * auto_orig
 *
 * Drives forward 1 m, then strafes right 1 m, using the three odometry
 * pods for position feedback.
 *
 * RUN odo_check FIRST. It gives you ODO_TICKS_PER_MM and tells you whether
 * any of the three sign constants below need flipping. The placeholder
 * values here are guesses and will send the robot the wrong way.
 *
 * Port map (encoder socket is independent of the motor on that port):
 *   port 0  motor FL  encoder = RIGHT parallel pod
 *   port 1  motor FR  encoder = PERPENDICULAR pod
 *   port 2  motor BR  encoder = LEFT parallel pod
 *   port 3  motor BL  encoder = unused
 *
 * The drive motors have no working encoders - every one of them is behind
 * an odometry pod. All feedback comes from the pods.
 *
 * The arm is left completely alone here. It rests on its stop with no
 * power, which is where it should be at the start of a match.
 */
@Autonomous(name = "auto_orig", group = "Auto")
public class auto_orig extends LinearOpMode {

    // =============================================================
    // >>> MEASURE THESE WITH odo_check FIRST <<<
    // =============================================================

    /**
     * Encoder ticks per millimetre of forward travel.
     *
     * goBILDA publishes these and the pods are factory tuned, so no
     * calibration push is needed - just pick the one you have:
     *
     *   4-Bar pod     (2000 CPR, 32mm wheel)  ->  19.894
     *   Swingarm pod  (2000 CPR, 48mm wheel)  ->  13.263
     *
     * 4-Bar suits chassis wheels around 96-104mm. Swingarm is for larger
     * wheels with more ground clearance, e.g. a 140mm mecanum chassis.
     */
    private static final double ODO_TICKS_PER_MM = 19.894;   // 4-Bar

    /** +1 if the pod counts up when the robot moves FORWARD, else -1.
     *  MEASURED: pushing forward gave BR +12854 and FL -13453, so the two
     *  mirrored pods need opposite signs. Verified correct. */
    private static final int ODO_LEFT_SIGN  = 1;    // BR port
    private static final int ODO_RIGHT_SIGN = -1;   // FL port

    /** +1 if the perpendicular pod counts up when the robot strafes RIGHT.
     *  MEASURED: strafing LEFT by hand gave FR +18235, so the pod counts
     *  up to the left and this must be -1. */
    private static final int ODO_PERP_SIGN = -1;    // FR port

    /** +1 if positive "x" in the mecanum formula strafes the robot RIGHT.
     *  Flip if the first strafe goes the wrong way. */
    private static final int STRAFE_SIGN = 1;

    // =============================================================
    // MOVEMENT TUNING
    // =============================================================

    /** Cruising power. Keep it low for the first runs. */
    private static final double DRIVE_POWER = 0.35;

    /** Minimum power - below this the robot stalls instead of creeping in. */
    private static final double MIN_POWER = 0.12;

    /** Distance gain: power per mm of remaining error. */
    private static final double KP_DISTANCE = 0.004;

    /** Heading gain: correction per mm of left/right pod mismatch.
     *  This is what keeps the robot straight. Raise it if the robot
     *  curves; lower it if the robot wobbles. */
    private static final double KP_HEADING = 0.002;

    /** Hard cap on the heading and cross-axis corrections. Without this a
     *  bad pod sign makes the correction dominate and the robot spins in
     *  place instead of driving. */
    private static final double MAX_CORRECTION = 0.25;

    /** Cross-axis gain: corrects sideways drift during a forward move,
     *  and forward drift during a strafe. */
    private static final double KP_CROSS = 0.003;

    /** Close enough, in mm. */
    private static final double TOLERANCE_MM = 15.0;

    /** Give up on a move after this long, in seconds. Stops the robot
     *  grinding into a wall for the whole autonomous period. */
    private static final double MOVE_TIMEOUT_S = 6.0;

    /** No-progress abort. If the robot has driven for this long without
     *  covering at least MIN_PROGRESS_MM, something is wrong - almost
     *  always a pod sign. Stop the whole autonomous rather than thrash. */
    private static final double PROGRESS_CHECK_S = 1.5;
    private static final double MIN_PROGRESS_MM = 40.0;

    // =============================================================

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor odoRight, odoPerp, odoLeft;

    private final ElapsedTime moveTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        // Same directions as TeleOp so the mecanum formulas behave identically.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // MUST be RUN_WITHOUT_ENCODER. The encoder sockets on these ports
        // hold odometry pods (or nothing, on BL), so RUN_USING_ENCODER
        // would run a velocity PID against feedback that has nothing to do
        // with the motor - the wheel stalls or fights itself. Run modes
        // persist between OpModes, so this must be set explicitly every time.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Pods, read through the motor name sharing their port.
        odoRight = hardwareMap.get(DcMotor.class, "FL");  // port 0 encoder
        odoPerp  = hardwareMap.get(DcMotor.class, "FR");  // port 1 encoder
        odoLeft  = hardwareMap.get(DcMotor.class, "BR");  // port 2 encoder

        telemetry.addLine("auto_orig ready");
        telemetry.addData("ticks/mm", "%.4f", ODO_TICKS_PER_MM);
        telemetry.addLine("Forward 1000 mm, then strafe right 1000 mm.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        driveForward(1000);
        sleep(300);

        strafeRight(1000);
        sleep(300);

        stopMotors();
        telemetry.addLine("Done.");
        telemetry.update();
    }

    // ---------- Pod readings, in "robot" sense ----------

    private double leftMm()  { return odoLeft.getCurrentPosition()  * ODO_LEFT_SIGN  / ODO_TICKS_PER_MM; }
    private double rightMm() { return odoRight.getCurrentPosition() * ODO_RIGHT_SIGN / ODO_TICKS_PER_MM; }
    private double perpMm()  { return odoPerp.getCurrentPosition()  * ODO_PERP_SIGN  / ODO_TICKS_PER_MM; }

    /** Forward travel is the average of the two parallel pods. */
    private double forwardMm() { return (leftMm() + rightMm()) / 2.0; }

    /** Rotation shows up as the two parallel pods disagreeing. Zero means
     *  the robot has not turned since the baseline was taken. */
    private double turnMm() { return leftMm() - rightMm(); }

    // ---------- Moves ----------

    private void driveForward(double distanceMm) {
        double baseFwd  = forwardMm();
        double baseTurn = turnMm();
        double basePerp = perpMm();

        moveTimer.reset();

        while (opModeIsActive() && moveTimer.seconds() < MOVE_TIMEOUT_S) {
            double travelled = forwardMm() - baseFwd;
            double error = distanceMm - travelled;

            if (Math.abs(error) < TOLERANCE_MM) break;

            double drive = Range.clip(error * KP_DISTANCE, -DRIVE_POWER, DRIVE_POWER);
            drive = applyMinPower(drive);

            // Hold heading, and cancel any sideways drift.
            double turnCorr  = clampCorrection((turnMm() - baseTurn) * KP_HEADING);
            double crossCorr = clampCorrection((perpMm() - basePerp) * KP_CROSS);

            setMecanum(drive, -crossCorr, -turnCorr);
            report("FORWARD", distanceMm, travelled);

            if (noProgress(travelled)) return;
        }

        stopMotors();
    }

    private void strafeRight(double distanceMm) {
        double baseFwd  = forwardMm();
        double baseTurn = turnMm();
        double basePerp = perpMm();

        moveTimer.reset();

        while (opModeIsActive() && moveTimer.seconds() < MOVE_TIMEOUT_S) {
            double travelled = perpMm() - basePerp;
            double error = distanceMm - travelled;

            if (Math.abs(error) < TOLERANCE_MM) break;

            double strafe = Range.clip(error * KP_DISTANCE, -DRIVE_POWER, DRIVE_POWER);
            strafe = applyMinPower(strafe);

            // Hold heading, and cancel any forward drift.
            double turnCorr  = clampCorrection((turnMm() - baseTurn) * KP_HEADING);
            double crossCorr = clampCorrection((forwardMm() - baseFwd) * KP_CROSS);

            setMecanum(-crossCorr, strafe * STRAFE_SIGN, -turnCorr);
            report("STRAFE", distanceMm, travelled);

            if (noProgress(travelled)) return;
        }

        stopMotors();
    }

    // ---------- Helpers ----------

    /**
     * True if the robot has been driving for a while without getting
     * anywhere. Stops the motors and leaves a message on the Driver
     * Station. Returning true aborts the whole autonomous.
     */
    private boolean noProgress(double travelled) {
        if (moveTimer.seconds() < PROGRESS_CHECK_S) return false;
        if (Math.abs(travelled) >= MIN_PROGRESS_MM) return false;

        stopMotors();
        telemetry.addLine("ABORTED - no progress.");
        telemetry.addLine("The robot is driving but the pods say it is not");
        telemetry.addLine("moving. Check ODO_LEFT_SIGN / ODO_RIGHT_SIGN with");
        telemetry.addLine("odo_check: push forward, both must count the SAME way.");
        telemetry.update();
        sleep(4000);
        return true;
    }

    /** Caps a correction term so it can never overwhelm the main move. */
    private double clampCorrection(double c) {
        return Range.clip(c, -MAX_CORRECTION, MAX_CORRECTION);
    }

    /** Keeps a small command from being too weak to move the robot. */
    private double applyMinPower(double power) {
        if (Math.abs(power) < 1e-6) return 0.0;
        if (Math.abs(power) < MIN_POWER) {
            return Math.signum(power) * MIN_POWER;
        }
        return power;
    }

    /** Same formulas as TeleOp, so behaviour matches what you already tuned. */
    private void setMecanum(double y, double x, double rx) {
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void report(String phase, double target, double travelled) {
        telemetry.addData("Phase", phase);
        telemetry.addData("Target mm", "%.0f", target);
        telemetry.addData("Travelled mm", "%.0f", travelled);
        telemetry.addData("Heading drift", "%.0f", turnMm());
        telemetry.addData("Time", "%.1f", moveTimer.seconds());
        telemetry.update();
    }
}