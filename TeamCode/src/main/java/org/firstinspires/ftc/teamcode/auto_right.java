package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * auto_forward
 *
 * Raises the arm to its carry position, drives forward three quarters of
 * a metre, then strafes a metre to the right and stops.
 *
 * THE ARM
 * The arm goes into RUN_TO_POSITION and stays there, so the hub's own
 * controller holds it in hardware. That matters: a LinearOpMode blocks
 * inside the drive move, so a TeleOp-style hold loop would never get to
 * run and the arm would sag.
 *
 * The encoder zeroes at init, so the arm MUST be resting at its bottom
 * position when you press init - 100 ticks is measured from there.
 *
 * Runs off the drive motors' own encoders, so each motor's 4-pin encoder
 * cable must be in the encoder socket of its own port. A wheel that does
 * not turn at all is almost always an unplugged encoder: RUN_TO_POSITION
 * with no feedback means the hub sees zero movement and the motor sits
 * still.
 */
@Autonomous(name = "auto_left", group = "Auto")
public class auto_right extends LinearOpMode {

    /** goBILDA 5203 Yellow Jacket, 435 RPM, 13.7:1 gearbox. */
    private static final double TICKS_PER_REV = 383.6;

    /** Mecanum wheel diameter in mm. */
    private static final double WHEEL_DIAMETER_MM = 96.0;

    /** Derived: ticks per millimetre of wheel travel. */
    private static final double TICKS_PER_MM =
            TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    /** How far to drive. Positive forward, negative backward. */
    private static final double DISTANCE_MM = 1000;

    /** How far to strafe afterwards. Positive right, negative left. */
    private static final double STRAFE_MM = 1000;

    /**
     * Strafing covers less ground per tick than driving, because the
     * mecanum rollers scrub sideways. 1.0 means no compensation.
     *
     * Measure it: ask for 1000mm sideways, see what you actually get, and
     * set this to (asked / actual). Getting 850 when you asked for 1000
     * means 1000/850 = 1.18. Most robots land between 1.1 and 1.2.
     */
    private static final double STRAFE_CORRECTION = 1.0;

    /** Pause between the two moves so the robot settles. */
    private static final long SETTLE_MS = 150;

    /** Drive power. */
    private static final double POWER = 0.7;

    /** Strafing needs a bit more push to overcome roller scrub. */
    private static final double STRAFE_POWER = 0.5;

    /** Give up after this long, in seconds, in case a wheel is stuck. */
    private static final double TIMEOUT_S = 8.0;

    /** Where the arm goes at the start, in ticks from its resting spot.
     *  Matches ARM_POS_RAISED in the TeleOp. */
    private static final int ARM_TICKS = 100;

    /** Power given to the arm's position controller - enough to lift and
     *  then hold against gravity. */
    private static final double ARM_POWER = 0.5;

    /** How long to wait for the arm before carrying on with the driving. */
    private static final double ARM_TIMEOUT_S = 3.0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx armMotor;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");
        armMotor   = hardwareMap.get(DcMotorEx.class, "AM");

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

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("auto_forward ready");
        telemetry.addData("Forward", "%.0f mm", DISTANCE_MM);
        telemetry.addData("Strafe", "%.0f mm %s",
                Math.abs(STRAFE_MM), STRAFE_MM < 0 ? "left" : "right");
        telemetry.addData("Arm target", ARM_TICKS);
        telemetry.addLine("Arm must be resting at the BOTTOM right now.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        raiseArm();
        if (!opModeIsActive()) return;

        driveStraight(DISTANCE_MM);
        sleep(SETTLE_MS);

        if (!opModeIsActive()) return;
        strafe(STRAFE_MM);

        telemetry.addLine("Done.");
        telemetry.addData("Arm", armMotor.getCurrentPosition());
        telemetry.update();
        sleep(2000);
    }

    // ---------- Moves ----------

    /** Positive drives forward, negative backward. All four wheels equal. */
    private void driveStraight(double mm) {
        int t = (int) Math.round(mm * TICKS_PER_MM);
        runToTargets(t, t, t, t, POWER, "STRAIGHT");
    }

    /**
     * Positive strafes right, negative left.
     *
     * Strafing right means the diagonal pairs turn opposite ways:
     * front-left and back-right forward, front-right and back-left back.
     * Same pattern as the x term in the TeleOp mixing formulas.
     */
    private void strafe(double mm) {
        int t = (int) Math.round(mm * TICKS_PER_MM * STRAFE_CORRECTION);
        runToTargets(t, -t, -t, t, STRAFE_POWER, "STRAFE");
    }

    /**
     * Sets a per-wheel tick target and waits for the wheels to get there.
     * Targets are relative to wherever the encoders are now, so the moves
     * chain without needing a reset between them.
     *
     * setTargetPosition MUST come before RUN_TO_POSITION, or the Driver
     * Station throws "target position not set".
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
            telemetry.addData("FL", "%d -> %d",
                    frontLeft.getCurrentPosition(), frontLeft.getTargetPosition());
            telemetry.addData("FR", "%d -> %d",
                    frontRight.getCurrentPosition(), frontRight.getTargetPosition());
            telemetry.addData("BL", "%d -> %d",
                    backLeft.getCurrentPosition(), backLeft.getTargetPosition());
            telemetry.addData("BR", "%d -> %d",
                    backRight.getCurrentPosition(), backRight.getTargetPosition());
            telemetry.addData("Time", "%.1f", timer.seconds());
            telemetry.update();
        }

        setPower(0);

        // Back to a normal mode so the next OpMode does not inherit
        // RUN_TO_POSITION and refuse to drive.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ---------- Arm ----------

    /**
     * Sends the arm up and hands it to the hub's position controller,
     * which keeps holding it for the rest of the OpMode. Power stays on
     * deliberately - cutting it would let the arm fall, since it is not
     * self-supporting.
     */
    private void raiseArm() {
        armMotor.setTargetPosition(ARM_TICKS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);

        ElapsedTime armTimer = new ElapsedTime();
        while (opModeIsActive() && armMotor.isBusy()
                && armTimer.seconds() < ARM_TIMEOUT_S) {
            telemetry.addLine("ARM UP");
            telemetry.addData("Arm", "%d -> %d",
                    armMotor.getCurrentPosition(), ARM_TICKS);
            telemetry.update();
        }
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