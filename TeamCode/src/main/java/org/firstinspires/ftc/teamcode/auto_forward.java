package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * auto_forward
 *
 * Raises the arm to its carry position, then drives forward three
 * quarters of a metre and stops.
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
@Autonomous(name = "auto_forward", group = "Auto")
public class auto_forward extends LinearOpMode {

    /** goBILDA 5203 Yellow Jacket, 435 RPM, 13.7:1 gearbox. */
    private static final double TICKS_PER_REV = 383.6;

    /** Mecanum wheel diameter in mm. */
    private static final double WHEEL_DIAMETER_MM = 96.0;

    /** Derived: ticks per millimetre of wheel travel. */
    private static final double TICKS_PER_MM =
            TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    /** How far to drive. Positive forward, negative backward. */
    private static final double DISTANCE_MM = 1000;

    /** Drive power. */
    private static final double POWER = 0.7;

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

        int ticks = (int) Math.round(DISTANCE_MM * TICKS_PER_MM);

        telemetry.addLine("auto_forward ready");
        telemetry.addData("Distance", "%.0f mm", DISTANCE_MM);
        telemetry.addData("Target ticks", ticks);
        telemetry.addData("Arm target", ARM_TICKS);
        telemetry.addLine("Arm must be resting at the BOTTOM right now.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        raiseArm();
        if (!opModeIsActive()) return;

        // setTargetPosition MUST come before RUN_TO_POSITION, or the
        // Driver Station throws "target position not set".
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(POWER);

        ElapsedTime timer = new ElapsedTime();

        // Waiting on ALL four means one slow wheel cannot end the move early.
        while (opModeIsActive()
                && timer.seconds() < TIMEOUT_S
                && (frontLeft.isBusy() || frontRight.isBusy()
                || backLeft.isBusy() || backRight.isBusy())) {

            telemetry.addData("Target", ticks);
            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.addData("Time", "%.1f", timer.seconds());
            telemetry.update();
        }

        setPower(0);

        // Back to a normal mode so the next OpMode does not inherit
        // RUN_TO_POSITION and refuse to drive.
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Done.");
        telemetry.addData("Arm", armMotor.getCurrentPosition());
        telemetry.update();
        sleep(2000);
    }

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