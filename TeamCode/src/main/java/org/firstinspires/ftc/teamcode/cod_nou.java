package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * MecanumTeleOp
 *
 * Mecanum drive, intake servos, and a two-position arm.
 *
 * CONTROLS (gamepad 1)
 *   left stick       translate (forward/back + strafe)
 *   right stick x    rotate
 *   right bumper     slow mode for precision driving
 *   X                toggle intake on (direction A) / off
 *   Y                toggle intake on (direction B) / off
 *   right trigger    toggle arm between RAISED and DOWN
 *   left trigger     force arm DOWN from any state
 *   left bumper      re-zero the arm encoder
 *   Back             cut arm power - panic only, the arm WILL fall
 *   D-pad up/down    raw direction test, works only while arm power is cut
 *   A / B, D-pad     free (claw and lift code is commented out below)
 *
 * BEFORE INIT: the arm must be resting at its bottom position. The encoder
 * is zeroed at init and every arm position is measured from there.
 *
 * BEFORE STOP: lower the arm. Power is cut when the OpMode ends and the arm
 * is not self-supporting.
 *
 * HOW THE ARM WORKS
 * RUN_TO_POSITION is deliberately not used. The arm runs a proportional
 * loop in RUN_USING_ENCODER, plus a constant feedforward that cancels
 * gravity, because this arm falls when power is removed.
 *
 * Two separate sign constants handle direction, and they must stay
 * separate:
 *   ARM_ENCODER_SIGN  applied to the encoder reading only
 *   ARM_POWER_SIGN    applied to the output power only
 * Applying a single constant to both cancels itself out and does nothing.
 * setDirection() cannot fix a direction mismatch either, since it flips
 * the power and the encoder together. If the arm ever moves the wrong way,
 * cut power with Back and use the D-pad test to measure each sign
 * independently.
 *
 * Two guards watch the arm: one for leaving its travel range, one for an
 * error that keeps growing. Neither cuts power, because zero power is the
 * dangerous state for a gravity-loaded arm. Both freeze the target so the
 * arm holds where it is. Either trigger clears the fault.
 */
@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class cod_nou extends LinearOpMode {

    // =============================================================
    // ARM CONSTANTS - verified working on the current hardware
    // (new arm motor, leads wired correctly). Do not change these
    // without re-running the D-pad direction test.
    // =============================================================

    /** Applied to the ENCODER READING only. +1 if the raw count rises as
     *  the arm rises, -1 if it falls. */
    private static final int ARM_ENCODER_SIGN = 1;

    /** Applied to the OUTPUT POWER only. +1 if positive power raises the
     *  arm, -1 if it lowers it. Independent of ARM_ENCODER_SIGN. */
    private static final int ARM_POWER_SIGN = 1;

    /** Constant "up" power that cancels gravity. Positive means up. */
    private static final double ARM_FEEDFORWARD = 0.15;

    /** Max total power the arm loop may command. Keep it well above
     *  ARM_FEEDFORWARD so the position term has room to work. */
    private static final double ARM_MAX_POWER = 0.80;

    /** Proportional gain. */
    private static final double ARM_KP = 0.005;

    /** Top of the arm's usable travel, in ticks from the bottom. */
    private static final int ARM_MAX_TICKS = 1200;

    // =============================================================
    // ARM PRESETS
    // =============================================================

    /** Bottom position. Leave at 0 - this is where the encoder is zeroed. */
    private static final int ARM_POS_DOWN = 0;

    /** The slightly raised position. 100 ticks - this is the height that
     *  tested well: one trigger step from the bottom, held steady on
     *  ARM_FEEDFORWARD = 0.15. Raise it only in small increments, and
     *  re-check that the arm still holds without sagging. */
    private static final int ARM_POS_RAISED = 100;

    // =============================================================
    // ARM SAFETY / INTERNALS - normally no need to touch
    // =============================================================

    private static final int ARM_MIN_TICKS = 0;
    private static final int ARM_TOLERANCE = 12;
    private static final int ARM_RUNAWAY_MARGIN = 120;
    private static final int ARM_DIVERGE_TICKS = 800;
    private static final double ARM_TEST_POWER = 0.15;

    // ---- Drive motors ----
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---- Intake servos (continuous) ----
    private CRServo intakeLeft, intakeRight;

    // ---- Claw servos (continuous) - DISABLED, code kept for later use ----
    // private CRServo clawLeft, clawRight;

    // ---- Lift servos (continuous) - DISABLED, code kept for later use ----
    // private CRServo liftLeft, liftRight;

    // ---- Arm motor (with encoder) ----
    private DcMotorEx armMotor;

    private static final double INTAKE_POWER = 1.0;
    // private static final double CLAW_POWER = 1.0;
    // private static final double LIFT_POWER = 1.0;

    // ---- Arm state ----
    private int armTarget = ARM_POS_DOWN;
    private boolean armEnabled = true;
    private boolean armFaulted = false;   // soft fault: hold in place, never drop
    private String armFault = "";
    private int bestAbsError = Integer.MAX_VALUE;

    // ---- Toggle states ----
    // 0 = stopped, 1 = running "direction A", 2 = running "direction B"
    private int intakeState = 0;
    // private int clawState = 0;

    // ---- Previous button states (for edge detection / toggling) ----
    private boolean xPrev = false;
    private boolean yPrev = false;
    // private boolean aPrev = false;
    // private boolean bPrev = false;
    private boolean rightTriggerPrev = false;
    private boolean leftTriggerPrev = false;
    private boolean leftBumperPrev = false;

    @Override
    public void runOpMode() {
        // ---------- Hardware map ----------
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        intakeLeft  = hardwareMap.get(CRServo.class, "IL");
        intakeRight = hardwareMap.get(CRServo.class, "IR");

        // clawLeft  = hardwareMap.get(CRServo.class, "clawLeft");
        // clawRight = hardwareMap.get(CRServo.class, "clawRight");

        // liftLeft  = hardwareMap.get(CRServo.class, "liftLeft");
        // liftRight = hardwareMap.get(CRServo.class, "liftRight");

        armMotor = hardwareMap.get(DcMotorEx.class, "AM");

        // ---------- Motor directions ----------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- Servo directions ----------
        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.REVERSE);

        // clawLeft.setDirection(CRServo.Direction.FORWARD);
        // clawRight.setDirection(CRServo.Direction.REVERSE);

        // liftLeft.setDirection(CRServo.Direction.FORWARD);
        // liftRight.setDirection(CRServo.Direction.REVERSE);

        // ---------- Arm motor setup ----------
        // Leave direction FORWARD. All sign handling lives in the two
        // constants above - setDirection() flips power and encoder together
        // and cannot fix a direction mismatch.
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);
        armTarget = ARM_POS_DOWN;

        telemetry.addLine("Ready - waiting for start");
        telemetry.addLine("Arm must be resting at the BOTTOM right now.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =========================================================
            // DRIVE (gamepad1) - mecanum
            // =========================================================
            double y  = -gamepad1.left_stick_y;  // forward/back
            double x  = -gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotate

            double speedMultiplier = gamepad1.right_bumper ? 0.4 : 1.0;

            double frontLeftPower  = (y + x + rx) * speedMultiplier;
            double backLeftPower   = (y - x + rx) * speedMultiplier;
            double frontRightPower = (y - x - rx) * speedMultiplier;
            double backRightPower  = (y + x - rx) * speedMultiplier;

            double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(
                    Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / maxPower);
            backLeft.setPower(backLeftPower / maxPower);
            frontRight.setPower(frontRightPower / maxPower);
            backRight.setPower(backRightPower / maxPower);

            // =========================================================
            // INTAKE (gamepad1 X / Y) - toggle
            // =========================================================
            boolean xNow = gamepad1.x;
            boolean yNow = gamepad1.y;

            if (xNow && !xPrev) {
                intakeState = (intakeState == 1) ? 0 : 1;
            }
            if (yNow && !yPrev) {
                intakeState = (intakeState == 2) ? 0 : 2;
            }
            xPrev = xNow;
            yPrev = yNow;

            double intakePower;
            switch (intakeState) {
                case 1:
                    intakePower = INTAKE_POWER;
                    break;
                case 2:
                    intakePower = -INTAKE_POWER;
                    break;
                default:
                    intakePower = 0.0;
            }
            intakeLeft.setPower(intakePower);
            intakeRight.setPower(intakePower);

            // =========================================================
            // CLAW (gamepad1 A / B) - toggle - DISABLED
            // =========================================================
            // boolean aNow = gamepad1.a;
            // boolean bNow = gamepad1.b;
            //
            // if (aNow && !aPrev) {
            //     clawState = (clawState == 1) ? 0 : 1;
            // }
            // if (bNow && !bPrev) {
            //     clawState = (clawState == 2) ? 0 : 2;
            // }
            // aPrev = aNow;
            // bPrev = bNow;
            //
            // double clawPower;
            // switch (clawState) {
            //     case 1:  clawPower =  CLAW_POWER; break;
            //     case 2:  clawPower = -CLAW_POWER; break;
            //     default: clawPower =  0.0;
            // }
            // clawLeft.setPower(clawPower);
            // clawRight.setPower(clawPower);

            // =========================================================
            // LIFT (D-pad) - DISABLED. The D-pad is used by the arm
            // direction test below. Re-assign before re-enabling this.
            // =========================================================

            // =========================================================
            // ARM - two preset positions
            // =========================================================
            int armRaw = armMotor.getCurrentPosition();
            int armPos = armRaw * ARM_ENCODER_SIGN;   // positive = up

            // ---- Panic button: cut power (the arm will fall) ----
            if (gamepad1.back) {
                armEnabled = false;
                armFault = "Power cut by driver (Back).";
            }

            // ---- Left bumper: re-zero the encoder ----
            // Only with the arm physically resting at the bottom.
            boolean leftBumperNow = gamepad1.left_bumper;
            if (leftBumperNow && !leftBumperPrev) {
                armMotor.setPower(0);
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armRaw = 0;
                armPos = 0;
                armTarget = ARM_POS_DOWN;
                armEnabled = true;
                armFaulted = false;
                armFault = "";
                bestAbsError = Integer.MAX_VALUE;
            }
            leftBumperPrev = leftBumperNow;

            // ---- Triggers: pick a preset ----
            boolean rightTriggerNow = gamepad1.right_trigger > 0.5;
            boolean leftTriggerNow  = gamepad1.left_trigger  > 0.5;

            int oldTarget = armTarget;
            if (rightTriggerNow && !rightTriggerPrev) {
                // Toggle: raised -> down, anything else -> raised.
                // "Anything else" covers a frozen target after a soft fault,
                // so a fault always recovers upward rather than dropping.
                armTarget = (armTarget == ARM_POS_RAISED) ? ARM_POS_DOWN : ARM_POS_RAISED;
                armFaulted = false;
                armFault = "";
            }
            if (leftTriggerNow && !leftTriggerPrev) {
                // Always down, regardless of current state.
                armTarget = ARM_POS_DOWN;
                armFaulted = false;
                armFault = "";
            }
            rightTriggerPrev = rightTriggerNow;
            leftTriggerPrev  = leftTriggerNow;

            armTarget = Range.clip(armTarget, ARM_MIN_TICKS, ARM_MAX_TICKS);

            // A new target means a legitimately larger error - reset the
            // divergence baseline so it does not false-trip.
            if (armTarget != oldTarget) {
                bestAbsError = Integer.MAX_VALUE;
            }

            int armError = armTarget - armPos;   // positive = needs to go up
            int absError = Math.abs(armError);
            double armCommand;                   // positive = up
            double armMotorPower;                // what reaches the motor

            if (!armEnabled) {
                // ---- RAW DIRECTION TEST ----
                // Power here is NOT sign-converted, so you can observe the
                // true hardware behaviour.
                if (gamepad1.dpad_up) {
                    armMotorPower = ARM_TEST_POWER;
                } else if (gamepad1.dpad_down) {
                    armMotorPower = -ARM_TEST_POWER;
                } else {
                    armMotorPower = 0.0;
                }
                armCommand = 0.0;
                bestAbsError = Integer.MAX_VALUE;
            } else {
                // ---- Travel limit: soft fault, park in place ----
                if (armPos > ARM_MAX_TICKS + ARM_RUNAWAY_MARGIN
                        || armPos < ARM_MIN_TICKS - ARM_RUNAWAY_MARGIN) {
                    armFaulted = true;
                    armFault = "Travel limit exceeded - holding position.";
                }

                // ---- Divergence: soft fault, park in place ----
                bestAbsError = Math.min(bestAbsError, absError);
                if (absError > bestAbsError + ARM_DIVERGE_TICKS) {
                    armFaulted = true;
                    armFault = "Not tracking - holding position.";
                }

                // A soft fault freezes the target here. Power stays on so
                // the arm holds instead of falling. Either trigger clears it.
                if (armFaulted) {
                    armTarget = Range.clip(armPos, ARM_MIN_TICKS, ARM_MAX_TICKS);
                    armError = armTarget - armPos;
                    absError = Math.abs(armError);
                    bestAbsError = absError;
                }

                // ---- Proportional term ----
                double p = (absError < ARM_TOLERANCE) ? 0.0 : armError * ARM_KP;

                // ---- Gravity feedforward ----
                // Applied whenever the arm is off its bottom stop, including
                // while lowering, so it descends instead of dropping.
                double ff = (armPos > ARM_TOLERANCE) ? ARM_FEEDFORWARD : 0.0;

                armCommand = Range.clip(p + ff, -ARM_MAX_POWER, ARM_MAX_POWER);

                // Convert "up" into the motor's actual polarity.
                armMotorPower = armCommand * ARM_POWER_SIGN;
            }

            armMotor.setPower(armMotorPower);

            // =========================================================
            // TELEMETRY
            // =========================================================
            telemetry.addData("Drive", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.addData("Slow mode", gamepad1.right_bumper ? "ON" : "off");
            telemetry.addData("Intake state", intakeState);
            telemetry.addLine();
            telemetry.addData("Arm preset",
                    (armTarget == ARM_POS_RAISED) ? "RAISED"
                            : (armTarget == ARM_POS_DOWN) ? "DOWN" : "held");
            telemetry.addData("Arm target", armTarget);
            telemetry.addData("Arm pos", armPos);
            telemetry.addData("Arm error", armError);
            telemetry.addData("Arm motor power", "%.2f", armMotorPower);
            telemetry.addData("Arm current (A)", "%.2f",
                    armMotor.getCurrent(CurrentUnit.AMPS));
            if (!armFault.isEmpty()) {
                telemetry.addLine("ARM: " + armFault);
            }
            if (!armEnabled) {
                telemetry.addLine("ARM POWER OFF. Left bumper at bottom = re-enable.");
            }
            telemetry.update();
        }
    }
}