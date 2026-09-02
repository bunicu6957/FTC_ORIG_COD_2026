package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * MecanumTeleOp
 *
 * Mecanum drive (gamepad1) + intake continuous servos + arm motor.
 *
 * NOTE: Claw and lift servo code is present but fully commented out below
 * (declarations, hardware map, directions, and control logic) so only the
 * intake servos, drive motors, and arm motor are active. Search for
 * "CLAW" / "LIFT" and uncomment those blocks to bring them back.
 *
 * Controls:
 *  Gamepad 1 (driver): mecanum drive
 *    - left stick    = translate (x/y)
 *    - right stick x = rotate
 *    - right bumper  = slow mode (precision driving)
 *    - X             = toggle intake servos ON (direction A) / OFF
 *    - Y             = toggle intake servos ON (direction B) / OFF
 *    - Right trigger = arm motor moves +45° (ticks) per press
 *    - A / B / D-pad Up / D-pad Down = disabled (claw/lift code commented out)
 *
 * IMPORTANT: Update the hardware map names below (the strings in
 * hardwareMap.get(...)) to match whatever you named things in your robot
 * configuration on the Driver Station / Control Hub.
 */
@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class cod_de_test extends LinearOpMode {

    // ---- Drive motors ----
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ---- Intake servos (continuous) ----
    private CRServo intakeLeft, intakeRight;

    // ---- Claw servos (continuous) — DISABLED, code kept for later use ----
    // private CRServo clawLeft, clawRight;

    // ---- Lift servos (continuous) — DISABLED, code kept for later use ----
    // private CRServo liftLeft, liftRight;

    // ---- Arm motor (with encoder) ----
    private DcMotorEx armMotor;

    // Power applied to auxiliary servos when running
    private static final double INTAKE_POWER = 1.0;
    // private static final double CLAW_POWER = 1.0;
    // private static final double LIFT_POWER = 1.0;

    // goBILDA 312 RPM motor: 537.7 ticks per output-shaft revolution.
    // ARM_MOVE_TICKS = 537.7 * (45.0 / 360.0) ≈ 67 ticks.
    // This assumes the motor's output shaft drives the arm 1:1 with NO
    // external gearing/chain/pulley reduction. If there IS external gearing,
    // multiply 67 by that gear ratio (e.g. 3:1 external reduction -> 201).
    // Verify on the actual robot with a protractor and adjust if needed —
    // belt slip/backlash can make the real angle differ slightly from ideal.
    private static final int ARM_MOVE_TICKS = 67;

    private static final double ARM_POWER = 0.6;

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

    // ---- Arm state ----
    private int armTargetPosition = 0;

    @Override
    public void runOpMode() {
        // ---------- Hardware map ----------
        // Names in quotes MUST match your robot configuration exactly.
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
        // Standard mecanum wiring: reverse the LEFT side so positive power
        // on all four motors drives the robot forward. Flip these if your
        // robot drives backward/sideways when it shouldn't.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake when power is zero, instead of coasting
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- Servo directions ----------
        // Each pair spins "opposite" directions by reversing one side, so
        // setting the same power value on both drives them in mirrored
        // physical directions (e.g. both intake wheels grabbing inward).
        // Flip these if a pair spins the wrong way relative to each other.
        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.REVERSE);

        // clawLeft.setDirection(CRServo.Direction.FORWARD);
        // clawRight.setDirection(CRServo.Direction.REVERSE);

        // liftLeft.setDirection(CRServo.Direction.FORWARD);
        // liftRight.setDirection(CRServo.Direction.REVERSE);

        // ---------- Arm motor setup ----------
        armMotor.setDirection(DcMotor.Direction.FORWARD); // flip if it moves the wrong way
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armTargetPosition = armMotor.getCurrentPosition();

        telemetry.addLine("Ready — waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =========================================================
            // DRIVE (gamepad1) — mecanum
            // =========================================================
            double y  = -gamepad1.left_stick_y;  // forward/back (inverted: stick up = negative)
            double x  = -gamepad1.left_stick_x;  // strafe
            double rx =  gamepad1.right_stick_x; // rotate

            // Slow mode for precision driving
            double speedMultiplier = gamepad1.right_bumper ? 0.4 : 1.0;

            double frontLeftPower  = (y + x + rx) * speedMultiplier;
            double backLeftPower   = (y - x + rx) * speedMultiplier;
            double frontRightPower = (y - x - rx) * speedMultiplier;
            double backRightPower  = (y + x - rx) * speedMultiplier;

            // Normalize so no motor is asked for more than 1.0 power
            double maxPower = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(
                    Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

            frontLeft.setPower(frontLeftPower / maxPower);
            backLeft.setPower(backLeftPower / maxPower);
            frontRight.setPower(frontRightPower / maxPower);
            backRight.setPower(backRightPower / maxPower);

            // =========================================================
            // INTAKE (gamepad1 X / Y) — toggle
            // =========================================================
            boolean xNow = gamepad1.x;
            boolean yNow = gamepad1.y;

            if (xNow && !xPrev) {
                // X just pressed
                intakeState = (intakeState == 1) ? 0 : 1;
            }
            if (yNow && !yPrev) {
                // Y just pressed
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
            // CLAW (gamepad1 A / B) — toggle — DISABLED
            // =========================================================
            // boolean aNow = gamepad1.a;
            // boolean bNow = gamepad1.b;
            //
            // if (aNow && !aPrev) {
            //     // A just pressed
            //     clawState = (clawState == 1) ? 0 : 1;
            // }
            // if (bNow && !bPrev) {
            //     // B just pressed
            //     clawState = (clawState == 2) ? 0 : 2;
            // }
            // aPrev = aNow;
            // bPrev = bNow;
            //
            // double clawPower;
            // switch (clawState) {
            //     case 1:
            //         clawPower = CLAW_POWER;
            //         break;
            //     case 2:
            //         clawPower = -CLAW_POWER;
            //         break;
            //     default:
            //         clawPower = 0.0;
            // }
            // clawLeft.setPower(clawPower);
            // clawRight.setPower(clawPower);

            // =========================================================
            // LIFT (gamepad1 D-pad Up / Down) — hold to run — DISABLED
            // =========================================================
            // double liftPower;
            // if (gamepad1.dpad_up) {
            //     liftPower = LIFT_POWER;
            // } else if (gamepad1.dpad_down) {
            //     liftPower = -LIFT_POWER;
            // } else {
            //     liftPower = 0.0;
            // }
            // liftLeft.setPower(liftPower);
            // liftRight.setPower(liftPower);

            // =========================================================
            // ARM MOTOR (gamepad1 right trigger) — move by fixed ticks per press
            // =========================================================
            boolean rightTriggerNow = gamepad1.right_trigger > 0.5;

            if (rightTriggerNow && !rightTriggerPrev) {
                armTargetPosition += ARM_MOVE_TICKS;
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(ARM_POWER);
            }
            rightTriggerPrev = rightTriggerNow;

            // Once it reaches target, stop actively holding power.
            // NOTE: if this arm fights gravity, removing this (and leaving
            // power applied) will be needed to hold position — otherwise it
            // may drift/fall back down after reaching the target.
            if (!armMotor.isBusy() && armMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                armMotor.setPower(0);
            }

            // =========================================================
            // TELEMETRY
            // =========================================================
            telemetry.addData("Drive", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.addData("Intake state", intakeState);
            // telemetry.addData("Claw state", clawState);
            // telemetry.addData("Lift power", liftPower);
            telemetry.addData("Arm target", armTargetPosition);
            telemetry.addData("Arm current", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}