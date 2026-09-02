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
 * Mecanum drive (gamepad1) + intake / claw / lift continuous servos.
 *
 * Controls:
 *  Gamepad 1 (driver): mecanum drive
 *    - left stick    = translate (x/y)
 *    - right stick x = rotate
 *    - right bumper  = slow mode (precision driving)
 *    - X             = toggle intake servos ON (direction A) / OFF
 *    - Y             = toggle intake servos ON (direction B) / OFF
 *    - A             = toggle claw servos ON (direction A) / OFF
 *    - B             = toggle claw servos ON (direction B) / OFF
 *    - D-pad Up      = lift servos run one way while held
 *    - D-pad Down    = lift servos run the other way while held
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

    // ---- Claw servos (continuous) ----
    private CRServo clawLeft, clawRight;

    // ---- Lift servos (continuous) ----
    private CRServo liftLeft, liftRight;

    // Power applied to auxiliary servos when running
    private static final double INTAKE_POWER = 1.0;
    private static final double CLAW_POWER = 1.0;
    private static final double LIFT_POWER = 1.0;

    // ---- Toggle states ----
    // 0 = stopped, 1 = running "direction A", 2 = running "direction B"
    private int intakeState = 0;
    private int clawState = 0;

    // ---- Previous button states (for edge detection / toggling) ----
    private boolean xPrev = false;
    private boolean yPrev = false;
    private boolean aPrev = false;
    private boolean bPrev = false;

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

        clawLeft  = hardwareMap.get(CRServo.class, "CL");
        clawRight = hardwareMap.get(CRServo.class, "CR");

        liftLeft  = hardwareMap.get(CRServo.class, "LL");
        liftRight = hardwareMap.get(CRServo.class, "LR");

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

        clawLeft.setDirection(CRServo.Direction.FORWARD);
        clawRight.setDirection(CRServo.Direction.REVERSE);

        liftLeft.setDirection(CRServo.Direction.FORWARD);
        liftRight.setDirection(CRServo.Direction.REVERSE);

        telemetry.addLine("Ready — waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // =========================================================
            // DRIVE (gamepad1) — mecanum
            // =========================================================
            double y  = -gamepad1.left_stick_y;  // forward/back (inverted: stick up = negative)
            double x  =  gamepad1.left_stick_x;  // strafe
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
            // CLAW (gamepad1 A / B) — toggle
            // =========================================================
            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;

            if (aNow && !aPrev) {
                // A just pressed
                clawState = (clawState == 1) ? 0 : 1;
            }
            if (bNow && !bPrev) {
                // B just pressed
                clawState = (clawState == 2) ? 0 : 2;
            }
            aPrev = aNow;
            bPrev = bNow;

            double clawPower;
            switch (clawState) {
                case 1:
                    clawPower = CLAW_POWER;
                    break;
                case 2:
                    clawPower = -CLAW_POWER;
                    break;
                default:
                    clawPower = 0.0;
            }
            clawLeft.setPower(clawPower);
            clawRight.setPower(clawPower);

            // =========================================================
            // LIFT (gamepad1 D-pad Up / Down) — hold to run
            // =========================================================
            double liftPower;
            if (gamepad1.dpad_up) {
                liftPower = LIFT_POWER;
            } else if (gamepad1.dpad_down) {
                liftPower = -LIFT_POWER;
            } else {
                liftPower = 0.0;
            }
            liftLeft.setPower(liftPower);
            liftRight.setPower(liftPower);

            // =========================================================
            // TELEMETRY
            // =========================================================
            telemetry.addData("Drive", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.addData("Intake state", intakeState);
            telemetry.addData("Claw state", clawState);
            telemetry.addData("Lift power", liftPower);
            telemetry.update();
        }
    }
}