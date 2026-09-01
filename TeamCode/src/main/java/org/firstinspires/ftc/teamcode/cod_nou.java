package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * MecanumTeleOp
 *
 * Currently: mecanum drive only (wheels).
 * Imports for DcMotorEx and Servo are kept in place for when slider/arm/claw
 * code gets added back in.
 *
 * Controls:
 *  Gamepad 1 (driver): mecanum drive
 *    - left stick    = translate (x/y)
 *    - right stick x = rotate
 *    - right bumper  = slow mode (precision driving)
 *
 * IMPORTANT: Update the hardware map names below (the strings in getX(...))
 * to match whatever you named things in your robot configuration on the
 * Driver Station / Control Hub.
 */
@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class cod_nou extends LinearOpMode {

    // ---- Drive motors ----
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {

        // ---------- Hardware map ----------
        // Names in quotes MUST match your robot configuration exactly.
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft   = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

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
            // TELEMETRY
            // =========================================================
            telemetry.addData("Drive", "y=%.2f x=%.2f rx=%.2f", y, x, rx);
            telemetry.update();
        }
    }
}