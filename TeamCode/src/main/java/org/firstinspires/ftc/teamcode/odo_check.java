package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * odo_check
 *
 * Not a driving OpMode. It only reads the three odometry pods and prints
 * them, so you can measure what auto_orig needs.
 *
 * Port map on the Control Hub (encoder socket is independent of the motor
 * plugged into that port):
 *   port 0  motor FL  encoder = RIGHT parallel pod
 *   port 1  motor FR  encoder = PERPENDICULAR pod
 *   port 2  motor BR  encoder = LEFT parallel pod
 *   port 3  motor BL  encoder = unused
 *
 * No motor is ever powered here. Push the robot by hand.
 *
 * TEST 1 - directions
 *   Push the robot STRAIGHT FORWARD about half a metre.
 *     left and right should both change, perp should stay near zero.
 *     If one parallel pod counts down while the other counts up, note it -
 *     that is a sign constant, not a wiring fault.
 *   Push the robot SIDEWAYS to the right.
 *     perp should change, left and right should stay near zero.
 *   Rotate the robot in place, clockwise seen from above.
 *     left and right should move in OPPOSITE directions.
 *
 * TEST 2 - ticks per mm
 *   Mark a start line on the floor. Press A to zero the counters.
 *   Push the robot straight forward exactly 1000 mm (use a tape measure,
 *   go slow, do not let it rotate).
 *   Read "TICKS PER MM" off the screen. That is ODO_TICKS_PER_MM.
 *   Do it three times and average - this number sets how accurate every
 *   autonomous move will be.
 */
@TeleOp(name = "odo_check", group = "Test")
public class odo_check extends LinearOpMode {

    // Distance used for the ticks-per-mm calculation, in mm.
    private static final double CAL_DISTANCE_MM = 1000.0;

    private DcMotor odoRight, odoPerp, odoLeft;

    @Override
    public void runOpMode() {
        // Pods are read through the motor name that shares their port.
        // These names describe the POD, not the motor.
        odoRight = hardwareMap.get(DcMotor.class, "FL");  // port 0 encoder
        odoPerp  = hardwareMap.get(DcMotor.class, "FR");  // port 1 encoder
        odoLeft  = hardwareMap.get(DcMotor.class, "BR");  // port 2 encoder

        int baseLeft = 0, baseRight = 0, basePerp = 0;
        boolean aPrev = false;

        telemetry.addLine("Push the robot by hand. A = zero the counters.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean aNow = gamepad1.a;
            if (aNow && !aPrev) {
                baseLeft  = odoLeft.getCurrentPosition();
                baseRight = odoRight.getCurrentPosition();
                basePerp  = odoPerp.getCurrentPosition();
            }
            aPrev = aNow;

            int left  = odoLeft.getCurrentPosition()  - baseLeft;
            int right = odoRight.getCurrentPosition() - baseRight;
            int perp  = odoPerp.getCurrentPosition()  - basePerp;

            // Average of the two parallel pods, ignoring sign differences
            // for now - you are reading these to work the signs out.
            double avgParallel = (left + right) / 2.0;

            telemetry.addLine("A = zero counters");
            telemetry.addLine();
            telemetry.addData("left  (BR port)", left);
            telemetry.addData("right (FL port)", right);
            telemetry.addData("perp  (FR port)", perp);
            telemetry.addLine();
            telemetry.addData("avg parallel", "%.1f", avgParallel);
            telemetry.addData("left - right", left - right);
            telemetry.addLine();
            telemetry.addData("TICKS PER MM", "%.4f", avgParallel / CAL_DISTANCE_MM);
            telemetry.addLine("(valid only after pushing exactly "
                    + (int) CAL_DISTANCE_MM + " mm forward)");
            telemetry.update();
        }
    }
}