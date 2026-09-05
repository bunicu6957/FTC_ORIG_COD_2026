package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * auto_match
 *
 * Sequence:
 *    1. raise the arm to 100 ticks and hold it there
 *    2. forward 100 ticks
 *    3. turn left (43)
 *    4. lower the arm to 1 tick, start the intake running INWARD
 *    5. forward 1000 ticks
 *    6. stop the intake
 *    7. turn left (90)
 *    8. forward 3500 ticks
 *   9a. raise the arm to 125 ticks
 *   9b. run the intake OUTWARD for 2 seconds
 *   10. strafe left 150 ticks
 *   11. backward 2000 ticks
 *
 * DRIVING DISTANCES ARE RAW WHEEL TICKS, not millimetres. At roughly
 * 1.27 ticks per mm on 96mm wheels, 1000 ticks is about 790mm and 3500 is
 * about 2.75m.
 *
 * THE TURNS ARE NOT TICKS
 * turnLeft() takes the same degrees-style parameter as auto_ticks, and
 * uses the identical geometry constants, so turnLeft(43) does exactly
 * what TURN_1_DEG = 43 does in that file - which on this robot is a real
 * 90 degree turn.
 *
 * That means step 7's turnLeft(90) is roughly a 180 degree turn, since it
 * goes through the same fudged scale. If you wanted a real quarter turn
 * there, change TURN_2 to 43.
 *
 * THE ARM
 * RUN_TO_POSITION, held by the hub's own controller. A LinearOpMode
 * blocks inside each drive move, so a TeleOp-style hold loop would never
 * get to run and the arm would sag. Power stays on throughout - cutting
 * it lets the arm fall.
 *
 * The encoder zeroes at init, so the arm MUST be resting at its bottom
 * position when you press init.
 */
@Autonomous(name = "auto_match", group = "Auto")
public class auto_orig extends LinearOpMode {

    // =============================================================
    // GEOMETRY - same values as auto_ticks, do not change in isolation
    // =============================================================

    /** goBILDA 5203 Yellow Jacket, 435 RPM, 13.7:1 gearbox. */
    private static final double TICKS_PER_REV = 383.6;

    /** Mecanum wheel diameter in mm. */
    private static final double WHEEL_DIAMETER_MM = 96.0;

    private static final double TICKS_PER_MM =
            TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    private static final double TRACK_WIDTH_MM = 350.0;
    private static final double WHEEL_BASE_MM = 300.0;

    private static final double TURN_RADIUS_MM =
            (TRACK_WIDTH_MM + WHEEL_BASE_MM) / 2.0;

    private static final double TURN_CORRECTION = 1.0;

    // =============================================================
    // THE SEQUENCE
    // =============================================================

    private static final int ARM_UP_TICKS = 100;    // step 1 ridicam bratul
    private static final int LEG_1_TICKS = 100;     // step 2 iesim din parcare
    private static final double TURN_1 = 43;        // step 3 intorc la stanga
    private static final int ARM_DOWN_TICKS = 1;    // step 4 las jos bratul
    private static final int LEG_2_TICKS = 1000;    // step 5 merg in fata
    private static final double TURN_2 = 90;        // step 7 intorc la 180
    private static final int LEG_3_TICKS = 3500;    // step 8 merg la hp
    private static final int ARM_OUT_TICKS = 125;   // before the outtake ridic bratul la 125
    private static final long OUTTAKE_MS = 2000;    // step 9 cac afara 2s
    private static final int STRAFE_TICKS = -150;   // step 10, negative = left strafe la stanga
    private static final int LEG_4_TICKS = -2000;   // step 11, negative = back merg incet cu spatele

    // =============================================================
    // POWERS AND TIMING
    // =============================================================

    private static final double POWER = 0.7;
    private static final double STRAFE_POWER = 0.5;
    private static final double TURN_POWER = 0.3;

    private static final double ARM_POWER = 0.5;
    private static final double ARM_TIMEOUT_S = 3.0;

    /** Positive runs the intake INWARD, negative runs it OUTWARD. */
    private static final double INTAKE_POWER = 1.0;

    private static final long SETTLE_MS = 150;
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

        // Same directions as TeleOp: positive power drives all four forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // One forward, one reversed - the original mirrored-pair setup.
        intakeLeft.setDirection(CRServo.Direction.FORWARD);
        intakeRight.setDirection(CRServo.Direction.REVERSE);
        setIntake(0);

        // Arm zeroes here, so it must be resting at the bottom at init.
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("auto_match ready");
        telemetry.addLine("Arm must be resting at the BOTTOM right now.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // 1. arm up, and it stays up
        armTo(ARM_UP_TICKS, "1 - ARM UP");
        if (!opModeIsActive()) return;

        // 2. forward
        driveStraight(LEG_1_TICKS, "2 - FORWARD");
        sleep(SETTLE_MS);
        if (!opModeIsActive()) return;

        // 3. turn left
        turnLeft(TURN_1, "3 - TURN LEFT");
        sleep(SETTLE_MS);
        if (!opModeIsActive()) return;

        // 4. arm down, intake in
        armTo(ARM_DOWN_TICKS, "4 - ARM DOWN");
        setIntake(INTAKE_POWER);
        if (!opModeIsActive()) { setIntake(0); return; }

        // 5. forward, intake still running
        driveStraight(LEG_2_TICKS, "5 - FORWARD (intaking)");
        sleep(SETTLE_MS);

        // 6. intake off
        setIntake(0);
        if (!opModeIsActive()) return;

        // 7. turn left
        turnLeft(TURN_2, "7 - TURN LEFT");
        sleep(SETTLE_MS);
        if (!opModeIsActive()) return;

        // 8. forward
        driveStraight(LEG_3_TICKS, "8 - FORWARD");
        sleep(SETTLE_MS);
        if (!opModeIsActive()) return;

        // 9a. arm up to firing height, before anything comes out
        armTo(ARM_OUT_TICKS, "9a - ARM TO 125");
        if (!opModeIsActive()) return;

        // 9b. outtake for two seconds
        telemetry.addLine("9b - OUTTAKE");
        telemetry.update();
        setIntake(-INTAKE_POWER);
        sleep(OUTTAKE_MS);
        setIntake(0);
        if (!opModeIsActive()) return;

        // 10. strafe left
        strafe(STRAFE_TICKS, "10 - STRAFE LEFT");
        sleep(SETTLE_MS);
        if (!opModeIsActive()) return;

        // 11. back up
        driveStraight(LEG_4_TICKS, "11 - BACKWARD");

        setIntake(0);
        telemetry.addLine("Done.");
        telemetry.addData("Arm", armMotor.getCurrentPosition());
        telemetry.update();
        sleep(2000);
    }

    // ---------- Arm ----------

    /**
     * Sends the arm to a tick position and hands it to the hub's position
     * controller, which keeps holding it. Power is left on deliberately.
     */
    private void armTo(int ticks, String label) {
        armMotor.setTargetPosition(ticks);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && armMotor.isBusy()
                && timer.seconds() < ARM_TIMEOUT_S) {
            telemetry.addData("Phase", label);
            telemetry.addData("Arm", "%d -> %d",
                    armMotor.getCurrentPosition(), ticks);
            telemetry.update();
        }
    }

    // ---------- Intake ----------

    /** Positive is inward, negative is outward. */
    private void setIntake(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    // ---------- Moves ----------

    /** Positive forward, negative backward. All four wheels equal. */
    private void driveStraight(int ticks, String label) {
        runToTargets(ticks, ticks, ticks, ticks, POWER, label);
    }

    /**
     * Positive strafes right, negative left. The diagonal pairs turn
     * opposite ways: front-left and back-right one way, front-right and
     * back-left the other.
     */
    private void strafe(int ticks, String label) {
        runToTargets(ticks, -ticks, -ticks, ticks, STRAFE_POWER, label);
    }

    /**
     * Positive turns LEFT. Takes the same degrees-style parameter as
     * auto_ticks and converts it the same way, so a value of 43 gives the
     * turn that file already produces.
     */
    private void turnLeft(double degrees, String label) {
        double arcMm = Math.toRadians(degrees) * TURN_RADIUS_MM;
        int t = (int) Math.round(arcMm * TICKS_PER_MM * TURN_CORRECTION);
        runToTargets(-t, t, -t, t, TURN_POWER, label);
    }

    // ---------- Machinery ----------

    /**
     * Sets a per-wheel tick target and waits for the wheels to get there.
     * Targets are relative to the current encoder reading, so the moves
     * chain without a reset between them.
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