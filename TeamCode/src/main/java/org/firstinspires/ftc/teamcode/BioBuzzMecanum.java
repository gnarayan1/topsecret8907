package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Robot-relative mecanum driving; requires four independently powered motors. */
@TeleOp(name = "Bio Buzz Mecanum", group = "Bio Buzz")
public class BioBuzzMecanum extends LinearOpMode {
    private static final double DRIVE_POWER = 0.50;
    private static final double SLOW_POWER = 0.25;
    private static final double CHECK_POWER = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor fr = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor bl = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor br = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor[] motors = {fl, fr, bl, br};

        // Matches the existing LarryBoi FConstants. Verify after mechanical changes.
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Ready: left stick moves, right stick turns.");
        telemetry.addLine("Left bumper: slow. Right bumper: wheel check mode.");
        telemetry.addLine("Wheel check: hold X=FL, Y=FR, A=BL, B=BR.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                double[] powers;
                if (gamepad1.right_bumper) {
                    // Overrides sticks. One wheel at a time, only while button held.
                    powers = new double[4];
                    if (gamepad1.x) powers[0] = CHECK_POWER;
                    else if (gamepad1.y) powers[1] = CHECK_POWER;
                    else if (gamepad1.a) powers[2] = CHECK_POWER;
                    else if (gamepad1.b) powers[3] = CHECK_POWER;
                } else {
                    double forward = deadband(-gamepad1.left_stick_y);
                    double right = deadband(gamepad1.left_stick_x);
                    double clockwise = deadband(gamepad1.right_stick_x);
                    powers = new double[] {
                        forward + right + clockwise,
                        forward - right - clockwise,
                        forward - right + clockwise,
                        forward + right - clockwise
                    };
                    double divisor = 1.0;
                    for (double power : powers) divisor = Math.max(divisor, Math.abs(power));
                    double speed = gamepad1.left_bumper ? SLOW_POWER : DRIVE_POWER;
                    for (int i = 0; i < powers.length; i++) powers[i] *= speed / divisor;
                }
                for (int i = 0; i < motors.length; i++) motors[i].setPower(powers[i]);
                telemetry.addData("Mode", gamepad1.right_bumper ? "WHEEL CHECK" : "DRIVE");
                telemetry.addData("FL / FR", "%.2f / %.2f", powers[0], powers[1]);
                telemetry.addData("BL / BR", "%.2f / %.2f", powers[2], powers[3]);
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotor motor : motors) motor.setPower(0);
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < 0.05 ? 0.0 : value;
    }
}
