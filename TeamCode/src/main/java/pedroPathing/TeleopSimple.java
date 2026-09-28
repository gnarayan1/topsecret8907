package pedroPathing;

import static pedroPathing.Config.ConfigFile.CLOSE_ERROR_THRESHOLD;
import static pedroPathing.Config.ConfigFile.CONFIGkD;
import static pedroPathing.Config.ConfigFile.CONFIGkDClose;
import static pedroPathing.Config.ConfigFile.CONFIGkI;
import static pedroPathing.Config.ConfigFile.CONFIGkIClose;
import static pedroPathing.Config.ConfigFile.CONFIGkP;
import static pedroPathing.Config.ConfigFile.CONFIGkPClose;
import static pedroPathing.Config.ConfigFile.DEADZONE;
import static pedroPathing.Config.ConfigFile.REV_CLOSE_THRESHOLD;
import static pedroPathing.Config.ConfigFile.REVkD;
import static pedroPathing.Config.ConfigFile.REVkDClose;
import static pedroPathing.Config.ConfigFile.REVkP;
import static pedroPathing.Config.ConfigFile.REVkPClose;
import static pedroPathing.Config.ConfigFile.TICK_STEP;
import static pedroPathing.Config.ConfigFile.loopTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TeleopSimple", group = "adon")
public class TeleopSimple extends OpMode {

    private Follower follower;
    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx revolver;

    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime revolverTimer = new ElapsedTime();

    private double kP, kI, kD;
    private double error;
    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0;
    private boolean launcherOn = false;
    private double targetVelocity = 0;
    private double PidInputSpeed = 0.68;
    private double targetTicks = 0;

    private double revolverLastError = 0;
    private boolean prevRightBumper = false;
    private boolean prevLeftTrigger = false;
    private boolean prevRightTrigger = false;
    private boolean prevLeftStickButton = false;
    private boolean prevRightStickButton = false;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));
        pidTimer.reset();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorSimple.Direction.FORWARD);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPower(0);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        launcherOn = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double runPIDFHybrid(double target, double current, double currentPower) {
        error = target - current;

        if (Math.abs(error) > DEADZONE) {
            integralSum += error * loopTime;
        }

        double derivative = (error - lastError) / loopTime;
        double pidCorrection = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;

        return currentPower + pidCorrection;
    }

    private void revolverSpin(double targetTicks) {
        double dt = revolverTimer.seconds();
        revolverTimer.reset();
        if (dt <= 0) return;

        double currentTicks = revolver.getCurrentPosition();
        double error = targetTicks - currentTicks;

        double pGain;
        double dGain;

        if (Math.abs(error) > REV_CLOSE_THRESHOLD) {
            pGain = REVkP;
            dGain = REVkD;
        } else {
            pGain = REVkPClose;
            dGain = REVkDClose;
        }

        double derivative = (error - revolverLastError) / dt;
        revolverLastError = error;

        double power = (pGain * error) + (dGain * derivative);
        power = clamp(power, -1.0, 1.0);

        revolver.setPower(power);
    }

    @Override
    public void loop() {
        double currentVelocity = launcher.getVelocity();
        error = targetVelocity - currentVelocity;

        if (Math.abs(error) > CLOSE_ERROR_THRESHOLD) {
            kP = CONFIGkP;
            kI = CONFIGkI;
            kD = CONFIGkD;
        } else {
            kP = CONFIGkPClose;
            kI = CONFIGkIClose;
            kD = CONFIGkDClose;
        }

        if (gamepad1.right_bumper && !prevRightBumper) {
            launcherOn = !launcherOn;
        }

        if (gamepad1.left_stick_button && !prevLeftStickButton) {
            PidInputSpeed -= 0.01;
        }

        if (gamepad1.right_stick_button && !prevRightStickButton) {
            PidInputSpeed += 0.01;
        }

        PidInputSpeed = clamp(PidInputSpeed, 0.0, 1.0);

        if (gamepad1.left_trigger > 0.1 && !prevLeftTrigger) {
            targetTicks -= TICK_STEP;
        }

        if (gamepad1.right_trigger > 0.1 && !prevRightTrigger) {
            targetTicks += TICK_STEP;
        }

        if (launcherOn) {
            targetVelocity = (Math.round((PidInputSpeed * 1960) / 20.0) * 20) - 10;
            if (pidTimer.seconds() >= loopTime) {
                pidOutput = runPIDFHybrid(targetVelocity, currentVelocity, launcher.getPower());
                launcher.setPower(pidOutput);
                launcher2.setPower(pidOutput);
                pidTimer.reset();
            }
        } else {
            launcher.setPower(0);
            launcher2.setPower(0);
            targetVelocity = 0;
            pidOutput = 0;
            integralSum = 0;
            lastError = 0;
        }

        revolverSpin((int) targetTicks);

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
        }

        telemetry.addData("Launcher On", launcherOn);
        telemetry.addData("Launcher Power", launcher.getPower());
        telemetry.addData("Launcher Target", targetVelocity);
        telemetry.addData("Launcher Speed", currentVelocity);
        telemetry.addData("Launcher Input %", PidInputSpeed);
        telemetry.addData("Revolver Target Ticks", targetTicks);
        telemetry.addData("Revolver Error", error);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        prevRightBumper = gamepad1.right_bumper;
        prevLeftTrigger = gamepad1.left_trigger > 0.1;
        prevRightTrigger = gamepad1.right_trigger > 0.1;
        prevLeftStickButton = gamepad1.left_stick_button;
        prevRightStickButton = gamepad1.right_stick_button;

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
