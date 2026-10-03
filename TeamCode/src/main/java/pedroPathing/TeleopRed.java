package pedroPathing;

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
import static pedroPathing.Config.ConfigFile.SERVO_POS_DOWN;
import static pedroPathing.Config.ConfigFile.SERVO_POS_MID;
import static pedroPathing.Config.ConfigFile.SERVO_POS_UP;
import static pedroPathing.Config.ConfigFile.TICK_STEP;
import static pedroPathing.Config.ConfigFile.loopTime;
import static pedroPathing.Config.ConfigFile.revolverPower;
import static pedroPathing.Config.randomClass.autoEndBlue;
import static pedroPathing.Config.randomClass.autoEndRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TeleopRed", group = "adon")
public class TeleopRed extends OpMode {

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx revolver;
    //private CRServo intakeServo;
    private Servo pushServo;
    private Follower follower;
    private FtcDashboard dashboard;
   // private Limelight3A limelight;
    private double kP, kI, kD;
    private double error;

    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0;

    private ElapsedTime pidTimer = new ElapsedTime();

    private boolean launcherOn = false;
    private double targetVelocity = 0;
    private boolean prevShare, prevLeftJoystick, prevRightJoystick = false;
    double PidInputSpeed = 0.68;
    double pushServoValue = SERVO_POS_MID;

    private double targetTicks = 0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private final double LOOP_TIME = loopTime;

    private final double CLOSE_ERROR_THRESHOLD = 150;

    private final double nudgeDistance = 0.75;
    private final Pose testingStartPose = new Pose(88.625, 9.125, Math.toRadians(90));
    private final Pose poseAarush = new Pose(20, 15, Math.toRadians(0));
    private final Pose pose2 = new Pose(85, 83, Math.toRadians(223)); //tri
    private final Pose pose3 = new Pose(72, 72, Math.toRadians(223));
    private final Pose pose4 = new Pose(84.7, 20.8, Math.toRadians(244)); //cross
    private final Pose Park = new Pose(37.7, 33.3, Math.toRadians(90));

    private int pathState = 0;

    private void setPathState(int newState) {
        pathState = newState;
    }

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEndRed);
        pidTimer.reset();

       // limelight = hardwareMap.get(Limelight3A.class, "limelight");
       // limelight.pipelineSwitch(4);
       // limelight.start();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        //intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        pushServo = hardwareMap.get(Servo.class, "pushServo");

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorSimple.Direction.FORWARD);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPower(0);
        pidTimer.reset();
    }

    private double runPIDFHybrid(double target, double current, double currentPower) {
        error = target - current;

        if (Math.abs(error) > DEADZONE) {
            integralSum += error * LOOP_TIME;
        }

        double derivative = (error - lastError) / LOOP_TIME;
        double pidCorrection = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;


        return currentPower + pidCorrection;
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        pushServo.setPosition(SERVO_POS_MID);
        launcherOn = true;
    }



    private double revolverLastError = 0;
    private ElapsedTime revolverTimer = new ElapsedTime();

    private void revolverSpin(double targetTicks) {

        double dt = revolverTimer.seconds();
        revolverTimer.reset();
        if (dt <= 0) return;

        double currentTicks = revolver.getCurrentPosition();
        double error = targetTicks - currentTicks;

        double kP, kD;

        if (Math.abs(error) > REV_CLOSE_THRESHOLD) {
            kP = REVkP;
            kD = REVkD;
        } else {
            kP = REVkPClose;
            kD = REVkDClose;
        }

        double derivative = (error - revolverLastError) / dt;
        revolverLastError = error;

        double power = (kP * error) + (kD * derivative);

        power = Math.max(-1.0, Math.min(1.0, power));

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

        double LRDeltaX = nudgeDistance * Math.cos(-((3.14159 / 2) - follower.getPose().getHeading()));
        double LRDeltaY = nudgeDistance * Math.sin(-((3.14159 / 2) - follower.getPose().getHeading()));

        double FBDeltaX = nudgeDistance * Math.cos(follower.getPose().getHeading());
        double FBDeltaY = nudgeDistance * Math.sin(follower.getPose().getHeading());

        if (gamepad1.right_bumper) {
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad1.share) {
            follower.setPose(new Pose(9, 9, Math.toRadians(90)));
        }

        if (gamepad1.options) {
            follower.setPose(new Pose(9, 9, Math.toRadians(90)));
        }

        if (gamepad2.share && !prevShare) {
            launcherOn = !launcherOn;
        }

        if (launcherOn) {
            targetVelocity = (Math.round((PidInputSpeed * 1960) / 20.0) * 20) - 10;
            if (pidTimer.seconds() >= LOOP_TIME) {
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

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Velocity", currentVelocity);
        packet.put("Target Velocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet);

        if (gamepad2.left_stick_button && !prevLeftJoystick) {
            PidInputSpeed -= 0.01;
        }

        if (gamepad2.right_stick_button && !prevRightJoystick) {
            PidInputSpeed += 0.01;
        }

        PidInputSpeed = Math.max(-1.0, Math.min(1.0, PidInputSpeed));

        //if (gamepad2.dpad_down) intakeServo.setPower(1.0);
        //else if (gamepad2.dpad_up) intakeServo.setPower(-1.0);
        //else if (gamepad2.dpad_left) intakeServo.setPower(0.0);

        if (gamepad2.cross) pushServoValue = SERVO_POS_DOWN;
        else if (gamepad2.square) pushServoValue = SERVO_POS_MID;

        if (!gamepad2.triangle) pushServo.setPosition(pushServoValue);
        else pushServo.setPosition(SERVO_POS_UP);

        if (gamepad2.left_bumper && !lastLeftBumper) targetTicks -= TICK_STEP;
        if (gamepad2.right_bumper && !lastRightBumper) targetTicks += TICK_STEP;

        revolverSpin((int)targetTicks);

        telemetry.addData("Launcher Input Percentage", PidInputSpeed);
        telemetry.addData("Launcher Velocity Error", error);
        telemetry.addData("Revolver Ticks", targetTicks);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("LauncherPOWER", launcher.getPower());

        prevLeftJoystick = gamepad2.left_stick_button;
        prevRightJoystick = gamepad2.right_stick_button;
        prevShare = gamepad2.share;
        lastLeftBumper = gamepad2.left_bumper;
        lastRightBumper = gamepad2.right_bumper;

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
        }

        if (gamepad1.left_bumper) {
            Pose HOLD = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
            follower.setMaxPower(1);
            follower.holdPoint(HOLD);
        }

        if (gamepad1.ps) follower.holdPoint(Park); //park
        if (gamepad1.circle) { follower.holdPoint(poseAarush); PidInputSpeed = -0.38; } //HP
        if (gamepad1.triangle) { follower.holdPoint(pose2); PidInputSpeed = 0.62; } //close
        if (gamepad1.square) { follower.holdPoint(pose3); PidInputSpeed = 0.65; } //goofy
        if (gamepad1.cross) { follower.holdPoint(pose4); PidInputSpeed = 0.74; } //far

        if (gamepad1.dpad_left) {
            Pose lefttemp = new Pose(follower.getPose().getX() - LRDeltaX, follower.getPose().getY() - LRDeltaY, follower.getPose().getHeading());
            follower.setMaxPower(1);
            follower.holdPoint(lefttemp);
        }

        if (gamepad1.dpad_right) {
            Pose righttemp = new Pose(follower.getPose().getX() + LRDeltaX, follower.getPose().getY() + LRDeltaY, follower.getPose().getHeading());
            follower.setMaxPower(1);
            follower.holdPoint(righttemp);
        }

        if (gamepad1.dpad_up) {
            Pose forwardtemp = new Pose(follower.getPose().getX() + FBDeltaX, follower.getPose().getY() + FBDeltaY, follower.getPose().getHeading());
            follower.setMaxPower(1);
            follower.holdPoint(forwardtemp);
        }

        if (gamepad1.dpad_down) {
            Pose backwardtemp = new Pose(follower.getPose().getX() - FBDeltaX, follower.getPose().getY() - FBDeltaY, follower.getPose().getHeading());
            follower.setMaxPower(1);
            follower.holdPoint(backwardtemp);
        }

        if (gamepad1.left_trigger > 0.1) {
            Pose turnLeftTemp = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() - 0.1);
            follower.setMaxPower(1);
            follower.holdPoint(turnLeftTemp);
        }

        if (gamepad1.right_trigger > 0.1) {
            Pose turnRightTemp = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() + 0.1);
            follower.setMaxPower(1);
            follower.holdPoint(turnRightTemp);
        }

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
