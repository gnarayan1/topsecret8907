package pedroPathing.Meet2AutoPaths;

import static pedroPathing.Config.ConfigFile.CONFIGkD;
import static pedroPathing.Config.ConfigFile.CONFIGkDClose;
import static pedroPathing.Config.ConfigFile.CONFIGkI;
import static pedroPathing.Config.ConfigFile.CONFIGkIClose;
import static pedroPathing.Config.ConfigFile.CONFIGkP;
import static pedroPathing.Config.ConfigFile.CONFIGkPClose;
import static pedroPathing.Config.ConfigFile.DEADZONE;
import static pedroPathing.Config.ConfigFile.SERVO_POS_DOWN;
import static pedroPathing.Config.ConfigFile.SERVO_POS_MID;
import static pedroPathing.Config.ConfigFile.SERVO_POS_UP;
import static pedroPathing.Config.ConfigFile.TICK_STEP;
import static pedroPathing.Config.ConfigFile.loopTime;
import static pedroPathing.Config.ConfigFile.revolverPower;
import static pedroPathing.Config.randomClass.autoEndBlue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous(name = "FarPreloadBlueOLD", group = "Adon")
public class FarPreloadBlueOLD extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer autoTimer;
    private int pathState;
    private boolean holding = false;
    String motif = "Unknown";
    private double targetVelocity = (Math.round((0.78 * 1960) / 20.0) * 20)-10; // ticks/sec
    private Timer actionTimer = new Timer();
    private boolean waiting = false;
    private double waitDuration = 0;

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx revolver;
    private CRServo intakeServo;
    private Servo pushServo;
    private FtcDashboard dashboard;
    private double targetTicks = 0;

    private Limelight3A limelight;
    private int detectedTagID = -1; // stores first seen AprilTag ID (21, 22, or 23)
    private int initDetectingID = -1;

    private final Pose startPose = new Pose(55.375, 8.125, Math.toRadians(90));
    private final Pose SHOOT = new Pose((59.3), (19.9), Math.toRadians(293));
    private final Pose PARK = new Pose(41, 35, Math.toRadians(90));


    private PathChain goToShoot1, goToPreIntake1, goToIntake1, goToPreIntake2, goToIntake2, goToPreIntake3, goToIntake3, goToShoot2, goToPark;

    private double kP, kI, kD;
    double error;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0; // current motor power

    private ElapsedTime pidTimer = new ElapsedTime();

    private final double LOOP_TIME = loopTime;

    // Threshold for switching PID sets
    private final double CLOSE_ERROR_THRESHOLD = 150; // ticks/sec

    @Override
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                initDetectingID = result.getFiducialResults().get(0).getFiducialId();
            }
        }
        telemetry.addData("detectedTag", initDetectingID);
        telemetry.update();

        if (detectedTagID == 21)  motif = "GPP";
        if (detectedTagID == 22)  motif = "PGP";
        if (detectedTagID == 23)  motif = "PPG";
        telemetry.addData("motif", motif);
        detectedTagID = initDetectingID;
    }

    public void buildPaths() {
        goToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(SHOOT)))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT.getHeading())
                .setPathEndTimeoutConstraint(300)
                .build();
        goToPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SHOOT), new Point(PARK)))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), PARK.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                detectedTagID = initDetectingID;
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(goToShoot1);
                    pushServo.setPosition(SERVO_POS_MID);
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }
                    if (detectedTagID == 21) {
                        targetTicks = 0;
                    }
                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 21) {
                        targetTicks += 2*TICK_STEP;
                    }
                    if (detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }
                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if (actionTimer.getElapsedTimeSeconds() >= 4) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 21) {
                        targetTicks += 2*TICK_STEP;
                    }

                    if (detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }

                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9:
                if (actionTimer.getElapsedTimeSeconds() >= 4) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 3) {
                    pushServo.setPosition(SERVO_POS_DOWN);
                    actionTimer.resetTimer();
                    follower.followPath(goToPark);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
                    targetVelocity = 0;
                    autoEndBlue = follower.getPose();
                    setPathState(-1);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setMaxPower(1);
        pathTimer = new Timer();
        autoTimer = new Timer();
        follower.setStartingPose(startPose);
        buildPaths();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        detectedTagID = initDetectingID;

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        pushServo = hardwareMap.get(Servo.class, "pushServo");
        pushServo.setPosition(SERVO_POS_MID);

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorSimple.Direction.FORWARD);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setTargetPosition(0);
        revolver.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        revolver.setPower(0);
        pidTimer.reset();
    }

    @Override
    public void start() {
        setPathState(0);
        autoTimer.resetTimer();
        pushServo.setPosition(SERVO_POS_MID);
    }

    private double runPID(double target, double current, double currentPower) {
        error = target - current;

        if (Math.abs(error) > DEADZONE) {
            integralSum += error * LOOP_TIME;
        }
        double derivative = (error - lastError) / LOOP_TIME;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;

        return currentPower + deltaPower; // relative adjustment
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

        if (pidTimer.seconds() >= LOOP_TIME) {
            pidOutput = runPID(targetVelocity, currentVelocity, pidOutput);
            pidOutput = Math.max(0.0, Math.min(1.0, pidOutput)); // clamp to [-1,1]
            launcher.setPower(pidOutput);
            launcher2.setPower(pidOutput);
            pidTimer.reset();
        }

        if (autoTimer.getElapsedTime() >= 29500 && !holding) {
            holding = true;
            Pose holdPose = follower.getPose();
            follower.holdPoint(holdPose);
            autoEndBlue = follower.getPose();
        } else if (!holding) {
            autonomousPathUpdate();
        }

        autonomousPathUpdate();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty() && detectedTagID == -1) {
                detectedTagID = result.getFiducialResults().get(0).getFiducialId();
            }
        }

        if (detectedTagID == 21)  motif = "GPP";
        if (detectedTagID == 22)  motif = "PGP";
        if (detectedTagID == 23)  motif = "PPG";

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("theta", follower.getPose().getHeading());
        telemetry.addData("Auto Time", autoTimer.getElapsedTime());
        telemetry.addData("Detected Tag ID", detectedTagID == -1 ? "None yet" : detectedTagID);
        telemetry.addData("CurrentVelocity", currentVelocity);
        telemetry.addData("TargetVelocity", targetVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("motif", motif);


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher Velocity", currentVelocity);
        packet.put("Target Velocity", targetVelocity);
        dashboard.sendTelemetryPacket(packet);

        telemetry.update();

        follower.update();
    }

    @Override
    public void stop() {
        follower.update();
        autoEndBlue = follower.getPose();
    }
}
