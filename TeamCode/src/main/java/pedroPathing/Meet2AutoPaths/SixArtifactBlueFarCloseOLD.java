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
@Autonomous(name = "SixArtifactBlueFarCloseOLD", group = "Adon")
public class SixArtifactBlueFarCloseOLD extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer autoTimer;
    private int pathState;
    private boolean holding = false;
    String motif = "Unknown";
    private double targetVelocity = (Math.round((0.69 * 1960) / 20.0) * 20)-10; // ticks/sec
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
    private final Pose SHOOT = new Pose((59), (77), Math.toRadians(313));
    private final Pose preIntake1 = new Pose(48, 82, Math.toRadians(180));
    private final Pose Intake1 = new Pose(33, 82, Math.toRadians(180));
    private final Pose preIntake2 = new Pose(48, 59, Math.toRadians(180));
    private final Pose Intake2 = new Pose(33, 59, Math.toRadians(180));
    private final Pose preIntake3 = new Pose(48, 35.5, Math.toRadians(180));
    private final Pose Intake3 = new Pose(33, 35.5, Math.toRadians(180));
    private final Pose PARK = new Pose(41, 80, Math.toRadians(270));


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
        detectedTagID = initDetectingID;
        telemetry.addData("detectedTag", initDetectingID);
        telemetry.update();

        if (detectedTagID == 21)  motif = "GPP";
        if (detectedTagID == 22)  motif = "PGP";
        if (detectedTagID == 23)  motif = "PPG";
        telemetry.addData("motif", motif);
    }

    public void buildPaths() {
        goToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(SHOOT)))
                .setLinearHeadingInterpolation(startPose.getHeading(), SHOOT.getHeading())
                .setPathEndTimeoutConstraint(300)
                .build();
        goToPreIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SHOOT), new Point(preIntake1)))
                .setLinearHeadingInterpolation(SHOOT.getHeading(), preIntake1.getHeading())
                .build();
        goToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake1), new Point(Intake1)))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), Intake1.getHeading())
                .build();
        goToPreIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Intake1), new Point(preIntake2)))
                .setLinearHeadingInterpolation(Intake1.getHeading(), preIntake2.getHeading())
                .build();
        goToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake2), new Point(Intake2)))
                .setLinearHeadingInterpolation(preIntake2.getHeading(), Intake2.getHeading())
                .build();
        goToPreIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Intake2), new Point(preIntake3)))
                .setLinearHeadingInterpolation(Intake2.getHeading(), preIntake3.getHeading())
                .build();
        goToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake3), new Point(Intake3)))
                .setLinearHeadingInterpolation(preIntake3.getHeading(), Intake3.getHeading())
                .build();
        goToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Intake3), new Point(SHOOT)))
                .setLinearHeadingInterpolation(Intake3.getHeading(), SHOOT.getHeading())
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
                        targetTicks += 0;
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
                if (actionTimer.getElapsedTimeSeconds() >= 0.7) {
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
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
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
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10:
                if (actionTimer.getElapsedTimeSeconds() >= 0.5) {
                    pushServo.setPosition(SERVO_POS_DOWN);
                    intakeServo.setPower(1.0);
                    actionTimer.resetTimer();
                    follower.followPath(goToPreIntake1);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(goToIntake1);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (actionTimer.getElapsedTimeSeconds() >= 0.9) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    targetTicks += 2*TICK_STEP;
                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    follower.followPath(goToPreIntake2);
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    pushServo.setPosition(SERVO_POS_DOWN);
                    follower.followPath(goToIntake2);
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (actionTimer.getElapsedTimeSeconds() >= 0.9) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    targetTicks += 2*TICK_STEP;
                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    follower.followPath(goToPreIntake3);
                    actionTimer.resetTimer();
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    pushServo.setPosition(SERVO_POS_DOWN);
                    follower.followPath(goToIntake3);
                    actionTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21:
                if (actionTimer.getElapsedTimeSeconds() >= 0.9) {
                    pushServo.setPosition(SERVO_POS_MID);
                    follower.followPath(goToShoot2);
                    actionTimer.resetTimer();
                    setPathState(22);
                }
                break;
            case 22:
                if (actionTimer.getElapsedTimeSeconds() >= 1) {
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }
                    if (detectedTagID == 21) {
                        targetTicks += 0;
                    }

                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    actionTimer.resetTimer();
                    setPathState(23);
                }
                break;
            case 23:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(24);
                }
                break;
            case 24:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(25);
                }
                break;
            case 25:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(26);
                }
                break;
            case 26:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 21) {
                        targetTicks += 2*TICK_STEP;
                    }
                    if (detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }

                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    actionTimer.resetTimer();
                    setPathState(27);
                }
                break;
            case 27:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(28);
                }
                break;
            case 28:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_MID);
                    actionTimer.resetTimer();
                    setPathState(29);
                }
                break;
            case 29:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    if (detectedTagID == 22 || detectedTagID == -1 || detectedTagID == 21) {
                        targetTicks += 2*TICK_STEP;
                    }

                    if (detectedTagID == 23) {
                        targetTicks += TICK_STEP;
                    }

                    revolver.setTargetPosition((int) targetTicks);
                    revolver.setPower(revolverPower);
                    actionTimer.resetTimer();
                    setPathState(30);
                }
                break;
            case 30:
                if (actionTimer.getElapsedTimeSeconds() >= 0.8) {
                    pushServo.setPosition(SERVO_POS_UP);
                    actionTimer.resetTimer();
                    setPathState(31);
                }
                break;
            case 31:
                if (actionTimer.getElapsedTimeSeconds() >= 0.4) {
                    follower.followPath(goToPark);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
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
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // clamp to [-1,1]
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
        telemetry.addData("Detected Tag ID", detectedTagID);
        telemetry.addData("motif", motif);
        telemetry.addData("CurrentVelocity", currentVelocity);
        telemetry.addData("TargetVelocity", targetVelocity);
        telemetry.addData("Error", error);
        telemetry.update();


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
