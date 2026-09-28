package pedroPathing.examples;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Disabled
@TeleOp(name = "TeleopBlueOLD", group = "adon")
public class TeleopBlueOLD extends OpMode {

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotorEx revolver;
    private CRServo intakeServo;
    private Servo pushServo;
    private Follower follower;
    private FtcDashboard dashboard;
    // PID gains
    private double kP, kI, kD;
    double error;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0; // current motor power

    private ElapsedTime pidTimer = new ElapsedTime();

    private boolean launcherOn = false;
    private double targetVelocity = 0;
    private boolean prevShare, prevLeftJoystick, prevRightJoystick = false;
    double PidInputSpeed = 0.69;
    double pushServoValue = SERVO_POS_MID;

    private double targetTicks = 0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private final double LOOP_TIME = loopTime;

    // Threshold for switching PID sets
    private final double CLOSE_ERROR_THRESHOLD = 150; // ticks/sec

    private final double nudgeDistance = 0.75;
    private final Pose testingStartPose = new Pose((55.375), (8.125), Math.toRadians(90));
    private final Pose poseAarush = new Pose(12.5, (12), Math.toRadians(0)); //circle
    private final Pose pose2 = new Pose((59), (77), Math.toRadians(310)); //triangle
    private final Pose pose3 = new Pose((48), (96), Math.toRadians(314)); //square
    private final Pose pose4 = new Pose((59.3), (19.9), Math.toRadians(292)); //cross
    private final Pose Park = new Pose((106.3), (33.3), Math.toRadians(90));
    private PathChain drivetoOne, drivetoTwo, drivetoThree, drivetoFour, PARK;

    private int pathState = 0;

    private void setPathState(int newState) {
        pathState = newState;
    }

    public void buildPaths() {
        drivetoOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(poseAarush)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), poseAarush.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        drivetoTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(pose2)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), pose2.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        drivetoThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(pose3)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), pose3.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        drivetoFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(pose4)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), pose4.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();

        PARK = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(Park)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Park.getHeading())
                .setPathEndTimeoutConstraint(500)
                .build();
    }

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEndBlue);
        buildPaths();
        pidTimer.reset();

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
    public void init_loop() {
        buildPaths();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
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

        double LRDeltaX = (nudgeDistance * (Math.cos((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));
        double LRDeltaY = (nudgeDistance * (Math.sin((-1 * (((3.14159 / 2) - (follower.getPose().getHeading())))))));

        double FBDeltaX = (nudgeDistance * (Math.cos(follower.getPose().getHeading())));
        double FBDeltaY = (nudgeDistance * (Math.sin(follower.getPose().getHeading())));

        if (gamepad1.right_bumper) {
            setPathState(0);
            follower.breakFollowing();
            follower.setMaxPower(1.0);
            follower.startTeleopDrive();
        }

        if (gamepad2.share && !prevShare) {
            launcherOn = !launcherOn;
        }

        if (launcherOn) {
            targetVelocity = Math.round((PidInputSpeed * 1960) / 20.0) * 20; // ticks/sec
            if (pidTimer.seconds() >= LOOP_TIME) {
                pidOutput = runPID(targetVelocity, currentVelocity, pidOutput);
                pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
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


        // Always send telemetry to dashboard
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

        if (gamepad2.dpad_down) {
            intakeServo.setPower(1.0);
        } else if (gamepad2.dpad_up) {
            intakeServo.setPower(-1.0);
        } else if (gamepad2.dpad_left) {
            intakeServo.setPower(0.0);
        }

        if (gamepad2.cross) {
            pushServoValue = SERVO_POS_DOWN;
        } else if (gamepad2.square) {
            pushServoValue = SERVO_POS_MID;
        }

        if (!gamepad2.triangle) {
            pushServo.setPosition(pushServoValue);
        } else {
            pushServo.setPosition(SERVO_POS_UP);
        }

        if (gamepad2.left_bumper && !lastLeftBumper) {
            targetTicks -= TICK_STEP;
        }

        if (gamepad2.right_bumper && !lastRightBumper) {
            targetTicks += TICK_STEP;
        }

        revolver.setTargetPosition((int) targetTicks);
        revolver.setPower(revolverPower);


        telemetry.addData("Launcher Input Percentage", PidInputSpeed);
        telemetry.addData("Launcher Velocity Error", error);
        telemetry.addData("Revolver Ticks", targetTicks);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        prevLeftJoystick = gamepad2.left_stick_button;
        prevRightJoystick = gamepad2.right_stick_button;
        prevShare = gamepad2.share;
        lastLeftBumper = gamepad2.left_bumper;
        lastRightBumper = gamepad2.right_bumper;



        switch (pathState) {
            case 0:

                if (!follower.isBusy()) {
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
                }

                if (gamepad1.left_bumper) {
                    Pose HOLD = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    follower.setMaxPower(1);
                    follower.holdPoint(HOLD);
                }

                if (gamepad1.ps) {
                    buildPaths();
                    setPathState(5);
                }

                if (gamepad1.circle) {
                    buildPaths();
                    PidInputSpeed = -0.29;
                    setPathState(1);
                }

                if (gamepad1.triangle) {
                    buildPaths();
                    PidInputSpeed = 0.69;
                    setPathState(2);
                }

                if (gamepad1.square) {
                    buildPaths();
                    PidInputSpeed = 0.66;
                    setPathState(3);
                }

                if (gamepad1.cross) {
                    buildPaths();
                    PidInputSpeed = 0.81;
                    setPathState(4);
                }

                if (gamepad1.dpad_left) {
                    Pose lefttemp = new Pose((follower.getPose().getX() - LRDeltaX), (follower.getPose().getY() - LRDeltaY), follower.getPose().getHeading());
                    follower.setMaxPower(1);
                    follower.holdPoint(lefttemp);
                }

                if (gamepad1.dpad_right) {
                    Pose righttemp = new Pose((follower.getPose().getX() + LRDeltaX), (follower.getPose().getY() + LRDeltaY), follower.getPose().getHeading());
                    follower.setMaxPower(1);
                    follower.holdPoint(righttemp);
                }

                if (gamepad1.dpad_up) {
                    Pose forwardtemp = new Pose((follower.getPose().getX() + FBDeltaX), (follower.getPose().getY() + FBDeltaY), follower.getPose().getHeading());
                    follower.setMaxPower(1);
                    follower.holdPoint(forwardtemp);
                }

                if (gamepad1.dpad_down) {
                    Pose backwardtemp = new Pose((follower.getPose().getX() - FBDeltaX), (follower.getPose().getY() - FBDeltaY), follower.getPose().getHeading());
                    follower.setMaxPower(1);
                    follower.holdPoint(backwardtemp);
                }

                if (gamepad1.left_trigger>0.1) {
                    Pose turnLeftTemp = new Pose((follower.getPose().getX()), (follower.getPose().getY()), follower.getPose().getHeading()-0.1);
                    follower.setMaxPower(1);
                    follower.holdPoint(turnLeftTemp);
                }

                if (gamepad1.right_trigger>0.1) {
                    Pose turnRightTemp = new Pose((follower.getPose().getX()), (follower.getPose().getY()), follower.getPose().getHeading()+0.1);
                    follower.setMaxPower(1);
                    follower.holdPoint(turnRightTemp);
                }

                break;

            case 1:
                follower.followPath(drivetoOne, 1, true);
                setPathState(6);
                break;

            case 2:
                follower.followPath(drivetoTwo, 1, true);
                setPathState(6);
                break;

            case 3:
                follower.followPath(drivetoThree, 1, true);
                setPathState(6);
                break;

            case 4:
                follower.followPath(drivetoFour, 1, true);
                setPathState(6);
                break;

            case 5:
                follower.followPath(PARK, 1, true);
                setPathState(6);
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.startTeleopDrive();
                    setPathState(0);
                }
                break;
        }

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
