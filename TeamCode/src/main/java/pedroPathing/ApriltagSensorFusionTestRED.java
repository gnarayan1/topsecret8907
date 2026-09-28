package pedroPathing;

import static pedroPathing.Config.LimelightHeadingCorrection.LimelightCorrection;
import static pedroPathing.Config.randomClass.autoEndRed;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.Config.LimelightHeadingCorrection;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "ApriltagSensorFusionTestRED", group = "adon")
public class ApriltagSensorFusionTestRED extends OpMode {
    private Limelight3A limelight;
    private Follower follower;

    static double tx;
    static double ty;
    static int detectedTagID;



    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4); // Sensor fusion AprilTag pipeline
        limelight.start();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoEndRed);
    }


    @Override
    public void loop() {
        follower.startTeleopDrive();
        follower.setMaxPower(1.0);

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {

                tx = result.getFiducialResults().get(0).getTargetXDegrees();
                ty = result.getFiducialResults().get(0).getTargetYDegrees();
                detectedTagID = result.getFiducialResults().get(0).getFiducialId();

                telemetry.addData("AprilTag tx (deg)", tx);
                telemetry.addData("AprilTag ty (deg)", ty);
                telemetry.addData("DetectedTagID", detectedTagID);
            }
        }


        if (gamepad1.options) {
            Pose Correction = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(LimelightCorrection(tx, ty, follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()), detectedTagID)));
            follower.setPose(Correction);
        }


        telemetry.addData("Robot angle DEG", (LimelightCorrection(tx, ty, follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()), detectedTagID)));
        telemetry.addLine("Press OPTIONS on gamepad1 to set the heading!");

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
