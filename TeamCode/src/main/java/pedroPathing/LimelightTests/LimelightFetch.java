package pedroPathing.LimelightTests;

import static pedroPathing.Config.LimelightManager.xOffset;
import static pedroPathing.Config.LimelightManager.yOffset;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import pedroPathing.Config.LimelightManager;
import pedroPathing.Config.randomClass;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
@Disabled
@TeleOp(name = "LimelightFetch", group = "Adon")
public class LimelightFetch extends OpMode {

    private LimelightManager lm;
    private Follower follower;
    private boolean lastPS = false;
    private PathChain fetchPath;
    private Pose artifact;

    @Override
    public void init() {
        lm = new LimelightManager(hardwareMap.get(Limelight3A.class, "limelight"));
        lm.setMode(LimelightManager.Mode.NEURAL);
        lm.setColorMode(null);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.update();
    }


    public void buildPaths() {
        if (artifact != null) {
            fetchPath = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(follower.getPose()), new Point(artifact)))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), artifact.getHeading())
                    .build();
        }
    }

    public void fetch(double tx, double distance) {
        double robotArtifactDeltaX = Math.sin(Math.toRadians(tx)) * distance + xOffset;
        double robotArtifactDeltaY = Math.cos(Math.toRadians(tx)) * distance + yOffset;
        double robotArtifactDeltaHeadingRad = Math.atan2(robotArtifactDeltaX, robotArtifactDeltaY);

        artifact = new Pose(
                follower.getPose().getX() + distance * Math.cos(robotArtifactDeltaHeadingRad),
                follower.getPose().getY() + distance * Math.sin(robotArtifactDeltaHeadingRad),
                follower.getPose().getHeading() - robotArtifactDeltaHeadingRad
        );

        telemetry.addData("Artifact Delta X", robotArtifactDeltaX);
        telemetry.addData("Artifact Delta Y", robotArtifactDeltaY);
        telemetry.addData("Artifact Delta Heading (rad)", robotArtifactDeltaHeadingRad);
    }

    @Override
    public void loop() {
        lm.update();

        LimelightManager.TargetData latest = lm.latest();
        LimelightManager.TargetData saved = lm.saved();

        if (gamepad1.ps && !lastPS) {
            if (latest != null) lm.save();
            if (saved != null) fetch(saved.tx, lm.savedDistance());
        }
        lastPS = gamepad1.ps;



        follower.setMaxPower(1);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
        follower.update();



        if (latest != null) telemetry.addData("Current", "%s TX:%.2f TY:%.2f", latest.color != null ? latest.color : "N/A", latest.tx, latest.ty);
        else telemetry.addData("Current", "No target");

        if (saved != null) telemetry.addData("Saved", "%s TX:%.2f TY:%.2f D:%.2f", saved.color != null ? saved.color : "N/A", saved.tx, saved.ty, lm.savedDistance());
        else telemetry.addData("Saved", "None");

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Theta", follower.getPose().getHeading());
        telemetry.update();
    }
}
