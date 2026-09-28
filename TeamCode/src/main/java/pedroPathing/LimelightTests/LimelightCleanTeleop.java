package pedroPathing.LimelightTests;

import pedroPathing.Config.LimelightManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "LimelightCleanTeleop", group = "Adon")
public class LimelightCleanTeleop extends OpMode {

    private LimelightManager lm;
    private boolean lastCross, lastCircle, lastTriangle, lastDpadUp, lastDpadDown;

    private LimelightManager.TargetData latest, saved;
    public Double savedTx, savedTy, savedDistance;
    public String currentColor;

    @Override
    public void init() {
        lm = new LimelightManager(hardwareMap.get(Limelight3A.class, "limelight"));
    }

    private void limelightInput() {
        // Toggle Neural color: Purple -> Green -> Any
        if (gamepad1.dpad_up && !lastDpadUp) lm.toggleColor();

        // Switch between Neural <-> AprilTag
        if (gamepad1.dpad_down && !lastDpadDown) {
            if (lm.getMode() == LimelightManager.Mode.NEURAL) lm.setMode(LimelightManager.Mode.APRILTAG);
            else lm.setMode(LimelightManager.Mode.NEURAL);
        }

        // Save / clear
        if (gamepad1.circle && !lastCircle) lm.save();
        if (gamepad1.triangle && !lastTriangle) lm.clear();

        // Update last button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastCircle = gamepad1.circle;
        lastTriangle = gamepad1.triangle;

        // Update Limelight data
        lm.update();
        latest = lm.latest();
        saved = lm.saved();
        currentColor = lm.currentColor();

        if (saved != null) {
            savedTx = saved.tx;
            savedTy = saved.ty;
            savedDistance = lm.savedDistance();
        } else {
            savedTx = savedTy = savedDistance = null;
        }
    }

    @Override
    public void loop() {
        limelightInput();

        telemetry.addData("System Mode", lm.getMode().toString());
        if (lm.getMode() == LimelightManager.Mode.NEURAL)
            telemetry.addData("Color Mode", currentColor.toUpperCase());

        // Latest detection
        if (latest != null) {
            if (lm.getMode() == LimelightManager.Mode.NEURAL) {
                telemetry.addData("Current", "%s TX:%.2f TY:%.2f",
                        latest.color != null ? latest.color : "N/A", latest.tx, latest.ty);
            } else {
                telemetry.addData("Current", "Tag %s TX:%.2f TY:%.2f",
                        latest.tagId != null ? latest.tagId.toString() : "N/A", latest.tx, latest.ty);
            }
        } else {
            telemetry.addData("Current", "No target detected");
        }

        // Saved detection
        if (saved != null) {
            if (lm.getMode() == LimelightManager.Mode.NEURAL) {
                telemetry.addData("Saved", "%s TX:%.2f TY:%.2f D:%.2f",
                        saved.color != null ? saved.color : "N/A", saved.tx, saved.ty, savedDistance);
            } else {
                telemetry.addData("Saved", "Tag %s TX:%.2f TY:%.2f D:%.2f",
                        saved.tagId != null ? saved.tagId.toString() : "N/A", saved.tx, saved.ty, savedDistance);
            }
        } else {
            telemetry.addData("Saved", "No target saved");
        }

        telemetry.update();
    }
}
