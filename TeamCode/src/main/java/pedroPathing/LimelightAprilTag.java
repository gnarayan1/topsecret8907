package pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "LimelightAprilTag", group = "Adon")
public class LimelightAprilTag extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // Get the Limelight from the config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        // Start streaming data
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Grab AprilTag results
            List<FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Loop through all detected tags
                for (FiducialResult fr : fiducials) {
                    int tagId = fr.getFiducialId();

                    telemetry.addData("Tag ID", tagId);
                }
            } else {
                telemetry.addLine("Valid result, but no AprilTags detected.");
            }
        } else {
            telemetry.addLine("No target detected.");
        }

        telemetry.update();
    }
}
