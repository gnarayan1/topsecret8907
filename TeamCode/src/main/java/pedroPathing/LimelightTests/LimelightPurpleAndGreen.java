package pedroPathing.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@Disabled

@TeleOp(name = "Limelight Purple and Green", group = "Adon")
public class LimelightPurpleAndGreen extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Use pipeline 1 for purple/green detection
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Limelight Initialized - Pipeline 1 (Purple & Green)");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();

            // Filter for purple and green artifacts only
            List<LLResultTypes.DetectorResult> targetDetections = new ArrayList<>();
            for (LLResultTypes.DetectorResult dr : detectors) {
                String className = dr.getClassName().toLowerCase();
                if (className.equals("purple") || className.equals("green")) {
                    targetDetections.add(dr);
                }
            }

            if (!targetDetections.isEmpty()) {
                for (LLResultTypes.DetectorResult dr : targetDetections) {
                    telemetry.addData("Artifact", dr.getClassName());
                    telemetry.addData("TX (deg)", "%.2f", dr.getTargetXDegrees());
                    telemetry.addData("TY (deg)", "%.2f", dr.getTargetYDegrees());
                    telemetry.addLine(); // add space between detections
                }
            } else {
                telemetry.addData("No Purple or Green Artifacts Found", true);
            }
        } else {
            telemetry.addData("No Valid Limelight Data", true);
        }

        telemetry.update();
    }
}
