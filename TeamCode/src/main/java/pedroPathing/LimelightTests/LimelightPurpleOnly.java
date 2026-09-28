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

@TeleOp(name = "Limelight Purple Only", group = "Adon")
public class LimelightPurpleOnly extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // Initialize Limelight from hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to pipeline 1 (purple detector)
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Limelight Initialized - Pipeline 1 (Purple)");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();

            // Filter only purple detections, ignore all others (especially green)
            List<LLResultTypes.DetectorResult> purpleDetections = new ArrayList<>();
            for (LLResultTypes.DetectorResult dr : detectors) {
                if (dr.getClassName().equalsIgnoreCase("purple")) {
                    purpleDetections.add(dr);
                }
            }

            if (!purpleDetections.isEmpty()) {
                for (LLResultTypes.DetectorResult dr : purpleDetections) {
                    telemetry.addData("Class", dr.getClassName());
                    telemetry.addData("TX (deg)", "%.2f", dr.getTargetXDegrees());
                    telemetry.addData("TY (deg)", "%.2f", dr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("No Purple Artifacts Found", true);
            }
        } else {
            telemetry.addData("No Artifacts detected", true);
        }

        telemetry.update();
    }
}
