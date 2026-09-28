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

@TeleOp(name = "LimelightTogglePurpleAndGreen", group = "Adon")
public class LimelightTogglePurpleAndGreen extends OpMode {

    private Limelight3A limelight;
    private boolean detectPurple = true;  // start with purple
    private boolean lastCross = false;    // used for button edge detection

    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Use pipeline 1 for both purple and green detection
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Initialized (Pipeline 1)");
        telemetry.addData("Current Mode", "Detecting PURPLE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Handle button toggle ---
        boolean crossPressed = gamepad1.cross; // PS4 "X" or Logitech "A"

        if (crossPressed && !lastCross) {
            detectPurple = !detectPurple; // toggle color mode
        }
        lastCross = crossPressed;

        String targetColor = detectPurple ? "purple" : "green";

        // --- Get Limelight results ---
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();

            // Filter for the selected color
            List<LLResultTypes.DetectorResult> detections = new ArrayList<>();
            for (LLResultTypes.DetectorResult dr : detectors) {
                if (dr.getClassName().equalsIgnoreCase(targetColor)) {
                    detections.add(dr);
                }
            }

            if (!detections.isEmpty()) {
                for (LLResultTypes.DetectorResult dr : detections) {
                    telemetry.addData("Artifact", dr.getClassName());
                    telemetry.addData("TX (deg)", "%.2f", dr.getTargetXDegrees());
                    telemetry.addData("TY (deg)", "%.2f", dr.getTargetYDegrees());
                    telemetry.addLine();
                }
            } else {
                telemetry.addData("No " + targetColor + " Artifacts Found", true);
            }
        } else {
            telemetry.addData("No Valid Limelight Data", true);
        }

        telemetry.addData("Mode", "Detecting " + targetColor.toUpperCase());
        telemetry.update();
    }
}
