package pedroPathing.LimelightTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "Limelight MultiMode", group = "Adon")
public class LimelightMultiMode extends OpMode {

    private Limelight3A limelight;

    private boolean detectPurple = true;   // Start detecting purple by default
    private boolean useAprilTags = false;  // Start in neural mode (purple/green)
    private boolean lastCross = false;     // Debounce for color toggle
    private boolean lastSquare = false;    // Debounce for mode toggle

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Start in neural detection pipeline (purple/green)
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Neural Detection (Purple/Green)");
        telemetry.addData("Detecting", "PURPLE");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Button Handling ---

        // Cross toggles between Purple <-> Green (only if not in AprilTag mode)
        boolean crossPressed = gamepad1.cross;
        if (crossPressed && !lastCross && !useAprilTags) {
            detectPurple = !detectPurple;
        }
        lastCross = crossPressed;

        // Square toggles between AprilTag mode and Neural mode
        boolean squarePressed = gamepad1.square;
        if (squarePressed && !lastSquare) {
            useAprilTags = !useAprilTags;
            if (useAprilTags) {
                limelight.pipelineSwitch(0); // AprilTag pipeline
            } else {
                limelight.pipelineSwitch(1); // Neural pipeline
            }
        }
        lastSquare = squarePressed;

        // --- Mode Processing ---
        if (useAprilTags) {
            runAprilTagMode();
        } else {
            runNeuralMode();
        }

        telemetry.update();
    }

    private void runAprilTagMode() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Mode", "AprilTag Tracking");

        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (FiducialResult fr : fiducials) {
                    telemetry.addData("Tag ID", fr.getFiducialId());
                    telemetry.addData("TX (deg)", "%.2f", fr.getTargetXDegrees());
                    telemetry.addData("TY (deg)", "%.2f", fr.getTargetYDegrees());
                    telemetry.addLine();
                }
            } else {
                telemetry.addData("No AprilTags Detected", true);
            }
        } else {
            telemetry.addData("No Valid Limelight Data", true);
        }
    }

    private void runNeuralMode() {
        LLResult result = limelight.getLatestResult();
        String targetColor = detectPurple ? "purple" : "green";

        telemetry.addData("Mode", "Neural Detection");
        telemetry.addData("Detecting", targetColor.toUpperCase());

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();
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
    }
}
