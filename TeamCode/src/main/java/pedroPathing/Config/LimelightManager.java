package pedroPathing.Config;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimelightManager {

    public enum Mode { NEURAL, APRILTAG }

    private final Limelight3A limelight;
    private String currentColor = "purple"; // "purple", "green", or null = any
    private Mode currentMode = Mode.NEURAL;

    public static double targetH = 2.5; //inches
    public static double camH = 1.25; //inches

    public static double xOffset = 0; //inches
    public static double yOffset = 7; //inches





    private TargetData latest = null;
    private TargetData saved = null;
    private Double savedDistance = null;

    public LimelightManager(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setMode(Mode mode) {
        currentMode = mode;
        switch (mode) {
            case NEURAL:
                limelight.pipelineSwitch(1);
                break;
            case APRILTAG:
                limelight.pipelineSwitch(0);
                break;
        }
    }

    public Mode getMode() { return currentMode; }

    // Toggle Purple -> Green -> Any -> Purple ...
    public void toggleColor() {
        if ("purple".equals(currentColor)) currentColor = "green";
        else if ("green".equals(currentColor)) currentColor = null;
        else currentColor = "purple";
    }

    /**
     * Explicitly set Neural color mode.
     * @param color "purple", "green", or null for any
     */
    public void setColorMode(String color) {
        if (color == null || "purple".equalsIgnoreCase(color) || "green".equalsIgnoreCase(color)) {
            currentColor = color;
        } else {
            throw new IllegalArgumentException("Invalid color. Must be 'purple', 'green', or null for any.");
        }
    }

    public String currentColor() { return currentColor != null ? currentColor : "ANY"; }

    public void update() {
        latest = (currentMode == Mode.NEURAL) ? detectNeural() : detectAprilTag();
        if (saved != null) savedDistance = distance(saved.tx, saved.ty);
    }

    public void save() {
        if (latest != null) {
            saved = latest;
            savedDistance = distance(saved.tx, saved.ty);
        }
    }

    public void clear() {
        saved = null;
        savedDistance = null;
    }

    private TargetData detectNeural() {
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return null;

        for (DetectorResult dr : res.getDetectorResults()) {
            if (currentColor == null || dr.getClassName().equalsIgnoreCase(currentColor))
                return new TargetData(dr.getClassName(), dr.getTargetXDegrees(), dr.getTargetYDegrees(), null);
        }
        return null;
    }

    private TargetData detectAprilTag() {
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return null;
        List<FiducialResult> tags = res.getFiducialResults();
        if (tags != null && !tags.isEmpty()) {
            FiducialResult fr = tags.get(0);
            return new TargetData(null, fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getFiducialId());
        }
        return null;
    }

    private double distance(double tx, double ty) {
        return Math.abs((camH - targetH) / Math.tan(Math.toRadians(Math.abs(ty))));
    }

    public TargetData latest() { return latest; }
    public TargetData saved() { return saved; }
    public Double savedDistance() { return savedDistance; }

    public static class TargetData {
        public final String color;
        public final double tx, ty;
        public final Integer tagId;

        public TargetData(String color, double tx, double ty, Integer tagId) {
            this.color = color;
            this.tx = tx;
            this.ty = ty;
            this.tagId = tagId;
        }
    }
}
