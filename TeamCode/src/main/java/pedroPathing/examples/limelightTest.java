package pedroPathing.examples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Disabled
@TeleOp(name = "limelightTest", group = "testing")
public class limelightTest extends OpMode {

    private Limelight3A limelight;
    private boolean crossPrev = false;
    private Double lastAngleDeg = null;

    private double highestCornerX = 0;
    private double highestCornerY = 0;

    private double bottomCornerX = 0;
    private double bottomCornerY = 0;

    private double rightmostCornerX = 0;
    private double rightmostCornerY = 0;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // Use color object detection pipeline
        limelight.start();
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("tx (deg)", result.getTx());
            telemetry.addData("ty (deg)", result.getTy());

            List<ColorResult> colorResults = result.getColorResults();
            if (colorResults != null && !colorResults.isEmpty()) {
                ColorResult cr = colorResults.get(0);
                List<List<Double>> corners = cr.getTargetCorners();

                if (corners != null && corners.size() == 4) {
                    double x0 = corners.get(0).get(0);
                    double y0 = corners.get(0).get(1);
                    double x1 = corners.get(1).get(0);
                    double y1 = corners.get(1).get(1);
                    double x2 = corners.get(2).get(0);
                    double y2 = corners.get(2).get(1);
                    double x3 = corners.get(3).get(0);
                    double y3 = corners.get(3).get(1);

                    double[][] all = {
                            {x0, y0},
                            {x1, y1},
                            {x2, y2},
                            {x3, y3}
                    };

                    highestCornerX = bottomCornerX = rightmostCornerX = x0;
                    highestCornerY = bottomCornerY = rightmostCornerY = y0;

                    for (double[] pair : all) {
                        double x = pair[0];
                        double y = pair[1];

                        if (y > highestCornerY) {
                            highestCornerX = x;
                            highestCornerY = y;
                        }

                        if (y < bottomCornerY) {
                            bottomCornerX = x;
                            bottomCornerY = y;
                        }

                        if (x > rightmostCornerX) {
                            rightmostCornerX = x;
                            rightmostCornerY = y;
                        }
                    }


                    double deltaA = Math.hypot(rightmostCornerY - bottomCornerY, rightmostCornerX - bottomCornerX);
                    double deltaB = Math.hypot(highestCornerY - rightmostCornerY, rightmostCornerX - highestCornerX);

                    if (deltaA < deltaB) {
                        lastAngleDeg = Math.toDegrees(Math.atan2(rightmostCornerX - bottomCornerX, rightmostCornerY - bottomCornerY));
                    } else {
                        lastAngleDeg = Math.toDegrees(Math.atan2(rightmostCornerX - bottomCornerX, rightmostCornerY - bottomCornerY)) + 90;
                    }

                } else {
                    telemetry.addLine("Corners missing or incomplete.");
                }
            } else {
                telemetry.addLine("No color results found.");
            }
        } else {
            telemetry.addLine("No target detected.");
        }

        if (lastAngleDeg != null) {
            telemetry.addData("Angle", lastAngleDeg);
        }

        telemetry.addData("angle var", (lastAngleDeg != null) ? lastAngleDeg : 0);
        crossPrev = gamepad1.cross;
        telemetry.update();
    }
}
