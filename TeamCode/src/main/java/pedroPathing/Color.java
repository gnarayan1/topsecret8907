package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name = "Color", group = "Test")
public class Color extends OpMode {

    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ColorSensor colorSensor3;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");
        colorSensor3 = hardwareMap.get(ColorSensor.class, "color3");
    }

    @Override
    public void loop() {

        telemetry.addData("Red c1", colorSensor1.red());
        telemetry.addData("Green c1", colorSensor1.green());
        telemetry.addData("Blue c1", colorSensor1.blue());

        telemetry.addData("Red c2", colorSensor2.red());
        telemetry.addData("Green c2", colorSensor2.green());
        telemetry.addData("Blue c2", colorSensor2.blue());

        telemetry.addData("Red c3", colorSensor3.red());
        telemetry.addData("Green c3", colorSensor3.green());
        telemetry.addData("Blue c3", colorSensor3.blue());

        telemetry.update();
    }

    @Override
    public void stop() {
        telemetry.addLine("Color Sensor Stopped");
        telemetry.update();
    }
}
