package pedroPathing;

import static pedroPathing.Config.ConfigFile.REV_CLOSE_THRESHOLD;
import static pedroPathing.Config.ConfigFile.REVkD;
import static pedroPathing.Config.ConfigFile.REVkDClose;
import static pedroPathing.Config.ConfigFile.REVkP;
import static pedroPathing.Config.ConfigFile.REVkPClose;
import static pedroPathing.Config.ConfigFile.TICK_STEP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SPIN", group = "testing")
public class SPIN extends OpMode {

    private DcMotorEx revolver;
    private FtcDashboard dashboard;

    private double targetTicks = 0;

    private double revolverLastError = 0;
    private ElapsedTime revolverTimer = new ElapsedTime();

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        revolver = hardwareMap.get(DcMotorEx.class, "revolver");
        revolver.setDirection(DcMotorSimple.Direction.FORWARD);
        revolver.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        revolver.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        revolver.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        revolver.setPower(0);

        revolverTimer.reset();
    }

    private void revolverSpin(double targetTicks) {
        double dt = revolverTimer.seconds();
        revolverTimer.reset();
        if (dt <= 0) return;

        double currentTicks = revolver.getCurrentPosition();
        double error = targetTicks - currentTicks;

        double kP, kD;
        if (Math.abs(error) > REV_CLOSE_THRESHOLD) {
            kP = REVkP;
            kD = REVkD;
        } else {
            kP = REVkPClose;
            kD = REVkDClose;
        }

        double derivative = (error - revolverLastError) / dt;
        revolverLastError = error;

        double power = (kP * error) + (kD * derivative);
        power = Math.max(-1.0, Math.min(1.0, power));

        revolver.setPower(power);
    }

    @Override
    public void loop() {


        if (gamepad2.left_stick_button) targetTicks -= TICK_STEP;
        if (gamepad2.right_stick_button) targetTicks += TICK_STEP;

        revolverSpin(targetTicks);

        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Revolver Ticks", revolver.getCurrentPosition());
        packet.put("Target Ticks", targetTicks);
        packet.put("Error", targetTicks - revolver.getCurrentPosition());
        dashboard.sendTelemetryPacket(packet);
    }
}

