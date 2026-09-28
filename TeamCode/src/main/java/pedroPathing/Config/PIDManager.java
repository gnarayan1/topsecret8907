package pedroPathing.Config;

import static pedroPathing.Config.ConfigFile.CONFIGkP;
import static pedroPathing.Config.ConfigFile.CONFIGkI;
import static pedroPathing.Config.ConfigFile.CONFIGkD;
import static pedroPathing.Config.ConfigFile.CONFIGkPClose;
import static pedroPathing.Config.ConfigFile.CONFIGkIClose;
import static pedroPathing.Config.ConfigFile.CONFIGkDClose;
import static pedroPathing.Config.ConfigFile.loopTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDManager {

    private DcMotorEx launcher, launcher2;
    private FtcDashboard dashboard;

    private double kP, kI, kD;
    private double error = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0;

    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double lastPowerInput = 0;
    private final double LOOP_TIME = loopTime;
    private final double CLOSE_ERROR_THRESHOLD = 200;

    private final ElapsedTime pidTimer = new ElapsedTime();

    public PIDManager(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);

        pidTimer.reset();
    }

    private double runPIDCalculation(double target, double current, double currentPower) {
        error = target - current;
        integralSum += error * LOOP_TIME;
        double derivative = (error - lastError) / LOOP_TIME;
        double deltaPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;
        return currentPower + deltaPower;
    }

    public void runPID(double powerInput) {
        lastPowerInput = powerInput;
        currentVelocity = launcher.getVelocity();

        if (Math.abs(targetVelocity - currentVelocity) > CLOSE_ERROR_THRESHOLD) {
            kP = CONFIGkP;
            kI = CONFIGkI;
            kD = CONFIGkD;
        } else {
            kP = CONFIGkPClose;
            kI = CONFIGkIClose;
            kD = CONFIGkDClose;
        }

        targetVelocity = powerInput * 1960;

        if (pidTimer.seconds() >= LOOP_TIME) {
            pidOutput = runPIDCalculation(targetVelocity, currentVelocity, pidOutput);
            pidOutput = Math.max(0.0, Math.min(1.0, pidOutput));
            launcher.setPower(pidOutput);
            launcher2.setPower(pidOutput);
            pidTimer.reset();
        }
    }

    public void stop() {
        launcher.setPower(0);
        launcher2.setPower(0);
        targetVelocity = 0;
        pidOutput = 0;
        integralSum = 0;
        lastError = 0;
        lastPowerInput = 0;
        currentVelocity = 0;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }
}
