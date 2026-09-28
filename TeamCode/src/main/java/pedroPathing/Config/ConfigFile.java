package pedroPathing.Config;

import com.acmerobotics.dashboard.config.Config;
@Config
public class ConfigFile {
    public static double power = 0.8;
    public static double CONFIGkP = 0.0008;
    public static double CONFIGkI = 0;
    public static double CONFIGkD = 0;
    public static double CONFIGkPClose = 0.0002;
    public static double CONFIGkIClose = 0;
    public static double CONFIGkDClose = 0.000025;

    public static double loopTime = 0.005; //5 msec
    public static double DEADZONE = 50;

    public static double SERVO_POS_DOWN = 0.9;
    public static double SERVO_POS_MID = 0.505;
    public static double SERVO_POS_UP = 0.29;

    public static double TICK_STEP = 179.23;

    public static final double revolverPower = 1;





    public static double limelightOffsetForward=8.75;
    public static double limelightOffsetRightwards=-1.25;
    public static double limelightHeight=12.25;
    public static double goalHeight=29.5;
    static double limelightDistance=0;


    public static double REVkP = 1;
    public static double REVkD = 0;

    public static double REVkPClose = 0.003;
    public static double REVkDClose = 0.00015;

    public static double REV_CLOSE_THRESHOLD = 60;
}