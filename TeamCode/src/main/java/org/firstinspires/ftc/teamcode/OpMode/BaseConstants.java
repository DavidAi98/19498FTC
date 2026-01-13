package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import java.util.TreeMap;

@Config
public class BaseConstants {

    /* =======================
       ODOMETRY (PINPOINT)
       ======================= */

    // forward = x+
    // left    = y+
    public static double ODO_X_OFFSET = 166.771;   // strafe pod
    public static double ODO_Y_OFFSET = -68;       // forward pod
    public static double ODO_YAW_SCALAR = 1.000861242911554;
    // Red is source of truth
    public static double GOAL_CENTER_X = 132;

    // Alliance-specific
    public static double RED_GOAL_CENTER_Y  = 132;
    public static double BLUE_GOAL_CENTER_Y = -132;
    public static int RED_APRILTAG_ID = 24;
    public static int BLUE_APRILTAG_ID = 20;

    /* =======================
       FLYWHEEL PIDF
       ======================= */

    public static double kP = 0.0029;
    public static double kI = 0.0;
    public static double kD = 0.00004;
    public static double kV = 0.0003804471334;
    public static double kS = 0.04348887;
    public static double NOMINAL_VOLTAGE = 12.67;

    /* =======================
       TURRET
       ======================= */

    public static double TURRET_LEFT_MAX = 0.01;
    public static double TURRET_RIGHT_MAX = 0.925;
    public static double TURRET_INIT = 0.01;
    public static double TURRET_ANTIBACKLASH = 0.0005;

    /* =======================
       HOOD / PIVOT
       ======================= */

    public static double HOOD_INIT = 0.6;
    public static double HOOD_MAX = 0.17;

    public static double LEFT_PIVOT_UP = 0.05;
    public static double RIGHT_PIVOT_UP = 0.05;
    public static double PIVOT_DOWN = 1.0;

    /* =======================
       SPINDEXER
       ======================= */

    public static double SPINDEXER_INTAKE_POS1 = 0.06;
    public static double SPINDEXER_INTAKE_POS2 = 0.35;
    public static double SPINDEXER_INTAKE_POS3 = 0.62;

    public static double SPINDEXER_OUTTAKE_POS1 = 0.48;
    public static double SPINDEXER_OUTTAKE_POS2 = 0.76;
    public static double SPINDEXER_OUTTAKE_POS3 = 0.2;

    public static double SPINDEXER_INIT = 0.06;
    public static double SPINDEXER_ANTIBACKLASH = 0.003;

    /* =======================
       INTAKE
       ======================= */

    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_REVERSE = -0.8;

    /* =======================
       SHOOTER LOOKUP TABLE
       ======================= */

    public static final TreeMap<Double, double[]> SHOOTING_TABLE = new TreeMap<>();

    static {
        SHOOTING_TABLE.put(12.0,  new double[]{1400, 25});
        SHOOTING_TABLE.put(20.0,  new double[]{1460, 27});
        SHOOTING_TABLE.put(30.0,  new double[]{1500, 30});
        SHOOTING_TABLE.put(40.0,  new double[]{1540, 35});
        SHOOTING_TABLE.put(50.0,  new double[]{1640, 40});
        SHOOTING_TABLE.put(60.0,  new double[]{1700, 45});
        SHOOTING_TABLE.put(70.0,  new double[]{1780, 45});
        SHOOTING_TABLE.put(80.0,  new double[]{1840, 45});
        SHOOTING_TABLE.put(90.0,  new double[]{1900, 45});
        SHOOTING_TABLE.put(100.0, new double[]{2030, 45});
        SHOOTING_TABLE.put(130.0, new double[]{2280, 45});
        SHOOTING_TABLE.put(140.0, new double[]{2380, 45});
        SHOOTING_TABLE.put(150.0, new double[]{2480, 45});
        SHOOTING_TABLE.put(160.0, new double[]{2560, 45});
        SHOOTING_TABLE.put(170.0, new double[]{2700, 45});
    }

    /* =======================
       CAMERA GEOMETRY
       ======================= */

    public static double hTarget = 0.7477125;    // meters
    public static double hCamera = 0.35941;  // meters
    public static double cameraAngle = 15;   // degrees
}
