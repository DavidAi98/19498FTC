package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;

import java.util.TreeMap;

@Config
public class Constant {

    // --- System Identifiers ---
    public static String motif = "PPG"; // The target color pattern

    // --- Drive & Odometry ---
    // forward = x+
    // left = y+
    public static double deltaTime = 0.1;
    public static double ODO_X_OFFSET = 121.271; //straf pod
    public static double ODO_Y_OFFSET = -60; //forward pod
    public static double ODO_YAW_SCALAR = 1.000861242911554;
    public static double ODO_TICK_PER_UNIT = 0; //2.919045586*(119/115); why tf is the document formula like this

    // Field Coordinates (174.734281)
    public static double GOAL_CENTER_X = 132; //expected 127.110236
    public static double BLUE_GOAL_CENTER_Y = -132; //expected 129.826772
    public static double RED_GOAL_CENTER_Y = 132;


//    // PIDF Coefficients
//    public static double kP = 0.0029;
//    public static double kI = 0.0;
//    public static double kD = 0.00004;
//    public static double kV = 0.0003804471334;
//    public static double kS = 0.04348887;
//    public static double NOMINAL_VOLTAGE = 12.5;
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.08;
    public static double kV = 0.00036114812667215992241;
    public static double kS = 0.067;
    public static double NOMINAL_VOLTAGE = 12.67;

    // --- Turret ---
    public static double TURRET_MIN = 0.059;
    public static double TURRET_MAX = 0.9725;
    public static double TURRET_RANGE = TURRET_MAX-TURRET_MIN;
    public static double TURRET_INIT = TURRET_RANGE/2 + TURRET_MIN;
    public static double TURRET_ANTIBACKLASH = 0.0005;

    // --- Hood ---
    public static double HOOD_INIT = 0.6;
    public static double HOOD_MAX = 0.17;

    // --- Pivot ---
    public static double PIVOT_UP = 0.05;
    public static double PIVOT_DOWN = 1.0;

    // --- Spindexer Positions ---
    // Intake
    public static double SPINDEXER_INTAKE_POS1 = 0.0025;
    public static double SPINDEXER_INTAKE_POS2 = 0.145;
    public static double SPINDEXER_INTAKE_POS3 = 0.2825;

    // Outtake
    public static double SPINDEXER_OUTTAKE_POS1 = 0.2135;
    public static double SPINDEXER_OUTTAKE_POS2 = 0.355;
    public static double SPINDEXER_OUTTAKE_POS3 = 0.0775;

    public static double SPINDEXER_INIT = SPINDEXER_INTAKE_POS1;
    public static double SPINDEXER_ANTIBACKLASH = 0.003;

    // --- Intake Motor ---
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_REVERSE = -0.8;


    // Distance Lookup Table (Distance In : {RPM, HoodAngle})
    public static final TreeMap<Double, double[]> SHOOTING_TABLE = new TreeMap<>();

    static {
        // Distance (Inches) -> {Target Velocity (RPM), Hood Angle (Degrees)}
        // Formula: Angle = 90 - atan(180 / distance)
        // Velocity = SPEED_MULTI * sqrt(2 * 9.81 * dist^0.25 / sin(theta)^2)
        SHOOTING_TABLE.put(12.0,  new double[]{1400, 25});//1540-1560,
        SHOOTING_TABLE.put(20.0,  new double[]{1460, 27});//1540-1560
        SHOOTING_TABLE.put(30.0,  new double[]{1500, 30});//1540-1560
        SHOOTING_TABLE.put(40.0,  new double[]{1540, 35});//1540-1560
        SHOOTING_TABLE.put(50.0,  new double[]{1640, 40});//1640-1660, change hood later
        SHOOTING_TABLE.put(60.0,  new double[]{1700, 45});//1700-1720
        SHOOTING_TABLE.put(70.0,  new double[]{1780, 45});//1780-1800
        SHOOTING_TABLE.put(80.0,  new double[]{1840, 45});//1840-1860
        SHOOTING_TABLE.put(90.0,  new double[]{1900, 45.0});//1920-1940
        SHOOTING_TABLE.put(100.0, new double[]{2030, 45.0});
        SHOOTING_TABLE.put(130.0, new double[]{2280, 45.0});//2280-2300
        SHOOTING_TABLE.put(140.0, new double[]{2380, 45});//2380-2400
        SHOOTING_TABLE.put(150.0, new double[]{2480, 45.0});//2480-2500
        SHOOTING_TABLE.put(160.0, new double[]{2560, 45.0});//2540-2520
        SHOOTING_TABLE.put(170.0, new double[]{2700, 45.0});//2540-2520
    }

    // camera
    public static double hTarget = 0.747;   // meters
    public static double hCamera = 0.3468015;   // meters
    public static double cameraAngle = 15;  // degrees
}