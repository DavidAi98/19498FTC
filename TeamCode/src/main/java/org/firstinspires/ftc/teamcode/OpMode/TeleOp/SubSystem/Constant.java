package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;

import java.util.TreeMap;

@Config
public class Constant {

    // --- Drive & Odometry ---
    // forward = x+
    // left = y+
    public static double ODO_X_OFFSET = 121.271; // straf pod, mm
    public static double ODO_Y_OFFSET = -60; // forward pod, mm
    public static double ODO_YAW_SCALAR = 1.000861242911554;
    public static double ODO_TICK_PER_UNIT = 0; // might use later

    // Field Coordinates
    public static double GOAL_CENTER_X = 124.272815; // inch
    public static double BLUE_GOAL_CENTER_Y = -125.287402; // inch
    public static double RED_GOAL_CENTER_Y = 125.287402; // inch


    // PIDF Coefficients
    public static double kP = 0.0029;
    public static double kI = 0.0;
    public static double kD = 0.00004;
    public static double kV = 0.0003804471334;
    public static double kS = 0.04348887;
    public static double NOMINAL_VOLTAGE = 12.5;

    // --- Turret ---
    public static double TURRET_MIN = 0.059;//left
    public static double TURRET_MAX = 0.9725;//right
    public static double TURRET_RANGE = TURRET_MAX-TURRET_MIN;
    public static double TURRET_INIT = TURRET_RANGE/2 + TURRET_MIN;
    public static double TURRET_ANTIBACKLASH = 0.0005;

    // --- Hood ---
    public static double HOOD_INIT = 0.65;
    public static double HOOD_MAX = 0.22;

    // --- Pivot ---
    public static double PIVOT_UP = 0.35;
    public static double PIVOT_DOWN = 0.03;

    // --- Spindexer Positions 1,2,3 -> 2,3,1 ---
    // Intake
    public static double INTAKE_POS1 = 0.014;
    public static double INTAKE_POS2 = 0.1525;
    public static double INTAKE_POS3 = 0.2925;

    // Outtake
    public static double OUTTAKE_POS1 = 0.221;
    public static double OUTTAKE_POS2 = 0.361;
    public static double OUTTAKE_POS3 = 0.0825;


    // Encoder Ticks Values
    public static final int OUTTAKE_POS1_TICK = 1365*3;
    public static final int OUTTAKE_POS2_TICK = 1365*5;
    public static final int OUTTAKE_POS3_TICK = 1365;
    public static final int INTAKE_POS1_TICK = 0;
    public static final int INTAKE_POS2_TICK = 1365*2;
    public static final int INTAKE_POS3_TICK = 1365*4;

    // TIMERS
    public static final int INVERSE_TIMER = 1000;
    public static final int INTAKE_TICK_TOLERANCE = 500;
    public static final int OUTTAKE_TICK_TOLERANCE = 300;
    public static final int PIVOT_UP_TIMER = 90;
    public static final int PIVOT_DOWN_TIMER = PIVOT_UP_TIMER + 60;
    public static final int ANTI_STUCK_TIMER = 500;
    public static final float CALIBRATE_TIMER = 3000;

    // --- Distance Lookup Table (Distance In : {RPM, HoodAngle}) ---
    public static final TreeMap<Double, double[]> SHOOTING_TABLE = new TreeMap<>();

    static {
        SHOOTING_TABLE.put(12.0,  new double[]{1400, 25});//1540-1560,
        SHOOTING_TABLE.put(20.0,  new double[]{1460, 27});//1540-1560
        SHOOTING_TABLE.put(30.0,  new double[]{1500, 30});//1540-1560
        SHOOTING_TABLE.put(40.0,  new double[]{1540, 35});//1540-1560
        SHOOTING_TABLE.put(50.0,  new double[]{1640, 40});//1640-1660
        SHOOTING_TABLE.put(60.0,  new double[]{1700, 45});//1700-1720
        SHOOTING_TABLE.put(70.0,  new double[]{1780, 45});//1780-1800
        SHOOTING_TABLE.put(80.0,  new double[]{1840, 45});//1840-1860
        SHOOTING_TABLE.put(90.0,  new double[]{1900, 45});//1920-1940
        SHOOTING_TABLE.put(100.0, new double[]{1980, 45});
        SHOOTING_TABLE.put(130.0, new double[]{2280, 45});//2280-2300
        SHOOTING_TABLE.put(140.0, new double[]{2380, 45});//2380-2400
        SHOOTING_TABLE.put(150.0, new double[]{2480, 45});//2480-2500
        SHOOTING_TABLE.put(160.0, new double[]{2500, 45});//2540-2520
        SHOOTING_TABLE.put(170.0, new double[]{2500, 45});//2540-2520
    }

    // LimeLight constants
    public static double hTarget = 0.747;   // meters
    public static double hCamera = 0.3468015;   // meters
    public static double cameraAngle = 15;  // degrees
}