package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;

import java.util.TreeMap;

@Config
public class Constant {

    // --- Auton to Teleop values ---
    public static String ALLIANCE = "BLUE";
    public static double AUTON_LAST_X = 0;
    public static double AUTON_LAST_Y = 0;
    public static double AUTON_LAST_HEADING_DEG = 0;
    public static double AUTON_LAST_HEADING_RAD = 0;

    // --- Drive & Odometry ---
    // forward = x+, left = y+
    public static final double ODO_X_OFFSET = 121.271;
    public static final double ODO_Y_OFFSET = -60;
    public static final double ODO_YAW_SCALAR = 1.000861242911554;

    // Field Coordinates
    public static double OFFCENTER_X = 23.1;
    public static double GOAL_CENTER_X = 124.272815 - OFFCENTER_X;
    public static double BLUE_GOAL_CENTER_Y  = -125.287402;
    public static double RED_GOAL_CENTER_Y   =  125.287402;
    public static final double TURRET_OFFSET = 2.13320866;

    // --- PIDF ---
    public static double overwritenVelocity = -1;
    public static double kP = 0.0032;
    public static double kI = 0;
    public static double kD = 10e-7;
    public static double kV = 0.000341754;
    public static double kS = 0.12;
    public static double NOMINAL_VOLTAGE = 12.8;

    // --- Turret ---
    public static double TURRET_MIN         = 0.063;
    public static double TURRET_MAX         = TURRET_MIN + 0.9135;
    public static double TURRET_RANGE       = 0.9135;
    public static double TURRET_INIT        = TURRET_RANGE / 2 + TURRET_MIN;
    public static double TURRET_ANTIBACKLASH = 0.001;
    public static double TURRET_LOOKAHEAD_SEC = 0.12;
    public static double MAX_TURRET_OMEGA_DEG_S = 60.0;

    // --- Hood ---
    public static double HOOD_INIT = 0.14;
    public static double HOOD_MAX  = 0.81 + HOOD_INIT;

    // --- Pivot ---
    public static double PIVOT_UP   = 0.35;
    public static double PIVOT_DOWN = 0.03;

    // --- Spindexer Positions ---
    public static double INTAKE_POS1  = 0.014;
    public static double INTAKE_POS2  = 0.1525;
    public static double INTAKE_POS3  = 0.2925;
    public static double OUTTAKE_POS1 = 0.221;
    public static double OUTTAKE_POS2 = 0.361;
    public static double OUTTAKE_POS3 = 0.0825;

    // --- Encoder Tick Values (final — positions are fixed by hardware) ---
    public static final int OUTTAKE_POS1_TICK = 1365 * 3;
    public static final int OUTTAKE_POS2_TICK = 1365 * 5;
    public static final int OUTTAKE_POS3_TICK = 1365;
    public static final int INTAKE_POS1_TICK  = 0;
    public static final int INTAKE_POS2_TICK  = 1365 * 2;
    public static final int INTAKE_POS3_TICK  = 1365 * 4;

    // --- Timers & Tolerances ---
    public static int INVERSE_TIMER        = 1000;
    public static int INTAKE_TICK_TOLERANCE  = 650;
    public static int OUTTAKE_TICK_TOLERANCE = 400;
    public static int VELOCITY_TOLERANCE = 100;

    // PIVOT timers (ms) — how long pivot travels up and back down before next shot
    public static int PIVOT_UP_TIMER   = 90;
    public static int PIVOT_DOWN_TIMER = 150;   // was PIVOT_UP_TIMER + 60 = 150, same value

    // ANTI_STUCK_TIMER — ms before spindexer retries if it can't reach target ticks
    public static int ANTI_STUCK_TIMER = 750;

    public static float CALIBRATE_TIMER = 10;

    // --- Distance Lookup Table {RPM, HoodAngle} ---
    public static final TreeMap<Double, double[]> SHOOTING_TABLE = new TreeMap<>();
    static {
        SHOOTING_TABLE.put(30.0,  new double[]{1380, 30});
        SHOOTING_TABLE.put(40.0,  new double[]{1440, 35});
        SHOOTING_TABLE.put(50.0,  new double[]{1500, 40});
        SHOOTING_TABLE.put(60.0,  new double[]{1500, 45});
        SHOOTING_TABLE.put(70.0,  new double[]{1540, 45});
        SHOOTING_TABLE.put(80.0,  new double[]{1595, 45});
        SHOOTING_TABLE.put(90.0,  new double[]{1680, 45});
        SHOOTING_TABLE.put(100.0, new double[]{1740, 45});
        SHOOTING_TABLE.put(110.0, new double[]{1840, 45});
        SHOOTING_TABLE.put(120.0, new double[]{1940, 45});
        SHOOTING_TABLE.put(130.0, new double[]{2020, 45});
        SHOOTING_TABLE.put(140.0, new double[]{2080, 45});
        SHOOTING_TABLE.put(150.0, new double[]{2160, 45});
    }

    // Velocity
    public static double VEL_ALPHA = 0.80;  // increased — faster velocity response
    public static double VEL_ALPHA_DECAY = 0.90;
    public static double VEL_DEADBAND_IPS = 1.5;
    public static double VEL_DEADBAND_RPS = 0.05;
    public static double COMP_MIN_DIST = 25.0;
    public static double COMP_MAX_DIST = 55.0;

    // Shooter
    public static double SHOOTER_WHEEL_DIAMETER_INCH = 3.4;

    // Ball exit speed efficiency vs flywheel surface speed (0.0-1.0).
    public static double BALL_SPEED_EFFICIENCY = 0.9;
    public static double BALL_TRANSFER_TIME = 0.3;

    public static double MAX_RPM_STEP_PER_LOOP = 50.0;

    public static double LATERAL_COMP_BOOST = 1.38;

    public static double MOVING_SPEED_THRESHOLD = 4.0;

    // Limelight
    public static double hTarget      = 0.747;
    public static double hCamera      = 0.3468015;
    public static double cameraAngle  = 15;

    public static double APRIL_REST_DECAY_RATE = 0.12;
    public static double APRIL_MOVING_DECAY_RATE = 0.25;
    public static double APRIL_MAX_DEG = 6.0;
    public static int LIMELIGHT_SETTLE_LOOPS = 8;
    public static double EXTRAP_GAIN = 0.3;
}