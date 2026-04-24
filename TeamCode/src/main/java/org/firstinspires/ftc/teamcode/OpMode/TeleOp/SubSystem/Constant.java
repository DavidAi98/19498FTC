package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.config.Config;

import java.util.TreeMap;

@Config
public class Constant {

    // -----------------------------------------------------------------------
    // AUTON → TELEOP HANDOFF
    // Written at end of auton so TeleOp starts with correct robot position.
    // Don't touch manually — auton overwrites these automatically.
    // -----------------------------------------------------------------------
    public static String ALLIANCE = "BLUE";
    public static double AUTON_LAST_X = 0;
    public static double AUTON_LAST_Y = 0;
    public static double AUTON_LAST_HEADING_DEG = 0;
    public static double AUTON_LAST_HEADING_RAD = 0;

    // -----------------------------------------------------------------------
    // DRIVE & ODOMETRY
    // Physical pod positions — don't change without remeasuring hardware.
    // -----------------------------------------------------------------------
    public static final double ODO_X_OFFSET = 60;      // strafe pod, mm from center
    public static final double ODO_Y_OFFSET = -121.271;          // forward pod, mm from center
    public static final double ODO_YAW_SCALAR = 1.000861242911554; // heading drift correction

    // -----------------------------------------------------------------------
    // FIELD COORDINATES (inches)
    // All fixed by game field — don't change.
    // -----------------------------------------------------------------------
    public static double OFFCENTER_X       = 23.1 * 1;
    public static double GOAL_CENTER_X     = 124.272815 - OFFCENTER_X;
    public static double BLUE_GOAL_CENTER_Y = -126.287402;
    public static double RED_GOAL_CENTER_Y  =  126.287402;
    public static final double TURRET_OFFSET = 2.13320866;  // turret behind robot center

    // -----------------------------------------------------------------------
    // SHOOTER PIDF
    // kV and kS are the main tuning knobs. kP only if RPM oscillates.
    // -----------------------------------------------------------------------
    // Force a fixed RPM target regardless of distance. -1 = disabled (use table).
    // Set on Dashboard to test a specific RPM without moving.
    public static double overwritenVelocity = -1;

    public static double kP = 0.0032;       // proportional — increase if slow to reach target
    public static double kI = 0;            // integral — leave at 0, causes windup
    public static double kD = 10e-7;        // derivative — leave at 0 effectively
    public static double kV = 0.000341754;  // feedforward per RPM — primary steady-state tuning knob
    public static double kS = 0.12;         // static friction offset
    public static double NOMINAL_VOLTAGE = 12.8; // voltage PIDF was tuned at, auto-compensates battery drain

    // -----------------------------------------------------------------------
    // TURRET
    // MIN/MAX/RANGE are hardware limits — don't change without recalibrating.
    // -----------------------------------------------------------------------
    public static double TURRET_MIN          = 0.063;
    public static double TURRET_MAX          = TURRET_MIN + 0.9135;
    public static double TURRET_RANGE        = 0.9135;
    public static double TURRET_INIT         = TURRET_RANGE / 2 + TURRET_MIN;
    public static double TURRET_ANTIBACKLASH = 0.001; // tiny offset to remove gear slop, don't change

    // How far ahead in time (seconds) the turret pre-aims based on virtual goal angular velocity.
    // Compensates for servo mechanical lag.
    //   Too high → overshoots when you stop suddenly
    //   Too low  → trails behind while moving
    //   Watch "lookahead deg" on Dashboard — should read 2–5° while strafing fast
    public static double TURRET_LOOKAHEAD_SEC = 0.125;

    // Hard cap on turret angular rate for the lookahead term (deg/s).
    // Prevents blowup if turretOmega spikes at mid-range. Rarely hits this.
    public static double MAX_TURRET_OMEGA_DEG_S = 25.0;

    // -----------------------------------------------------------------------
    // HOOD
    // -----------------------------------------------------------------------
    public static double HOOD_INIT = 0.14;           // rest position (not shooting)
    public static double HOOD_MAX  = 0.81 + HOOD_INIT;

    // -----------------------------------------------------------------------
    // PIVOT
    // -----------------------------------------------------------------------
    public static double PIVOT_UP   = 0.35;
    public static double PIVOT_DOWN = 0.03;

    // -----------------------------------------------------------------------
    // SPINDEXER SERVO POSITIONS
    // Hardware-specific — don't change without re-measuring.
    // -----------------------------------------------------------------------

    public static double BRIGHTNESS_GATE = 2250;


    public static double INTAKE_POS1  = 0.014; // a
    public static double INTAKE_POS2  = 0.1528;//(b-a)(2/5)+a
    public static double INTAKE_POS3  = 0.2916;//(b-a)(4/5)+a
    public static double OUTTAKE_POS1 = 0.2222;//(b-a)(3/5)+a
    public static double OUTTAKE_POS2 = 0.361; // b
    public static double OUTTAKE_POS3 = 0.0834; //(b-a)(1/5)+a

    // -----------------------------------------------------------------------
    // ENCODER TICK VALUES — fixed by hardware gearing, don't change
    // -----------------------------------------------------------------------
    public static final int HALF_SLOT_TICK = 1365;
    public static final int OUTTAKE_POS1_TICK = HALF_SLOT_TICK * 3;
    public static final int OUTTAKE_POS2_TICK = HALF_SLOT_TICK * 5;
    public static final int OUTTAKE_POS3_TICK = HALF_SLOT_TICK;
    public static final int INTAKE_POS1_TICK  = 0;
    public static final int INTAKE_POS2_TICK  = HALF_SLOT_TICK * 2;
    public static final int INTAKE_POS3_TICK  = HALF_SLOT_TICK * 4;

    // -----------------------------------------------------------------------
    // TIMERS & TOLERANCES
    // -----------------------------------------------------------------------
    public static int INVERSE_TIMER         = 900;
    public static int INTAKE_TICK_TOLERANCE = 600;

    // How many encoder ticks away from target still counts as "in slot".
    //   Artifacts not firing → INCREASE (spindexer settling slowly)
    //   Artifacts fire at wrong slot → DECREASE (too loose)
    public static int OUTTAKE_TICK_TOLERANCE = 600;

    // RPM error allowed before isReady() returns true and fires the ball.
    // Note: isReady() has a (2200/currentVelo)^2 term making this looser than it looks.
    //   Balls firing before shooter is up to speed → DECREASE (try 60–80)
    public static int VELOCITY_TOLERANCE = 1000; //120

    // How long pivot travels up before coming back down (ms).
    public static int PIVOT_UP_TIMER   = 90;
    public static int PIVOT_DOWN_TIMER = 160;

    // Milliseconds before spindexer gives up and retries if it can't reach position.
    //   Getting stuck often → INCREASE
    public static int ANTI_STUCK_TIMER = 750;

    public static float CALIBRATE_TIMER = 10;

    // -----------------------------------------------------------------------
    // DISTANCE → RPM + HOOD ANGLE LOOKUP TABLE
    // Key = distance in inches. Value = {RPM, hood angle degrees}.
    // Interpolated between entries. Tune RPM values if shots land short/long.
    // -----------------------------------------------------------------------
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
        SHOOTING_TABLE.put(140.0, new double[]{2100, 45});
        SHOOTING_TABLE.put(150.0, new double[]{2180, 45});
    }

    // -----------------------------------------------------------------------
    // VELOCITY FILTER
    // Tune these FIRST before adjusting any solver constants.
    // -----------------------------------------------------------------------
    // Low-pass blend: filtered = filtered + ALPHA*(raw - filtered) each loop.
    //   0.0 = frozen, 1.0 = raw unfiltered.
    //   Too low  → turret barely reacts when you start moving
    //   Too high → turret jitters at rest
    public static double VEL_ALPHA = 0.80;

    // Separate alpha for when velocity is decreasing (robot slowing/stopping).
    // Higher than VEL_ALPHA so the turret snaps back faster when you stop.
    public static double VEL_ALPHA_DECAY = 0.90;

    // Velocity below this (in/s) is treated as zero — prevents turret jitter from sensor noise.
    //   Too high → compensation doesn't kick in until moving fast
    //   Too low  → turret jitters at rest
    public static double VEL_DEADBAND_IPS = 1.5;

    // Angular velocity below this (rad/s) treated as zero (~3 deg/s). Rarely needs changing.
    public static double VEL_DEADBAND_RPS = 0.05;

    // -----------------------------------------------------------------------
    // COMPENSATION DISTANCE RANGE
    // -----------------------------------------------------------------------
    // Below COMP_MIN_DIST: zero move-while-shoot compensation.
    //   At close range small virtual goal shifts = huge angle errors, so disable it.
    // Above COMP_MAX_DIST: full compensation.
    // Between: linearly blended.
    //   Turret still drifts when close → raise COMP_MIN_DIST
    public static double COMP_MIN_DIST = 40.0; // inches
    public static double COMP_MAX_DIST = 55.0; // inches

    // -----------------------------------------------------------------------
    // BALL PHYSICS — affects virtual goal position calculation
    // -----------------------------------------------------------------------
    // Physical flywheel diameter (inches). Affects RPM → ball speed conversion.
    public static double SHOOTER_WHEEL_DIAMETER_INCH = 3.4;

    // Ratio of ball exit speed to flywheel surface speed (0.0–1.0).
    // Accounts for compression and slip losses.
    //   Drive TOWARD goal at full speed: balls land SHORT → decrease, LONG → increase
    public static double BALL_SPEED_EFFICIENCY = 0.9;

    // Seconds from fire command to ball physically leaving the shooter.
    // Includes pivot travel and feed time.
    //   Stationary shots accurate but moving shots slightly off → adjust this
    public static double BALL_TRANSFER_TIME = 0.325; // was 0.3

    // -----------------------------------------------------------------------
    // RPM RATE LIMITER
    // -----------------------------------------------------------------------
    // Max RPM change per loop in calculatedTargetVelocity.
    // Prevents sudden distance jumps (e.g. when outtake starts) from spiking
    // the RPM target → exceeding 10% threshold → max power slam.
    //   Shooter still spikes sometimes → decrease to 30
    //   Shooter feels slow to adjust when you move a lot → increase to 80
    public static double MAX_RPM_STEP_PER_LOOP = 50.0;

    // -----------------------------------------------------------------------
    // DIRECTIONAL ASYMMETRY BOOST
    // -----------------------------------------------------------------------
    // The goal is far away in Y (-125in blue, +125in red).
    // Strafing further in that direction produces less turret angle change
    // than strafing the other way — geometry, not a bug.
    // This multiplier boosts the under-compensated direction.
    //   Blue: applies when strafing left (velY > 0)
    //   Red:  applies when strafing right (velY < 0) — auto-detected
    //   Still under on the weak side → increase in 0.1 steps (watch "effective dist" on Dashboard)
    //   Over-compensating → decrease
    //   WARNING: too high pushes effectiveDistInch past 150in → 2160 RPM target → max power spike
    public static double LATERAL_COMP_BOOST = 1.38;

    // -----------------------------------------------------------------------
    // LIMELIGHT INTEGRATION CONTROL
    // -----------------------------------------------------------------------
    // Speed (in/s) at which limelight correction fully cuts out.
    // Below this: limelight fine-tunes gradually. Above: solver controls heading entirely.
    //   Too low  → limelight re-engages too quickly and fights the solver
    //   Too high → limelight stays active at speed and double-corrects
    public static double MOVING_SPEED_THRESHOLD = 4.0;

    // Limelight camera geometry — used for distance estimation
    public static double hTarget     = 0.747;      // target height, meters
    public static double hCamera     = 0.3468015;  // camera height, meters
    public static double cameraAngle = 15;         // camera mount angle, degrees

    // How fast filteredAprilX decays toward zero while stationary.
    // Without this it accumulates indefinitely → turret drifts randomly at rest.
    //   stable value = aprilx * 0.1 / REST_DECAY_RATE (e.g. 8° tag × 0.1 / 0.12 = 6.7°)
    //   Too low  → still drifts at rest
    //   Too high → limelight correction drains before it can help
    public static double APRIL_REST_DECAY_RATE = 1.2;

    // How fast filteredAprilX drains while moving.
    // Prevents stale limelight data from holding the turret off-target after stopping.
    public static double APRIL_MOVING_DECAY_RATE = 0.25;

    // Hard cap on limelight correction (degrees). Limelight can never move turret beyond this.
    public static double APRIL_MAX_DEG = 30;

    // Consecutive loops at zero velocity before limelight re-engages after stopping.
    // Limelight has 100–200ms pipeline latency — without this delay, stale data from
    // when you were moving gets fed in immediately and pulls the turret the wrong way.
    public static int LIMELIGHT_SETTLE_LOOPS = 8;

    // Forward-predicts velocity one loop ahead to close Pinpoint's one-loop measurement delay.
    // Adds (EXTRAP_GAIN × velocity change this loop) as a prediction for next loop.
    //   Turret still slightly behind at full speed → increase toward 0.5
    //   Turret overshoots on sudden direction change → decrease toward 0.15
    public static double EXTRAP_GAIN = 0.3;
}