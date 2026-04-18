package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    Pose2D pos;
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public GoBildaPinpointDriver pinpoint;

    private double dx, dy;
    public double botX, botY;
    public double turretX, turretY;
    public double headingDeg, headingRad;
    public boolean resetingPos = false;
    public double velX, velY, omega;
    private static double rawVelX, rawVelY, rawOmega; // unfiltered, for telemetry
    private double prevHeadingRad = 0;
    private ElapsedTime loopTimer = new ElapsedTime(); // for omega derivation
    private double prevRawVelX = 0, prevRawVelY = 0; // for velocity extrapolation
    public int settledLoops = 0; // how many consecutive loops velocity has been at deadband

    public MecanumDrive(HardwareMap hwMap) {
        // Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "LeftFrontMotor");
        leftBack  = hwMap.get(DcMotor.class, "LeftBackMotor");
        rightFront = hwMap.get(DcMotor.class, "RightFrontMotor");
        rightBack  = hwMap.get(DcMotor.class, "RightBackMotor");

        // Set Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Brake Behavior
        DcMotor[] motors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize Pinpoint Odometry
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constant.ODO_X_OFFSET, Constant.ODO_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setYawScalar(Constant.ODO_YAW_SCALAR);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Constant.AUTON_LAST_X+2.5 , Constant.AUTON_LAST_Y - 2.5, AngleUnit.RADIANS, Constant.AUTON_LAST_HEADING_RAD));
    }

    public void update(double targetX, double targetY) {
        // elementry pos
        pinpoint.update();
        pos = pinpoint.getPosition();
        botX = pos.getX(DistanceUnit.INCH) - Constant.MAGIC_OFFSET * Math.cos(headingRad);
        botY = pos.getY(DistanceUnit.INCH) + Constant.MAGIC_OFFSET * Math.cos(headingRad);
        headingRad = pos.getHeading(AngleUnit.RADIANS);
        headingDeg = Math.toDegrees(headingRad);

        // advance calculations
        turretX = botX - Constant.TURRET_OFFSET * Math.cos(headingRad);
        turretY = botY - Constant.TURRET_OFFSET * Math.sin(headingRad);
        dx = turretX - targetX;
        dy = turretY - targetY;

        // Read velocity from Pinpoint and apply low-pass filter + deadband.
        rawVelX  = pinpoint.getVelX(DistanceUnit.INCH);
        rawVelY  = pinpoint.getVelY(DistanceUnit.INCH);

        // Derive omega (angular velocity) from heading change over time.
        double dt = loopTimer.seconds();
        loopTimer.reset();
        double deltaHeading = headingRad - prevHeadingRad;
        // Wrap deltaHeading to [-pi, pi] so a 359->1 degree transition doesn't give 358 deg/s
        while (deltaHeading >  Math.PI) deltaHeading -= 2 * Math.PI;
        while (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
        rawOmega = (dt > 0.001 && dt < 0.5) ? deltaHeading / dt : 0;
        prevHeadingRad = headingRad;

        // Asymmetric filter: ramp up uses VEL_ALPHA, decay uses VEL_ALPHA_DECAY.
        // Turret snap back

        double alphaX = (Math.abs(rawVelX)  < Math.abs(velX))  ? Constant.VEL_ALPHA_DECAY : Constant.VEL_ALPHA;
        double alphaY = (Math.abs(rawVelY)  < Math.abs(velY))  ? Constant.VEL_ALPHA_DECAY : Constant.VEL_ALPHA;
        double alphaO = (Math.abs(rawOmega) < Math.abs(omega)) ? Constant.VEL_ALPHA_DECAY : Constant.VEL_ALPHA;
        velX  = velX  + alphaX * (rawVelX  - velX);
        velY  = velY  + alphaY * (rawVelY  - velY);
        omega = omega + alphaO * (rawOmega - omega);

        // Deadband — snap to zero if below threshold
        if (Math.abs(velX)  < Constant.VEL_DEADBAND_IPS) velX  = 0;
        if (Math.abs(velY)  < Constant.VEL_DEADBAND_IPS) velY  = 0;
        if (Math.abs(omega) < Constant.VEL_DEADBAND_RPS) omega = 0;

        if (velX == 0 && velY == 0) settledLoops = Math.min(settledLoops + 1, 30);
        else settledLoops = 0;

        // Velocity extrapolation: predict one loop ahead by adding the change this loop.
        double extrapX = velX + Constant.EXTRAP_GAIN * (rawVelX - prevRawVelX);
        double extrapY = velY + Constant.EXTRAP_GAIN * (rawVelY - prevRawVelY);
        // Don't extrapolate below deadband — prevents noise amplification at rest
        velX = (Math.abs(velX) > 0) ? extrapX : 0;
        velY = (Math.abs(velY) > 0) ? extrapY : 0;
        prevRawVelX = rawVelX;
        prevRawVelY = rawVelY;

        // reset pos
        if (resetingPos) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -2.5, -2.5, AngleUnit.RADIANS, 0));
            resetingPos = false;
        }
    }

    // Mecanum drive
    public void drive(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = rotX;
            y = rotY;
        }
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower( (y + x + rx) / denom );
        leftBack.setPower(  (y - x + rx) / denom );
        rightFront.setPower((y - x - rx) / denom );
        rightBack.setPower( (y + x - rx) / denom );
    }

    // --- Helper methods ---
    public double distanceToGoal() {
        return Math.hypot(dx, dy);
    }

    public double angleToGoal() {
        double referenceAngle = -Math.toDegrees(Math.atan2(dy, dx)) + 90;
        return referenceAngle + headingDeg;
    }

    // Raw (unfiltered) velocity for telemetry — compare to velX/Y to see filter effect
    public static double getRawVelX() { return rawVelX; }
    public static double getRawVelY() { return rawVelY; }
    public double getRawOmega() { return rawOmega; }

    // Same angle formula as angleToGoal() but aimed at an arbitrary field position.
    // Used by TeleOp to aim the turret at the virtual goal from the solver.
    public double angleToPoint(double pointX, double pointY) {
        double pdx = turretX - pointX;
        double pdy = turretY - pointY;
        double referenceAngle = -Math.toDegrees(Math.atan2(pdy, pdx)) + 90;
        return referenceAngle + headingDeg;
    }
}