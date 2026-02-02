package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;

import java.util.List;

@Config
@TeleOp(name = "Shooter Tuner (BLUE)")
public class ShooterTunerTeleOp extends OpMode {

    // ================= DASHBOARD TUNING =================
    public static double TARGET_RPM = 0;
    public static double HOOD_ANGLE = 45; // degrees (25–45)
    public double aprilx;
    public double filteredAprilX;

    // ================= HARDWARE =================
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx leftShooter, rightShooter;
    private Servo leftPivot, rightPivot, hood;
    private Servo turret1, turret2;

    private GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;

    // ================= CONTROL =================
    private PIDFController shooterPID;
    private FtcDashboard dashboard;

    private Limelight3A limelight;
    double ty;

    private boolean fieldCentric = false;
    private boolean prevA = false;

    @Override
    public void init() {

        // ---- Drive ----
        leftFront  = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        leftBack   = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        rightFront = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBack  = hardwareMap.get(DcMotor.class, "RightBackMotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Shooter ----
        leftShooter  = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Pivot & Hood ----
        leftPivot  = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
        hood       = hardwareMap.get(Servo.class, "RightHood");

        leftPivot.setPosition(Constant.PIVOT_DOWN);
        rightPivot.setPosition(Constant.PIVOT_DOWN);
        hood.setPosition(Constant.HOOD_INIT);

        // ---- Turret ----
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turret1.setPosition(Constant.TURRET_INIT);
        turret2.setPosition(Constant.TURRET_INIT);

        // ---- Control ----
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooterPID = new PIDFController(
                new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD)
        );
        dashboard = FtcDashboard.getInstance();

        // ---- Odometry ----
        // Pinpoint Odo
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(Constant.ODO_X_OFFSET, Constant.ODO_Y_OFFSET, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setYawScalar(Constant.ODO_YAW_SCALAR);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    @Override
    public void loop() {

        // ================= ODOMETRY =================
        odo.update();
        Pose2D pos = odo.getPosition();

        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);
        filteredAprilX += aprilx * 0.08;

        double dx = x - Constant.GOAL_CENTER_X;
        double dy = y - Constant.RED_GOAL_CENTER_Y;
        double distance = Math.hypot(dx, dy);

        LLResult results = limelight.getLatestResult();

        if (results != null && results.isValid()) {
            List<LLResultTypes.FiducialResult> detection = results.getFiducialResults();
            for (LLResultTypes.FiducialResult april : detection) {
                if (april.getFiducialId() == 20) {
                    aprilx = april.getTargetXDegrees();
                    break;
                }
            }
        } else {
            aprilx = 0;
        }
        // ================= DRIVE =================
        if (gamepad1.a && !prevA) {
            fieldCentric = !fieldCentric;
            odo.resetPosAndIMU();
        }
        double limelightDistanceMeter = (Constant.hTarget - Constant.hCamera) / Math.tan(Math.toRadians(Constant.cameraAngle + ty));
        prevA = gamepad1.a;

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double driveRX = gamepad1.right_stick_x;

        if (fieldCentric) {
            double rotX = driveX * Math.cos(-headingRad) - driveY * Math.sin(-headingRad);
            double rotY = driveX * Math.sin(-headingRad) + driveY * Math.cos(-headingRad);
            driveX = rotX;
            driveY = rotY;
        }

        double denom = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(driveRX), 1);
        leftFront.setPower((driveY + driveX + driveRX) / denom);
        leftBack.setPower((driveY - driveX + driveRX) / denom);
        rightFront.setPower((driveY - driveX - driveRX) / denom);
        rightBack.setPower((driveY + driveX - driveRX) / denom);

        // ================= PIVOT =================
        if (gamepad1.left_bumper) {
            leftPivot.setPosition(Constant.PIVOT_UP);
            rightPivot.setPosition(Constant.PIVOT_UP);
        } else {
            leftPivot.setPosition(Constant.PIVOT_DOWN);
            rightPivot.setPosition(Constant.PIVOT_DOWN);
        }

        // ================= HOOD =================
        double hoodAngle = Math.max(25, Math.min(45, HOOD_ANGLE));
        double hoodPos =
                (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25)
                        * (hoodAngle - 45)
                        + Constant.HOOD_MAX;
        hood.setPosition(hoodPos);

        // ================= TURRET AUTO-AIM =================
        double referenceAngle = -Math.toDegrees(Math.atan2(dy, dx));
        double turretHeading = referenceAngle + headingDeg;
        turretHeading += filteredAprilX;

        double turretRange = Constant.TURRET_MAX - Constant.TURRET_MIN;
        double normalizedHeading = turretHeading + 90;

        if (normalizedHeading > 360) {
            normalizedHeading -= 360;
        } else if (normalizedHeading < 0) {
            normalizedHeading += 360;
        }

        double turretPos =
                Constant.TURRET_MIN
                        + (normalizedHeading / 360.0) * turretRange;

        turretPos = Math.max(
                Constant.TURRET_MIN,
                Math.min(Constant.TURRET_MAX, turretPos)
        );

        turret1.setPosition(turretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(turretPos + Constant.TURRET_ANTIBACKLASH);

        // ================= SHOOTER =================
        applyShooterPower(TARGET_RPM);

        // ================= TELEMETRY =================
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance (in)", distance);
        packet.put("Target RPM", TARGET_RPM);
        packet.put("Actual RPM", leftShooter.getVelocity());
        packet.put("Hood Angle", hoodAngle);
        packet.put("Turret Pos", turretPos);
        packet.put("Turret Heading", turretHeading);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Distance", "%.2f", distance);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Actual RPM", "%.1f", leftShooter.getVelocity());
        telemetry.addData("Hood Angle", hoodAngle);
        telemetry.update();
    }

    // ================= PID + FF =================
    private void applyShooterPower(double targetVelo) {

        double currentVelo = leftShooter.getVelocity();
        double voltage = batteryVoltageSensor.getVoltage();

        double ff = (Constant.kV * targetVelo) + (targetVelo > 0 ? Constant.kS : 0);

        shooterPID.setTargetPosition(targetVelo);
        double pid = shooterPID.update(currentVelo);

        double power = (pid + ff) * (Constant.NOMINAL_VOLTAGE / voltage);
        double safe = Math.max(0, Math.min(1.0, power));

        leftShooter.setPower(safe);
        rightShooter.setPower(safe);
    }
}
