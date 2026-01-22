package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;
import java.util.Map;

public class Shooter {
    public DcMotorEx leftShooter, rightShooter;
    public Servo turret1, turret2, hood;
    private PIDFController shooterPID;
    private VoltageSensor battery;
    private Limelight3A limelight;

    public double calculatedTargetVelocity = 0.0;
    private double calculatedHoodAngle = 0.0;
    public double calculatedTurretPos = 0.0;

    // Original tracking variables
    public double limelightDistanceInch = -1;
    public double last_Distance = -1;
    private double filteredAprilX = 0;
    private double aprilx = 0;

    public Shooter(HardwareMap hwMap) {
        leftShooter = hwMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hwMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret1 = hwMap.get(Servo.class, "turret1");
        turret2 = hwMap.get(Servo.class, "turret2");
        hood = hwMap.get(Servo.class, "RightHood");
        turret1.setPosition(Constant.TURRET_INIT);
        turret2.setPosition(Constant.TURRET_INIT);

        shooterPID = new PIDFController(new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD));
        battery = hwMap.voltageSensor.iterator().next();

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void updateShootingParams(double odoDistance, int aprilTagID) {
        LLResult results = limelight.getLatestResult();

        if (results != null && results.isValid()) {
            List<LLResultTypes.FiducialResult> detection = results.getFiducialResults();
            for (LLResultTypes.FiducialResult april : detection) {
                if (april.getFiducialId() == aprilTagID) {
                    aprilx = april.getTargetXDegrees();
                    double ty = april.getTargetYDegrees();

                    // --- Your Original Polynomial Conversion ---
                    double limelightDistanceMeter = (Constant.hTarget - Constant.hCamera) / Math.tan(Math.toRadians(Constant.cameraAngle + ty));
                    limelightDistanceInch = limelightDistanceMeter * 39.3700787;
                    linearInterpolation(limelightDistanceInch);
                    last_Distance = limelightDistanceInch;
                }
            }
        } else {
            filteredAprilX = 0;
            aprilx = 0;
            linearInterpolation(odoDistance);
//            if (last_Distance != -1) {
//                linearInterpolation(last_Distance);
//            } else {
//                linearInterpolation(odoDistance);
//            }
        }
    }

    public void updateTurret(double rawTurretAngle) {
        // Original logic: turretHeading = referenceAngle + headingDeg + filteredAprilX
        filteredAprilX += aprilx * 0.08;
        double turretHeading = rawTurretAngle + filteredAprilX;

        // Normalize 0-360
        if (turretHeading > 360) turretHeading -= 360;
        if (turretHeading < 0) turretHeading += 360;

        calculatedTurretPos = Constant.TURRET_MIN + (turretHeading / 360.0) * Constant.TURRET_RANGE;

        calculatedTurretPos = Math.max(Constant.TURRET_MIN, Math.min(Constant.TURRET_MAX, calculatedTurretPos));

        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);
    }

    private void linearInterpolation(double distance) {
        Map.Entry<Double, double[]> low = Constant.SHOOTING_TABLE.floorEntry(distance);
        Map.Entry<Double, double[]> high = Constant.SHOOTING_TABLE.ceilingEntry(distance);

        if (low != null && high != null && !low.equals(high)) {
            double factor = (distance - low.getKey()) / (high.getKey() - low.getKey());
            calculatedTargetVelocity = low.getValue()[0] + (high.getValue()[0] - low.getValue()[0]) * factor;
            calculatedHoodAngle = low.getValue()[1] + (high.getValue()[1] - low.getValue()[1]) * factor;
        } else if (low != null) {
            calculatedTargetVelocity = low.getValue()[0];
            calculatedHoodAngle = low.getValue()[1];
        }

        if (calculatedTargetVelocity != 0) {
            double hoodServoPos = (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25) * (calculatedHoodAngle - 45) + Constant.HOOD_MAX;
            hood.setPosition(hoodServoPos);
        }
    }

    public void runShooter(boolean active) {
        double targetVelo = active ? calculatedTargetVelocity : 0.0;
        double currentVelo = leftShooter.getVelocity();
        double ff = (Constant.kV * targetVelo) + (targetVelo > 0 ? Constant.kS : 0);
        shooterPID.setTargetPosition(targetVelo);

        double power = (shooterPID.update(currentVelo) + ff) * (Constant.NOMINAL_VOLTAGE / battery.getVoltage());
        leftShooter.setPower(active ? Math.max(0, Math.min(1.0, power)) : 0);
        rightShooter.setPower(active ? Math.max(0, Math.min(1.0, power)): 0);
    }

    public boolean isReady() {
        return calculatedTargetVelocity > 0 && Math.abs(leftShooter.getVelocity() - calculatedTargetVelocity) <= 40;
    }
}