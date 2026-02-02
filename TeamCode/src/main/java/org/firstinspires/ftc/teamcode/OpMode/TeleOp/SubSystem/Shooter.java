package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
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

    public double calculatedTargetVelocity, calculatedHoodAngle, calculatedTurretPos;

    // Original tracking variables
    public double filteredAprilX, aprilx;

    public double lastKP, lastKI, lastKD;

    public Shooter(HardwareMap hwMap) {
        leftShooter = hwMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hwMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret1 = hwMap.get(Servo.class, "turret1");
        turret2 = hwMap.get(Servo.class, "turret2");
        hood = hwMap.get(Servo.class, "RightHood");
        hood.setPosition(Constant.HOOD_INIT);
        turret1.setPosition(Constant.TURRET_INIT);
        turret2.setPosition(Constant.TURRET_INIT);

        battery = hwMap.voltageSensor.iterator().next();

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterPID = new PIDFController(
                new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD)
        );
    }

    public void updateShootingParams(double odoDistance, int aprilTagID) {
        LLResult results = limelight.getLatestResult();
        linearInterpolation(odoDistance);

        if (results != null && results.isValid()) {
            List<LLResultTypes.FiducialResult> detection = results.getFiducialResults();
            for (LLResultTypes.FiducialResult april : detection) {
                if (april.getFiducialId() == aprilTagID) {
                    aprilx = april.getTargetXDegrees();
                    break;
                }
            }
        } else {
            aprilx = 0;
        }
    }

    public void updateTurret(double rawTurretAngle) {
        filteredAprilX += aprilx * 0.08;
        double turretHeading = rawTurretAngle + filteredAprilX;

        // Normalize 0-360
        turretHeading = ((turretHeading % 360) + 360) % 360;

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

        if (leftShooter.getVelocity() > 100) {
            double hoodServoPos = (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25) * (calculatedHoodAngle - 45) + Constant.HOOD_MAX;
            hoodServoPos= Math.max(Constant.HOOD_MAX, Math.min(Constant.HOOD_INIT, hoodServoPos));
            hood.setPosition(hoodServoPos);
        } else {
            hood.setPosition(Constant.HOOD_INIT);
        }
    }

    public void runShooter(boolean active) {
        if (!active) {
            leftShooter.setPower(0.5);
            rightShooter.setPower(0.5);
            return;
        }

        if (lastKP != Constant.kP ||
                lastKI != Constant.kI ||
                lastKD != Constant.kD) {

            shooterPID = new PIDFController(
                    new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD)
            );

            lastKP = Constant.kP;
            lastKI = Constant.kI;
            lastKD = Constant.kD;
        }

        if (Constant.overwritenVelocity != -1) {
            calculatedTargetVelocity = Constant.overwritenVelocity;
        }

        double currentVelo = leftShooter.getVelocity();
        double voltageComp = Constant.NOMINAL_VOLTAGE / battery.getVoltage();
        double ff = ((Constant.kV * calculatedTargetVelocity) + Constant.kS) * voltageComp;
        double error = Math.abs(currentVelo - calculatedTargetVelocity);

        shooterPID.setTargetPosition(calculatedTargetVelocity);
        double pidContribution = shooterPID.update(currentVelo);
        double totalPower = pidContribution + ff;

        totalPower = Math.max(0, Math.min(1.0, totalPower));
        if (error > 0.075 * leftShooter.getVelocity() && totalPower > 0.2) {
            leftShooter.setPower(1);
            rightShooter.setPower(1);
        } else {
            leftShooter.setPower(totalPower);
            rightShooter.setPower(totalPower);
        }
    }

    public boolean isReady() {
        double error = Math.abs(leftShooter.getVelocity() - calculatedTargetVelocity);
        return calculatedTargetVelocity > 0 && ((error <= 0.075 * leftShooter.getVelocity()) || (error <= 100));
    }
}