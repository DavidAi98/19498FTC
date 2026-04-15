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
    private double prevTargetVelocity = 0; // for rate-limiting RPM jumps

    // Original tracking variables
    public double filteredAprilX, aprilx;
    public double lastKP, lastKI, lastKD;
    String motif = "Null";

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

        battery = hwMap.voltageSensor.iterator().next();

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterPID = new PIDFController(
                new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD)
        );
    }

    public void updateShootingParams(double odoDistance, int aprilTagID, boolean active) {
        LLResult results = limelight.getLatestResult();
        linearInterpolation(odoDistance, active);

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

    public void updateTurret(double rawTurretAngle, double AUTON) {

        filteredAprilX += aprilx * 0.2 * AUTON;
        double turretHeading = rawTurretAngle + filteredAprilX;

        // Normalize 0-360
        turretHeading = ((turretHeading % 360) + 360) % 360;

        calculatedTurretPos = Constant.TURRET_MIN + (turretHeading / 360.0) * Constant.TURRET_RANGE;

        calculatedTurretPos = Math.max(Constant.TURRET_MIN, Math.min(Constant.TURRET_MAX, calculatedTurretPos));

        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);
    }

    // movingScale: 0 = stationary (full limelight correction), 1 = fast (no correction)
    public double movingScale = 0.0;

    public void updateTurret(double rawTurretAngle) {
        filteredAprilX += 2 * aprilx * (1.0 - movingScale);

        // Decay toward zero:
        //   While moving (movingScale=1): fast decay — drains stale values quickly
        double restDecay   = Constant.APRIL_REST_DECAY_RATE;    // slow bleed at rest
        double movingDecay = Constant.APRIL_MOVING_DECAY_RATE;  // fast drain while moving
        filteredAprilX *= (1.0 - restDecay - movingScale * (movingDecay - restDecay));

        // Hard cap — even with decay, clamp to a sane correction range.
        filteredAprilX = Math.max(-Constant.APRIL_MAX_DEG, Math.min(Constant.APRIL_MAX_DEG, filteredAprilX));

        double turretHeading = rawTurretAngle + filteredAprilX;

        // Normalize 0-360
        turretHeading = ((turretHeading % 360) + 360) % 360;

        calculatedTurretPos = Constant.TURRET_MIN + (turretHeading / 360.0) * Constant.TURRET_RANGE;

        calculatedTurretPos = Math.max(Constant.TURRET_MIN, Math.min(Constant.TURRET_MAX, calculatedTurretPos));

        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);
    }


    private void linearInterpolation(double distance, boolean active) {
        Map.Entry<Double, double[]> low = Constant.SHOOTING_TABLE.floorEntry(distance);
        Map.Entry<Double, double[]> high = Constant.SHOOTING_TABLE.ceilingEntry(distance);

        double rawTarget;
        if (low != null && high != null && !low.equals(high)) {
            double factor = (distance - low.getKey()) / (high.getKey() - low.getKey());
            rawTarget = low.getValue()[0] + (high.getValue()[0] - low.getValue()[0]) * factor;
            calculatedHoodAngle = low.getValue()[1] + (high.getValue()[1] - low.getValue()[1]) * factor;
        } else if (low != null) {
            rawTarget = low.getValue()[0];
            calculatedHoodAngle = low.getValue()[1];
        } else if (high != null) {
            rawTarget = high.getValue()[0];
            calculatedHoodAngle = high.getValue()[1];
        } else {
            rawTarget = prevTargetVelocity;
        }

        double step = rawTarget - prevTargetVelocity;
        if (Math.abs(step) > Constant.MAX_RPM_STEP_PER_LOOP) {
            rawTarget = prevTargetVelocity + Math.signum(step) * Constant.MAX_RPM_STEP_PER_LOOP;
        }
        calculatedTargetVelocity = rawTarget;
        prevTargetVelocity = calculatedTargetVelocity;

        if (active) {
            double hoodServoPos = (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25) * (calculatedHoodAngle - 25) + Constant.HOOD_INIT;
            hoodServoPos= Math.max(Constant.HOOD_INIT, Math.min(Constant.HOOD_MAX, hoodServoPos));
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

        double currentVelo = rightShooter.getVelocity();
        double voltageComp = Constant.NOMINAL_VOLTAGE / battery.getVoltage();
        double ff = ((Constant.kV * calculatedTargetVelocity) + Constant.kS) * voltageComp;
        double error = Math.abs(currentVelo - calculatedTargetVelocity);

        shooterPID.setTargetPosition(calculatedTargetVelocity);
        double pidContribution = shooterPID.update(currentVelo);
        double totalPower = pidContribution + ff;

        totalPower = Math.max(0, Math.min(1.0, totalPower));
        if (error > 0.1 * calculatedTargetVelocity && calculatedTargetVelocity > currentVelo) {
            leftShooter.setPower(1);
            rightShooter.setPower(1);
        } else {
            leftShooter.setPower(totalPower);
            rightShooter.setPower(totalPower);
        }
    }

    public void setTurretPosition(double position){
        double calculatedTurretPos = position * (Constant.TURRET_MAX-Constant.TURRET_MIN) + Constant.TURRET_MIN;
        calculatedTurretPos = Math.max(Constant.TURRET_MIN,Math.min(Constant.TURRET_MAX, calculatedTurretPos));
        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);
    }

    public String detectMotif() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) return motif;

        List<LLResultTypes.FiducialResult> aprils = result.getFiducialResults();
        if (aprils == null || aprils.isEmpty()) return motif;

        for (LLResultTypes.FiducialResult april : aprils) {
            switch (april.getFiducialId()) {
                case 21: motif = "GPP"; break;
                case 22: motif = "PGP"; break;
                case 23: motif = "PPG"; break;
            }
        }
        return motif;
    }


    public boolean isReady() {
        double currentVelo = rightShooter.getVelocity();
        double voltageComp = Constant.NOMINAL_VOLTAGE / battery.getVoltage();
        double error = Math.abs(currentVelo - calculatedTargetVelocity);
        return calculatedTargetVelocity > 0 && (error < Constant.VELOCITY_TOLERANCE * (1/voltageComp) * Math.pow((2200/currentVelo),2));
    }
}