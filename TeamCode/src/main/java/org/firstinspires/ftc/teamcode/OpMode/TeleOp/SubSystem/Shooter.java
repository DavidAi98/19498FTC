package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

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
    public double limelightDistanceInch = -1;
    public double last_Distance = -1;
    public double filteredAprilX, aprilx;
    private boolean motifDetected = false;


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
                    limelightDistanceInch = limelightDistanceMeter * 37.5;
                    linearInterpolation(odoDistance);
                    last_Distance = limelightDistanceInch;
                    break;
                }
            }
        } else {
//            filteredAprilX = 0;
            aprilx = 0;
        }
    }

    public void updateTurret(double rawTurretAngle) {
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

        if (leftShooter.getVelocity() > 100) {
            double hoodServoPos = (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25) * (calculatedHoodAngle - 45) + Constant.HOOD_MAX;
            hood.setPosition(hoodServoPos);
        } else {
            hood.setPosition(Constant.HOOD_INIT);
        }
    }

    public void runShooter(boolean active) {
        if (!active) {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
            return;
        }

        double currentVelo = leftShooter.getVelocity();
        double voltageComp = Constant.NOMINAL_VOLTAGE / battery.getVoltage();
        double ff = ((Constant.kV * calculatedTargetVelocity) + Constant.kS) * voltageComp;

        shooterPID.setTargetPosition(calculatedTargetVelocity);
        double pidContribution = shooterPID.update(currentVelo);
        double totalPower = pidContribution + ff;
        totalPower = Math.max(0, Math.min(1.0, totalPower));

        leftShooter.setPower(totalPower);
        rightShooter.setPower(totalPower);
    }

    public void setTurretPosition(double position){
        double calculatedTurretPos = position * (Constant.TURRET_MAX-Constant.TURRET_MIN) + Constant.TURRET_MIN;
        calculatedTurretPos = Math.max(Constant.TURRET_MIN,Math.min(Constant.TURRET_MAX, calculatedTurretPos));
        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);
    }
    public void setHoodPosition(double position){
        hood.setPosition(position);
    }

    public String detectMotif() {

        String motif = "null";
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> aprils = result.getFiducialResults();

        if (!aprils.isEmpty()) {
            for (LLResultTypes.FiducialResult april : aprils)
                switch (april.getFiducialId()) {
                    case 21:
                        motif = "GPP";
                        break;
                    case 22:
                        motif = "PGP";
                        break;
                    case 23:
                        motif = "PPG";
                        break;
                }

        }
        return motif;


    }


    public boolean isReady() {
        double error = Math.abs(leftShooter.getVelocity() - calculatedTargetVelocity);
        return calculatedTargetVelocity > 0 && ((error <= 0.075 * leftShooter.getVelocity()) || (error <= 80));
    }
}