package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends OpMode {

    // --- Hardware ---
    private DcMotor leftFront, rightFront, leftBack, rightBack, intake;
    private DcMotorEx leftShooter, rightShooter;
    private Servo spindexer1, spindexer2, leftPivot, rightPivot, turret1, turret2, hood;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private GoBildaPinpointDriver odo;
    private VoltageSensor batteryVoltageSensor;
    private Limelight3A limelight;

    // --- Control Objects ---
    private PIDFController shooterPID;

    // --- Anti-Silly / Debounce Latches ---
    private boolean prevA1 = false;
    private boolean prevLB2 = false;
    private boolean prevY2 = false;

    // --- State Machine Variables ---
    private ArrayList<Artifact> artifacts = new ArrayList<>();
    private int intakeIndex = 0;
    private int outtakeIndex = 1;
    private double currentSpindexerPos;
    private String targetColor = "";
    private int nearestIndex = -1;
    private double nearestPosition = Double.MAX_VALUE;

    public static int intakeStage = 1;
    public static int outtakeStage = -1;
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean endGame = false;
    private int colorCount = 0;
    private boolean intakeReverse = false;
    private int inventory = 0;

    // Timers
    private ElapsedTime intakeTime = new ElapsedTime();
    private ElapsedTime pivotTime = new ElapsedTime();
    private ElapsedTime outtakeTime = new ElapsedTime();
    private ElapsedTime shooterTime = new ElapsedTime();

    // --- Drive & Auto-Aim Variables ---
    private boolean isFieldCentric = false;
    private double calculatedTargetVelocity = 0.0;
    private double calculatedHoodAngle = 0.0;
    public double aprilx;
    private double filteredAprilX;
    private double ty;
    double limelightDistanceMeter;
    double limelightDistanceInch;
    double last_Distance;
    private double currentVelo = 0;
    private int outtakeSpindexerTime = 0;


    @Override
    public void init() {
        // Drivetrain
        leftFront = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        rightFront = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        rightBack = hardwareMap.get(DcMotor.class, "RightBackMotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Spindexer & Intake
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        spindexer1 = hardwareMap.get(Servo.class, "spindexer1");
        spindexer2 = hardwareMap.get(Servo.class, "spindexer2");
        spindexer1.setPosition(Constant.SPINDEXER_INIT);
        spindexer2.setPosition(Constant.SPINDEXER_INIT);

        // Pivot
        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
        leftPivot.setPosition(Constant.PIVOT_DOWN);
        rightPivot.setPosition(Constant.PIVOT_DOWN);

        // Turret & Hood
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood = hardwareMap.get(Servo.class, "RightHood");
        hood.setPosition(Constant.HOOD_INIT);

        // Shooter
        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Control
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        shooterPID = new PIDFController(new PIDCoefficients(Constant.kP, Constant.kI, Constant.kD));
        spindexer1.setPosition(Constant.SPINDEXER_INTAKE_POS1);
        spindexer2.setPosition(Constant.SPINDEXER_INTAKE_POS1);


        // Pinpoint Odo
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(Constant.ODO_X_OFFSET, Constant.ODO_Y_OFFSET, DistanceUnit.INCH);
        odo.setEncoderResolution(Constant.ODO_TICK_PER_UNIT, DistanceUnit.INCH);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setYawScalar(Constant.ODO_YAW_SCALAR);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // Camera
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready to Start");
        telemetry.update();

        // re init everything
        intakeStage = 1;
        outtakeStage = -1;
        currentSpindexerPos = Constant.SPINDEXER_INTAKE_POS1;
        shooterOn = false;
        inventory = 0;
        intakeIndex = 0;
        outtakeIndex = 1;
        last_Distance =-1;
    }

    @Override
    public void loop() {
        // --- 1. LOCALIZATION & TARGETING ---

        odo.update();
        Pose2D pos = odo.getPosition();
        double posX = pos.getX(DistanceUnit.INCH);
        double posY = pos.getY(DistanceUnit.INCH);
        double headingRad = pos.getHeading(AngleUnit.RADIANS);
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);

//        double xVel = odo.getVelX(DistanceUnit.INCH);       // Velocity in mm/sec (default)
//        double yVel = odo.getVelY(DistanceUnit.INCH);       // Velocity in mm/sec
//        double hVel = odo.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized());

        double dx = posX - Constant.GOAL_CENTER_X;
        double dy = posY - Constant.BLUE_GOAL_CENTER_Y;
        double odoDistance = Math.hypot(dx, dy);

        // Turret Math + LimeLight
        double referenceAngle = -Math.toDegrees(Math.atan2(dy, dx)) + 90; // field to turret frame
        double turretHeading = referenceAngle + headingDeg; // relative to robot
        filteredAprilX += aprilx * 0.08;

        turretHeading += filteredAprilX; // fine adjustment from Limelight
        double turretRange = Constant.TURRET_RIGHT_MAX - Constant.TURRET_LEFT_MAX;
        double normalizedHeading = turretHeading;
        LLResult results = limelight.getLatestResult();

        if (results.isValid()) {
            List<LLResultTypes.FiducialResult> detection = results.getFiducialResults();

            for (LLResultTypes.FiducialResult april : detection) {
                if (april.getFiducialId() == 20) {
                    aprilx = april.getTargetXDegrees() + Constant.magicConstant;
                    ty = april.getTargetYDegrees();
                }
            }

            limelightDistanceMeter = (Constant.hTarget - Constant.hCamera) / Math.tan(Math.toRadians(Constant.cameraAngle + ty));
            limelightDistanceInch = 1.48380597*Math.pow(limelightDistanceMeter,2)+26.65271153*(limelightDistanceMeter)+20;
            interpolateShootingParams(limelightDistanceInch);
            last_Distance = limelightDistanceInch;
        } else {
            aprilx = 0;
            if (last_Distance != -1) {
                interpolateShootingParams(last_Distance);
            } else {
                interpolateShootingParams(odoDistance);
            }
        }

        if (normalizedHeading > 360) normalizedHeading -= 360;
        if (normalizedHeading < 0) normalizedHeading += 360;
        double calculatedTurretPos =
                Constant.TURRET_LEFT_MAX + (normalizedHeading / 360.0) * turretRange;
        calculatedTurretPos = Math.max(Constant.TURRET_LEFT_MAX,
                Math.min(Constant.TURRET_RIGHT_MAX, calculatedTurretPos));

        turret1.setPosition(calculatedTurretPos - Constant.TURRET_ANTIBACKLASH);
        turret2.setPosition(calculatedTurretPos + Constant.TURRET_ANTIBACKLASH);

        // Update Target Velocity and Hood Angle from Lookup Table

        // --- 2. DRIVETRAIN ---
        if (gamepad1.a && !prevA1) {
            isFieldCentric = !isFieldCentric;
            odo.resetPosAndIMU();
            filteredAprilX = 0;
        }
        if (gamepad1.x) {
            odo.resetPosAndIMU();
            filteredAprilX = 0;
        }
        prevA1 = gamepad1.a;

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double driveRX = gamepad1.right_stick_x;

        if (isFieldCentric) {
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

        // --- 3. INTAKE STATE MACHINE ---
        if (gamepad1.rightBumperWasPressed() && artifacts.size() < 3) {
            intakeOn = !intakeOn;
            intakeReverse = false;
            if (!intakeOn){
                intakeIndex = artifacts.size();
                intakeStage = 1;
                //artifacts.clear();
            } else{
                intakeStage = 0;
            }
            if (outtakeStage != -1) outtakeStage = -1;
        }

        if (gamepad1.b) {
            intakeReverse = true;
            intakeOn = false;
        } else {
            intakeReverse = false;
        }

        if (intakeOn) {
            intake.setPower(Constant.INTAKE_POWER);
        } else if (intakeReverse) {
            intake.setPower(Constant.INTAKE_REVERSE);
        } else {
            intake.setPower(0);
        }


        // --- 4. OUTTAKE STATE MACHINE ---
        if (gamepad2.yWasPressed()) {
            shooterOn = !artifacts.isEmpty() && !shooterOn;
            outtakeStage = shooterOn ? endGame ? 0 : 1 : -1;
            outtakeSpindexerTime = shooterOn ? endGame ? 350 : 200 : 0;
            intakeStage = -1;
        }

        switch (intakeStage) {
            case 0: // Detection
                if (artifacts.size() < 3) {
                    double b = colorSensor.blue();
                    double g = colorSensor.green();
                    if (b >= 100 && g >= 100) {
                        String detected = (b > g) ? "P" : "G";
                        double tPos = (intakeIndex == 1) ? Constant.SPINDEXER_OUTTAKE_POS1 :
                                (intakeIndex == 2) ? Constant.SPINDEXER_OUTTAKE_POS2 : Constant.SPINDEXER_OUTTAKE_POS3;
                        artifacts.add(new Artifact(detected, tPos));
                        intakeStage = 1;
                    }
                    currentSpindexerPos = (intakeIndex == 1) ? Constant.SPINDEXER_INTAKE_POS1 :
                            (intakeIndex == 2) ? Constant.SPINDEXER_INTAKE_POS2 : Constant.SPINDEXER_INTAKE_POS3;
                }
                break;
            case 1: // Indexing
                intakeIndex = (intakeIndex % 3) + 1;
                double nPos = (intakeIndex == 1) ? Constant.SPINDEXER_INTAKE_POS1 :
                        (intakeIndex == 2) ? Constant.SPINDEXER_INTAKE_POS2 : Constant.SPINDEXER_INTAKE_POS3;
                spindexer1.setPosition(nPos);
                spindexer2.setPosition(nPos);
                intakeTime.reset();
                intakeStage = 2;
                break;
            case 2:
                if (!intakeOn || artifacts.size() == 3 || artifacts.isEmpty()){
                    intakeStage = -1;
                    currentSpindexerPos = Constant.SPINDEXER_INTAKE_POS3;
                } else if (intakeTime.milliseconds() >= 300){
                    intakeStage = 0;
                }
                break;
            case -1:
                intakeOn = false;
                break;
        }

        if (gamepad2.aWasPressed()) {
            endGame = !endGame;
        }
        if (intakeStage == -1 && outtakeStage == -1 && endGame) {
            if (colorCount < artifacts.size()) {
                if (gamepad2.dpadUpWasPressed()) {
                    targetColor += "P";
                    colorCount++;
                }
                if (gamepad2.dpadDownWasPressed()) {
                    targetColor += "G";
                    colorCount++;
                }
            }

        }


        // --- 5. SHOOTER POWER ---
        double targetVelo = (outtakeStage != -1) ? calculatedTargetVelocity : 0.0;
        applyShooterPower(targetVelo);

        switch (outtakeStage) {
            case 0: // Setup Motif
                if (targetColor.isEmpty() && artifacts.size() == 3) targetColor = Constant.motif;
                outtakeStage = 1;
                break;
            case 1: // Sorting Logic
                if (outtakeIndex <= artifacts.size() && endGame) {
                    Artifact art = artifacts.get(outtakeIndex - 1);
                    boolean match = !targetColor.isEmpty() && art.getColor().equals(targetColor.substring(0, 1));
                    if (match && Math.abs(nearestPosition - currentSpindexerPos) > Math.abs(art.getPosition() - currentSpindexerPos)) {
                        nearestIndex = outtakeIndex;
                        nearestPosition = art.getPosition();
                    }
                    outtakeIndex++;
                } else {
                    if (nearestIndex == -1) {
//                        switch (artifacts.size()){
//                            case 3:
//                                nearestIndex = 3;
//                                break;
//                            case 2:
//                                nearestIndex = inventory == 3 ? 1 : 2;
//                                break;
//                            case 1:
//                                nearestIndex = 1;
//                                break;
//                        }
                        nearestIndex = artifacts.size() == 3 ? 3 : 1;
                        nearestPosition = artifacts.get(nearestIndex - 1).getPosition();
                    }
                    outtakeStage = 2;
                }
                break;
            case 2: // Align
                spindexer1.setPosition(nearestPosition);
                spindexer2.setPosition(nearestPosition);
                outtakeTime.reset();
                outtakeStage = 3;
                break;
            case 3: // Fire (Wait for LB)
                if (outtakeTime.milliseconds() >= outtakeSpindexerTime && Math.abs(currentVelo - calculatedTargetVelocity) <= 80 && gamepad2.left_bumper) {
                    targetColor = targetColor.length() > 1 ? targetColor.substring(1) : "";
                    if (nearestIndex != -1) artifacts.remove(nearestIndex - 1);
                    currentSpindexerPos = artifacts.isEmpty() ? 0 : nearestPosition;
                    nearestIndex = -1;
                    nearestPosition = Double.MAX_VALUE;
                    outtakeIndex = 1;
                    leftPivot.setPosition(Constant.LEFT_PIVOT_UP);
                    rightPivot.setPosition(Constant.RIGHT_PIVOT_UP);
                    pivotTime.reset();
                    outtakeStage = 4;
                }
                break;
            case 4: // Reset Pivot
                if (pivotTime.milliseconds() >= 200) {
                    leftPivot.setPosition(Constant.PIVOT_DOWN);
                    rightPivot.setPosition(Constant.PIVOT_DOWN);
                    outtakeStage = 5;
                }
                break;
            case 5: // Loop or End
                if (pivotTime.milliseconds() >= 400) {
                    if (artifacts.isEmpty()) {
                        outtakeStage = 6;
                        shooterTime.reset();
                    } else {
                        outtakeStage = 1;
                    }
                }
                break;
            case 6:
                if (shooterTime.milliseconds() >= 500){
                    outtakeStage = -1;
                    intakeStage = 1;
                    intakeIndex = 0;
                    inventory = 0;
                }
                break;
            case -1:
                shooterOn = false;
                colorCount = 0;
                break;
        }
        prevLB2 = gamepad2.left_bumper;


        telemetry.addData("Field Centric", isFieldCentric);
        telemetry.addData("filteredAprilX", filteredAprilX);
        telemetry.addData("referenceAngle", referenceAngle);
        telemetry.addData("headingDeg", headingDeg);
        telemetry.addData("target velo", targetVelo);
        telemetry.addData("target velo error", targetVelo-currentVelo);
        telemetry.addData("Distance (odo)", "%.2f", odoDistance);
        telemetry.addData("Distance (ll)", "%.2f", limelightDistanceInch);
        telemetry.addData("X coordinate", posX);
        telemetry.addData("Y coordinate", posY);
        telemetry.addData("array", artifacts);
        telemetry.addData("Intake Stage", intakeStage);
        telemetry.addData("Outtake Stage", outtakeStage);
        telemetry.addData("ty", ty);
        telemetry.addData("Distance (ll) new",limelightDistanceMeter*39.37);
//        telemetry.addData("Inventory", inventory);

//        telemetry.addData("Robot Heading", headingDeg);
//        telemetry.addData("Spindexer count", spindexerCount);
//        telemetry.addData("Turret heading", turretHeading);
//        telemetry.addData("adj angle", aprilx);
//        telemetry.addData("Nearest Index", nearestIndex);
        telemetry.update();
    }

    private void interpolateShootingParams(double distance) {
        Map.Entry<Double, double[]> low = Constant.SHOOTING_TABLE.floorEntry(distance);
        Map.Entry<Double, double[]> high = Constant.SHOOTING_TABLE.ceilingEntry(distance);

        if (low != null && high != null && !low.equals(high)) {
            double distDiff = high.getKey() - low.getKey();
            double factor = (distance - low.getKey()) / distDiff;
            calculatedTargetVelocity = low.getValue()[0] + (high.getValue()[0] - low.getValue()[0]) * factor;
            calculatedHoodAngle = low.getValue()[1] + (high.getValue()[1] - low.getValue()[1]) * factor;
        } else if (low != null) {
            calculatedTargetVelocity = low.getValue()[0];
            calculatedHoodAngle = low.getValue()[1];
        }

        double hoodServoPos = (Constant.HOOD_MAX - Constant.HOOD_INIT) / (45 - 25) * (calculatedHoodAngle - 45) + Constant.HOOD_MAX;
        if (calculatedTargetVelocity != 0) {
            hood.setPosition(hoodServoPos);
        }
    }

    private void applyShooterPower(double targetVelo) {
        currentVelo = leftShooter.getVelocity();
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double ff = (Constant.kV * targetVelo) + (targetVelo > 0 ? Constant.kS : 0);
        shooterPID.setTargetPosition(targetVelo);
        double pid = shooterPID.update(currentVelo);
        double power = (pid + ff) * (Constant.NOMINAL_VOLTAGE / currentVoltage);
        double safePower = Math.max(0, Math.min(1.0, power));
        leftShooter.setPower(safePower);
        rightShooter.setPower(safePower);
    }
}