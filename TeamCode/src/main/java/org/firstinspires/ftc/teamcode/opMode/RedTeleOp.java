package org.firstinspires.ftc.teamcode.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDCalculator;
import org.firstinspires.ftc.teamcode.opMode.Constant;

// ... all imports remain the same ...

@TeleOp(name = "RedTeleOp", group = "Main")
public class RedTeleOp extends OpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    // Mechanisms
    private DcMotor intake;
    private DcMotorEx leftShooter, rightShooter;
    private Servo leftPivot, rightPivot, rightHood;

    // IMU for heading
    private IMU imu;

    // Vision / sensors
    private HuskyLens huskylens;
    private VoltageSensor voltageSensor;

    // PID controllers
    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);

    // Toggles
    private boolean intakeOn = false, intakeOff = false;
    private boolean nearShooterOn = false, farShooterOn = false;
    private boolean lastX = false, lastY = false, lastRB = false, lastB = false, lastA = false;
    private boolean fieldCentric = false; // default to field-centric
    private double initialHeading = 0; // will set in init
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    private double rx = 0;
    private double targetAngle = -0.5;

    // Shooter
    private double hood_position = Constant.rightHoodINIT;
    private double velocity = 0, targetVelocity = 0, shooterPower = 0;
    private double distance = 0;
    private double botHeading = 0;
    private PIDCalculator aim = new PIDCalculator(1.5, 0, 0.3);

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        // ... hardware mapping same as before ...
        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
        rightHood = hardwareMap.get(Servo.class, "RightHood");
        imu = hardwareMap.get(IMU.class, "imu");
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // initial heading
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        pidFar.reset();
        pidNear.reset();

        rightHood.setPosition(Constant.rightHoodINIT);
        rightPivot.setPosition(Constant.rightPivotINIT);
        leftPivot.setPosition(Constant.leftPivotINIT);
    }

    @Override
    public void loop() {
        // --- Toggle field-centric / robot-centric (A button) ---
        if (gamepad1.a && !lastA) {
            fieldCentric = !fieldCentric;
            if (fieldCentric) {
                // store heading as field-zero when switching
                initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI/2;
            }
        }
        lastA = gamepad1.a;

        // --- Joystick inputs ---
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        // --- Field-centric adjustment ---


        if (nearShooterOn){
            HuskyLens.Block[] blocks = huskylens.blocks();
            if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
                distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
                targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
                rx = Math.abs(blocks[0].x - Constant.targetPosition) > 3 ? aiming.calculator(Constant.targetPosition, blocks[0].x) : 0;
            }
        } else if (farShooterOn){
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double pid = aim.calculator(targetAngle, botHeading);
            rx += Math.abs(botHeading - targetAngle) > 0.02 ? pid:0;
        }

        if (fieldCentric) {
            if (gamepad1.startWasPressed()) {
                imu.resetYaw();
            }

            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        }else {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
        }

        // --- Mecanum drive calculation ---
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // --- Intake toggles ---
        if (gamepad1.x && !lastX) {
            intakeOn = !intakeOn;
            intake.setPower(intakeOn ? 1.0 : 0.0);
        }
        lastX = gamepad1.x;

        if (gamepad1.b && !lastB) {
            intakeOff = !intakeOff;
            intake.setPower(intakeOff ? -1 : 0);
        }
        lastB = gamepad1.b;

        // --- Shooter toggles ---
        if (gamepad1.right_bumper && !lastRB) {
            farShooterOn = !farShooterOn;
            nearShooterOn = false;
            hood_position = 0.4;
            pidFar.reset();
        }
        lastRB = gamepad1.right_bumper;

        if (gamepad1.y && !lastY) {
            nearShooterOn = !nearShooterOn;
            farShooterOn = false;
            hood_position = 0.3;
            pidNear.reset();
        }
        lastY = gamepad1.y;

        // --- Hood & pivot ---
        if (gamepad1.left_bumper) {
            leftPivot.setPosition(Constant.leftPivotActivated);
            rightPivot.setPosition(Constant.rightPivotActivated);
        } else {
            leftPivot.setPosition(Constant.leftPivotINIT);
            rightPivot.setPosition(Constant.rightPivotINIT);
        }

        if (gamepad1.dpad_up) {
            hood_position = Math.min(hood_position + 0.005, Constant.rightHoodMax);
        } else if (gamepad1.dpad_down) {
            hood_position = Math.max(hood_position - 0.005, Constant.rightHoodINIT);
        }
        rightHood.setPosition(hood_position);

        // --- Shooter PID / HuskyLens aiming ---
        if (farShooterOn) {
            velocity = rightShooter.getVelocity();
            double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
            shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);
        } else if (nearShooterOn) {
            velocity = rightShooter.getVelocity();
            double output = pidNear.calculator(targetVelocity, velocity);
            shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);
        } else {
            shooterPower = 0.2;
        }

        leftShooter.setPower(shooterPower);
        rightShooter.setPower(shooterPower);

        // --- Telemetry ---
        telemetry.addData("Field-Centric", fieldCentric);
        telemetry.addData("Angle Error", Math.abs(botHeading - targetAngle));
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("rx", rx);
        telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Intake On", intakeOn);
        telemetry.update();
    }
}
