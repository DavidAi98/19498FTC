
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opMode.Constant;


@TeleOp(name = "ShooterTurningPlusProMaxUltra")
@Config
public class ShooterTurning extends OpMode {

    // === Dashboard adjustable values ===

    public static double kV = 0.0;
    public static double kP = 0.0;
    public static double kS = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static int targetVelocity =0;
    private DcMotorEx leftShooter, rightShooter;
    private boolean intakeOn = false;
    private Servo leftPivot, rightPivot;
    private VoltageSensor voltageSensor;
    private Servo rightHood;
    private double hood_position = Constant.rightHoodINIT;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private DcMotor frontLeft, frontRight, backLeft, backRight, intake;

    private PIDCalculator pid;
    private boolean shooterOn = false;

    private IMU imu;


    @Override
    public void init() {
        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        rightHood = hardwareMap.get(Servo.class, "RightHood");

        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightHood = hardwareMap.get(Servo.class, "RightHood");
        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientation));
        pid = new PIDCalculator(kP, kI, kD);
        pid.reset();
    }

    @Override
    public void loop() {


        double botHeading = -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double voltage = voltageSensor.getVoltage();

        double currentVelocity = leftShooter.getVelocity();

        double feedforward = kV * targetVelocity*Constant.voltageStand / voltage;

        double feedback = pid.calculator(targetVelocity, currentVelocity);

        double motorPower = feedforward + feedback + kS;

        // Clamp motor power to valid range (-1 to 1)
        motorPower = Math.max(0, Math.min(1.0, motorPower));

        if (gamepad1.yWasPressed()) {
            shooterOn = !shooterOn;
            rightHood.setPosition(0.3);
        }
        // Apply power to the motor
        leftShooter.setPower(shooterOn?motorPower:0);
        rightShooter.setPower(shooterOn?motorPower:0);

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower(0.9 * (y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower(0.9 * (y + x - rx) / denominator);

        if (gamepad1.xWasPressed()){
            intakeOn = !intakeOn;
            intake.setPower(intakeOn?1:0);
        }
        if (gamepad1.left_bumper) {
            leftPivot.setPosition(Constant.leftPivotActivated);
            rightPivot.setPosition(Constant.rightPivotActivated);
        } else {
            leftPivot.setPosition(Constant.leftPivotINIT);
            rightPivot.setPosition(Constant.rightPivotINIT);
        }

        if (gamepad1.dpad_up){
            hood_position = Math.min(hood_position + 0.005, Constant.rightHoodMax);
            telemetry.addData("up?", true);

        } else if (gamepad1.dpad_down){
            hood_position = Math.max(hood_position - 0.005, Constant.rightHoodINIT);
            telemetry.addData("up?", false);
        }
        rightHood.setPosition(hood_position);

        // Telemetry for debugging
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("kV", kV);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("PID",feedback);
        telemetry.addData("Shooter", shooterOn);
        telemetry.addData("Voltage", voltage);
        telemetry.addData("Botheading", botHeading);
        telemetry.update();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Velocity", targetVelocity);
        packet.put("Target Velocity for FF",targetVelocity*Constant.voltageStand / voltage);
        packet.put("Current Velocity", currentVelocity);
        packet.put("Motor Power", motorPower);
        packet.put("kV", kV);
        packet.put("kP", kP);
        packet.put("kS", kS);
        packet.put("Voltage", voltage);

        dashboard.sendTelemetryPacket(packet);


    }
}
