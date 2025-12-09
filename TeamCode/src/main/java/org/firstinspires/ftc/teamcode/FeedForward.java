////kV first, then kP
//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.opMode.Constant;
//
//@TeleOp(name = "Configure Feedforward")
//@Config
//public class FeedForward extends OpMode {
//
//    // === Dashboard adjustable values ===
//    private boolean shooterOn = false;
//    public static double kV = 0.0;
//    public static double kP = 0.0;
//    public static double kS = 0.0;
//    public static int targetVelocity =0;
//    private DcMotorEx leftShooter, rightShooter;
//    private boolean intakeOn = false;
//    private Servo leftPivot, rightPivot;
//    private VoltageSensor voltageSensor;
//    private Servo rightHood;
//    private double hood_position = Constant.rightHoodINIT;
//    private FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    private DcMotor intake;
//    @Override
//    public void init() {
//        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
//        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//
//        // Set zero power behavior (e.g., BRAKE or FLOAT)
//        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        // Disable built-in velocity control to allow direct power setting
//        leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//    }
//
//    @Override
//    public void loop(){
//        if (gamepad1.yWasPressed()) {
//            shooterOn = !shooterOn;
//            rightHood.setPosition(0.3);
//        }
//
//        double voltage = voltageSensor.getVoltage();
//        // Get current velocity from the motor's encoder
//        double currentVelocity = leftShooter.getVelocity();
//
//        // Calculate feedforward component
//        double feedforward = Constant.kV_near_standard * targetVelocity;
//
//        // Calculate proportional feedback component
//        double feedback = kP * (targetVelocity - currentVelocity);
//        // Combine feedforward and feedback for the total motor power
//        double motorPower = feedforward + feedback + kS;
//
//        // Clamp motor power to valid range (-1 to 1)
//        motorPower = Math.max(0, Math.min(1.0, motorPower));
//        motorPower = Math.min(motorPower*11.67 / voltage, 1);
//
//        // Apply power to the motor
//        leftShooter.setPower(shooterOn?motorPower:0);
//        rightShooter.setPower(shooterOn?motorPower:0);
//
//        if (gamepad1.xWasPressed()){
//            intakeOn = !intakeOn;
//            intake.setPower(intakeOn?1:0);
//        }
//        if (gamepad1.left_bumper) {
//            leftPivot.setPosition(Constant.leftPivotActivated);
//            rightPivot.setPosition(Constant.rightPivotActivated);
//        } else {
//            leftPivot.setPosition(Constant.leftPivotINIT);
//            rightPivot.setPosition(Constant.rightPivotINIT);
//        }
//        // Telemetry for debugging
//        telemetry.addData("Target Velocity", targetVelocity);
//        telemetry.addData("Current Velocity", currentVelocity);
//        telemetry.addData("Motor Power", motorPower);
//        telemetry.addData("kP", kP);
//        telemetry.addData("kV", kV);
//        telemetry.addData("Shooter", shooterOn);
//        telemetry.addData("Voltage", voltage);
//        telemetry.update();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Target Velocity", targetVelocity);
//        packet.put("Current Velocity", currentVelocity);
//        packet.put("Motor Power", motorPower);
//        packet.put("kV", kV);
//        packet.put("kP", kP);
//        packet.put("kS", kS);
//        packet.put("Voltage", voltage);
//        dashboard.sendTelemetryPacket(packet);
//        if (gamepad1.dpad_up){
//            hood_position = Math.min(hood_position + 0.005, Constant.rightHoodMax);
//            telemetry.addData("up?", true);
//
//        } else if (gamepad1.dpad_down){
//            hood_position = Math.max(hood_position - 0.005, Constant.rightHoodINIT);
//            telemetry.addData("up?", false);
//        }
//        rightHood.setPosition(hood_position);
//    }
//
//}