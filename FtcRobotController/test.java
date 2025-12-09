package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class shooter extends OpMode {
    private double integral = 0;
    private double derivative = 0;
    private double lastTime = 0, time = 0;
    private double position, lastPosition, lastVError;

    private double shooter_power = 0;

    private double verror;
    private DcMotor left_shooter, right_shooter;
    private Servo right_hood, left_pivot, right_pivot;
    private double hood_position = Constants.rightHoodINIT;
    private boolean shooterOn = false;
    private boolean lastRB = false;
    ElapsedTime timer = new ElapsedTime();

    PIDCalculator pid = new PIDCalculator(Constants.shooter_kP, Constants.shooter_kI, Constants.shooter_kD);

    public void init(){
        left_shooter = hardwareMap.get(DcMotor.class, "LeftShooterMotor");
        right_shooter = hardwareMap.get(DcMotor.class, "RightShooterMotor");
        right_shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        left_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_hood = hardwareMap.get(Servo.class, "RightHood");
        left_pivot = hardwareMap.get(Servo.class, "LeftPivot");
        right_pivot = hardwareMap.get(Servo.class, "RightPivot");
        lastTime = timer.seconds();
        lastPosition = left_shooter.getCurrentPosition();
        left_pivot.setPosition(Constants.leftPivotINIT);
        right_pivot.setPosition(Constants.rightPivotINIT);
        right_hood.setPosition(Constants.rightHoodINIT);
        pid.reset();
    }
    public void loop(){
        if (gamepad1.right_bumper && !lastRB) {
            shooterOn = !shooterOn;
            //shooter power PID
            time = getRuntime();
            position = left_shooter.getCurrentPosition();
            double dt = time - lastTime;
            double dp = position - lastPosition;
            double velocity = dp / dt;
            pid.calculator(Constants.targetVelocity, velocity, false);
            shooter_power = Math.abs(verror) > 30 ? shooter_power * 1 : 0;
            shooter_power = Math.min(Math.max(0, shooter_power), 1.0);
            //Reset variables for next PID calculation
            lastTime = time;
            lastPosition = position;
            lastVError = verror;
            left_shooter.setPower(shooterOn ? shooter_power : 0);
            right_shooter.setPower(shooterOn ? shooter_power : 0);
            telemetry.addData("Shooter", "On");
            telemetry.update();
        }
        lastRB = gamepad1.right_bumper;

        if (gamepad1.left_bumper){
            left_pivot.setPosition(Constants.leftPivotActivated);
            right_pivot.setPosition(Constants.rightPivotActivated);
        } else{
            left_pivot.setPosition(Constants.leftPivotINIT);
            right_pivot.setPosition(Constants.rightPivotINIT);
        }

        if (gamepad1.dpad_up){
            hood_position = Math.max(hood_position + 0.001, Constants.rightHoodMax);
            telemetry.addData("up?", true);

        } else if (gamepad1.dpad_down){
            hood_position = Math.min(hood_position - 0.001, Constants.rightHoodINIT);
            telemetry.addData("up?", false);
        }
        telemetry.addData("up?", "not pressed");
        right_hood.setPosition(hood_position);
        telemetry.update();


    }
}