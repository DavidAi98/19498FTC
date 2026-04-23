package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Leave Auton")
public class leave extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {

            sleep(27500);

            leftFront.setPower(1);
            leftBack.setPower(1);
            rightFront.setPower(1);
            rightBack.setPower(1);

            sleep(300);

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            requestOpModeStop();
        }
    }
}
