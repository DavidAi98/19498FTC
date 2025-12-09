package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "MeasureTicksToInches", group = "Calibration")
public class measureOWheel extends LinearOpMode {
    private long prevLeft = 0, prevRight = 0, prevCenter = 0;
    DcMotorEx leftOdom, rightOdom ,strafeOdom;
    double headingRadians=0;
    double TRACK_WIDTH=9.875;

    @Override
    public void runOpMode() throws InterruptedException {
        // Replace these names with your actual odometry encoder names
        leftOdom = hardwareMap.get(DcMotorEx.class, "LeftBackMotor");
        rightOdom = hardwareMap.get(DcMotorEx.class, "RightFrontMotor");
        strafeOdom = hardwareMap.get(DcMotorEx.class, "BackOdometry");

        // Reset encoders
        leftOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.update();

        waitForStart();

        // Wait until you press STOP on the driver station
        while (opModeIsActive()) {

            updateHeading();

            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(headingRadians));
            telemetry.addData("Heading (rad)", "%.4f", headingRadians);
            telemetry.addData("Encoders L,R,C", "%d, %d, %d",
                    leftOdom.getCurrentPosition(),
                    rightOdom.getCurrentPosition(),
                    strafeOdom.getCurrentPosition());
            telemetry.update();
            idle();
        }


        // give time to read
    }

    private void updateHeading() {
        long currLeft = leftOdom.getCurrentPosition();
        long currRight = rightOdom.getCurrentPosition();
        long currCenter = strafeOdom.getCurrentPosition();

        double dLeft = (currLeft - prevLeft);
        double dRight = (currRight - prevRight);

        prevLeft = currLeft;
        prevRight = currRight;
        prevCenter = currCenter;

        // Compute change in heading
        double dTheta = (dRight - dLeft) / TRACK_WIDTH;

        // Accumulate heading
        headingRadians += dTheta;
    }
}

//
//
//@TeleOp(name = "ReturnToOrigin_Odometry")
//public class measureOWheel extends LinearOpMode {
//
//    // 驱动电机（四轮麦轮）
//    DcMotorEx frontLeft, frontRight, backLeft, backRight;
//
//    // 三个Odometry轮
//    DcMotorEx leftOdom, rightOdom, centerOdom;
//
//    // 里程计参数
//    static final double TICKS_PER_REV = 8192;
//    static final double WHEEL_DIAMETER_IN = 1.5;
//    static final double TRACK_WIDTH = 9.875;
//    static final double GEAR_RATIO = 1.0;
//    final double TICKS_TO_INCH = (Math.PI * WHEEL_DIAMETER_IN * GEAR_RATIO) / TICKS_PER_REV;
//
//    // 当前位置
//    double x = 0, y = 0, heading = 0;
//    long prevL = 0, prevR = 0, prevC = 0;
//
//    // PID / 比例参数
//    double kPos = 0.05;    // 位置比例系数
//    double kTurn = 0.02;   // 角度比例系数
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // 映射驱动电机
//        frontLeft = hardwareMap.get(DcMotorEx.class, "LeftFrontMotor");
//        frontRight = hardwareMap.get(DcMotorEx.class, "RightFrontMotor");
//        backLeft = hardwareMap.get(DcMotorEx.class, "LeftBackMotor");
//        backRight = hardwareMap.get(DcMotorEx.class, "RightBackMotor");
//
//        // 映射三个里程计
//        leftOdom = hardwareMap.get(DcMotorEx .class, "LeftBackMotor");
//        rightOdom = hardwareMap.get(DcMotorEx.class, "RightBackMotor");
//        centerOdom = hardwareMap.get(DcMotorEx.class, "BackOdometry");
//
//        // 初始化编码器
//        leftOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        centerOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        leftOdom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightOdom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        centerOdom.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        prevL = leftOdom.getCurrentPosition();
//        prevR = rightOdom.getCurrentPosition();
//        prevC = centerOdom.getCurrentPosition();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            updatePose();
//
//            // ------ 计算误差 ------
//            double targetX = 0;
//            double targetY = 0;
//            double targetHeading = 0;
//
//            double errorX = targetX - x;
//            double errorY = targetY - y;
//            double errorHeading = targetHeading - heading;
//
//            // ------ 转换到机器人坐标系 ------
//            double cosH = Math.cos(heading);
//            double sinH = Math.sin(heading);
//            double robotX = errorX * cosH + errorY * sinH;
//            double robotY = -errorX * sinH + errorY * cosH;
//
//            // ------ 计算控制输出 ------
//            double vx = robotY * kPos;      // 前后方向
//            double vy = robotX * kPos;      // 侧向
//            double omega = errorHeading * kTurn;  // 转动
//
//            // ------ 驱动输出（Mecanum标准分配）
//            double fl = vx + vy + omega;
//            double fr = vx - vy - omega;
//            double bl = vx - vy + omega;
//            double br = vx + vy - omega;
//
//            frontLeft.setPower(fl);
//            frontRight.setPower(fr);
//            backLeft.setPower(bl);
//            backRight.setPower(br);
//
//            telemetry.addData("x (in)", x);
//            telemetry.addData("y (in)", y);
//            telemetry.addData("heading (deg)", Math.toDegrees(heading));
//            telemetry.addData("errorDist", Math.hypot(errorX, errorY));
//            telemetry.update();
//        }
//    }
//
//    private void updatePose() {
//        long curL = leftOdom.getCurrentPosition();
//        long curR = rightOdom.getCurrentPosition();
//        long curC = centerOdom.getCurrentPosition();
//
//        double dL = (curL - prevL);
//        double dR = (curR - prevR);
//        double dC = (curC - prevC);
//
//        prevL = curL;
//        prevR = curR;
//        prevC = curC;
//
//        double dTheta = (dR - dL) / TRACK_WIDTH;
//        double dx = (dL + dR) / 2.0;
//        double dy = dC;
//
//        // 转到全局坐标
//        double sinH = Math.sin(heading);
//        double cosH = Math.cos(heading);
//
//        x += dx * cosH - dy * sinH;
//        y += dx * sinH + dy * cosH;
//        heading += dTheta;
//    }
//}