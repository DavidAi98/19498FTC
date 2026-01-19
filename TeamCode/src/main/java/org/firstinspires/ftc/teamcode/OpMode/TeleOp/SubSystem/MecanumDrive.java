package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public GoBildaPinpointDriver pinpoint;
    private double vx, vy;
    private double dx, dy;
    private double dx2, dy2;
    private double headingDeg;
    Pose2D pos;

    public MecanumDrive(HardwareMap hwMap) {
        // Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "LeftFrontMotor");
        leftBack = hwMap.get(DcMotor.class, "LeftBackMotor");
        rightFront = hwMap.get(DcMotor.class, "RightFrontMotor");
        rightBack = hwMap.get(DcMotor.class, "RightBackMotor");

        // Set Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set Brake Behavior
        DcMotor[] motors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Initialize Pinpoint Odometry
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constant.ODO_X_OFFSET, Constant.ODO_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setYawScalar(Constant.ODO_YAW_SCALAR);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    public void updateRelativePostion(double targetX, double targetY) {
        pos = pinpoint.getPosition();
        vx = pinpoint.getVelX(DistanceUnit.INCH);
        vy = pinpoint.getVelY(DistanceUnit.INCH);
        dx = pos.getX(DistanceUnit.INCH) - targetX;
        dy = pos.getY(DistanceUnit.INCH) - targetY;
        headingDeg = pos.getHeading(AngleUnit.DEGREES);
        // TBD for use in moving while shooting
        dx2 = dx + vx * Constant.deltaTime;
        dy2 = dy + vy * Constant.deltaTime;
    }

    /**
     * Standard Mecanum Drive Method
     */
    public void drive(double y, double x, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            double rotX = x * Math.cos(-headingDeg) - y * Math.sin(-headingDeg);
            double rotY = x * Math.sin(-headingDeg) + y * Math.cos(-headingDeg);
            x = rotX;
            y = rotY;
        }

        // Normalize powers so no motor exceeds 1.0
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((y + x + rx) / denom);
        leftBack.setPower((y - x + rx) / denom);
        rightFront.setPower((y - x - rx) / denom);
        rightBack.setPower((y + x - rx) / denom);
    }

    /**
     * Calculates distance from current position to a target coordinate
     */
    public double distanceToGoal() {
        return Math.hypot(dx, dy);
        // return Math.hypot(dx2, dy2);
    }

    /**
     * Calculates the angle the robot needs to face to look at the goal
     */
    public double getAngleToGoal() {
        double referenceAngle = -Math.toDegrees(Math.atan2(dy, dx)) + 90;
        // double referenceAngle = -Math.toDegrees(Math.atan2(dy2, dx2)) + 90;
        return referenceAngle + headingDeg;
    }
}