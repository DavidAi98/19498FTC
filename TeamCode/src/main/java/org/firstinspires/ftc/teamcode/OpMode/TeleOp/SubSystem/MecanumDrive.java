package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    Pose2D pos;
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public GoBildaPinpointDriver pinpoint;

    private double dx, dy;
    public double botX, botY;
    public double turretX, turretY;
    public double headingDeg, headingRad;
    public boolean resetingPos = false;

    public MecanumDrive(HardwareMap hwMap) {
        // Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "LeftFrontMotor");
        leftBack  = hwMap.get(DcMotor.class, "LeftBackMotor");
        rightFront = hwMap.get(DcMotor.class, "RightFrontMotor");
        rightBack  = hwMap.get(DcMotor.class, "RightBackMotor");

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

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, Constant.AUTON_LAST_X, Constant.AUTON_LAST_Y, AngleUnit.RADIANS, Constant.AUTON_LAST_HEADING_RAD));

    }

    public void update(double targetX, double targetY) {
        // elementry pos
        pinpoint.update();
        pos = pinpoint.getPosition();
        botX = pos.getX(DistanceUnit.INCH);
        botY = pos.getY(DistanceUnit.INCH);
        headingRad = pos.getHeading(AngleUnit.RADIANS);
        headingDeg = Math.toDegrees(headingRad);

        // advance calculations
        turretX = botX - Constant.TURRET_OFFSET * Math.cos(headingRad);
        turretY = botY - Constant.TURRET_OFFSET * Math.sin(headingRad);
        dx = turretX - targetX;
        dy = turretY - targetY;

        // reset pos
        if (resetingPos) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            resetingPos = false;
        }
    }

    // Mecanum drive
    public void drive(double y, double x, double rx, boolean fieldCentric) {

        if (fieldCentric) {
            double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = rotX;
            y = rotY;
        }
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        leftFront.setPower( (y + x + rx) / denom );
        leftBack.setPower( (y - x + rx) / denom );
        rightFront.setPower( (y - x - rx) / denom );
        rightBack.setPower( (y + x - rx) / denom );
    }

    // --- Helper methods ---
    public double distanceToGoal() {
        return Math.hypot(dx,dy);
    }
    public double angleToGoal() {
        double referenceAngle = -Math.toDegrees(Math.atan2(dy, dx)) + 90;

        return referenceAngle + headingDeg;
    }
}