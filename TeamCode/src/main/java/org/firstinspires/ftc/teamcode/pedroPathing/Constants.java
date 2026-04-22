package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(11.75)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.11870544538707178, 0.002052759378465075))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.18, 0))
            .headingPIDFSwitch(10)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.15,0.015));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("RightFrontMotor")
            .rightRearMotorName("RightBackMotor")
            .leftRearMotorName("LeftBackMotor")
            .leftFrontMotorName("LeftFrontMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.68)
            .yVelocity(61.81) //(54.4996131236159 + 55.765637495386315 + 55.52764892578125 + 53.797435850609006) / 4
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);






    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.3622047) //4.7744488
            .strafePodX(-4.7744488) //-2.3622047
            .yawScalar(Constant.ODO_YAW_SCALAR)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.95,//0.995
            0.1,
            0.5,
            0.009,
            10,
            0.5,
            10,
            1);//in the path constraints

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
