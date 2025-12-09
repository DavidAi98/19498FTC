package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.43)

            .forwardZeroPowerAcceleration(-31.037090499911248)
            .lateralZeroPowerAcceleration(-56.89847403045257)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.0001, 0.04, 0))//F: the motor trying to move but not move(sound); P: set correct line(vertical; D: more slowly back
            .headingPIDFCoefficients(new PIDFCoefficients(2.3, 0.001, 0.15, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0044, 0.6, 0))//line test contains the t and h pidf while drive tuner not; keep the same f with forwardtunner
            .centripetalScaling(0.0008);

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
            .xVelocity(71.82933097479244)
            .yVelocity(51.824077460148466)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);

    ;


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(5.38378847695754E-4)
            .strafeTicksToInches(5.20553813315335E-4)
            .turnTicksToInches(5.41841247367179E-4)
            .leftPodY(5)
            .rightPodY(-4.73)
            .strafePodX(-7.16)
            .leftEncoder_HardwareMapName("LeftBackMotor")
            .rightEncoder_HardwareMapName("RightFrontMotor")
            .strafeEncoder_HardwareMapName("BackOdometry")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));



    public static PathConstraints pathConstraints = new PathConstraints(0.995,
            0.1,
            0.1,
            0.009,
            30,
            1.2,
            10,
            1);//in the path constraints

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}