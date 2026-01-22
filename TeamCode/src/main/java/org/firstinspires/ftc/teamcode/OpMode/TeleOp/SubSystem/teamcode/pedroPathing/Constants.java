package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.75)

            .forwardZeroPowerAcceleration(-(31.45+29.87+31.45+32.63+30.18+33.83)/6
            )
            .lateralZeroPowerAcceleration(-(62.97+68.21+60.48+62)/4)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.39, 0.00001, 0.033, 0))//F: the motor trying to move but not move(sound); P: set correct line(vertical; D: more slowly back
            .headingPIDFCoefficients(new PIDFCoefficients(2.8, 0, 0.16, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.000001, 0.0027, 0.6, 0))//line test contains the t and h pidf while drive tuner not; keep the same f with forwardtunner
            .centripetalScaling(0.0005);

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
            .xVelocity((67.002404881127+64.08+68.81+67+67.42)/5)
            .yVelocity((50.28+49.4+50.48)/3) //(54.4996131236159 + 55.765637495386315 + 55.52764892578125 + 53.797435850609006) / 4
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);






    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.08562992)
            .strafePodX(-6.94866142)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(2.9734409722222*0.99454819444)//2.82174406647
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
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
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
