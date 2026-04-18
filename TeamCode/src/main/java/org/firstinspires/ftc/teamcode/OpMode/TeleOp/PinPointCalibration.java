package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;

@TeleOp(name = "pinpointCalibration", group="01")
public class PinPointCalibration extends OpMode {
    public GoBildaPinpointDriver pinpoint;
    public ElapsedTime calibrateTimer = new ElapsedTime();

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(Constant.ODO_X_OFFSET, Constant.ODO_Y_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setYawScalar(Constant.ODO_YAW_SCALAR);
    }

    @Override
    public void loop() {

        if (calibrateTimer.seconds() > Constant.CALIBRATE_TIMER) {
            requestOpModeStop();
        } else if (calibrateTimer.seconds() > 0.3) {
            pinpoint.resetPosAndIMU();

        }
    }
}
