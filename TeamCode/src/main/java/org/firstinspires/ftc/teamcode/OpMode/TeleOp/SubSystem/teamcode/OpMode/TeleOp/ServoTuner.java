package org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Config
public class ServoTuner extends OpMode {
    private Servo spindexer1, spindexer2;
    public static double spindexer1Position, spindexer2Position;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public void init(){
        spindexer1 = hardwareMap.get(Servo.class, "LeftPivot");
        spindexer2 = hardwareMap.get(Servo.class, "RightPivot");
        spindexer1.setDirection(Servo.Direction.REVERSE);
    }
    public void loop(){
        packet.put("Spindexer 1 Position", spindexer1Position);
        packet.put("Spindexer 2 Position", spindexer2Position);
        dashboard.sendTelemetryPacket(packet);
        spindexer1.setPosition(spindexer1Position);
        // 0.06 -> 3
        spindexer2.setPosition(spindexer2Position);

    }

}
