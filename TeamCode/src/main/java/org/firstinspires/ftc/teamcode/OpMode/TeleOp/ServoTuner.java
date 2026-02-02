package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Config
public class ServoTuner extends OpMode {
    private Servo spindexer1, spindexer2, turret1, turret2, hood;
    public static double spindexer1Position, spindexer2Position, turret1pos, turret2pos, hoodPos;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();

    public void init(){
        spindexer1 = hardwareMap.get(Servo.class, "spindexer1");
        spindexer2 = hardwareMap.get(Servo.class, "spindexer2");
        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        hood = hardwareMap.get(Servo.class, "RightHood");
    }
    public void loop(){
        packet.put("Spindexer 1 Position", spindexer1Position);
        packet.put("Spindexer 2 Position", spindexer2Position);
        packet.put("turret 1 Position", turret1pos);
        packet.put("turret 2 Position", turret2pos);
        packet.put("hood Position", hoodPos);
        dashboard.sendTelemetryPacket(packet);
        spindexer1.setPosition(spindexer1Position);
        spindexer2.setPosition(spindexer2Position);
        turret1.setPosition(turret1pos);
        turret2.setPosition(turret2pos);
        hood.setPosition(hoodPos);
    }
}
