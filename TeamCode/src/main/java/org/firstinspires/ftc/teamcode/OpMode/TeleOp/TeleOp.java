package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "01")
public class TeleOp extends OpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Spindexer spindexer;
    private boolean FieldCentric = true;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        spindexer.onStart = true;
    }

    @Override
    public void loop() {
        // 1. DRIVE & UTILITY
        drive.update(Constant.GOAL_CENTER_X, Constant.ALLIANCE.equalsIgnoreCase("RED") ? Constant.RED_GOAL_CENTER_Y : Constant.BLUE_GOAL_CENTER_Y);
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FieldCentric);

        // Toolkit:

        // A: Field/Robot toggle
        // X: reset pos
        // B: reset spindexer
        // Y: shooter

        // Left bumper: shoot Any

        // Dpad Up: shoot P
        // Dpad Down: shoot G
        // Dpad Left: calibrate odo + imu
        // Dpad Right: swap color sensor

        // Right stick button: swap alliance

        if (gamepad1.aWasPressed()) {
            FieldCentric = !FieldCentric;
        }
        // reset odometry
        if (gamepad1.xWasPressed()) {
            drive.resetingPos = true;
            shooter.filteredAprilX = 0;
        }
        // reset spindexer
        if (gamepad1.bWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
        }

        // change opMode
//        if (gamepad1.rightStickButtonWasPressed()) {
//            Constant.ALLIANCE = Constant.ALLIANCE.equalsIgnoreCase("RED") ? "BLUE" : "RED";
//        }
        // change color sensor
        if (gamepad1.dpadRightWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
            spindexer.sensorInUse = spindexer.sensorInUse == 1 ? 2 : 1;
        }
        // disable color sensor (ONLY IF BOTH COLOR SENSOR DC)
        if (gamepad1.dpadLeftWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
            spindexer.sensorInUse = -1;
        }

        // 2. VISION & AIMING
        double dist = drive.distanceToGoal();
        double rawTurretAngle = drive.angleToGoal();
        shooter.updateShootingParams(dist, Constant.ALLIANCE.equalsIgnoreCase("RED") ? 24 : 20, spindexer.outtakeStage != -1);
        shooter.updateTurret(rawTurretAngle);

        // 3. INTAKE/OUTTAKE CONTROL
        if (gamepad1.rightBumperWasPressed()) {
            if (spindexer.intakeStage == -1) spindexer.startIntake();
            else spindexer.stopIntake();
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            if (spindexer.outtakeStage == -1) spindexer.startOuttake();
            else spindexer.stopOuttake();
        }

        // 4. SUBSYSTEM UPDATES
        shooter.runShooter(spindexer.outtakeStage != -1);
        // Passing Fire Button (Left Bumper) and Shooter Ready state
        spindexer.update(gamepad1.left_bumper || gamepad2.left_bumper,
                shooter.isReady(),
                gamepad1.dpadUpWasPressed(), // P Artifact
                gamepad1.dpadDownWasPressed(), // G Artifact
                gamepad1.leftBumperWasPressed()); // skipslot

        // 5. Visual Slot Logic
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if (spindexer.slots[i] == null) {
                slotVisual.append("⚪ "); // Empty
            } else if (spindexer.slots[i].getColor().equals("P")) {
                slotVisual.append("\uD83D\uDFE3 "); // Purple
            } else if (spindexer.slots[i].getColor().equals("G")) {
                slotVisual.append("\uD83D\uDFE2 "); // Green
            }
        }

        // 6. TELEMETRY
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("current velocity", shooter.leftShooter.getVelocity());
        packet.put("target velocity", shooter.calculatedTargetVelocity);
        packet.put("target velocity", shooter.calculatedTargetVelocity);
        packet.put("target velocity", shooter.calculatedTargetVelocity);
        

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Spindexer Slots", slotVisual.toString());
        telemetry.addData("Field Centric", FieldCentric);
        telemetry.addData("OpMode", Constant.ALLIANCE);
        telemetry.addData("Sensor in use", spindexer.sensorInUse);
        telemetry.addData("Intake Stage", spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
        telemetry.addData("Velo Error", "%.1f", shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Distance (odo)", "%.2f", dist);
        telemetry.addData("target ticks", spindexer.targetTicks);
        telemetry.addData("current ticks", spindexer.spindexerEncoder.getCurrentPosition());
        telemetry.addData("filteredAprilX", shooter.filteredAprilX);
        telemetry.addData("Drive Pos",
                "X=%.1f  Y=%.1f",
                drive.botX,
                drive.botY);
        telemetry.addData("Turret Pos",
                "X=%.1f  Y=%.1f",
                drive.turretX,
                drive.turretY);
        telemetry.update();
    }
}