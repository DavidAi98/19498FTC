package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.*;

@TeleOp(name = "RED TeleOp")
public class RedTeleOp extends OpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Spindexer spindexer;
    private boolean FieldCentric = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
    }

    @Override
    public void loop() {
        // 1. DRIVE
        drive.pinpoint.update();
        drive.updateRelativePostion(Constant.GOAL_CENTER_X, Constant.RED_GOAL_CENTER_Y);
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FieldCentric);

        if (gamepad1.aWasPressed()) {
            FieldCentric = !FieldCentric;
        }
        // reset odometry
        if (gamepad1.xWasPressed()) {
            drive.calibrateTimer.reset();
            drive.calibrating = true;
            shooter.filteredAprilX = 0;
        }
        // reset spindexer
        if (gamepad1.bWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
        }

        // 2. VISION & AIMING
        double dist = drive.distanceToGoal();
        double rawTurretAngle = drive.getAngleToGoal();
        shooter.updateShootingParams(dist, 24);
        shooter.updateTurret(rawTurretAngle);

        // 3. INTAKE/OUTTAKE CONTROL
        if (gamepad1.rightBumperWasPressed()) {
            if (spindexer.intakeStage == -1) spindexer.startIntake();
            else spindexer.stopIntake();
        }
        if (gamepad2.yWasPressed()) {
            if (spindexer.outtakeStage == -1) spindexer.startOuttake();
            else spindexer.stopOuttake();
        }

        // 4. SUBSYSTEM UPDATES
        shooter.runShooter(spindexer.outtakeStage != -1);
        // Passing Fire Button (Left Bumper) and Shooter Ready state
        spindexer.update(gamepad2.left_bumper, shooter.isReady(), gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());

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
        telemetry.addData("Spindexer Slots", slotVisual.toString());
        telemetry.addData("Field Centric", FieldCentric);
        telemetry.addData("Intake Stage", spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.addData("Robot Heading", "%.2f", drive.headingDeg);
        telemetry.addData("Velo Error", "%.1f", shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Distance (odo)", "%.2f", dist);
        telemetry.addData("Distance (ll)", "%.2f", shooter.limelightDistanceInch);
        telemetry.addData("target ticks", spindexer.targetTicks);
        telemetry.addData("current ticks", spindexer.currentTicks);
        telemetry.addData("filteredAprilX", shooter.filteredAprilX);
        telemetry.update();
    }
}