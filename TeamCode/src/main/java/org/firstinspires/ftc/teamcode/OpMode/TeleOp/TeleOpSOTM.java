package org.firstinspires.ftc.teamcode.OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.VirtualGoalSolver;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (SOTM)", group = "00")
public class TeleOpSOTM extends OpMode {
    private MecanumDrive drive;
    private Shooter shooter;
    private Spindexer spindexer;
    private boolean FieldCentric = true;
    private FtcDashboard dashboard;
    private String motif = "Null";
    private VirtualGoalSolver.ShotSolution lastSolution = null;
    public static double virtualGoalAngle;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void start() {
        spindexer.encoderResetDone = false;
        spindexer.sensorInUse = 1;
        FieldCentric = true;
    }

    @Override
    public void loop() {
//        NormalizedRGBA colors = colorSensor1.getNormalizedColors();
        // 1. DRIVE & UTILITY
        drive.update(Constant.GOAL_CENTER_X, Constant.ALLIANCE.equalsIgnoreCase("RED") ? Constant.RED_GOAL_CENTER_Y : Constant.BLUE_GOAL_CENTER_Y);
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, FieldCentric);

        // Toolkit:
        // A:           Field/Robot toggle
        // X:           Reset pos
        // B:           Reset spindexer
        // Y:           Start/stop outtake
        // Left bumper: Shoot any
        // Dpad Up:     Shoot by motif (press once to start, hold to keep running)
        // Dpad Left:   Disable color sensor (if both dead)
        // Dpad Right:  Swap color sensor

        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            Constant.ALLIANCE = Constant.ALLIANCE.equalsIgnoreCase("RED") ? "BLUE" : "RED";
        }
        if (gamepad1.aWasPressed()) {
            FieldCentric = !FieldCentric;
        }
        if (gamepad1.xWasPressed()) {
            drive.resetingPos = true;
            shooter.filteredAprilX = 0;
        }
        if (gamepad1.bWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
        }
        if (gamepad1.dpadRightWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
            spindexer.sensorInUse = spindexer.sensorInUse == 1 ? 2 : 1;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            spindexer.resetTimer.reset();
            spindexer.encoderResetDone = false;
            spindexer.sensorInUse = -1;
        }

        // 2. VISION & AIMING — Virtual Goal Solver
        double goalY = Constant.ALLIANCE.equalsIgnoreCase("RED")
                ? Constant.RED_GOAL_CENTER_Y
                : Constant.BLUE_GOAL_CENTER_Y;
        int aprilID = Constant.ALLIANCE.equalsIgnoreCase("RED") ? 24 : 20;

        lastSolution = VirtualGoalSolver.solve(
                drive.turretX, drive.turretY,
                drive.velX,    drive.velY,
                drive.omega,   drive.headingRad,
                Constant.GOAL_CENTER_X, goalY
        );

        double speed = Math.hypot(drive.velX, drive.velY);
        double speedScale = Math.min(1.0, speed / Constant.MOVING_SPEED_THRESHOLD);
        double settleScale = (drive.settledLoops >= Constant.LIMELIGHT_SETTLE_LOOPS) ? 0.0 : 1.0;
        shooter.movingScale = Math.max(speedScale, settleScale);

        // Lookahead: pre-aims ahead of virtual goal angular velocity to beat servo lag.
        virtualGoalAngle = drive.angleToPoint(lastSolution.virtGoalX, lastSolution.virtGoalY);
        double lookaheadDeg = Math.toDegrees(lastSolution.turretOmegaRad) * Constant.TURRET_LOOKAHEAD_SEC;

        // Use REAL distance for RPM/hood — not effectiveDistInch.
        shooter.updateShootingParams(drive.distanceToGoal(), aprilID, spindexer.outtakeStage != -1);
        shooter.updateTurret(virtualGoalAngle + lookaheadDeg);

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
        motif = shooter.detectMotif();

        if ((gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed())
                && spindexer.outtakeStage == -1) {
            spindexer.autonColor = 1;
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            spindexer.update(motif, shooter.isReady());
        } else {
            spindexer.update(
                    gamepad1.left_bumper || gamepad2.left_bumper,
                    shooter.isReady(),
                    false,
                    false,
                    false);
        }

        // 5. Visual Slot Logic
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if (spindexer.slots[i] == null) {
                slotVisual.append("⚪ ");
            } else if (spindexer.slots[i].getColor().equals("P")) {
                slotVisual.append("\uD83D\uDFE3 ");
            } else if (spindexer.slots[i].getColor().equals("G")) {
                slotVisual.append("\uD83D\uDFE2 ");
            }
        }

        // 6. TELEMETRY
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("current velocity",    shooter.rightShooter.getVelocity());
        packet.put("target velocity",     shooter.calculatedTargetVelocity);
        packet.put("isReady",             shooter.isReady());
        packet.put("real dist (in)",      drive.distanceToGoal());
        packet.put("effective dist (in)", lastSolution.effectiveDistInch);
        packet.put("comp scale",          lastSolution.compScale);
        packet.put("moving scale",        shooter.movingScale);
        packet.put("settled loops",        drive.settledLoops);
        packet.put("lookahead deg",       lookaheadDeg);
        packet.put("robot speed (in/s)",  speed);
        packet.put("velX filtered",       drive.velX);
        packet.put("velY filtered",       drive.velY);
//        packet.put("velX raw",            drive.getRawVelX());
//        packet.put("velY raw",            drive.getRawVelY());
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Spindexer Slots", slotVisual.toString());
        telemetry.addData("Field Centric",   FieldCentric);
        telemetry.addData("Distance1",  "%.2f", spindexer.colorSensor1.getDistance(DistanceUnit.MM));
        telemetry.addData("Distance2",  "%.2f", spindexer.colorSensor2.getDistance(DistanceUnit.MM));
        telemetry.addData("Alliance",        Constant.ALLIANCE);
//        telemetry.addData("Motif",           motif);
        telemetry.addData("Sensor in use",   spindexer.sensorInUse);
        telemetry.addData("Intake Stage",    spindexer.intakeStage);
        telemetry.addData("Outtake Stage",   spindexer.outtakeStage);
//        telemetry.addData("Shooter Ready",   shooter.isReady());
        telemetry.addData("Current Veloity", shooter.rightShooter.getVelocity());
        telemetry.addData("Velo Error",      "%.1f", shooter.calculatedTargetVelocity - shooter.rightShooter.getVelocity());
        telemetry.addData("Real Dist (in)",  "%.2f", drive.distanceToGoal());
//        telemetry.addData("Eff Dist (in)",   "%.2f", lastSolution.effectiveDistInch);
//        telemetry.addData("Moving Scale",    "%.2f", shooter.movingScale);
//        telemetry.addData("target ticks",    spindexer.targetTicks);
//        telemetry.addData("current ticks",   spindexer.spindexerEncoder.getCurrentPosition());
//        telemetry.addData("x Vel", MecanumDrive.getRawVelX());
//        telemetry.addData("y Vel", MecanumDrive.getRawVelY());
        telemetry.addData("filteredAprilX",  shooter.filteredAprilX);
        telemetry.addData("Drive Pos",       "X=%.1f  Y=%.1f", drive.botX, drive.botY);
        telemetry.addData("Robot Heading",   "%.2f", drive.headingDeg);

//        telemetry.addData("Turret Pos",      "X=%.1f  Y=%.1f", drive.turretX, drive.turretY);
//
//        telemetry.addData("CS2 Blue",        spindexer.colorSensor2.blue());
//        telemetry.addData("CS2 Green",       spindexer.colorSensor2.green());
//        telemetry.addData("CS2 Sum (B+G)",   spindexer.colorSensor2.blue() + spindexer.colorSensor2.green());
//        telemetry.addData("CS2 Gap (B-G)",   spindexer.colorSensor2.blue() - spindexer.colorSensor2.green());

        telemetry.update();
    }
}