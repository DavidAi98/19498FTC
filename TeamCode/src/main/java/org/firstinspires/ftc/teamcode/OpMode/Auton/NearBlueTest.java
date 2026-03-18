package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE Test")
public class NearBlueTest extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public static class Paths {
        public PathChain MoveToShootPreload;
        public PathChain MoveToSecondRow;
        public PathChain IntakeSecondRow;
        public PathChain ShootSecondRow;
        public PathChain MoveToGate;
        public PathChain MoveBack;
        public PathChain GateIntake;
        public PathChain ShootIntaked;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(32.729, 136.953),
                            new Pose(58.100, 85.549)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            MoveToSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.100, 85.549),
                            new Pose(62.063, 57.007),
                            new Pose(43.15496368038741, 59.499)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .build();

            IntakeSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(43.15496368038741, 59.499),
                            new Pose(15.634, 59.475)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ShootSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(15.634, 59.475),
                            new Pose(58.100, 85.549)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            MoveToGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58.100, 85.549),
                            new Pose(42.241, 71.732),
                            new Pose(18.153, 66.688)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-151), Math.toRadians(170))
                    .build();

            MoveBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(18.153, 66.688),
                            new Pose(19, 64.081)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(170))
                    .build();

            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(20.931, 66.081),
                            new Pose(16.826, 57.282),
                            new Pose(16.4, 55.574)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(170), Math.toRadians(130))
                    .build();

            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(11.983, 55.574),
                            new Pose(33.977, 67.562),
                            new Pose(58.100, 85.549)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(58.100, 85.549),
                            new Pose(21, 83.598)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21, 83.598),
                            new Pose(52.173, 110.530)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }

    // =========================================================================
    //  FIELDS
    // =========================================================================

    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer;
    private int       pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 40;
    private double odoDist     = 70;
    private String targetMotif = "Null";

    public static final Pose START_POS = new Pose(31.160032833282813, 136.08094169150246, Math.toRadians(-90));

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            // Move to shooting position; start intake to stage preload
            case 0:
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            // Wait for arrival + motif lock
            case 1:
                if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
                break;

            // Wait for preload to stage then fire
            case 2:
                if (spindexer.intakeStage == -1 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    angle = 40;
                    spindexer.startOuttake();
                    setPathState(3);
                }
                break;

            // Wait for preload shot to clear
            case 3:
                if (spindexer.outtakeStage == -1) {
                    setPathState(4);
                }
                break;

            // ── SECOND ROW ───────────────────────────────────────────────────

            // Curve down to front of second row; start intake
            case 4:
                follower.followPath(paths.MoveToSecondRow, true);
                spindexer.startIntake();
                setPathState(5);
                break;

            // Wait to arrive at second row start, then sweep
            case 5:
                if (!follower.isBusy()) {
                    angle = 340;
                    follower.followPath(paths.IntakeSecondRow, true);
                    setPathState(6);
                }
                break;

            // Sweep done OR intake full; stop intake and head back to shoot
            case 6:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootSecondRow, true);
                    setPathState(7);
                }
                break;

            // Wait to arrive at shooting position then fire
            case 7:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(8);
                }
                break;

            // Wait for second row shot to clear
            case 8:
                if (spindexer.outtakeStage == -1) {
                    setPathState(10);
                }
                break;

            // ── GATE CYCLE 1 ─────────────────────────────────────────────────

            // Head to gate (no intake yet — start at MoveBack)
            case 10:
                follower.followPath(paths.MoveToGate, true);
                setPathState(11);
                break;

            // Arrived at gate; start intake then nudge back
            case 11:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(12);
                }
                break;

            // Nudge done; sweep gate arc
            case 12:
                if (!follower.isBusy()) {
                    angle = 315;
                    follower.followPath(paths.GateIntake, true);
                    setPathState(13);
                }
                break;

            // Intake until full or 2s timeout; stop intake and head to shoot
            case 13:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(14);
                }
                break;

            // Wait to arrive then fire gate cycle 1
            case 14:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(15);
                }
                break;

            // Wait for gate cycle 1 shot to clear
            case 15:
                if (spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE CYCLE 2 ─────────────────────────────────────────────────

            // Head to gate (no intake yet)
            case 20:
                follower.followPath(paths.MoveToGate, true);
                setPathState(21);
                break;

            // Arrived; start intake then nudge back
            case 21:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(22);
                }
                break;

            // Nudge done; sweep gate arc
            case 22:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GateIntake, true);
                    setPathState(23);
                }
                break;

            // Intake until full or 2s timeout; stop intake and head to shoot
            case 23:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(24);
                }
                break;

            // Wait to arrive then fire gate cycle 2
            case 24:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    angle = 315;
                    spindexer.startOuttake();
                    setPathState(25);
                }
                break;

            // Wait for gate cycle 2 shot to clear
            case 25:
                if (spindexer.outtakeStage == -1) {
                    setPathState(30);
                }
                break;

            // ── FIRST ROW ────────────────────────────────────────────────────

            // Sweep across first row with intake on
            case 30:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                setPathState(31);
                break;

            // Wait for path done or intake full; stop intake and head to shoot
            case 31:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(32);
                }
                break;

            // Wait to arrive then fire first row
            case 32:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    angle = 300;
                    spindexer.startOuttake();
                    setPathState(33);
                }
                break;

            // Wait for first row shot to clear
            case 33:
                if (spindexer.outtakeStage == -1) {
                    setPathState(99);
                }
                break;

            // ── DONE ─────────────────────────────────────────────────────────
            case 99:
                break;
        }
    }

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================

    @Override
    public void init() {
        Constant.ALLIANCE = "BLUE";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        shooter.setTurretPosition(0.25);

        follower = Constants.createFollower(hardwareMap);
        paths    = new Paths(follower);
        follower.setStartingPose(START_POS);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Poll every loop — never misses the motif
        if (targetMotif.equals("Null")) {
            targetMotif = shooter.detectMotif();
        }

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);

        // Hold at 40° facing goal until motif is seen, then use per-shot angle
        if (targetMotif.equals("Null")) {
            shooter.updateTurret(100, 0);
        } else {
            shooter.updateTurret(angle, 0);
        }

        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif, shooter.isReady());

        autonomousPathUpdate();

        // Slot visual
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
        }

        // Save pose for TeleOp hand-off
        Pose p = follower.getPose();
        Constant.AUTON_LAST_X           = 103 - p.getX();
        Constant.AUTON_LAST_Y           =   3 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.addData("Velo Error",    "%.1f",
                shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Target Color",  spindexer.targetColor);
        telemetry.addData("Max Power",     follower.getMaxPowerScaling());
        telemetry.addData("Heading",       follower.getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}