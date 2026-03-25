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

@Autonomous(name = "BLUE \uD83D\uDD35 18 Coop (NOT Sorted)")
public class NearBlueTest extends OpMode {

    // =========================================================================
    //  PATHS
    // =========================================================================

    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain MoveToSecondRow;
        public PathChain ShootSecondRow;
        public PathChain GateIntake;
        public PathChain MoveBack;
        public PathChain ShootIntaked;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31, 135),
                            new Pose(58, 76)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> fireAny())
                    .build();

            MoveToSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(58, 76),
                            new Pose(58, 68),
                            new Pose(47, 61),
                            new Pose(13.000, 60.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ShootSecondRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13, 60),
                            new Pose(56, 80)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 80),
                            new Pose(31, 68),
                            new Pose(20, 68)
                    ))
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierLine(
                            new Pose(20, 68),
                            new Pose(20, 64)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(new BezierCurve(
                            new Pose(20, 64),
                            new Pose(16.826, 57.282),
                            new Pose(12, 52)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                    .build();

            ShootIntaked = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(14, 52),
                            new Pose(33.977, 67.562),
                            new Pose(56, 80)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 80),
                            new Pose(21, 85)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            ShootFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21, 85),
                            new Pose(51, 115)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> fireAny())
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

    private double angle       = 50;
    private double odoDist     = 78;
    private String targetMotif = "PPG";

    public static final Pose START_POS = new Pose(31, 135, Math.toRadians(-90));

    private void fireAny() {
        spindexer.noSort = true;
        spindexer.startOuttake();
    }

    // =========================================================================
    //  STATE MACHINE
    // =========================================================================

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── STATE 0: Start preload path ───────────────────────────────────
            case 0:
                angle = 50;
                follower.setMaxPower(1.0);
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            // ── STATE 1: Wait for preload path, then fire ─────────────────────
            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            // ── STATE 2: Wait for preload outtake to finish ───────────────────
            case 2:
                if (spindexer.outtakeStage == -1) {
                    setPathState(3);
                }
                break;

            // ── STATE 3: Drive to second row + intake ─────────────────────────
            case 3:
                follower.followPath(paths.MoveToSecondRow, true);
                spindexer.startIntake();
                angle = 345;
                odoDist = 60;
                setPathState(4);
                break;

            // ── STATE 4: Wait for full intake or arrival, then drive to shoot ─
            case 4:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootSecondRow, true);
                    setPathState(5);
                }
                break;

            // ── STATE 5: Wait for shoot path, then fire ───────────────────────
            case 5:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    fireAny();
                    setPathState(6);
                }
                break;

            // ── STATE 6: Wait for second row outtake to finish ────────────────
            case 6:
                if (spindexer.outtakeStage == -1) {
                    setPathState(7);
                }
                break;

            // ── STATE 7: Drive to gate (gate cycle 1) ─────────────────────────
            case 7:
                odoDist = 70;
                follower.followPath(paths.GateIntake, true);
                setPathState(8);
                break;

            // ── STATE 8: Wait for gate, start intake + drive back/intake path ─
            case 8:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(9);
                }
                break;

            // ── STATE 9: Wait for intake or timeout, then shoot ───────────────
            case 9:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(10);
                }
                break;

            // ── STATE 10: Wait for shoot path, then fire ──────────────────────
            case 10:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    fireAny();
                    setPathState(11);
                }
                break;

            // ── STATE 11: Wait for gate cycle 1 outtake to finish ────────────
            case 11:
                if (spindexer.outtakeStage == -1) {
                    setPathState(12);
                }
                break;

            // ── STATE 12: Drive to gate (gate cycle 2) ────────────────────────
            case 12:
                follower.followPath(paths.GateIntake, true);
                setPathState(13);
                break;

            // ── STATE 13: Wait for gate, start intake + drive back/intake path
            case 13:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(14);
                }
                break;

            // ── STATE 14: Wait for intake or timeout, then shoot ──────────────
            case 14:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(15);
                }
                break;

            // ── STATE 15: Wait for shoot path, then fire ──────────────────────
            case 15:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    fireAny();
                    setPathState(16);
                }
                break;

            // ── STATE 16: Wait for gate cycle 2 outtake to finish ────────────
            case 16:
                if (spindexer.outtakeStage == -1) {
                    setPathState(17);
                }
                break;

            // ── STATE 17: Drive to gate (gate cycle 3) ────────────────────────
            case 17:
                follower.followPath(paths.GateIntake, true);
                setPathState(18);
                break;

            // ── STATE 18: Wait for gate, start intake + drive back/intake path
            case 18:
                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(paths.MoveBack, true);
                    setPathState(19);
                }
                break;

            // ── STATE 19: Wait for intake or timeout, then shoot ──────────────
            case 19:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 1.75) {
                    follower.followPath(paths.ShootIntaked, true);
                    setPathState(20);
                }
                break;

            // ── STATE 20: Wait for shoot path, then fire ──────────────────────
            case 20:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    fireAny();
                    setPathState(21);
                }
                break;

            // ── STATE 21: Wait for gate cycle 3 outtake to finish ────────────
            case 21:
                if (spindexer.outtakeStage == -1) {
                    setPathState(22);
                }
                break;

            // ── STATE 22: Drive to first row + intake ─────────────────────────
            case 22:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                setPathState(23);
                break;

            // ── STATE 23: Wait for full intake or arrival, then drive to shoot
            case 23:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    angle = 340;
                    odoDist = 26;
                    setPathState(24);
                }
                break;

            // ── STATE 24: Wait for shoot path, then fire ──────────────────────
            case 24:
                if (!follower.isBusy()) {
                    setPathState(25);
                }
                break;

            // ── STATE 25: Wait for first row outtake to finish ────────────────
            case 25:
                if (spindexer.outtakeStage == -1) {
                    setPathState(26);
                }
                break;

            // ── STATE 26: DONE ────────────────────────────────────────────────
            case 26:
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
        shooter.setTurretPosition(0.3);


        follower = Constants.createFollower(hardwareMap);
        paths    = new Paths(follower);
        follower.breakFollowing();
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

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);
        shooter.updateTurret(angle, 0);
        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif, shooter.isReady());

        autonomousPathUpdate();

        // Slot visual
        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P")) slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G")) slotVisual.append("\uD83D\uDFE2 ");
        }

        // Save pose for TeleOp hand-off
        Pose p = follower.getPose();
        Constant.AUTON_LAST_X           = 103 - p.getX();
        Constant.AUTON_LAST_Y           =   3 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Encoder Reset",  spindexer.encoderResetDone);
        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}