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

@Autonomous(name = "Current blue 15")
public class CurrentNearBlue15 extends OpMode {

    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain SecondRowToGate;
        public PathChain GateToShoot;
        public PathChain GateIntake;
        public PathChain ShootGate;
        public PathChain MoveToThirdRow;
        public PathChain ShootThirdRow;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {

            MoveToShootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(31, 135),
                            new Pose(52, 81.5)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            IntakeSecondRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(52.000, 81.500),
                            new Pose(57.604, 57.275),
                            new Pose(31.362, 62.304),
                            new Pose(12, 60.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            SecondRowToGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(12, 60),
                            new Pose(30, 58),
                            new Pose(20, 73)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(185))
                    .build();

            GateToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(20, 73),
                            new Pose(42.241, 65),
                            new Pose(56, 84)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.9, () -> { spindexer.stopIntake(); spindexer.startOuttake(); })
                    .build();

            GateIntake = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 84),
                            new Pose(35.000, 55),
                            new Pose(12, 59)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(138))
                    .build();

            ShootGate = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(12, 59),
                            new Pose(33.977, 60),
                            new Pose(56, 83)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            MoveToThirdRow = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56, 83),
                            new Pose(56.38366101694915, 48.03529055690071),
                            new Pose(56.117447941888635, 33.57518644067797),
                            new Pose(13.128, 36.745)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            ShootThirdRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(13.128, 36.745),
                            new Pose(56, 83)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            IntakeFirstRow = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56, 83),
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
                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
                    .build();
        }
    }

    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer;
    private int       pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 45;
    private double odoDist     = 64;
    private String targetMotif = "Null";

    public static final Pose START_POS = new Pose(31, 135, Math.toRadians(-90));

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(1.0);
                follower.followPath(paths.MoveToShootPreload, true);
                spindexer.startIntake();
                setPathState(1);
                break;

            case 1:
                if (opmodeTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(2);
                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    setPathState(2);
                }
                break;

            case 2:
                angle = 50;
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2.0) {
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(3);
                }
                break;

            case 3:
                if (spindexer.outtakeStage == -1) {
                    setPathState(10);
                }
                break;

            // ── SECOND ROW ───────────────────────────────────────────────────

            case 10:
                follower.followPath(paths.IntakeSecondRow, true);
                spindexer.startIntake();
                angle = 15;
                odoDist = 64;
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SecondRowToGate, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    follower.followPath(paths.GateToShoot, true);
                    setPathState(13);
                }
                break;

            case 13:
                spindexer.stopIntake();
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE INTAKE ───────────────────────────────────────────────────

            case 20:
                follower.followPath(paths.GateIntake, true);
                spindexer.startIntake(); // called once here, not in a loop
                angle = 8;
                setPathState(21);
                break;

            // Wait for robot to finish the gate path
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;

            // Stay at gate for 2 seconds (or leave early if full)
            case 22:
                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(23);
                }
                break;

            // ShootGate callbacks handle stopIntake + startOuttake automatically
            case 23:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(30);
                }
                break;

            // ── THIRD ROW ─────────────────────────────────────────────────────

            case 30:
                spindexer.startIntake();
                follower.followPath(paths.MoveToThirdRow, true);
                angle = 5;
                odoDist = 70;
                setPathState(31);
                break;

            case 31:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootThirdRow, true);
                    setPathState(32);
                }
                break;

            case 32:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(40);
                }
                break;

            // ── FIRST ROW ─────────────────────────────────────────────────────

            case 40:
                spindexer.startIntake();
                follower.followPath(paths.IntakeFirstRow, true);
                angle = 335;
                odoDist = 18;
                setPathState(41);
                break;

            case 41:
                if (!follower.isBusy() || spindexer.intakeStage == -1) {
                    follower.followPath(paths.ShootFirstRow, true);
                    setPathState(42);
                }
                break;

            case 42:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(99);
                }
                break;

            // ── DONE ─────────────────────────────────────────────────────────
            case 99:
                break;
        }
    }

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
        follower.setStartingPose(START_POS);
        spindexer.noSort = false;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        if (targetMotif.equals("Null")) {
            targetMotif = shooter.detectMotif();
        }

        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);

        if (targetMotif.equals("Null")) {
            shooter.updateTurret(100, 0);
        } else {
            shooter.updateTurret(angle, 0);
        }

        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif, shooter.isReady());

        autonomousPathUpdate();

        StringBuilder slotVisual = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
        }

        Pose p = follower.getPose();
        Constant.AUTON_LAST_X           = 103 - p.getX();
        Constant.AUTON_LAST_Y           =   3 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Odo Dist",      odoDist);
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