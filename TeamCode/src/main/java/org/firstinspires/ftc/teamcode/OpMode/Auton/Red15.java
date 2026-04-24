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

@Autonomous(name = "\uD83D\uDD34 15 SORTED")
public class Red15 extends OpMode {

    private double mirror_length = 139.5;
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
            MoveToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(31.000, 135.000).mirror(mirror_length),

                                    new Pose(58.5, 77).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                    .build();

            IntakeSecondRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.5, 77).mirror(mirror_length),
                                    new Pose(56.658, 59.540).mirror(mirror_length),
                                    new Pose(41, 59.5).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(41, 59.5).mirror(mirror_length),

                                    new Pose(10, 59.5).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            SecondRowToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10, 59.5).mirror(mirror_length),
                                    new Pose(26.870, 63.746).mirror(mirror_length),
                                    new Pose(15.8, 69.199).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(5))
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.8, 69.199).mirror(mirror_length),
                                    new Pose(37.379, 69.553).mirror(mirror_length),
                                    new Pose(58, 77).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(5), Math.toRadians(0))
                    .addParametricCallback(0.9, () -> { spindexer.stopIntake(); spindexer.startOuttake(); })
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58, 77).mirror(mirror_length),
                                    new Pose(42.000, 64.000).mirror(mirror_length),
                                    new Pose(16, 65).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(16, 65).mirror(mirror_length),

                                    new Pose(11.8, 62).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
                    .build();

            ShootGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(11.8, 62).mirror(mirror_length),

                                    new Pose(58.565, 76.845).mirror(mirror_length)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            MoveToThirdRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.565, 76.845).mirror(mirror_length),
                                    new Pose(58.000, 46.000).mirror(mirror_length),
                                    new Pose(52.000, 37.5).mirror(mirror_length),
                                    new Pose(48, 37.5).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(48, 37.5).mirror(mirror_length),

                                    new Pose(10, 37.5).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ShootThirdRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 37.5).mirror(mirror_length),

                                    new Pose(58.565, 77.292).mirror(mirror_length)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

            IntakeFirstRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.565, 77.292).mirror(mirror_length),
                                    new Pose(48.911, 85.447).mirror(mirror_length),
                                    new Pose(16, 83.553).mirror(mirror_length)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ShootFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16, 83.553).mirror(mirror_length),

                                    new Pose(48.764, 113.435).mirror(mirror_length)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.8, () -> spindexer.stopIntake())
                    .addParametricCallback(0.9, () -> spindexer.startOuttake())
                    .build();
        }
    }


    private Follower  follower;
    private Paths     paths;
    private Timer     pathTimer, opmodeTimer, intakeTimer;
    private int      pathState;

    private Shooter   shooter;
    private Spindexer spindexer;

    private double angle       = 130;
    private double odoDist     = 70;
    private String targetMotif = "Null";

    public final Pose START_POS = new Pose(31, 135, Math.toRadians(-90)).mirror(mirror_length);

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── PRELOAD ───────────────────────────────────────────────────────

            case 0:
                follower.setMaxPower(0.9);
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
                angle = 220;
                odoDist = 76;
                setPathState(11);
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SecondRowToGate, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.GateToShoot, true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    setPathState(20);
                }
                break;

            // ── GATE INTAKE ───────────────────────────────────────────────────

            case 20:
                follower.followPath(paths.GateIntake, true);
                spindexer.startIntake();
                angle = 202;
                setPathState(21);
                break;

            // Wait for robot to finish the gate path
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                    intakeTimer.resetTimer();
                }
                break;

            // Stay at gate for 2 seconds (or leave early if full)
            case 22:
                if (spindexer.artifactCount == 3 || intakeTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(23);
                }
                break;

            // ShootGate callbacks handle stopIntake + startOuttake automatically
            case 23:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    follower.followPath(paths.MoveToThirdRow, true);
                    spindexer.startIntake();
                    setPathState(30);
                }
                break;

            // ── THIRD ROW ─────────────────────────────────────────────────────

            case 30:
                angle = 180;
                odoDist = 74;
                if (!follower.isBusy()) {
                    follower.followPath(paths.ShootThirdRow, true);
                    setPathState(31);
                }
                break;

            case 31:
                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
                    follower.followPath(paths.IntakeFirstRow, true);
                    spindexer.startIntake();
                    setPathState(40);
                }
                break;

//            case 32:
//                if (spindexer.artifactCount == 0) {
//                    follower.followPath(paths.IntakeFirstRow, true);
//                    spindexer.startIntake();
//                    setPathState(40);
//                }
//                break;

            // ── FIRST ROW ─────────────────────────────────────────────────────

            case 40:
                angle = 195;
                odoDist = 15;
                setPathState(41);
                break;

            case 41:
                if (!follower.isBusy()) {
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
        Constant.ALLIANCE = "RED";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        intakeTimer = new Timer();

        shooter   = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        spindexer.setSpindexer(Constant.INTAKE_POS1);
        shooter.setTurretPosition(0.2);

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
        intakeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        if (targetMotif.equals("Null")) {
            targetMotif = shooter.detectMotif();
        }

        shooter.updateShootingParams(odoDist, 24, spindexer.outtakeStage != -1);

        if (targetMotif.equals("Null")) {
            shooter.updateTurret(70, 0);
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
        Constant.AUTON_LAST_X           = 113.5 - p.getX();
        Constant.AUTON_LAST_Y           =   8 - p.getY();
        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        telemetry.addData("Slots",         slotVisual.toString());
        telemetry.addData("artifactCount", spindexer.artifactCount);
        telemetry.addData("Path State",    pathState);
        telemetry.addData("Intake Stage",  spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);

        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Motif",         targetMotif);
        telemetry.addData("Turret Angle",  angle);
        telemetry.addData("Odo Dist",      odoDist);
        telemetry.addData("Velo Error",    "%.1f",
                shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("Target Color",  spindexer.targetColor);
        telemetry.addData("Max Power",     follower.getMaxPowerScaling());
        telemetry.addData("Drive Pos",       "X=%.1f  Y=%.1f", p.getX(), p.getY());
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