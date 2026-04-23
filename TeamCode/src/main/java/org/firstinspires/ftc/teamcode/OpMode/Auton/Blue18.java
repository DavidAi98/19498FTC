package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "\uD83D\uDD35 18")
public class Blue18 extends OpMode {


    public class Paths {
        public PathChain MoveToShootPreload;
        public PathChain IntakeSecondRow;
        public PathChain SecondRowToGate;
        public PathChain GateToShoot;
        public PathChain GateIntake;
        public PathChain ShootGate;
//        public PathChain MoveToThirdRow;
//        public PathChain ShootThirdRow;
        public PathChain IntakeFirstRow;
        public PathChain ShootFirstRow;

        public Paths(Follower follower) {
            MoveToShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(31.000, 135.000),

                                    new Pose(58.789, 77.292)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                    .build();

            IntakeSecondRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.789, 77.292),
                                    new Pose(56.658, 59.540),
                                    new Pose(41, 59.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(41, 59.5),

                                    new Pose(10, 59.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            SecondRowToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10, 59.5),
                                    new Pose(26.870, 63.746),
                                    new Pose(15, 69.199)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(175))
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15, 69.199),
                                    new Pose(37.379, 69.553),
                                    new Pose(58.789, 77.068)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))
                    .addParametricCallback(0.9, () -> { spindexer.stopIntake(); spindexer.startOuttake(); })
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.789, 77.068),
                                    new Pose(42.000, 64.000),
                                    new Pose(15, 65.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(15, 65.5),

                                    new Pose(10.2, 62.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            ShootGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.2, 62.7),

                                    new Pose(58.565, 76.845)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
                    .build();

//            MoveToThirdRow = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(58.565, 76.845),
//                                    new Pose(58.000, 46.000),
//                                    new Pose(52.000, 38.000),
//                                    new Pose(48, 38)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(48, 38),
//
//                                    new Pose(10, 38)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .build();
//
//            ShootThirdRow = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(10, 38),
//
//                                    new Pose(58.565, 77.292)
//                            )
//                    ).setTangentHeadingInterpolation()
//                    .setReversed()
//                    .addParametricCallback(0.7, () -> spindexer.stopIntake())
//                    .addParametricCallback(0.8, () -> spindexer.startOuttake())
//                    .build();

            IntakeFirstRow = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.565, 76.845),
                                    new Pose(48.911, 85.447),
                                    new Pose(16, 83.553)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            ShootFirstRow = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16, 83.553),

                                    new Pose(48.764, 113.435)
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

    private double angle       = 44;
    private double odoDist     = 67;
    private String targetMotif = "Null";

    public final Pose START_POS = new Pose(31, 135, Math.toRadians(270));

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
                angle = 317;
                odoDist = 72;
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
                angle = 335;
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

//            // ShootGate callbacks handle stopIntake + startOuttake automatically
//            case 23:
//                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
//                    follower.followPath(paths.MoveToThirdRow, true);
//                    spindexer.startIntake();
//                    setPathState(30);
//                }
//                break;
//
//            // ── THIRD ROW ─────────────────────────────────────────────────────
//
//            case 30:
//                angle = 1;
//                odoDist = 68;
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.ShootThirdRow, true);
//                    setPathState(31);
//                }
//                break;

            case 23:
                follower.followPath(paths.GateIntake, true);
                spindexer.startIntake();
                setPathState(24);
                break;

            // Wait for robot to finish the gate path
            case 24:
                if (!follower.isBusy()) {
                    setPathState(25);
                    intakeTimer.resetTimer();
                }
                break;

            // Stay at gate for 2 seconds (or leave early if full)
            case 25:
                if (spindexer.artifactCount == 3 || intakeTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(27);
                }
                break;

                // Gate intake 3

            case 27:
                follower.followPath(paths.GateIntake, true);
                spindexer.startIntake();
                setPathState(28);
                break;

            // Wait for robot to finish the gate path
            case 28:
                if (!follower.isBusy()) {
                    setPathState(29);
                    intakeTimer.resetTimer();
                }
                break;

            // Stay at gate for 2 seconds (or leave early if full)
            case 29:
                if (spindexer.artifactCount == 3 || intakeTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(paths.ShootGate, true);
                    setPathState(30);
                }
                break;

            case 30:
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
                angle = 336;
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
        Constant.ALLIANCE = "BLUE";

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        intakeTimer = new Timer();

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
        intakeTimer.resetTimer();
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
            shooter.updateTurret(110, 0);
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