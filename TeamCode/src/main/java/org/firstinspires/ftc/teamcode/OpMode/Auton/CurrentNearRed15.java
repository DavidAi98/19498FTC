//package org.firstinspires.ftc.teamcode.OpMode.Auton;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "RED \uD83D\uDD34 15 Solo (Sorted)")
//public class CurrentNearRed15 extends OpMode {
//
//    public class Paths {
//        public PathChain MoveToShootPreload;
//        public PathChain IntakeSecondRow;
//        public PathChain SecondRowToGate;
//        public PathChain GateToShoot;
//        public PathChain GateIntake;
//        public PathChain ShootGate;
//        public PathChain MoveToThirdRow;
//        public PathChain ShootThirdRow;
//        public PathChain IntakeFirstRow;
//        public PathChain ShootFirstRow;
//
//        public Paths(Follower follower) {
//
//            // Blue: (31,135)→(52,81.5), heading -90°→-90°
//            // Red:  (113,135)→(92,81.5), heading -90°→-90° (unchanged — pointing in -y)
//            MoveToShootPreload = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(113, 135),
//                            new Pose(92, 81.5)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
//                    .build();
//
//            // Blue: (52,81.5)→ctrl(57.604,57.275)→ctrl(31.362,62.304)→(12,60)
//            // Red:  (92,81.5)→ctrl(86.396,57.275)→ctrl(112.638,62.304)→(132,60)
//            IntakeSecondRow = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(92.000, 81.500),
//                            new Pose(86.396, 57.275),
//                            new Pose(112.638, 62.304),
//                            new Pose(132, 60.000)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            // Blue: (12,60)→ctrl(30,54)→(20,73), heading 180°→185°
//            // Red:  (132,60)→ctrl(114,54)→(124,73), heading 0°→-5°
//            SecondRowToGate = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(132, 60),
//                            new Pose(114, 54),
//                            new Pose(124, 73)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-5))
//                    .build();
//
//            // Blue: (20,73)→ctrl(42.241,65)→(56,84), heading tangent reversed
//            // Red:  (124,73)→ctrl(101.759,65)→(88,84)
//            GateToShoot = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(124, 73),
//                            new Pose(101.759, 65),
//                            new Pose(88, 84)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
//                    .addParametricCallback(0.8,  () -> spindexer.startOuttake())
//                    .build();
//
//            // Blue: (56,84)→ctrl(35,55)→(13.5,59), heading const 138°
//            // Red:  (88,84)→ctrl(109,55)→(130.5,59), heading const 42°
//            GateIntake = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(88, 84),
//                            new Pose(109, 55),
//                            new Pose(130.5, 59)
//                    ))
//                    .setConstantHeadingInterpolation(Math.toRadians(42))
//                    .build();
//
//            // Blue: (13.5,59)→ctrl(33.977,60)→(56,83), tangent reversed
//            // Red:  (130.5,59)→ctrl(110.023,60)→(88,83)
//            ShootGate = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(130.5, 59),
//                            new Pose(110.023, 60),
//                            new Pose(88, 83)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .addParametricCallback(0.7,  () -> spindexer.stopIntake())
//                    .addParametricCallback(0.75, () -> spindexer.startOuttake())
//                    .build();
//
//            // Blue: (56,83)→ctrl(56.384,48.035)→ctrl(56.117,33.575)→(12.128,36.745)
//            // Red:  (88,83)→ctrl(87.616,48.035)→ctrl(87.883,33.575)→(131.872,36.745)
//            MoveToThirdRow = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(88, 83),
//                            new Pose(87.616, 48.035),
//                            new Pose(87.883, 33.575),
//                            new Pose(131.872, 36.745)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .build();
//
//            // Blue: (12.128,36.745)→(56,83), tangent reversed
//            // Red:  (131.872,36.745)→(88,83)
//            ShootThirdRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(131.872, 36.745),
//                            new Pose(88, 83)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .addParametricCallback(0.7,  () -> spindexer.stopIntake())
//                    .addParametricCallback(0.8,  () -> spindexer.startOuttake())
//                    .build();
//
//            // Blue: (56,83)→(21,85), heading 180°→180°
//            // Red:  (88,83)→(123,85), heading 0°→0°
//            IntakeFirstRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(88, 83),
//                            new Pose(123, 85)
//                    ))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            // Blue: (21,85)→(51,115), tangent reversed
//            // Red:  (123,85)→(93,115)
//            ShootFirstRow = follower.pathBuilder()
//                    .addPath(new BezierLine(
//                            new Pose(123, 85),
//                            new Pose(93, 115)
//                    ))
//                    .setTangentHeadingInterpolation()
//                    .setReversed()
//                    .addParametricCallback(0.75, () -> spindexer.stopIntake())
//                    .addParametricCallback(0.85, () -> spindexer.startOuttake())
//                    .build();
//        }
//    }
//
//    private Follower     follower;
//    private Paths        paths;
//    private Timer        pathTimer, opmodeTimer;
//    private int          pathState;
//    private FtcDashboard dashboard;
//
//    private Shooter   shooter;
//    private Spindexer spindexer;
//
//    // Turret angles: mirrored as 360 - blue_angle
//    // Blue 45 → Red 315, Blue 50 → Red 310, Blue 11.5 → Red 348.5
//    // Blue 5  → Red 355,  Blue 2  → Red 358,  Blue 335 → Red 25
//    private double angle       = 315;
//    private double odoDist     = 64;
//    private String targetMotif = "Null";
//
//    // Blue start: (31, 135, -90°) → Red: (113, 135, -90°)
//    public static final Pose START_POS = new Pose(113, 135, Math.toRadians(-90));
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//
//            // ── PRELOAD ───────────────────────────────────────────────────────
//
//            case 0:
//                spindexer.startIntake();
//                follower.setMaxPower(1.0);
//                follower.followPath(paths.MoveToShootPreload, true);
//                setPathState(1);
//                break;
//
//            case 1:
//                if (opmodeTimer.getElapsedTimeSeconds() > 1.5) {
//                    setPathState(2);
//                } else if (!follower.isBusy() && !targetMotif.equals("Null")) {
//                    setPathState(2);
//                }
//                break;
//
//            case 2:
//                angle = 310; // Blue 50 → Red 310
//                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 3.0) {
//                    spindexer.stopIntake();
//                    spindexer.startOuttake();
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (spindexer.outtakeStage == -1) {
//                    setPathState(10);
//                }
//                break;
//
//            // ── SECOND ROW ───────────────────────────────────────────────────
//
//            case 10:
//                follower.followPath(paths.IntakeSecondRow, true);
//                spindexer.startIntake();
//                angle = 348.5; // Blue 11.5 → Red 348.5
//                odoDist = 64;
//                setPathState(11);
//                break;
//
//            case 11:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.SecondRowToGate, true);
//                    setPathState(12);
//                }
//                break;
//
//            case 12:
//                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
//                    spindexer.stopIntake();
//                }
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
//                    follower.followPath(paths.GateToShoot, true);
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
//                    setPathState(20);
//                }
//                break;
//
//            // ── GATE INTAKE ───────────────────────────────────────────────────
//
//            case 20:
//                follower.followPath(paths.GateIntake, true);
//                spindexer.startIntake();
//                angle = 355; // Blue 5 → Red 355
//                setPathState(21);
//                break;
//
//            case 21:
//                if (!follower.isBusy()) {
//                    setPathState(22);
//                }
//                break;
//
//            case 22:
//                if (spindexer.intakeStage == -1 || pathTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(paths.ShootGate, true);
//                    setPathState(23);
//                }
//                break;
//
//            case 23:
//                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
//                    setPathState(30);
//                }
//                break;
//
//            // ── THIRD ROW ─────────────────────────────────────────────────────
//
//            case 30:
//                spindexer.startIntake();
//                follower.followPath(paths.MoveToThirdRow, true);
//                angle = 358; // Blue 2 → Red 358
//                odoDist = 70;
//                setPathState(31);
//                break;
//
//            case 31:
//                if (!follower.isBusy() || spindexer.intakeStage == -1) {
//                    follower.followPath(paths.ShootThirdRow, true);
//                    setPathState(32);
//                }
//                break;
//
//            case 32:
//                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
//                    setPathState(40);
//                }
//                break;
//
//            // ── FIRST ROW ─────────────────────────────────────────────────────
//
//            case 40:
//                spindexer.startIntake();
//                follower.followPath(paths.IntakeFirstRow, true);
//                angle = 25; // Blue 335 → Red 25
//                odoDist = 20;
//                setPathState(41);
//                break;
//
//            case 41:
//                if (!follower.isBusy() || spindexer.intakeStage == -1) {
//                    follower.followPath(paths.ShootFirstRow, true);
//                    setPathState(42);
//                }
//                break;
//
//            case 42:
//                if (!follower.isBusy() && spindexer.outtakeStage == -1) {
//                    setPathState(99);
//                }
//                break;
//
//            // ── DONE ─────────────────────────────────────────────────────────
//            case 99:
//                break;
//        }
//    }
//
//    @Override
//    public void init() {
//        Constant.ALLIANCE = "RED";
//
//        pathTimer   = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        shooter   = new Shooter(hardwareMap);
//        spindexer = new Spindexer(hardwareMap);
//
//        spindexer.setSpindexer(Constant.INTAKE_POS1);
//        shooter.setTurretPosition(0.3);
//
//        follower = Constants.createFollower(hardwareMap);
//        paths    = new Paths(follower);
//        follower.setStartingPose(START_POS);
//        dashboard = FtcDashboard.getInstance();
//        spindexer.noSort = false;
//    }
//
//    @Override
//    public void init_loop() {}
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//
//        if (targetMotif.equals("Null")) {
//            targetMotif = shooter.detectMotif();
//        }
//
//        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);
//
//        if (targetMotif.equals("Null")) {
//            shooter.updateTurret(100, 0);
//        } else {
//            shooter.updateTurret(angle, 0);
//        }
//
//        shooter.runShooter(spindexer.outtakeStage != -1);
//        spindexer.update(targetMotif, shooter.isReady());
//
//        autonomousPathUpdate();
//
//        StringBuilder slotVisual = new StringBuilder();
//        for (int i = 0; i < 3; i++) {
//            if      (spindexer.slots[i] == null)                    slotVisual.append("⚪ ");
//            else if (spindexer.slots[i].getColor().equals("P"))     slotVisual.append("\uD83D\uDFE3 ");
//            else if (spindexer.slots[i].getColor().equals("G"))     slotVisual.append("\uD83D\uDFE2 ");
//        }
//
//        Pose p = follower.getPose();
//        // Mirror of blue: AUTON_LAST_X = 103 - blue_x = 103 - (144 - red_x) = red_x - 41
//        Constant.AUTON_LAST_X           = p.getX() - 41;
//        Constant.AUTON_LAST_Y           = 3 - p.getY();
//        Constant.AUTON_LAST_HEADING_RAD = p.getHeading() - Math.PI;
//        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);
//
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("current velocity", shooter.rightShooter.getVelocity());
//        packet.put("target velocity",  shooter.calculatedTargetVelocity);
//        dashboard.sendTelemetryPacket(packet);
//
//        telemetry.addData("Slots",         slotVisual.toString());
//        telemetry.addData("Path State",    pathState);
//        telemetry.addData("Motif",         targetMotif);
//        telemetry.addData("Turret Angle",  angle);
//        telemetry.addData("Odo Dist",      odoDist);
//        telemetry.addData("Intake Stage",  spindexer.intakeStage);
//        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
//        telemetry.addData("Velo Error",    "%.1f",
//                shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
//        telemetry.addData("Target Color",  spindexer.targetColor);
//        telemetry.addData("Max Power",     follower.getMaxPowerScaling());
//        telemetry.addData("Heading",       follower.getHeading());
//        telemetry.addData("Shooter Velocity", shooter.rightShooter.getVelocity());
//        telemetry.addData("CS2 Blue",        spindexer.colorSensor2.blue());
//        telemetry.addData("CS2 Green",       spindexer.colorSensor2.green());
//        telemetry.addData("CS2 Sum (B+G)",   spindexer.colorSensor2.blue() + spindexer.colorSensor2.green());
//        telemetry.addData("CS2 Gap (B-G)",   spindexer.colorSensor2.blue() - spindexer.colorSensor2.green());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {}
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}