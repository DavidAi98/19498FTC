//package org.firstinspires.ftc.teamcode.OpMode.Auton;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
//import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "RED Far (9 solo)", group = "04")
//public class FarRedTest extends OpMode {
//
//    private Follower follower;
//    private PathChain firstPickup1, firstPickup2, secondPickup1, secondPickup2, openRedGate2, firstPickupBreak1, firstPickupBreak2, endPath, secondPickupBreak1, secondPickupBreak2, pickup2ToGate, thirdPickup1, thirdPickup2,randomPickup1,randomPickup2,randomPickup3,randomPickup4;
//    private Path preLoadShot;
//    public static Pose initPos = new Pose(144-64, 6.807, Math.toRadians(0));
//    public static Pose shootingPos = new Pose(80, 16.391, Math.toRadians(0));
//    public static Pose redGate = new Pose(128, 71, Math.toRadians(0));
//    public static Pose pickup1Pos = new Pose(133.21, 35, Math.toRadians(0));
//    public static Pose pickup2Pos = new Pose(133.21, 58.89, Math.toRadians(0));
//    public static Pose pickup3Pos = new Pose(127, 83.989, Math.toRadians(0));
//    public static Pose random1 = new Pose(132,7.8,Math.toRadians(355));
//    public static Pose random2 = new Pose(132.6,12.219,Math.toRadians(5));
//    public static Pose endPos1 = new Pose(87.409, 27.298, Math.toRadians(0));
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    private int pathState, actionState;
//
//    private Shooter shooter;
//    private Spindexer spindexer;
//    private String targetMotif = "Null";
//
//
//    public void buildPaths() {
//
//        preLoadShot = new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());
//
//
//        firstPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        redGate,
//                        new Pose(64.840, 73.591),
//                        new Pose(62.652, 54.895)
//                ))
//                .setLinearHeadingInterpolation(redGate.getHeading(), redGate.getHeading())
//                .build();
//
//        firstPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(62.652, 54.895),   // from (81.348,54.895)
//                        new Pose(65.436, 33.017),   // from (78.564,33.017)
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(redGate.getHeading(), pickup1Pos.getHeading())
//                .build();
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(93.309, 36.644),
//                        new Pose(103.928, 35.964),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
//                .build();
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup1Pos,
//                        new Pose(106.497, 31.238),
//                        shootingPos))
//                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), shootingPos.getHeading())
//                .build();
//
//
//        pickup2ToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pos,
//                        new Pose(110, 64.483),
//                        redGate
//                ))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
//                .build();
//
//
//        openRedGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(96.365, 60.097),
//                        shootingPos
//                ))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(103.331, 63.580),
//                        new Pose(98.768, 62.199),
//                        pickup2Pos))
//                .setLinearHeadingInterpolation(shootingPos.getHeading(), pickup2Pos.getHeading(), 0.4)
//                .build();
//
//
//        thirdPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//
//                        new Pose(90.669, 74.157),
//                        new Pose(94.453, 88.467),
//                        pickup3Pos))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
//                .build();
//
//
//        thirdPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pickup3Pos,
//
//                        shootingPos))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
//                .build();
//
//        randomPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(98,15),
//                        new Pose(107.354,6.826),
//                        random1))
//                .setLinearHeadingInterpolation(shootingPos.getHeading(), random1.getHeading())
//                .build();
//        randomPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        random1,
//                        new Pose(124.7,10.84),
//                        random2))
//                .setLinearHeadingInterpolation(random1.getHeading(), random2.getHeading())
//                .build();
//        randomPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        random2,
//
//                        shootingPos))
//                .setLinearHeadingInterpolation(random2.getHeading(), shootingPos.getHeading())
//                .build();
//
//        randomPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        random1,
//
//                        shootingPos))
//                .setLinearHeadingInterpolation(random1.getHeading(), shootingPos.getHeading())
//                .build();
//
//        endPath = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        shootingPos,
//
//                        endPos1
//                ))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
//                .build();
//
//    }
//
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//
//
//                follower.followPath(preLoadShot);
//                spindexer.startIntake();
//                setActionState(0);
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//                targetMotif = shooter.detectMotif();
//
//                if (!follower.isBusy() && !targetMotif.equals("Null")) {
//                    setActionState(0);
//                    setPathState(99);
//
//                }
//
//                break;
//
//            case 99:
//                if (spindexer.intakeStage == -1 && pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    spindexer.startOuttake();
//                    setPathState(2);
//                }
//                break;
//            case 2:
//
//
//                if (spindexer.outtakeStage == -1 && pathTimer.getElapsedTimeSeconds() > 0.2) {
//                    setActionState(0);
//                    follower.followPath(secondPickup1, true);
//                    spindexer.startIntake();
//                    setPathState(3);
//
//
//                }
//                break;
//
//            case 3://
//
//                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
//                    follower.setMaxPower(0.8);
//                }
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//
//                    follower.followPath(pickup2ToGate, true);
//                    setPathState(4);
//
//                }
//
//                break;
//
//            case 4:
//
//
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    follower.followPath(openRedGate2, true);
//                    spindexer.stopIntake();
//                    setActionState(0);
//                    setPathState(5);
//
//                }
//
//                break;
//
//            case 5:
//
//
//                if (!follower.isBusy()) {
//                    spindexer.startOuttake();
//                    setPathState(6);
//
//                }
//                break;
//
//            case 6:
//
//                if (spindexer.outtakeStage == -1) {
//                    follower.followPath(firstPickup1, true);
//                    setActionState(0);
//
//                    spindexer.startIntake();
//                    setPathState(7);
//                }
//
//                break;
//            case 7:
//                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
//                    follower.setMaxPower(0.8);
//
//
//                }
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    spindexer.stopIntake();
//                    follower.followPath(firstPickup2, true);
////                    setActionState(0);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//
//                if (!follower.isBusy()) {
//                    spindexer.startOuttake();
//
//                    setActionState(0);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//
//                if (spindexer.outtakeStage == -1) {
//                    follower.followPath(endPath, true);
//                    setActionState(0);
//
//                    setPathState(13);
//                }
//                break;
//            case 10:
//
//                if (!follower.isBusy()) {
//                    follower.followPath(thirdPickup2, true);
//
//                    setActionState(0);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if (!follower.isBusy()) {
//                    spindexer.startOuttake();
//
//                    setActionState(0);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//
//                if (spindexer.outtakeStage == -1) {
////                    turret1.setPosition(Constant.TURRET_RIGHT_MAX-0.25*turretRange);
//                    follower.followPath(randomPickup1, true);
//                    spindexer.startIntake();
//
//                    setPathState(13);
//
//                }
//                break;
//            case 13:
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    follower.setMaxPower(0.6);
//                }
//                if(!follower.isBusy()&&spindexer.intakeStage!=-1){
//
//                    follower.followPath(randomPickup4);
//                    setPathState(15);
//                }else if(!follower.isBusy()){
//                    follower.followPath(randomPickup2);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                if(!follower.isBusy()){
//                    follower.followPath(randomPickup3);
//                    setPathState(15);
//
//                }
//                break;
//
//            case 15:
//                if(pathTimer.getElapsedTimeSeconds()>0.7){
//                    follower.setMaxPower(1);
//                    spindexer.stopIntake();
//                }
//                if(!follower.isBusy()){
//                    spindexer.startOuttake();
//                    setPathState(16);
//                }
//            case 16:
//                if(spindexer.outtakeStage==-1){
//                    follower.followPath(endPath);
//                }
//               break;
//
//
//        }
//    }
//
//
//    /**
//     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
//     **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        if (!targetMotif.equals("Null") && spindexer.outtakeStage != -1) {
//            shooter.updateShootingParams(125, 24, spindexer.outtakeStage != -1);
//            shooter.updateTurret(22+180);//0 is left max
//        }
//        shooter.runShooter(spindexer.outtakeStage != -1);
//        spindexer.update(targetMotif, shooter.isReady());
//
//        autonomousPathUpdate();
//
//        // 5. Visual Slot Logic
//        StringBuilder slotVisual = new StringBuilder();
//        for (int i = 0; i < 3; i++) {
//            if (spindexer.slots[i] == null) {
//                slotVisual.append("⚪ "); // Empty
//            } else if (spindexer.slots[i].getColor().equals("P")) {
//                slotVisual.append("\uD83D\uDFE3 "); // Purple
//            } else if (spindexer.slots[i].getColor().equals("G")) {
//                slotVisual.append("\uD83D\uDFE2 "); // Green
//            }
//        }
//
//        Pose currentPose = follower.getPose();
//        Constant.AUTON_LAST_X = currentPose.getX() - 32;
//        Constant.AUTON_LAST_Y = currentPose.getY() - 6.5;
//        Constant.AUTON_LAST_HEADING_RAD = currentPose.getHeading() - Math.PI;
//        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);
//
//        // 6. TELEMETRY
//        telemetry.addData("Spindexer Slots", slotVisual.toString());
//        telemetry.addData("Intake Stage", spindexer.intakeStage);
//        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
//        telemetry.addData("Velo Error", "%.1f", shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
//        telemetry.addData("target ticks", spindexer.targetTicks);
//        telemetry.addData("current ticks", spindexer.currentTicks);
//        telemetry.addData("Motif", targetMotif);
//        telemetry.addData("path state", pathState);
//        telemetry.addData("TargetColor", spindexer.targetColor);
//
////        telemetry.addData("x", follower.getPose().getX());
////        telemetry.addData("y", follower.getPose().getY());
////        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//
//    /**
//     * This method is called once at the init of the OpMode.
//     **/
//    @Override
//    public void init() {
//        Constant.ALLIANCE = "RED";
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//
//        opmodeTimer.resetTimer();
//
//        shooter = new Shooter(hardwareMap);
////        drive = new MecanumDrive(hardwareMap);
//        spindexer = new Spindexer(hardwareMap);
//
////        shooter.calculatedTargetVelocity = 1800;
//        spindexer.setSpindexer(Constant.INTAKE_POS1);
//        shooter.setTurretPosition(0.5);
////        shooter.setHoodPosition(Constant.HOOD_INIT);
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
////        drive.pinpoint.resetPosAndIMU();
//    }
//
//    /**
//     * This method is called continuously after Init while waiting for "play".
//     **/
//    @Override
//    public void init_loop() {
//    }
//
//    /**
//     * This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system
//     **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /**
//     * We do not use this because everything should automatically disable
//     **/
//    @Override
//    public void stop() {
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//
//    }
//
//    public void setActionState(int aState) {
//        actionState = aState;
//        actionTimer.resetTimer();
//
//    }
//}
