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
//
//@Autonomous(name = "BLUE Near (15 solo)", group = "01")
//public class NearBlue15 extends OpMode {
//
//    private Follower follower;
//    private PathChain
//            firstPickup1, firstPickup2, firstPickup3,firstPickup32,firstPickup4,
//            secondPickup1, secondPickup2,pickup2ToGate,
//            thirdPickup1, thirdPickup2,pickup3ToGate,
//            openRedGate1,openRedGate2, openRedGate3,
//            gateIntake1,gateIntake2,gateIntake3,gateIntake4,gateIntake5,
//            randomPickup1,randomPickup2,randomPickup3,
//            endPath;
//    private Path preLoadShot;
//    public static Pose initPos = new Pose(31.4, 135.293, Math.toRadians(270));
//    public static Pose shootingPos = new Pose(51.9, 84.133, Math.toRadians(270));
//    public static Pose redGate = new Pose(15, 69.9, Math.toRadians(180));
//
//    public static Pose pickup1Pos = new Pose(12, 35.403,Math.toRadians(180));
//    public static Pose pickup2Pos = new Pose(12.8, 60.282, Math.toRadians(180));
//    public static Pose pickup3Pos = new Pose(18.9, 84.04, Math.toRadians(180));
//    public static Pose endPos1 = new Pose(56.619, 36.669, Math.toRadians(180));
//
//    public static Pose randomPos1 = new Pose(11.138, 23.669,Math.toRadians(260));
//    public static Pose randomPos2 = new Pose(11.425, 12.807,Math.toRadians(260));
//    public static Pose shooting2Pos = new Pose(60.265, 103, Math.toRadians(225));
//
//    public static Pose gatePickup1 = new Pose(16.9,69,Math.toRadians(180));
//    public static Pose gatePickup2 = new Pose(13.5,57.6,Math.toRadians(130));
//    public static Pose gatePickup3 = new Pose(9.873,58.608,Math.toRadians(110));
//    public static Pose gatePickup4 = new Pose(16.702,59.873,Math.toRadians(110));
//
//
//    private Timer pathTimer, actionTimer, opmodeTimer,testTimer;
//
//    private int pathState, actionState;
//
//    private Shooter shooter;
//    private Spindexer spindexer;
//
//
//
//    private double odoDist =70;
//    private double angle = 40;
//    boolean ifCorrecting = true;
//
//    private String targetMotif = "Null";
//
//
//
//
//    public void buildPaths() {
//
//        preLoadShot = new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());
//
//
//        openRedGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(42, 65.98),
//                        redGate
//                ))
//                .setLinearHeadingInterpolation(redGate.getHeading(), Math.toRadians(180))
//                .build();
//
//        openRedGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(39.76,67.98),
//                        shootingPos
//                ))
//                .setLinearHeadingInterpolation(redGate.getHeading(),Math.toRadians(180))
//                .build();
//
//        openRedGate3 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(35.981, 70.301),
//                        shootingPos
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(52, 45),
//                        new Pose(54, 39.4),
//
//
//                        pickup1Pos
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pickup1Pos,
//                        new Pose(16.994, 43.369)
//                ))
//                .setLinearHeadingInterpolation(pickup1Pos.getHeading(),Math.toRadians(180))
//                .build();
//
//        firstPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(16.994, 43.369),
//                        new Pose(51.912, 85.724)
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        firstPickup32 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(16.994, 43.199),
//                        shooting2Pos
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        firstPickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Pose(42,35.6),
//
//                        pickup1Pos
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//
//
//        pickup2ToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pos,
//                        new Pose(36.025, 63.124),
//                        redGate
//                ))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(48.356, 68.4),
//                        new Pose(50.122, 63.9),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//
//                .build();
//
//        secondPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup2Pos,
//                        new Pose(25.939,59.171),
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//
//        thirdPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        shootingPos,
//
//                        pickup3Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        thirdPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        pickup3Pos,
//
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        pickup3ToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        redGate,
//                        new Pose(25.061, 76.575),
//                        shootingPos
//                ))
//                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
//                .build();
//
//
//
//        randomPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(18.613, 44.047),
//                        new Pose(13.635, 34.870),
//                        randomPos1
//                ))
//                .setTangentHeadingInterpolation()
//
//                .build();
//
//        randomPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        randomPos1,
//
//                        randomPos2
//                ))
//                .setLinearHeadingInterpolation(randomPos1.getHeading(), randomPos2.getHeading())
//                .build();
//
//        randomPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        randomPos2,
//
//                        shooting2Pos
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//
//        gateIntake1= follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//
//                        new Pose(27.5, 68.069),
//                        gatePickup1
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        gateIntake2= follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        gatePickup1,
//
//                        new Pose(21.768,64.086),
//                        new Pose(15.5,50.423),
//                        gatePickup2
//                ))
//                .setLinearHeadingInterpolation( gatePickup1.getHeading(),  gatePickup2.getHeading())
//                .build();
//        gateIntake3= follower.pathBuilder()
//                .addPath(new BezierLine(
//                        gatePickup2,
//
//                        gatePickup3
//                ))
//                .setLinearHeadingInterpolation(gatePickup2.getHeading(), gatePickup3.getHeading())
//                .build();
//
//        gateIntake4= follower.pathBuilder()
//                .addPath(new BezierLine(
//                        gatePickup3,
//
//                        gatePickup4
//                ))
//                .setLinearHeadingInterpolation(gatePickup3.getHeading(), gatePickup4.getHeading())
//                .build();
//
//        gateIntake5= follower.pathBuilder()
//                .addPath(new BezierLine(
//                        gatePickup2,
//
//                        shootingPos
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//
//
//    }
//
//
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
//
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//                targetMotif = shooter.detectMotif();
//
//                if (!follower.isBusy() && !targetMotif.equals("Null")) {
//                    angle=50;
//
//                    setPathState(2);
//
//                }
//
//                break;
//
//            case 2:
//                if(spindexer.intakeStage==-1&&pathTimer.getElapsedTimeSeconds()>0.5){
//                    spindexer.startOuttake();
//                    setPathState(201);
//                }
//                break;
//
//
//
//
//            case 101:
//
//                if (spindexer.outtakeStage==-1) {
//
//                    odoDist = 70;
////                    angle=34.5;
//                    angle=359;
//
//
//                    testTimer.resetTimer();
//                    follower.followPath(firstPickup1, true);
//                    spindexer.startIntake();
//
//                    setPathState(102);
//                }
//                break;
//            case 102:
//                if(pathTimer.getElapsedTimeSeconds()>2){
//                    follower.setMaxPower(0.8);
//                }
//
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    follower.followPath(firstPickup2, true);
//
//                    setPathState(104);
//                }
//                break;
//
//            case 103:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    odoDist = 75;
//
//                    follower.followPath(firstPickup3, true);
//
//                    setPathState(107);
//                }
//                break;
//            case 104:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//
//
//                    follower.followPath(firstPickup32, true);
//
//                    setPathState(105);
//                }
//                break;
//            case 105:
//
//
//                if(!follower.isBusy()){
//                    spindexer.stopIntake();
//                    spindexer.startOuttake();
//
//                    setPathState(13);
//                }
//                break;
//
//
//            case 106:
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(0.8);
//
//
//                    follower.followPath(firstPickup4, true);
//
//                    setPathState(102);
//                }
//                break;
//            case 107:
//                if(!follower.isBusy()){
//                    follower.turnTo(Math.toRadians(270));
//                    setPathState(105);
//                }
//                break;
//
//
//            case 201:
//
//
//                if (spindexer.outtakeStage==-1) {
//
//
//                    ifCorrecting=false;
//
//                    follower.followPath(secondPickup1, true);
//
//                    spindexer.startIntake();
//                    setPathState(202);
//
//
//                }
//                break;
//
//            case 202:
//
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    follower.setMaxPower(0.8);
//                }
//                if (!follower.isBusy()) {
//                    angle=319.5;
//
//                    follower.followPath(pickup2ToGate, true);
//                    setPathState(203);
//
//                }
//
//                break;
//
//            case 203:
//
//
//                if (!follower.isBusy()) {
//
//
//
//                    follower.setMaxPower(1);
//                    follower.followPath(openRedGate2, true);
//
//                    setPathState(205);
//
//                }
//
//                break;
//
//            case 204:
//
//                if(pathTimer.getElapsedTimeSeconds()>0.8){
//                    follower.setMaxPower(0.7);
//                }
//                if (!follower.isBusy()) {
//                    follower.setMaxPower(1);
//                    angle=2.6;
//                    follower.followPath(secondPickup2, true);
//                    setPathState(205);
//
//                }
//                break;
//
//            case 205:
//
//                if(!follower.isBusy()){
//                    spindexer.stopIntake();
//                    spindexer.startOuttake();
//                    setPathState(301);
//                }
//                break;
//
//
//
//            case 301:
//
//                if (spindexer.outtakeStage==-1) {
//                    follower.followPath(thirdPickup1, true);
//
//                    ifCorrecting=true;
//                    follower.setMaxPower(0.7);
//                    spindexer.startIntake();
//                    setPathState(304);
//                }
//                break;
//
//            case 302:
//                if(pathTimer.getElapsedTimeSeconds()>0.5){
//                    follower.setMaxPower(0.7);
//
//                }
//                if (!follower.isBusy()) {
//
//
//                    follower.followPath(pickup3ToGate, true);
//
//                    setPathState(303);
//                }
//                break;
//            case 303:
//                if(pathTimer.getElapsedTimeSeconds()>0.8){
//
//                    follower.setMaxPower(1);
//                }
//
//                if (!follower.isBusy()) {
//
//
//                    follower.followPath(openRedGate3, true);
//
//                    setPathState(305);
//                }
//                break;
//
//
//            case 304:
//
//                if(pathTimer.getElapsedTimeSeconds()>0.8){
//                    follower.setMaxPower(0.7);
//                }
//                if (!follower.isBusy()) {
//                    angle = 319.5;
//                    follower.setMaxPower(1);
//                    follower.followPath(thirdPickup2, true);
//
//                    setPathState(305);
//                }
//                break;
//
//            case 305:
//                if(pathTimer.getElapsedTimeSeconds()>0.8){
//                    follower.setMaxPower(1);
//                }
//                if (!follower.isBusy()) {
//
//                    spindexer.stopIntake();
//                    spindexer.startOuttake();
//                    setPathState(101);
//                }
//                break;
//
//
//            case 401:
//                if (spindexer.outtakeStage==-1) {
//
//                    ifCorrecting=false;
//                    testTimer.resetTimer();
//                    follower.followPath(gateIntake1, true);
//                    spindexer.startIntake();
//                    setPathState(402);
//
//                }
//                break;
//
//            case 402:
//
//
//                if(pathTimer.getElapsedTimeSeconds()>1.7) {
//
//                    angle=356;
//
//                    follower.followPath(gateIntake2, true);
//                    spindexer.startIntake();
//
//
//                    setPathState(405);
//                }
//                break;
//            case 403:
//
//                if(!follower.isBusy()){
//
//                    follower.followPath(gateIntake3 , true);
//                    setPathState(404);
//                }
//                break;
//            case 404:
//                if(!follower.isBusy()){
//
//
//                    follower.followPath(gateIntake4 , true);
//
//                    setPathState(405);
//                }
//                break;
//
//            case 405:
////                if(pathTimer.getElapsedTimeSeconds()>1){
////                    follower.setMaxPower(0.8);
////                }
//                if(spindexer.intakeStage==-1||pathTimer.getElapsedTimeSeconds()>2.4){
//                    follower.setMaxPower(1);
//
//                    follower.followPath(gateIntake5 , true);
//
//                    setPathState(406);
//                }
//                break;
//
//            case 406:
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    spindexer.stopIntake();
//                }
//                if(!follower.isBusy()){
//
//                    spindexer.stopIntake();
//                    spindexer.startOuttake();
//                    setPathState(301);
//                }
//                break;
//            case 407:
//                if(!follower.isBusy()){
//                    follower.turn(Math.toRadians(42),false);
//                    setPathState(406);
//                }
//                break;
//
//
//
//            case 501:
//                odoDist = 75;
//
//
//                if (spindexer.outtakeStage==-1) {
//                    angle=10;
//                    testTimer.resetTimer();
//                    follower.followPath(randomPickup1, true);
//                    spindexer.startIntake();
//                    setPathState(502);
//
//                }
//                break;
//            case 502:
//                if(!follower.isBusy()){
//
//                    follower.setMaxPower(0.7);
//
//                    follower.followPath(randomPickup2 , true);
//
//                    setPathState(503);
//                }
//                break;
//
//            case 503:
//                if(!follower.isBusy()){
//                    follower.setMaxPower(1);
//                    follower.followPath(randomPickup3 , true);
//
//                    setPathState(504);
//                }
//                break;
//
//
//            case 504:
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    spindexer.stopIntake();
//                }
//                if(!follower.isBusy()){
//                    spindexer.startOuttake();
//                    setPathState(1);
//                }
//                break;
//
//
//
//            case 13:
//
//                if(spindexer.outtakeStage==-1){
//                    angle=0;
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                break;
//        }
//    }
//
//
//
//
//
//
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        shooter.updateShootingParams(odoDist, 20, spindexer.outtakeStage != -1);
//
//        if(!targetMotif.equals("Null")&&ifCorrecting){
//            shooter.updateTurret(angle);//0 is left max
//        }else if(!targetMotif.equals("Null")){
//
//
//            shooter.updateTurret(angle,0);//0 is left max
//        }
//
//        shooter.runShooter(spindexer.outtakeStage != -1);
//        spindexer.update(targetMotif,shooter.isReady());
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
//
//        Pose currentPose = follower.getPose();
//        Constant.AUTON_LAST_X = 103 - currentPose.getX();
//        Constant.AUTON_LAST_Y = 3 - currentPose.getY(); // close blue should be similar
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
//        telemetry.addData("Motif",targetMotif);
//        telemetry.addData("path state", pathState);
//        telemetry.addData("TargetColor",spindexer.targetColor);
//        telemetry.addData("maxPower",follower.getMaxPowerScaling());
//        telemetry.addData("angle",follower.getHeading());
//
//
//        telemetry.addData("TestTimer",testTimer.getElapsedTimeSeconds());
////        telemetry.addData("x", follower.getPose().getX());
////        telemetry.addData("y", follower.getPose().getY());
////        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        Constant.ALLIANCE = "BLUE";
//
//        ifCorrecting = true;
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        testTimer=new Timer();
//
//        opmodeTimer.resetTimer();
//
//        shooter = new Shooter(hardwareMap);
////        drive = new MecanumDrive(hardwareMap);
//        spindexer = new Spindexer(hardwareMap);
//
//        spindexer.setSpindexer(Constant.INTAKE_POS1);
////        shooter.calculatedTargetVelocity = 1800;
//        shooter.setTurretPosition(0.25);
////        shooter.setHoodPosition(Constant.HOOD_INIT);
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
////        drive.pinpoint.resetPosAndIMU();
//    }
//
//
//
//
//
//
//
//
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
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
//
//
//}
//
//
