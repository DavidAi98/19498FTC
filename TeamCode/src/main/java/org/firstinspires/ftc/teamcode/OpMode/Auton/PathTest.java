package org.firstinspires.ftc.teamcode.OpMode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Constant;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Shooter;
import org.firstinspires.ftc.teamcode.OpMode.TeleOp.SubSystem.Spindexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "PathTest", group = "01")
public class PathTest extends OpMode {

    private Follower follower;
    private PathChain
            firstPickup1, firstPickup2, firstPickup3,firstPickup32,firstPickup4,
            secondPickup1, secondPickup2,pickup2ToGate,
            thirdPickup1, thirdPickup2,pickup3ToGate,
            openRedGate1,openRedGate2, openRedGate3,
            gateIntake1,gateIntake2,gateIntake3,gateIntake4,gateIntake5,
            randomPickup1,randomPickup2,randomPickup3,
            endPath;
    private Path preLoadShot;
    public static Pose initPos = new Pose(112.79, 135.293, Math.toRadians(-90));
    public static Pose shootingPos = new Pose(92.8, 84, Math.toRadians(-90));
    public static Pose redGate = new Pose(129.0, 66, Math.toRadians(0));

    public static Pose pickup1Pos = new Pose(132.4, 33.403, Math.toRadians(0));
    public static Pose pickup2Pos = new Pose(132.4, 57, Math.toRadians(0));
    public static Pose pickup3Pos = new Pose(126, 82.04, Math.toRadians(0));
    public static Pose endPos1 = new Pose(87.381, 34.669, Math.toRadians(0));

    public static Pose randomPos1 = new Pose(132.862, 21.669, Math.toRadians(-80));
    public static Pose randomPos2 = new Pose(132.575, 10.807, Math.toRadians(-80));
    public static Pose shooting2Pos = new Pose(83.735, 100.044, Math.toRadians(-45));

    public static Pose gatePickup1 = new Pose(128.7, 67, Math.toRadians(0));
    public static Pose gatePickup2 = new Pose(134.5, 51.5, Math.toRadians(40));
    public static Pose gatePickup3 = new Pose(134.127, 56.608, Math.toRadians(70));
    public static Pose gatePickup4 = new Pose(127.298, 57.873, Math.toRadians(70));

    private Timer pathTimer, actionTimer, opmodeTimer,testTimer;

    private int pathState, actionState;

    private Shooter shooter;
    private Spindexer spindexer;



    private double odoDist =75;
    private double angle = 40;


    private String targetMotif = "ppg";



    public void buildPaths() {

        preLoadShot = new Path(new BezierLine(initPos, shootingPos));
        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());

        openRedGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(102, 65.98),
                        redGate
                ))
                .setLinearHeadingInterpolation(redGate.getHeading(), Math.toRadians(0))
                .build();

        openRedGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(102, 65),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(redGate.getHeading(), Math.toRadians(0))
                .build();


        openRedGate3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(108.019, 68.301),
                        shootingPos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();



        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(88, 45.5),
                        new Pose(86, 39.9),
                        pickup1Pos
                ))
                .setTangentHeadingInterpolation()
                .build();

        firstPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickup1Pos,
                        new Pose(127.006, 41.369)
                ))
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), Math.toRadians(0))
                .build();

        firstPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(127.006, 41.369),
                        new Pose(92.088, 83.724)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        firstPickup32 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(127.006, 41.199),
                        shooting2Pos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        firstPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(102, 33.6),
                        pickup1Pos
                ))
                .setTangentHeadingInterpolation()
                .build();

        pickup2ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pos,
                        new Pose(107.975, 61.124),
                        redGate
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();




        secondPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(94.644, 66.5),
                        new Pose(92.878, 62.5),
                        pickup2Pos
                ))
                .setTangentHeadingInterpolation()
                .build();

        secondPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pos,
                        new Pose(118.061, 57.171),
                        shootingPos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();



        thirdPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootingPos,
                        pickup3Pos
                ))
                .setTangentHeadingInterpolation()
                .build();

        thirdPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickup3Pos,
                        shootingPos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickup3ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(118.939, 74.575),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();



        randomPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(125.387, 42.047),
                        new Pose(130.365, 32.870),
                        randomPos1
                ))
                .setTangentHeadingInterpolation()
                .build();

        randomPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        randomPos1,
                        randomPos2
                ))
                .setLinearHeadingInterpolation(randomPos1.getHeading(), randomPos2.getHeading())
                .build();

        randomPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        randomPos2,
                        shooting2Pos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();



        gateIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(116.5, 66.069),
                        gatePickup1
                ))
                .setTangentHeadingInterpolation()
                .build();

        gateIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        gatePickup1,
                        new Pose(122.232, 62.086),
                        new Pose(128.5, 48.423),
                        gatePickup2
                ))
                .setLinearHeadingInterpolation(gatePickup1.getHeading(), gatePickup2.getHeading())
                .build();

        gateIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup2,
                        gatePickup3
                ))
                .setLinearHeadingInterpolation(gatePickup2.getHeading(), gatePickup3.getHeading())
                .build();

        gateIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup3,
                        gatePickup4
                ))
                .setLinearHeadingInterpolation(gatePickup3.getHeading(), gatePickup4.getHeading())
                .build();

        gateIntake5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup2,
                        shootingPos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();



    }




    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:


                follower.followPath(preLoadShot);
                spindexer.startIntake();

                setPathState(1);

                break;

            case 1:

                targetMotif = shooter.detectMotif();

                if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    angle=130;

                    setPathState(2);

                }

                break;

            case 2:
                if(spindexer.intakeStage==-1&&pathTimer.getElapsedTimeSeconds()>0.5){
                    spindexer.startOuttake();
                    setPathState(201);
                }
                break;




            case 101:

                if (spindexer.outtakeStage==-1) {

                    angle=184;


                    follower.followPath(firstPickup1, true);
                    spindexer.startIntake();

                    setPathState(102);
                }
                break;
            case 102:
                if(pathTimer.getElapsedTimeSeconds()>2){
                    follower.setMaxPower(0.7);
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(firstPickup2, true);

                    setPathState(104);
                }
                break;

            case 103:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    odoDist = 75;

                    follower.followPath(firstPickup3, true);

                    setPathState(105);
                }
                break;
            case 104:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);


                    follower.followPath(firstPickup32, true);

                    setPathState(105);
                }
                break;
            case 105:


                if(!follower.isBusy()){
                    spindexer.stopIntake();
                    spindexer.startOuttake();

                    setPathState(13);
                }
                break;


            case 106:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.8);


                    follower.followPath(firstPickup4, true);

                    setPathState(102);
                }
                break;


            case 201:


                if (spindexer.outtakeStage==-1) {




                    follower.followPath(secondPickup1, true);

                    spindexer.startIntake();
                    setPathState(202);


                }
                break;

            case 202:

                if(pathTimer.getElapsedTimeSeconds()>1){
                    follower.setMaxPower(0.8);
                }
                if (!follower.isBusy()) {
                    angle=222;

                    follower.followPath(pickup2ToGate, true);
                    setPathState(203);

                }

                break;

            case 203:


                if (!follower.isBusy()) {



                    follower.setMaxPower(1);
                    follower.followPath(openRedGate2, true);

                    setPathState(205);

                }

                break;

            case 204:

                if(pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.setMaxPower(0.7);

                }
                if (!follower.isBusy()) {
                    angle=171.8;

                    follower.setMaxPower(1);
                    follower.followPath(secondPickup2, true);
                    setPathState(205);

                }
                break;

            case 205:

                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(301);
                }
                break;



            case 301:

                if (spindexer.outtakeStage==-1) {
                    follower.followPath(thirdPickup1, true);

                    follower.setMaxPower(0.7);
                    spindexer.startIntake();
                    setPathState(304);
                }
                break;

            case 302:
                if(pathTimer.getElapsedTimeSeconds()>0.5){
                    follower.setMaxPower(0.65);

                }
                if (!follower.isBusy()) {

                    angle = 185;
                    follower.followPath(pickup3ToGate, true);

                    setPathState(303);
                }
                break;
            case 303:
                if(pathTimer.getElapsedTimeSeconds()>0.8){

                    follower.setMaxPower(1);
                }

                if (!follower.isBusy()) {


                    follower.followPath(openRedGate3, true);

                    setPathState(305);
                }
                break;


            case 304:

                if(pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.setMaxPower(0.6);
                }
                if (!follower.isBusy()) {
                    angle = 224;
                    follower.setMaxPower(1);
                    follower.followPath(thirdPickup2, true);

                    setPathState(305);
                }
                break;

            case 305:
                if(pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.setMaxPower(1);
                }
                if (!follower.isBusy()) {

                    spindexer.stopIntake();
                    spindexer.startOuttake();
                    setPathState(101);
                }
                break;


            case 401:
                if (spindexer.outtakeStage==-1) {
                    angle=183;
                    testTimer.resetTimer();
                    follower.followPath(gateIntake1, true);
                    spindexer.startIntake();
                    setPathState(402);

                }
                break;

            case 402:


                if(pathTimer.getElapsedTimeSeconds()>1.55) {
                    follower.followPath(gateIntake2, true);
                    spindexer.startIntake();


                    setPathState(405);
                }
                break;
            case 403:

                if(!follower.isBusy()){

                    follower.followPath(gateIntake3 , true);
                    setPathState(404);
                }
                break;
            case 404:
                if(!follower.isBusy()){


                    follower.followPath(gateIntake4 , true);

                    setPathState(405);
                }
                break;

            case 405:
//                if(pathTimer.getElapsedTimeSeconds()>1){
//                    follower.setMaxPower(0.8);
//                }
                if(spindexer.intakeStage==-1||pathTimer.getElapsedTimeSeconds()>3.5){
                    follower.setMaxPower(1);

                    follower.followPath(gateIntake5 , true);

                    setPathState(406);
                }
                break;

            case 406:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    spindexer.stopIntake();
                }
                if(!follower.isBusy()){
                    spindexer.startOuttake();
                    setPathState(301);
                }
                break;

            case 501:
                odoDist = 75;


                if (spindexer.outtakeStage==-1) {
                    angle=170;
                    testTimer.resetTimer();
                    follower.followPath(randomPickup1, true);
                    spindexer.startIntake();
                    setPathState(502);

                }
                break;
            case 502:
                if(!follower.isBusy()){

                    follower.setMaxPower(0.7);

                    follower.followPath(randomPickup2 , true);

                    setPathState(503);
                }
                break;

            case 503:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(randomPickup3 , true);

                    setPathState(504);
                }
                break;


            case 504:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    spindexer.stopIntake();
                }
                if(!follower.isBusy()){
                    spindexer.startOuttake();
                    setPathState(1);
                }
                break;



            case 13:

                if(spindexer.outtakeStage==-1){
                    angle=0;
                    setPathState(14);
                }
                break;
            case 14:
                break;
        }
    }







    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
//        if(!targetMotif.equals("Null")&&spindexer.outtakeStage!=-1){
//            shooter.updateShootingParams(odoDist, 24, spindexer.outtakeStage != -1);
//
//        }

        if(!targetMotif.equals("Null")){
            shooter.updateTurret(angle);//0 is left max
        }
//
//        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif,true);

        autonomousPathUpdate();

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

        Pose currentPose = follower.getPose();
        Constant.AUTON_LAST_X = currentPose.getX() - 33.5;
        Constant.AUTON_LAST_Y = currentPose.getY() - 11.5;
        Constant.AUTON_LAST_HEADING_RAD = currentPose.getHeading();
        Constant.AUTON_LAST_HEADING_DEG = Math.toDegrees(Constant.AUTON_LAST_HEADING_RAD);

        // 6. TELEMETRY
        telemetry.addData("Spindexer Slots", slotVisual.toString());
        telemetry.addData("Intake Stage", spindexer.intakeStage);
        telemetry.addData("Outtake Stage", spindexer.outtakeStage);
        telemetry.addData("Velo Error", "%.1f", shooter.calculatedTargetVelocity - shooter.leftShooter.getVelocity());
        telemetry.addData("target ticks", spindexer.targetTicks);
        telemetry.addData("current ticks", spindexer.currentTicks);
        telemetry.addData("Motif",targetMotif);
        telemetry.addData("path state", pathState);
        telemetry.addData("TargetColor",spindexer.targetColor);
        telemetry.addData("maxPower",follower.getMaxPowerScaling());


        telemetry.addData("TestTimer",testTimer.getElapsedTimeSeconds());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }



    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Constant.ALLIANCE = "RED";

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        testTimer=new Timer();

        opmodeTimer.resetTimer();

        shooter = new Shooter(hardwareMap);
//        drive = new MecanumDrive(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

//        shooter.calculatedTargetVelocity = 1800;
        shooter.setTurretPosition(0.25);
//        shooter.setHoodPosition(Constant.HOOD_INIT);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(initPos);
//        drive.pinpoint.resetPosAndIMU();
    }









    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }

    public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();

    }


}


