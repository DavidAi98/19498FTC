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


@Autonomous(name = "BlueTest")
public class NearBlueTest extends OpMode {

    private Follower follower;
    private PathChain firstPickup1, firstPickup2, secondPickup1, secondPickup2, openRedGate2, firstPickupBreak1, firstPickupBreak2, endPath, secondPickupBreak1, secondPickupBreak2, pickup2ToGate, thirdPickup1, thirdPickup2,
            gateIntake1,gateIntake2,gateIntake3,gateIntake4,gateIntake5,
            firstPickupTest1,firstPickupTest2,firstPickupTest3,pickup2ToGate0,openRedGate20,randomIntake1,randomIntake2,randomIntake3;
    private Path preLoadShot;
    public static Pose initPos = new Pose(31.4, 135.8, Math.toRadians(270));
    //    public static Pose initPos = new Pose(33.000, 136.685, Math.toRadians(180));
    public static Pose shootingPos = new Pose(51.2, 84.133, Math.toRadians(260));
    //    public static Pose shootingPos = new Pose(50.387, 84.133, Math.toRadians(180));
    public static Pose redGate = new Pose(15, 69.9, Math.toRadians(270));
    public static Pose redGate0 =new Pose(14.5, 70.4, Math.toRadians(180));
    public static Pose pickup1Pos = new Pose(10.5, 3.4, Math.toRadians(180));
    public static Pose pickup2Pos = new Pose(10.5, 60.282, Math.toRadians(180));
    public static Pose pickup3Pos = new Pose(17.6, 84.04, Math.toRadians(180));
    public static Pose endPos1 = new Pose(56.619, 36.669, Math.toRadians(180));
    public static Pose shooting2Pos = new Pose(60, 102, Math.toRadians(225));

    public static Pose gatePickup1 = new Pose(16,68.591,Math.toRadians(185));
    public static Pose gatePickup2 = new Pose(11.138,52.906,Math.toRadians(100));
    public static Pose gatePickup3 = new Pose(9.873,58.608,Math.toRadians(110));
    public static Pose gatePickup4 = new Pose(16.702,59.873,Math.toRadians(110));
    public static Pose randomPickup1 = new Pose(9.148,23.072,Math.toRadians(250));
    public static Pose randomPickup2 = new Pose(9.238,9.524,Math.toRadians(250));


    public static Pose pickup1PosTest = new Pose(9.149, 35.403,Math.toRadians(180));

    private Timer pathTimer, actionTimer, opmodeTimer,testTimer;

    private int pathState,actionState;

    private Shooter shooter;
    private Spindexer spindexer;



    private double odoDist =75;
    private double angle = 40;//40


    private String targetMotif = "Null";









    public void buildPaths() {

        preLoadShot = new Path(new BezierLine(initPos, shootingPos));
        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());

        pickup2ToGate0 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pos,
                        new Pose(36.025, 63.124),
                        redGate0
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate0.getHeading())
                .build();


        openRedGate20 = follower.pathBuilder()
                .addPath(new BezierCurve(redGate0,
                        new Pose(48, 66),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate0.getHeading())
                .build();

        firstPickupTest1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(48.681, 46.08),
                        new Pose(49, 35.953),
                        new Pose(20.677, 33.326),
                        pickup1PosTest
                ))
                .setTangentHeadingInterpolation()
                .build();


        firstPickupTest2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickup1PosTest,
                        new Pose(16.398, 43.994)
                ))
                .setLinearHeadingInterpolation(pickup1PosTest.getHeading(),Math.toRadians(180))
                .build();

        firstPickupTest3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(16.398, 43.994),
                        shooting2Pos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        firstPickupBreak1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        redGate,
                        new Pose(64.840, 73.591),
                        new Pose(62.652, 54.895)
                ))
                .setLinearHeadingInterpolation(redGate.getHeading(), redGate.getHeading())
                .build();

        firstPickupBreak2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(62.652, 54.895),   // from (81.348,54.895)
                        new Pose(65.436, 33.017),   // from (78.564,33.017)
                        pickup1Pos))
                .setLinearHeadingInterpolation(redGate.getHeading(), pickup1Pos.getHeading())
                .build();

        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(49.088, 43.892),
                        new Pose(57.210, 32.970),
                        pickup1Pos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

        firstPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickup1Pos,

                        shooting2Pos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), shootingPos.getHeading())
                .build();


        pickup2ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pos,
                        new Pose(19.318,64.3),
                        redGate
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), redGate.getHeading())
                .build();


        openRedGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(redGate,
                        new Pose(39.76,67.98),
                        shootingPos
                ))
                .setLinearHeadingInterpolation(redGate.getHeading(),Math.toRadians(180))
                .build();

        secondPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(43, 63.45),
                        new Pose(37.193, 57.5),
                        pickup2Pos))
                .setTangentHeadingInterpolation()

                .build();

        secondPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pos,
                        new Pose(25.939,59.171),
                        shootingPos))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();



        thirdPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootingPos,

                        pickup3Pos))
                .setTangentHeadingInterpolation()
                .build();


        thirdPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pickup3Pos,

                        shootingPos))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        endPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        shootingPos,

                        endPos1
                ))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

        gateIntake1= follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,

                        new Pose(27.5, 68.069),
                        gatePickup1
                ))
                .setTangentHeadingInterpolation()
                .build();

        gateIntake2= follower.pathBuilder()
                .addPath(new BezierCurve(
                        gatePickup1,

                        new Pose(25.945,69.456),
                        new Pose(19.433,58.906),
                        gatePickup2
                ))
                .setLinearHeadingInterpolation( gatePickup1.getHeading(),  gatePickup2.getHeading())
                .build();
        gateIntake3= follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup2,

                        gatePickup3
                ))
                .setLinearHeadingInterpolation(gatePickup2.getHeading(), gatePickup3.getHeading())
                .build();

        gateIntake4= follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup3,

                        gatePickup4
                ))
                .setLinearHeadingInterpolation(gatePickup3.getHeading(), gatePickup4.getHeading())
                .build();

        gateIntake5= follower.pathBuilder()
                .addPath(new BezierLine(
                        gatePickup4,

                        shootingPos
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        randomIntake1= follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,

                        new Pose(11.652, 34.699),
                        new Pose(10.851, 26.914),
                        randomPickup1
                ))
                .setTangentHeadingInterpolation()
                .build();
        randomIntake2= follower.pathBuilder()
                .addPath(new BezierLine(
                        randomPickup1,

                        randomPickup2
                ))
                .setLinearHeadingInterpolation(randomPickup2.getHeading(), randomPickup2.getHeading())
                .build();
        randomIntake3= follower.pathBuilder()
                .addPath(new BezierLine(
                        randomPickup2,

                        shooting2Pos
                ))
                .setTangentHeadingInterpolation()
                .build();

    }




    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:


                follower.followPath(preLoadShot);
                spindexer.startIntake();
                setActionState(0);
                setPathState(1);

                break;

            case 1:

                targetMotif = shooter.detectMotif();

                if (!follower.isBusy() && !targetMotif.equals("Null")) {
                    angle=320;
                    setActionState(0);
                    setPathState(99);

                }

                break;

            case 99:
                if(spindexer.intakeStage==-1&&pathTimer.getElapsedTimeSeconds()>0.5){
                    spindexer.startOuttake();
                    setPathState(2);
                }
                break;
            case 88:
                if (spindexer.outtakeStage==-1&&pathTimer.getElapsedTimeSeconds()>1) {
                    setActionState(0);
                    angle=10;
                    testTimer.resetTimer();
                    follower.followPath(gateIntake1, true);
                    spindexer.startIntake();
                    setPathState(77);


                }
                break;

            case 77:
                if(pathTimer.getElapsedTimeSeconds()>1.6){
                    follower.setMaxPower(0.8);
                }

                if (!follower.isBusy()) {
                    spindexer.startIntake();
                    follower.followPath(gateIntake2, true);

                    setActionState(0);
                    setPathState(66);
                }
                break;
            case 66:
                if(!follower.isBusy()){

                    follower.followPath(gateIntake3 , true);
                    setPathState(55);
                }
                break;
            case 55:
                if(!follower.isBusy()){


                    follower.followPath(gateIntake4 , true);

                    setPathState(44);
                }
                break;

            case 44:
                if(!follower.isBusy()){
                    follower.setMaxPower(1);


                    follower.followPath(gateIntake5 , true);

                    setPathState(33);
                }
                break;

            case 33:
                if(pathTimer.getElapsedTimeSeconds()>1){
                    spindexer.stopIntake();
                }
                if(!follower.isBusy()){


                    spindexer.startOuttake();
                    setPathState(6);
                }
                break;

            case 2:


                if (spindexer.outtakeStage==-1&&pathTimer.getElapsedTimeSeconds()>0.2) {
                    setActionState(0);
                    angle=355;
                    follower.followPath(secondPickup1, true);

                    spindexer.startIntake();
                    setPathState(4);


                }
                break;

            case 3://

                if(pathTimer.getElapsedTimeSeconds()>1){
                    follower.setMaxPower(0.6);
                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    follower.followPath(pickup2ToGate0, true);
                    setPathState(4);

                }

                break;

            case 4:



                if(pathTimer.getElapsedTimeSeconds()>1){
                    follower.setMaxPower(0.6);
                }

                if (!follower.isBusy()) {

                    follower.followPath(secondPickup2, true);


                    setActionState(0);
                    setPathState(5);

                }

                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.setMaxPower(1);
                    spindexer.stopIntake();
                }


                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    spindexer.startOuttake();
                    setPathState(88);

                }
                break;

            case 6:

                if (spindexer.outtakeStage==-1) {
                    follower.followPath(thirdPickup1, true);
                    setActionState(0);

                    spindexer.startIntake();
                    setPathState(7);
                }

                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.8){
                    follower.setMaxPower(0.7);


                }
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);

                    follower.followPath(thirdPickup2, true);
//                    setActionState(0);
                    setPathState(8);
                }
                break;

            case 8:


                if(pathTimer.getElapsedTimeSeconds()>2){
                    spindexer.stopIntake();
                }
                if (!follower.isBusy()&&spindexer.intakeStage==-1) {

                    spindexer.startOuttake();

                    setActionState(0);
                    setPathState(9);
                }
                break;
            case 9:

                if (spindexer.outtakeStage==-1) {
                    odoDist = 75;
                    angle=60+180-Math.toDegrees(shooting2Pos.getHeading());
                    testTimer.resetTimer();
                    follower.followPath(firstPickupTest1, true);
                    spindexer.startIntake();
                    setActionState(0);

                    setPathState(10);
                }
                break;
            case 10:
                if(pathTimer.getElapsedTimeSeconds()>1.6){
                    follower.setMaxPower(0.8);
                }

                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(firstPickupTest2, true);

                    setActionState(0);
                    setPathState(100);
                }
                break;

            case 100:
                if (!follower.isBusy()) {
                    spindexer.stopIntake();
                    follower.setMaxPower(1);
                    follower.followPath(firstPickupTest3, true);

                    setActionState(0);
                    setPathState(11);
                }
                break;
            case 11:

                if(pathTimer.getElapsedTimeSeconds()>0.6){
                    spindexer.stopIntake();
                }
                if(!follower.isBusy()){


                    setActionState(0);
                    setPathState(12);
                }
                break;
            case 12:

                if (spindexer.outtakeStage==-1) {


                    setPathState(13);

                }
                break;
            case 13:
                break;
        }
    }








    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        if(!targetMotif.equals("Null")&&spindexer.outtakeStage!=-1){
            shooter.updateShootingParams(odoDist, 24, spindexer.outtakeStage != -1);

        }
        if(!targetMotif.equals("Null")){
            shooter.updateTurret(360-angle);//0 is left max
        }

        shooter.runShooter(spindexer.outtakeStage != -1);
        spindexer.update(targetMotif,shooter.isReady());

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
        Constant.AUTON_LAST_X = 103 - currentPose.getX();
        Constant.AUTON_LAST_Y = 3 - currentPose.getY(); // close blue should be similar
        Constant.AUTON_LAST_HEADING_RAD = currentPose.getHeading() - Math.PI;
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
        Constant.ALLIANCE = "BLUE";

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        testTimer=new Timer();

        opmodeTimer.resetTimer();

        shooter = new Shooter(hardwareMap);
//        drive = new MecanumDrive(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

//        shooter.calculatedTargetVelocity = 1800;
        shooter.setTurretPosition(0.25
                );
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


