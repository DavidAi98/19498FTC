package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.PIDCalculator;
import org.firstinspires.ftc.teamcode.opMode.Constant;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "NearBlue")
@Config
public class AutonNearBlue extends OpMode {
    private Follower follower;
    private PathChain  firstPickup1, firstPickup2, openBlueGate, firstPickupBreack1, firstPickupBreack2, secondPickup1, secondPickup2, thirdPickup1, thirdPickup2, pickup1ToGate, endPath;


    private Path preLoadShot;
    // Mirrored about x = 72 (x' = 144 - x). Headings mirrored: heading' = 180° - heading
    public static Pose initPos = new Pose(34.210, 136.840, Math.toRadians(180));
    public static Pose shootingPos = new Pose(48.000, 95.000, Math.toRadians(135));
    public static Pose pickup3Pos = new Pose(19, 84, Math.toRadians(180));
    public static Pose blueGate = new Pose(14.5, 73, Math.toRadians(180));
    public static Pose pickup1Pos = new Pose(9.5, 38, Math.toRadians(180));
    public static Pose pickup2Pos = new Pose(9, 62.3, Math.toRadians(180));
    public static Pose endPos1 = new Pose(46.3, 71.8, Math.toRadians(90));
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState, actionState;

    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);
    private DcMotorEx leftShooter, rightShooter;
    private DcMotor intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private HuskyLens huskylens;

    private Servo leftPivot, rightPivot, rightHood;
    private VoltageSensor voltageSensor;
    private double rx = 1;
    private double velocity = 0,  shooterPower = 0;
    private double targetVelocity = 1164;
    private double distance = 0;
    private double hood_position = Constant.rightHoodINIT;

    private double intakePower = 0, pivotPositionRight = Constant.rightPivotINIT, pivotPositionLeft = Constant.leftPivotINIT;
    private boolean shootComplete;
    private boolean shootNearOff, shootFarOff;
    private IMU imu;

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        if(!shootNearOff){
            shootNear();
        }
        if(!shootFarOff){
            shootFar();
        }

        leftShooter.setPower(shooterPower);
        rightShooter.setPower(shooterPower);

        leftPivot.setPosition(pivotPositionLeft);
        rightPivot.setPosition(pivotPositionRight);

        rightHood.setPosition(hood_position);
        intake.setPower(intakePower);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("action state", actionState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();

    }




    public void buildPaths() {

        preLoadShot = new Path(new BezierLine(initPos, shootingPos));
        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(), shootingPos.getHeading());

        firstPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(47.735, 28.641),
                        new Pose(46.939, 35.602),
                        pickup1Pos))
                .setLinearHeadingInterpolation(shootingPos.getHeading(),Math.toRadians(180),0.4)
                .build();

        firstPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup1Pos,
//                        new Pose(30.630, 44.155),
//                        new Pose(41.967, 46.740),
                        shootingPos))
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), shootingPos.getHeading())
                .build();

        secondPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(48.928, 65.834),
                        new Pose(44.751, 67.425),
                        new Pose(50.718, 58.276),
                        pickup2Pos))
//                .setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180),0.4)
                .build();

        secondPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup2Pos,
                        new Pose(54.1, 58.276),

                        shootingPos))
                .setLinearHeadingInterpolation(pickup2Pos.getHeading(), shootingPos.getHeading())
                .build();

        thirdPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootingPos,
                        new Pose(53.304, 82.939),
                        pickup3Pos))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        thirdPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pickup3Pos,
                        new Pose(49.724, 76.575),
                        shootingPos))
                .setLinearHeadingInterpolation(pickup3Pos.getHeading(), shootingPos.getHeading())
                .build();
        pickup1ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pos,
                        new Pose(36, 76.575),
                        blueGate))
                .setLinearHeadingInterpolation(pickup1Pos.getHeading(), blueGate.getHeading())
                .build();

        openBlueGate = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.762, 61.061),
//                        new Pose(75.381, 75.779),
//                        blueGate))
//                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(blueGate,

                        shootingPos))
                .setLinearHeadingInterpolation(blueGate.getHeading(), shootingPos.getHeading())
                .build();


        endPath = follower.pathBuilder()
                .addPath(new BezierLine(shootingPos,endPos1))
                .setLinearHeadingInterpolation(shootingPos.getHeading(),endPos1.getHeading())
                .build();




    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                intakePower=0.5;

                follower.followPath(preLoadShot);

                setPathState(1);

                break;

            case 1:

                intakePower=0.5;

                setActionState(0);
                shootNearOff = false;
                if(!follower.isBusy()) {

                    setPathState(2);

                }

                break;

            case 2:

                intakePower=1;

                shootThree();

                if(shootNearOff) {

                    follower.followPath(thirdPickup1,true);
                    setPathState(3);
                }
                break;

            case 3:

                intakePower =1;
                if(!follower.isBusy()){

                    follower.followPath(pickup1ToGate);
                    setPathState(4);

                }

                break;

            case 4:

                intakePower = 0.5;
                if(!follower.isBusy()){

                    follower.followPath(openBlueGate,true);
                    setPathState(5);
                }

                break;


            case 5:
                intakePower=0.5;
                setActionState(0);
                if(!follower.isBusy()) {
                    shootNearOff = false;
                    setPathState(6);
                }
                break;

            case 6:

                intakePower = 1;
                shootThree();
                if(shootNearOff){
                    follower.followPath(secondPickup1,true);
                    setPathState(7);
                }

                break;
            case 7:
                intakePower=1;
                if(!follower.isBusy()) {
                    follower.followPath(secondPickup2,true);
                    setPathState(8);
                }
                break;

            case 8:

                intakePower=0.5;
                setActionState(0);//
                if(!follower.isBusy()){
                    shootNearOff = false;
                    setPathState(9);
                }
                break;
            case 9:
                intakePower = 1;
                shootThree();
                if(shootNearOff){
                    follower.followPath(firstPickup1,true);
                    setPathState(10);
                }
                break;
            case 10:
                intakePower = 1;
                if(!follower.isBusy()){
                    follower.followPath(firstPickup2);
                    setPathState(11);
                }
                break;
            case 11:
                intakePower=0.5;
                setActionState(0);
                if(!follower.isBusy()){
                    shootNearOff = false;
                    setPathState(12);
                }
                break;
            case 12:
                intakePower = 1;
                shootThree();
                if(shootNearOff){
                    follower.followPath(endPath,true);
                    setPathState(13);
                }
                break;
            case 13:
                break;

        }
    }



    public void shootNear() {

        hood_position = 0.3;
        velocity = rightShooter.getVelocity();
        double output = pidNear.calculator(targetVelocity, velocity);
        shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);

//        HuskyLens.Block[] blocks = huskylens.blocks();
//        if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
//            distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
//            targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
//        }
    }

    public boolean nearError(){

        velocity = rightShooter.getVelocity();
        return Math.abs(velocity-targetVelocity)<75;

    }

    public void shootFar(){
        hood_position = 0.4;
        velocity = rightShooter.getVelocity();
        double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
        shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);

    }

    public void pivotControl(Timer actionTimer){

        if(actionTimer.getElapsedTimeSeconds()<0.3 && shooterPower!=0){
            shootComplete=false;
            pivotPositionLeft = Constant.leftPivotActivated;
            pivotPositionRight = Constant.rightPivotActivated;
        }else{
            pivotPositionLeft = Constant.leftPivotINIT;
            pivotPositionRight = Constant.rightPivotINIT;

        }
        if(actionTimer.getElapsedTimeSeconds()>1){
            shootComplete = true;
        }
    }
//    public boolean ifTurned(double targetAngle,double current,double tolar){
//        return Math.abs(targetAngle-current)<tolar;
//    }

    public void shootThree(){
        switch (actionState){


            case 0:
                if(nearError()){
                    setActionState(1);
                }


                break;
            case 1:
                pivotControl(actionTimer);
                if(shootComplete){
                    setActionState(2);
                }
                break;
            case 2:
                pivotControl(actionTimer);
                if(shootComplete){
                    setActionState(3);
                }
                break;
            case 3:
                pivotControl(actionTimer);
                if(actionTimer.getElapsedTimeSeconds()>0.5){
                    setActionState(4);
                }
                break;
            case 4:
                intakePower = 0;
                shootNearOff = true;
                shootFarOff = true;

                break;

        }
    }


    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

    }

    public void setActionState(int aState){
        actionState = aState;
        actionTimer.resetTimer();
        pidNear.reset();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();

        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
        rightHood = hardwareMap.get(Servo.class, "RightHood");
        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(initPos);

        rightHood.setPosition(Constant.rightHoodINIT);

        rightPivot.setPosition(Constant.rightPivotINIT);
        leftPivot.setPosition(Constant.leftPivotINIT);


        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        imu.resetYaw();
        shootFarOff=true;
        shootNearOff=true;


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
        setActionState(0);


    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}