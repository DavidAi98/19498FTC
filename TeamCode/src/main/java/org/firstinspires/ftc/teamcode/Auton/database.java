//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.PIDCalculator;
//import org.firstinspires.ftc.teamcode.opMode.Constant;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "FarBlue")
//@Config
//public class AutonFarBlue extends OpMode {
//    private Follower follower;
//    private Path preLoadShot;
//    private PathChain  goingFirstPickup, firstPickup1,firstPickup2, firstPickupShot,randomPickup, openBlueGate1,openBlueGate2,firstPickupBreack1,firstPickupBreack2,endPath,secondPickup1,secondPickup2;
//
//
//    public static Pose initPos = new Pose(65,8,Math.toRadians(90));
//    public static Pose shootingPos = new Pose(65, 17.105,Math.toRadians(111));
//    public static Pose redGate = new Pose(24.519, 70.608,Math.toRadians(180));
//    public static Pose pickup1Pos = new Pose (24,35,Math.toRadians(180));
//    public static Pose pickup2Pos = new Pose (24, 59.470,Math.toRadians(180));
//    public static Pose randomPickupPos = new Pose(12,28,Math.toRadians(137));
//    public static Pose endPos1 = new Pose(24,68,Math.toRadians(90));
//
//
//
//
//
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState,actionState;
//
//    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);
//    private DcMotorEx leftShooter, rightShooter;
//    private DcMotor intake;
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    private HuskyLens huskylens;
//
//    private Servo leftPivot, rightPivot, rightHood;
//    private VoltageSensor voltageSensor;
//    private double rx = 1;
//    private double velocity = 0,  shooterPower = 0;
//    private double targetVelocity = 500;
//    private double distance = 0;
//    private double hood_position = Constant.rightHoodINIT;
//
//    private double intakePower = 0,pivotPositionRight=Constant.rightPivotINIT,pivotPositionLeft = Constant.leftPivotINIT;
//
//    private boolean shootComplete;
//    private boolean shootNearOff=true,shootFarOff=true;
//
//    private IMU imu;
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        if(!shootNearOff){
//            shootNear();
//        }else if(!shootFarOff){
//            shootFar();
//        }
//
//        leftShooter.setPower(shooterPower);
//        rightShooter.setPower(shooterPower);
//
//        leftPivot.setPosition(pivotPositionLeft);
//        rightPivot.setPosition(pivotPositionRight);
//
//        rightHood.setPosition(hood_position);
//        intake.setPower(intakePower);
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("action state", actionState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//
//        telemetry.update();
//
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
//        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
//        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
//        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
//        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
//        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
//        imu = hardwareMap.get(IMU.class, "imu");
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
//
//        rightHood.setPosition(Constant.rightHoodINIT);
//
//        rightPivot.setPosition(Constant.rightPivotINIT);
//        leftPivot.setPosition(Constant.leftPivotINIT);
//
//        shootNearOff =true;
//
//        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//        );
//        imu.initialize(new IMU.Parameters(orientation));
//
//        imu.resetYaw();
//
//
//    }
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
//        setActionState(0);
//
//
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//
//
//    public void buildPaths() {
//
//
//
//        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
//        preLoadShot=new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(),shootingPos.getHeading());
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(95.271, 35.204),
//                        new Pose(80.155, 42.166),
//                        pickup1Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup1Pos,
//                        new Pose(48.729, 35.204),
//                        new Pose(63.845, 42.166),
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//        openBlueGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(65.238, 61.061),
//                        new Pose(68.619, 75.779),
//                        redGate))
//                .setTangentHeadingInterpolation()
//
//                .build();
//
//        openBlueGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(65.635, 73.790),
//                        new Pose(65.238, 68.022),
//                        initPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(65.834, 69.215),
//                        new Pose(53.702, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup2Pos,
//                        new Pose(55.89, 62.055),
//                        new Pose(61.657, 61.657),
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//        endPath = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(83.138, 72.994),
//                        endPos1
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//
//
//
//    }
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//
//                follower.followPath(preLoadShot);
//                setActionState(0);
//                shootFarOff = false;
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//                if(!follower.isBusy()) {
//                    setPathState(2);
//                }
//
//                break;
//
//            case 2:
//
//                intakePower=1;
//                shootThree();
//
//                if(shootFarOff) {
//                    follower.followPath(openBlueGate1,true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//
//                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>2){
//
//                    follower.followPath(openBlueGate2, true);
//                    setPathState(4);
//
//                }
//
//                break;
//
//            case 4:
//
//                if(!follower.isBusy()){
//                    follower.followPath(secondPickup1);
//                    setPathState(5);
//                }
//
//                break;
//            case 5:
//
//                intakePower = 1;
//                if(!follower.isBusy()){
//                    follower.followPath(secondPickup2);
//                    setPathState(6);
//
//                }
//
//                break;
//            case 6:
//
//                intakePower =0;
//                if(!follower.isBusy()) {
//                    follower.turnTo(Math.toRadians(111));
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//
//                if(!follower.isBusy()){
//                    shootFarOff = false;
//                    setActionState(0);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(firstPickup1,true);
//                    setPathState(8);
//
//                }
//
//                break;
//            case 8:
//                intakePower=1;
//                if(!follower.isBusy()) {
//                    follower.followPath(firstPickup2,true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                intakePower=0;
//                if(!follower.isBusy()) {
//                    follower.turnTo(Math.toRadians(111));
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if(!follower.isBusy()){
//                    shootFarOff = false;
//                    setActionState(0);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(endPath);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                break;
//
//        }
//    }
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//
//    }
//
//    public void setActionState(int aState){
//        actionState = aState;
//        actionTimer.resetTimer();
//        pidNear.reset();
//    }
//
//    public void shootNear() {
//
//        hood_position = 0.3;
//        velocity = rightShooter.getVelocity();
//        double output = pidNear.calculator(targetVelocity, velocity);
//        shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);
//
//        HuskyLens.Block[] blocks = huskylens.blocks();
//        if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
//            distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
//            targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
////            rx = Math.abs(blocks[0].x - Constant.targetPosition) > 10 ? aiming.calculator(Constant.targetPosition, blocks[0].x) : 0;
//        }
//    }
//
//    public void shootFar(){
//        hood_position = 0.4;
//        velocity = rightShooter.getVelocity();
//        double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
//        shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);
//
//    }
//
//    public void pivotControl(Timer actionTimer){
//
//        if(actionTimer.getElapsedTimeSeconds()<0.5 && shooterPower!=0){
//            shootComplete=false;
//            pivotPositionLeft = Constant.leftPivotActivated;
//            pivotPositionRight = Constant.rightPivotActivated;
//        }else{
//            pivotPositionLeft = Constant.leftPivotINIT;
//            pivotPositionRight = Constant.rightPivotINIT;
//
//        }
//        if(actionTimer.getElapsedTimeSeconds()>1.5){
//            shootComplete = true;
//        }
//    }
//    public boolean ifTurned(double targetAngle,double current,double tolar){
//        return Math.abs(targetAngle-current)<tolar;
//    }
//
//    public void shootThree(){
//        switch (actionState){
//            case 0:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(1);
//                }
//                break;
//            case 1:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(2);
//                }
//                break;
//            case 2:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(3);
//                }
//                break;
//            case 3:
//                intakePower = 0;
//                shootNearOff =true;
//                shootFarOff = true;
//                break;
//        }
//    }
//
//
//
//
//}
//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.PIDCalculator;
//import org.firstinspires.ftc.teamcode.opMode.Constant;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "FarRed")
//@Config
//public class AutonFarRed extends OpMode {
//    private Follower follower;
//    private PathChain gateToPickup1, firstPickup1,firstPickup2, secondPickup1,secondPickup2,firstPickupShot,randomPickup, openRedGate1,openRedGate2, firstPickupBreak1, firstPickupBreak2,endPath,secondPickupBreak1,secondPickupBreak2;
//
//
//    private Path preLoadShot;
//    public static Pose initPos = new Pose(79,8,Math.toRadians(90));
//    public static Pose shootingPos = new Pose(77, 20,Math.toRadians(64));
//    public static Pose redGate = new Pose(139.481, 70.608,Math.toRadians(0));
//    public static Pose pickup1Pos = new Pose (132,34.5,Math.toRadians(0));
//    public static Pose pickup2Pos = new Pose (131.271, 59.470,Math.toRadians(0));
//    public static Pose randomPickupPos = new Pose(132,28,Math.toRadians(43));
//    public static Pose endPos1 = new Pose(120,70,Math.toRadians(90));
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState,actionState;
//
//    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);
//    private DcMotorEx leftShooter, rightShooter;
//    private DcMotor intake;
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    private HuskyLens huskylens;
//
//    private Servo leftPivot, rightPivot, rightHood;
//    private VoltageSensor voltageSensor;
//    private double rx = 1;
//    private double velocity = 0,  shooterPower = 0;
//    private double targetVelocity = 500;
//    private double distance = 0;
//    private double hood_position = Constant.rightHoodINIT;
//
//    private double intakePower = 0,pivotPositionRight=Constant.rightPivotINIT,pivotPositionLeft = Constant.leftPivotINIT;
//    private boolean shootComplete;
//    private boolean shootNearOff,shootFarOff;
//    private IMU imu;
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        if(!shootNearOff){
//            shootNear();
//        }
//        if(!shootFarOff){
//            shootFar();
//        }
//
//        leftShooter.setPower(shooterPower);
//        rightShooter.setPower(shooterPower);
//
//        leftPivot.setPosition(pivotPositionLeft);
//        rightPivot.setPosition(pivotPositionRight);
//
//        rightHood.setPosition(hood_position);
//        intake.setPower(intakePower);
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("action state", actionState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//
//        telemetry.update();
//
//    }
//
//
//
//
//    public void buildPaths() {
//
//        preLoadShot=new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(),shootingPos.getHeading());
//
//
//        firstPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        redGate,
//                        new Pose(79.160, 73.591),
//                        new Pose(81.348, 54.895)
//                ))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(81.348, 54.895),
//                        new Pose(78.564, 33.017),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(79.359, 43.558),
//                        new Pose(95.271, 35.403),
//                        pickup1Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup1Pos,
//                        new Pose(95.271, 35.204),
//                        new Pose(80.155, 42.166),
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//        gateToPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(51.713, 78.365),
//                        new Pose(101.834, 46.740),
//                        new Pose(55.890, 36.796),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        secondPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(75.779, 57.282),
//                        new Pose(95.470, 59.470)
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(95.470, 59.470),
//                        new Pose(116.950, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.166, 69.215),
//                        new Pose(90.298, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup2Pos,
//                        new Pose(88.110, 62.055),
//                        new Pose(82.343, 61.657),
//                        shootingPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//
//        openRedGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.762, 61.061),
//                        new Pose(77.381, 75.779),
//                        redGate))
//                .setTangentHeadingInterpolation()
//                .build();
//        openRedGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(78.365, 73.790),
//                        new Pose(79.160, 68.022),
//                        initPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        endPath = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(83.138, 72.994),
//                        endPos1
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//
//                follower.followPath(preLoadShot);
//                shootFarOff = false;
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//
//                setActionState(0);
//
//                if(!follower.isBusy()) {
//                    setPathState(2);
//
//                }
//
//                break;
//
//            case 2:
//
//                intakePower=1;
//
//                shootThree();
//
//                if(shootFarOff) {
//
//                    follower.followPath(openRedGate1,true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//
//                if(!follower.isBusy()){
//                    follower.followPath(firstPickupBreak1, true);
//                    setPathState(4);
//
//                }
//
//                break;
//
//            case 4:
//
//                if(!follower.isBusy()){
//                    follower.followPath(firstPickupBreak2,true);
//                    setPathState(5);
//                }
//
//                break;
//            case 5:
//
//                follower.setMaxPower(0.7);
//                intakePower = 1;
//                if(!follower.isBusy()){
//                    follower.followPath(firstPickup2);
//                    setPathState(6);
//
//                }
//
//                break;
//            case 6:
//
//                follower.setMaxPower(1);
//                intakePower=0.5;
//                if(!follower.isBusy()) {
//                    follower.turn(0.47,false);
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//
//
//                setActionState(0);
//                if(follower.getHeadingError()<Math.toRadians(0.5)&&pathTimer.getElapsedTimeSeconds()>1){
//
//                    shootFarOff = false;
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(secondPickupBreak1,true);
//                    setPathState(8);
//                }
//
//                break;
//            case 8:
//                intakePower=0;
//                if(!follower.isBusy()) {
//                    follower.followPath(secondPickupBreak2,true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                intakePower=1;
//                follower.setMaxPower(0.7);
//                if(!follower.isBusy()) {
//                    follower.followPath(secondPickup2,true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//
//                intakePower=0.4;
//                follower.setMaxPower(1);
//                if(!follower.isBusy()){
//                    follower.turn(0.47,false);
//
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                setActionState(0);
//                if(follower.getHeadingError()<Math.toRadians(0.5)&&pathTimer.getElapsedTimeSeconds()>1){
//
//                    shootFarOff = false;
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(endPath,true);
//                    setPathState(15);
//                }
//                break;
//            case 15:
//                shootFarOff=true;
//                break;
//        }
//    }
//
//
//
//    public void shootNear() {
//
//        hood_position = 0.3;
//        velocity = rightShooter.getVelocity();
//        double output = pidNear.calculator(targetVelocity, velocity);
//        shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);
//
//        HuskyLens.Block[] blocks = huskylens.blocks();
//        if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
//            distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
//            targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
////            rx = Math.abs(blocks[0].x - Constant.targetPosition) > 10 ? aiming.calculator(Constant.targetPosition, blocks[0].x) : 0;
//        }
//    }
//
//    public void shootFar(){
//        hood_position = 0.4;
//        velocity = rightShooter.getVelocity();
//        double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
//        shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);
//
//    }
//    public boolean farError(){
//        velocity = rightShooter.getVelocity();
//        return Math.abs(velocity-Constant.targetVelocityFar)<100;
//    }
//
//    public void pivotControl(Timer actionTimer){
//
//        if(actionTimer.getElapsedTimeSeconds()<0.3 && shooterPower!=0){
//            shootComplete=false;
//            pivotPositionLeft = Constant.leftPivotActivated;
//            pivotPositionRight = Constant.rightPivotActivated;
//        }else{
//            pivotPositionLeft = Constant.leftPivotINIT;
//            pivotPositionRight = Constant.rightPivotINIT;
//
//        }
//        if(actionTimer.getElapsedTimeSeconds()>1.5){
//            shootComplete = true;
//        }
//    }
////    public boolean ifTurned(double targetAngle,double current,double tolar){
////        return Math.abs(targetAngle-current)<tolar;
////    }
//
//    public void shootThree(){
//        switch (actionState){
//
//
//            case 0:
//                if(farError()){
//                    setActionState(1);
//                }
//
//
//                break;
//            case 1:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(2);
//                }
//                break;
//            case 2:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(3);
//                }
//                break;
//            case 3:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(4);
//                }
//                break;
//            case 4:
//                intakePower = 0;
//                shootNearOff = true;
//                shootFarOff = true;
//
//                break;
//
//        }
//    }
//
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//
//    }
//
//    public void setActionState(int aState){
//        actionState = aState;
//        actionTimer.resetTimer();
//        pidNear.reset();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
//        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
//        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
//        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
//        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
//        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
//        imu = hardwareMap.get(IMU.class, "imu");
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
//
//        rightHood.setPosition(Constant.rightHoodINIT);
//
//        rightPivot.setPosition(Constant.rightPivotINIT);
//        leftPivot.setPosition(Constant.leftPivotINIT);
//
//        shootNearOff =true;
//
//        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//        );
//        imu.initialize(new IMU.Parameters(orientation));
//
//        imu.resetYaw();
//        shootFarOff=true;
//        shootNearOff=true;
//
//
//    }
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
//        setActionState(0);
//
//
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//}
//
//
//
//
//
//
//

//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.PIDCalculator;
//import org.firstinspires.ftc.teamcode.opMode.Constant;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "FarRed")
//@Config
//public class AutonFarRed extends OpMode {
//    private Follower follower;
//    private PathChain gateToPickup1, firstPickup1,firstPickup2, secondPickup1,secondPickup2,firstPickupShot,randomPickup, openRedGate1,openRedGate2, firstPickupBreak1, firstPickupBreak2,endPath,secondPickupBreak1,secondPickupBreak2,pickup2ToGate;
//
//
//    private Path preLoadShot;
//    public static Pose initPos = new Pose(79,8,Math.toRadians(90));
//    public static Pose shootingPos = new Pose(77, 20,Math.toRadians(64));
//    public static Pose redGate = new Pose(139.481, 70.608,Math.toRadians(0));
//    public static Pose pickup1Pos = new Pose (135,34.5,Math.toRadians(0));
//    public static Pose pickup2Pos = new Pose (135, 59.470,Math.toRadians(0));
//    public static Pose randomPickupPos = new Pose(132,28,Math.toRadians(43));
//    public static Pose endPos1 = new Pose(120,70,Math.toRadians(90));
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState,actionState;
//
//    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);
//    private DcMotorEx leftShooter, rightShooter;
//    private DcMotor intake;
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    private HuskyLens huskylens;
//
//    private Servo leftPivot, rightPivot, rightHood;
//    private VoltageSensor voltageSensor;
//    private double rx = 1;
//    private double velocity = 0,  shooterPower = 0;
//    private double targetVelocity = 500;
//    private double distance = 0;
//    private double hood_position = Constant.rightHoodINIT;
//
//    private double intakePower = 0,pivotPositionRight=Constant.rightPivotINIT,pivotPositionLeft = Constant.leftPivotINIT;
//    private boolean shootComplete;
//    private boolean shootNearOff,shootFarOff;
//    private IMU imu;
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        if(!shootNearOff){
//            shootNear();
//        }
//        if(!shootFarOff){
//            shootFar();
//        }
//
//        leftShooter.setPower(shooterPower);
//        rightShooter.setPower(shooterPower);
//
//        leftPivot.setPosition(pivotPositionLeft);
//        rightPivot.setPosition(pivotPositionRight);
//
//        rightHood.setPosition(hood_position);
//        intake.setPower(intakePower);
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("action state", actionState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//
//        telemetry.update();
//
//    }
//
//
//
//
//    public void buildPaths() {
//
//        preLoadShot=new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(),shootingPos.getHeading());
//
//
//        firstPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        redGate,
//                        new Pose(79.160, 73.591),
//                        new Pose(81.348, 54.895)
//                ))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(81.348, 54.895),
//                        new Pose(78.564, 33.017),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(79.359, 43.558),
//                        new Pose(95.271, 35.403),
//                        pickup1Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup1Pos,
//                        new Pose(95.271, 35.204),
//                        new Pose(80.155, 42.166),
//                        shootingPos))
////                .setTangentHeadingInterpolation()
////                .setReversed()
//                .setLinearHeadingInterpolation(0,shootingPos.getHeading())
//                .build();
//
//
//        pickup2ToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pos,
//                        new Pose(105.017, 56.088),
//                        new Pose(107.602, 77.967),redGate
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//        gateToPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(51.713, 78.365),
//                        new Pose(101.834, 46.740),
//                        new Pose(55.890, 36.796),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        secondPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(75.779, 57.282),
//                        new Pose(95.470, 59.470)
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(95.470, 59.470),
//                        new Pose(116.950, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.166, 69.215),
//                        new Pose(90.298, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup2Pos,
//                        new Pose(88.110, 62.055),
//                        new Pose(82.343, 61.657),
//                        shootingPos))
////                .setTangentHeadingInterpolation()
////                .setReversed()
//                .setLinearHeadingInterpolation(0,shootingPos.getHeading())
//                .build();
//
//
//        openRedGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.762, 61.061),
//                        new Pose(77.381, 75.779),
//                        redGate))
//                .setTangentHeadingInterpolation()
//                .build();
//        openRedGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(78.365, 73.790),
//                        new Pose(79.160, 68.022),
//                        initPos))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//
//        endPath = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(83.138, 72.994),
//                        endPos1
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//
//                follower.followPath(preLoadShot);
//                shootFarOff = false;
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//
//                setActionState(0);
//
//                if(!follower.isBusy()) {
//                    setPathState(2);
//
//                }
//
//                break;
//
//            case 2:
//
//                intakePower=1;
//
//                shootThree();
//
//                if(shootFarOff) {
//
//                    follower.followPath(openRedGate1,true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//
//                if(!follower.isBusy()){
//                    follower.followPath(gateToPickup1, true);
//                    setPathState(5);
//
//                }
//
//                break;
//
//            case 4:
//
//                if(!follower.isBusy()){
//                    follower.followPath(firstPickupBreak2,true);
//                    setPathState(5);
//                }
//
//                break;
//            case 5:
//
////                follower.setMaxPower(1);
//                intakePower = 1;
//                if(!follower.isBusy()){
//                    follower.followPath(firstPickup2);
//                    setPathState(6);
//
//                }
//
//                break;
//            case 6:
//
//                follower.setMaxPower(1);
//                intakePower=0.5;
//                setActionState(0);//
//                if(!follower.isBusy()) {
////                    follower.turn(0.47,false);
//                    shootFarOff = false;//
//                    setPathState(7);
//
//                }
//                break;
//
//            case 13:
//
//
//                setActionState(0);
//                if(follower.getHeadingError()<Math.toRadians(0.5)&&pathTimer.getElapsedTimeSeconds()>1){
//
//                    shootFarOff = false;
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(secondPickupBreak1,true);
//                    setPathState(8);
//                }
//
//                break;
//            case 8:
//                intakePower=0;
//                if(!follower.isBusy()) {
//                    follower.followPath(secondPickupBreak2,true);
//                    setPathState(9);
//                }
//                break;
//
//            case 9:
//                intakePower=1;
//                follower.setMaxPower(1);
//                if(!follower.isBusy()) {
//                    follower.followPath(secondPickup2,true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//
//                intakePower=0.5;
//                follower.setMaxPower(1);
//                setActionState(0);//
//                if(!follower.isBusy()){
////                    follower.turn(0.47,false);
//
//                    shootFarOff = false;//
//
//                    setPathState(12);
//                }
//                break;
//            case 11:
//                setActionState(0);
//                if(follower.getHeadingError()<Math.toRadians(0.5)&&pathTimer.getElapsedTimeSeconds()>1){
//
//                    shootFarOff = false;
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(endPath,true);
//                    setPathState(15);
//                }
//                break;
//            case 15:
//                shootFarOff=true;
//                break;
//        }
//    }
//
//
//
//    public void shootNear() {
//
//        hood_position = 0.3;
//        velocity = rightShooter.getVelocity();
//        double output = pidNear.calculator(targetVelocity, velocity);
//        shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);
//
//        HuskyLens.Block[] blocks = huskylens.blocks();
//        if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
//            distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
//            targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
////            rx = Math.abs(blocks[0].x - Constant.targetPosition) > 10 ? aiming.calculator(Constant.targetPosition, blocks[0].x) : 0;
//        }
//    }
//
//    public void shootFar(){
//        hood_position = 0.4;
//        velocity = rightShooter.getVelocity();
//        double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
//        shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);
//
//    }
//    public boolean farError(){
//        velocity = rightShooter.getVelocity();
//        return Math.abs(velocity-Constant.targetVelocityFar)<100;
//    }
//
//    public void pivotControl(Timer actionTimer){
//
//        if(actionTimer.getElapsedTimeSeconds()<0.3 && shooterPower!=0){
//            shootComplete=false;
//            pivotPositionLeft = Constant.leftPivotActivated;
//            pivotPositionRight = Constant.rightPivotActivated;
//        }else{
//            pivotPositionLeft = Constant.leftPivotINIT;
//            pivotPositionRight = Constant.rightPivotINIT;
//
//        }
//        if(actionTimer.getElapsedTimeSeconds()>1.5){
//            shootComplete = true;
//        }
//    }
////    public boolean ifTurned(double targetAngle,double current,double tolar){
////        return Math.abs(targetAngle-current)<tolar;
////    }
//
//    public void shootThree(){
//        switch (actionState){
//
//
//            case 0:
//                if(farError()){
//                    setActionState(1);
//                }
//
//
//                break;
//            case 1:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(2);
//                }
//                break;
//            case 2:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(3);
//                }
//                break;
//            case 3:
//                pivotControl(actionTimer);
//                if(actionTimer.getElapsedTimeSeconds()>0.4){
//                    setActionState(4);
//                }
//                break;
//            case 4:
//                intakePower = 0;
//                shootNearOff = true;
//                shootFarOff = true;
//
//                break;
//
//        }
//    }
//
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//
//    }
//
//    public void setActionState(int aState){
//        actionState = aState;
//        actionTimer.resetTimer();
//        pidNear.reset();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
//        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
//        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
//        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
//        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
//        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
//        imu = hardwareMap.get(IMU.class, "imu");
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
//
//        rightHood.setPosition(Constant.rightHoodINIT);
//
//        rightPivot.setPosition(Constant.rightPivotINIT);
//        leftPivot.setPosition(Constant.leftPivotINIT);
//
//        shootNearOff =true;
//
//        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//        );
//        imu.initialize(new IMU.Parameters(orientation));
//
//        imu.resetYaw();
//        shootFarOff=true;
//        shootNearOff=true;
//
//
//    }
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
//        setActionState(0);
//
//
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//}

//
//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.teamcode.PIDCalculator;
//import org.firstinspires.ftc.teamcode.opMode.Constant;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "FarRed")
//@Config
//public class AutonFarRed extends OpMode {
//    private Follower follower;
//    private PathChain gateToPickup1, firstPickup1,firstPickup2, secondPickup1,secondPickup2,firstPickupShot,randomPickup, openRedGate1,openRedGate2, firstPickupBreak1, firstPickupBreak2,endPath,secondPickupBreak1,secondPickupBreak2,pickup2ToGate;
//
//
//    private Path preLoadShot;
//    public static Pose initPos = new Pose(79,8,Math.toRadians(90));
//    public static Pose shootingPos = new Pose(77, 20,Math.toRadians(64));
//    public static Pose redGate = new Pose(139.481, 70.608,Math.toRadians(180));
//    public static Pose pickup1Pos = new Pose (135,34.5,Math.toRadians(0));
//    public static Pose pickup2Pos = new Pose (135, 59.470,Math.toRadians(0));
//    public static Pose randomPickupPos = new Pose(132,28,Math.toRadians(43));
//    public static Pose endPos1 = new Pose(120,70,Math.toRadians(90));
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState,actionState;
//
//    private PIDCalculator pidNear = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator pidFar = new PIDCalculator(Constant.shooter_kP, Constant.shooter_kI, Constant.shooter_kD);
//    private PIDCalculator aiming = new PIDCalculator(Constant.aiming_kP, Constant.aiming_kI, Constant.aiming_kD);
//    private DcMotorEx leftShooter, rightShooter;
//    private DcMotor intake;
//    private DcMotor frontLeft, frontRight, backLeft, backRight;
//    private HuskyLens huskylens;
//
//    private Servo leftPivot, rightPivot, rightHood;
//    private VoltageSensor voltageSensor;
//    private double rx = 1;
//    private double velocity = 0,  shooterPower = 0;
//    private double targetVelocity = 500;
//    private double distance = 0;
//    private double hood_position = Constant.rightHoodINIT;
//
//    private double intakePower = 0,pivotPositionRight=Constant.rightPivotINIT,pivotPositionLeft = Constant.leftPivotINIT;
//    private boolean shootComplete;
//    private boolean shootNearOff,shootFarOff;
//    private IMU imu;
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//
//        if(!shootNearOff){
//            shootNear();
//        }
//        if(!shootFarOff){
//            shootFar();
//        }
//
//        leftShooter.setPower(shooterPower);
//        rightShooter.setPower(shooterPower);
//
//        leftPivot.setPosition(pivotPositionLeft);
//        rightPivot.setPosition(pivotPositionRight);
//
//        rightHood.setPosition(hood_position);
//        intake.setPower(intakePower);
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState);
//        telemetry.addData("action state", actionState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//
//        telemetry.update();
//
//    }
//
//
//
//
//    public void buildPaths() {
//
//        preLoadShot=new Path(new BezierLine(initPos, shootingPos));
//        preLoadShot.setLinearHeadingInterpolation(initPos.getHeading(),shootingPos.getHeading());
//
//
//        firstPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        redGate,
//                        new Pose(79.160, 73.591),
//                        new Pose(81.348, 54.895)
//                ))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(81.348, 54.895),
//                        new Pose(78.564, 33.017),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        firstPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(79.359, 43.558),
//                        new Pose(95.271, 35.403),
//                        pickup1Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        firstPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup1Pos,
//                        new Pose(95.271, 35.204),
//                        new Pose(80.155, 42.166),
//                        shootingPos))
////                .setTangentHeadingInterpolation()
////                .setReversed()
//                .setLinearHeadingInterpolation(0,shootingPos.getHeading())
//                .build();
//
//
//        pickup2ToGate = follower.pathBuilder()
//                .addPath(new BezierCurve(pickup2Pos,
//                        new Pose(105.017, 56.088),
//                        new Pose(107.602, 77.967),
//                        redGate
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//        gateToPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(51.713, 78.365),
//                        new Pose(101.834, 46.740),
//                        new Pose(55.890, 36.796),
//                        pickup1Pos))
//                .setLinearHeadingInterpolation(0,0)
//                .build();
//
//        secondPickupBreak1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(75.779, 57.282),
//                        new Pose(95.470, 59.470)
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickupBreak2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Pose(95.470, 59.470),
//                        new Pose(116.950, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.166, 69.215),
//                        new Pose(90.298, 59.470),
//                        pickup2Pos))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        secondPickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        pickup2Pos,
//                        new Pose(88.110, 62.055),
//                        new Pose(82.343, 61.657),
//                        shootingPos))
////                .setTangentHeadingInterpolation()
////                .setReversed()
//                .setLinearHeadingInterpolation(0,shootingPos.getHeading())
//                .build();
//
//
//        openRedGate1 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(78.762, 61.061),
//                        new Pose(77.381, 75.779),
//                        redGate))
//                .setTangentHeadingInterpolation()
//                .build();
//        openRedGate2 = follower.pathBuilder()
//                .addPath(new BezierCurve(redGate,
//                        new Pose(78.365, 73.790),
//                        new Pose(79.160, 68.022),
//                        initPos))
////                .setTangentHeadingInterpolation()
////                .setReversed()
//                .setLinearHeadingInterpolation(redGate.getHeading(),shootingPos.getHeading())
//                .build();
//
//        endPath = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        shootingPos,
//                        new Pose(83.138, 72.994),
//                        endPos1
//                ))
//                .setTangentHeadingInterpolation()
//                .build();
//
//    }
//
//    public void autonomousPathUpdate() {
//
//        switch (pathState) {
//
//            case 0:
//
//                follower.followPath(preLoadShot);
//                shootFarOff = false;
//                setPathState(1);
//
//                break;
//
//            case 1:
//
//
//                setActionState(0);
//
//                if(!follower.isBusy()) {
//                    setPathState(2);
//
//                }
//
//                break;
//
//            case 2:
//
//                intakePower=1;
//
//                shootThree();
//
//                if(shootFarOff) {
//
//                    follower.followPath(secondPickup1,true);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//
//                intakePower =1;
//                if(!follower.isBusy()){
//                    follower.followPath(pickup2ToGate, true);
//                    setPathState(4);
//
//                }
//
//                break;
//
//            case 4:
//
//                intakePower = 0.5;
//                if(!follower.isBusy()){
//                    follower.followPath(openRedGate2,true);
//                    setPathState(5);
//                }
//
//                break;
//
//            case 5:
//                intakePower=0.5;
//                setActionState(0);//
//                if(!follower.isBusy()) {
//                    shootFarOff = false;
//                    setPathState(6);
//
//                }
//                break;
//
//            case 6:
//
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(firstPickup1,true);
//                    setPathState(7);
//                }
//
//                break;
//            case 7:
//                intakePower=1;
//                if(!follower.isBusy()) {
//                    follower.followPath(firstPickup2,true);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//
//                intakePower=0.5;
//                setActionState(0);//
//                if(!follower.isBusy()){
//                    shootFarOff = false;//
//
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                intakePower = 1;
//                shootThree();
//                if(shootFarOff){
//                    follower.followPath(endPath,true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                shootFarOff=true;
//                break;
//        }
//    }
//
//
//
//    public void shootNear() {
//
//        hood_position = 0.3;
//        velocity = rightShooter.getVelocity();
//        double output = pidNear.calculator(targetVelocity, velocity);
//        shooterPower = Math.min(output + Constant.kV_near_standard * Constant.voltage_near / voltageSensor.getVoltage() * targetVelocity + Constant.kS, 1);
//
//        HuskyLens.Block[] blocks = huskylens.blocks();
//        if (blocks != null && blocks.length == 1 && (blocks[0].id == 4 || blocks[0].id == 5)) {
//            distance = Constant.distance_slope / blocks[0].height + Constant.distance_intercept;
//            targetVelocity = Math.sqrt(Constant.velocity_slope * distance + Constant.velocity_intercept);
////            rx = Math.abs(blocks[0].x - Constant.targetPosition) > 10 ? aiming.calculator(Constant.targetPosition, blocks[0].x) : 0;
//        }
//    }
//
//    public void shootFar(){
//        hood_position = 0.4;
//        velocity = rightShooter.getVelocity();
//        double output = pidFar.calculator(Constant.targetVelocityFar, velocity);
//        shooterPower = Math.min(output + Constant.kV_far_standard * Constant.voltage_far / voltageSensor.getVoltage() * Constant.targetVelocityFar + Constant.kS, 1);
//
//    }
//    public boolean farError(){
//        velocity = rightShooter.getVelocity();
//        return Math.abs(velocity-Constant.targetVelocityFar)<100;
//    }
//
//    public void pivotControl(Timer actionTimer){
//
//        if(actionTimer.getElapsedTimeSeconds()<0.3 && shooterPower!=0){
//            shootComplete=false;
//            pivotPositionLeft = Constant.leftPivotActivated;
//            pivotPositionRight = Constant.rightPivotActivated;
//        }else{
//            pivotPositionLeft = Constant.leftPivotINIT;
//            pivotPositionRight = Constant.rightPivotINIT;
//
//        }
//        if(actionTimer.getElapsedTimeSeconds()>1.5){
//            shootComplete = true;
//        }
//    }
////    public boolean ifTurned(double targetAngle,double current,double tolar){
////        return Math.abs(targetAngle-current)<tolar;
////    }
//
//    public void shootThree(){
//        switch (actionState){
//
//
//            case 0:
//                if(farError()){
//                    setActionState(1);
//                }
//
//
//                break;
//            case 1:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(2);
//                }
//                break;
//            case 2:
//                pivotControl(actionTimer);
//                if(shootComplete){
//                    setActionState(3);
//                }
//                break;
//            case 3:
//                pivotControl(actionTimer);
//                if(actionTimer.getElapsedTimeSeconds()>0.5){
//                    setActionState(4);
//                }
//                break;
//            case 4:
//                intakePower = 0;
//                shootNearOff = true;
//                shootFarOff = true;
//
//                break;
//
//        }
//    }
//
//
//    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//
//    }
//
//    public void setActionState(int aState){
//        actionState = aState;
//        actionTimer.resetTimer();
//        pidNear.reset();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        actionTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
//        leftShooter = hardwareMap.get(DcMotorEx.class, "LeftShooterMotor");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "RightShooterMotor");
//        rightHood = hardwareMap.get(Servo.class, "RightHood");
//        leftPivot = hardwareMap.get(Servo.class, "LeftPivot");
//        rightPivot = hardwareMap.get(Servo.class, "RightPivot");
//        frontLeft = hardwareMap.get(DcMotor.class, "LeftFrontMotor");
//        frontRight = hardwareMap.get(DcMotor.class, "RightFrontMotor");
//        backLeft = hardwareMap.get(DcMotor.class, "LeftBackMotor");
//        backRight = hardwareMap.get(DcMotor.class, "RightBackMotor");
//        imu = hardwareMap.get(IMU.class, "imu");
//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        huskylens = hardwareMap.get(HuskyLens.class, "huskylens");
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(initPos);
//
//        rightHood.setPosition(Constant.rightHoodINIT);
//
//        rightPivot.setPosition(Constant.rightPivotINIT);
//        leftPivot.setPosition(Constant.leftPivotINIT);
//
//        shootNearOff =true;
//
//        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//        );
//        imu.initialize(new IMU.Parameters(orientation));
//
//        imu.resetYaw();
//        shootFarOff=true;
//        shootNearOff=true;
//
//
//    }
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
//        setActionState(0);
//
//
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {}
//
//}
//
//
//





