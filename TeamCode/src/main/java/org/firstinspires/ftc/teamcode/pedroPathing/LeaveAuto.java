package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Autonomous(name = "C Twelve Ball Auto", group = "Official")
public class LeaveAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Limelight3A limelight;
    DcMotor intakeMotor;
    DcMotorEx launcher1;
    DcMotorEx launcher2;
    Servo launcherServo;
    Servo sorterServo;
    Servo leftGateServo;
    Servo turretServo;
    final double launcherServoDown = 0.18;
    final double launcherServoUp = 0.49; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double sorterServoOpenLeft = 0.67; //DONE: SET THIS VALUE TO OPEN THE LEFT SIDE
    final double sorterServoOpenRight = 0.36; //DONE: SET THIS VALUE TO OPEN THE RIGHT SIDE
    final double closeLeftGateServo = 0.73; // DONE: GET THE GATE CLOSE VALUE
    final double openLeftGateServo = 0.44; //DONE: GET THE GATE OPEN VALUE

    // Turret Positions

    double turretScore, turretPreloadScore, turretScan;
    final double turretRest = 0.5;

    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1640; // DONE: FIND DESIRED LAUNCHER VELOCITY
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1580;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1660;
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1880; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    final double FAR_LAUNCHER_MIN_VELOCITY = 1840;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1920;
    final double STOP_SPEED = 0.0;
    final double MAX_FEED_TIME = 0.22;
    final double MAX_WAITING_TIME = 0.6;
    final double MAX_SCAN_TIME = 2.0;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    int shotCounter = 0;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.015, 0, 0.0015);
    ControlSystem launcherController;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        WAITING,
    }

    private LaunchState launchState;
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime gateTimer = new ElapsedTime();
    String motif = "Not Detected Yet";
    int detectedID;
    int obeliskID = 0; // Having separate detectedID and obeliskID prevents goal targeting April Tags from being mistaken as obelisk
    /*
    21: GPP
    22: PGP
    23: PPG
    */
    public static int colorAlliance = 0; // 1: Red 2: Blue
    public static int startingPlace = 0; // 1: Far 2: Close
    boolean lockedIn = false;
    private int pathState;
    // TODO: CHANGE THIS START TO CLOSE START
    private Pose startPose = new Pose(88, 8, Math.toRadians(270)); // Start Pose of our robot.
    private Pose parkPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.
    private Pose parkControlPose = new Pose(100, 114, Math.toRadians(270));
    PathChain park;
    void buildPaths() {

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        if (startingPlace == 1) { // Far
            startPose = new Pose(88, 8, Math.toRadians(0)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO FAR SCAN
            parkPose = new Pose(130, 11, Math.toRadians(0)); // Park Pose of our robot.

        } else if (startingPlace == 2) { // Close
            // DONE: CHANGE THIS START POSE TO CLOSE START
            startPose = new Pose(125, 120, Math.toRadians(217)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO CLOSE PARK
            parkPose = new Pose(104, 134, Math.toRadians(270)); // Park Pose of our robot.
        }

        if (colorAlliance == 1) {
            if(startingPlace == 1) {
                park = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, parkPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                        .build();
            } else {
                park = follower.pathBuilder()
                        .addPath(new BezierCurve(startPose, parkControlPose, parkPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                        .build();
            }

        } else if (colorAlliance == 2) { // Mirrored poses for blue side
            startPose = startPose.mirror();

            if(startingPlace == 1) {
                park = follower.pathBuilder()
                        .addPath(new BezierLine(startPose.mirror(), parkPose.mirror()))
                        .setLinearHeadingInterpolation(startPose.mirror().getHeading(), startPose.mirror().getHeading())
                        .build();
            } else {
                park = follower.pathBuilder()
                        .addPath(new BezierCurve(startPose.mirror(),parkControlPose.mirror(), parkPose.mirror()))
                        .setLinearHeadingInterpolation(startPose.mirror().getHeading(), parkPose.mirror().getHeading())
                        .build();
            }
        }

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(park);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        launchState = LaunchState.IDLE;

        follower = Constants.createFollower(hardwareMap);

        /* Limelight Stuff */
        // Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");

        launcher1.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .build();

        launcher1.setPower(STOP_SPEED);
        launcher2.setPower(STOP_SPEED);
        launcherController.setGoal(stopLauncherKineticState);

        launcherServo.setPosition(launcherServoDown);
        sorterServo.setPosition(sorterServoOpenRight);
        leftGateServo.setPosition(closeLeftGateServo);
        turretServo.setPosition(turretRest);

        // Ensure we're using pipeline 0
        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();


    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        if (!lockedIn) {
            telemetry.addLine("PLEASE REMEMBER TO LOCK IT IN!!! I'M TALKING TO YOU!!! LOCK ME IN!!!!!!");

            telemetry.addLine("---------- PRESS RIGHT TRIGGER TO LOCK IN ----------");
            telemetry.addLine("Press A for Red and B for Blue");

            if (gamepad1.aWasPressed()) {
                colorAlliance = 1; // Red
            } else if (gamepad1.bWasPressed()) {
                colorAlliance = 2; // Blue
            }

            if (colorAlliance == 1) {
                telemetry.addData("Selected Color", "Red");
            } else if (colorAlliance == 2) {
                telemetry.addData("Selected Color", "Blue");
            } else if (colorAlliance == 0) {
                telemetry.addData("Selected Color", "No Color Selected");
            }

            telemetry.addLine("Press X for Far and Y for Close");
            if (gamepad1.xWasPressed()) {
                startingPlace = 1; // Far
            } else if (gamepad1.yWasPressed()) {
                startingPlace = 2; // Close
            }

            if (startingPlace == 1) {
                telemetry.addData("Starting Position", "Far");
            } else if (startingPlace == 2) {
                telemetry.addData("Starting Position", "Close");
            } else if (startingPlace == 0) {
                telemetry.addData("Starting Position", "No Position Selected");
            }

            if (gamepad1.rightBumperWasPressed()) {
                buildPaths();
                follower.setStartingPose(startPose);
                lockedIn = true;
            }
        } else {
            telemetry.addLine("---------- LOCKED IN ----------");

            if (colorAlliance == 1) {
                telemetry.addData("Selected Color", "Red");
            } else if (colorAlliance == 2) {
                telemetry.addData("Selected Color", "Blue");
            } else if (colorAlliance == 0) {
                telemetry.addData("Selected Color", "No Color Selected");
            }

            if (startingPlace == 1) {
                telemetry.addData("Starting Position", "Far");
            } else if (startingPlace == 2) {
                telemetry.addData("Starting Position", "Close");
            } else if (startingPlace == 0) {
                telemetry.addData("Starting Position", "No Position Selected");
            }
        }

        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if (colorAlliance == 0) {
            colorAlliance = 1;
        }
        if (startingPlace == 0) {
            startingPlace = 2;
        }

        autoToTeleop.colorAlliance = colorAlliance;
        autoToTeleop.startingPosition = startingPlace;

        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.update();
        autoToTeleop.endAuto = follower.getPose();
    }

}
