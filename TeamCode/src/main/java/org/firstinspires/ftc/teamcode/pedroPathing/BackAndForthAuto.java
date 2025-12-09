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

@Autonomous(name = "Back And Forth Auto", group = "Official")
public class BackAndForthAuto extends OpMode {
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
    Servo hoodServo;
    final double launcherServoDown = 0.40;
    final double launcherServoUp = 0.15; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double sorterServoOpenLeft = 0.72; //DONE: SET THIS VALUE TO OPEN THE LEFT SIDE
    final double sorterServoOpenRight = 0.37; //DONE: SET THIS VALUE TO OPEN THE RIGHT SIDE
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
    final double hoodRest = 0;
    final double hoodScore = 0.4; // TODO: FIND DESIRED HOOD POSITION
    final double STOP_SPEED = 0.0;
    final double MAX_FEED_TIME = 0.22;
    final double MAX_WAITING_TIME = 0.75;
    final double HOLD_MAX_WAITING_TIME = 0.9;
    final double MAX_SCAN_TIME = 2.0;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    int shotCounter = 0;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.006, 0, 0.0006);
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
    private Pose scanPose = new Pose(88, 75, Math.toRadians(275)); // Scan Obelisk
    private Pose scorePose = new Pose(90, 90, Math.toRadians(225)); // Scoring Pose of our robot.
    private Pose scorePreloadPose = new Pose(90, 90, Math.toRadians(217));
    private final Pose pickup1Pose = new Pose(101, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(124, 84, Math.toRadians(0)); // Collect first set of artifacts
    private final Pose openGatePose = new Pose(132, 76, Math.toRadians(0)); // Opens Gate.
    private final Pose openGateControlPose = new Pose(116, 77, Math.toRadians(0)); // Control pose for Scoring 1 while avoiding Artifacts 2
    private final Pose score1ControlPose = new Pose(81, 83, Math.toRadians(90)); // Control pose to open Gate
    private final Pose pickup2Pose = new Pose(101, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose collect2Pose = new Pose(127, 58, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose pickup3Pose = new Pose(101, 37, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(127, 37, Math.toRadians(0)); // Collect third set of artifacts
    private final Pose collect4aPose = new Pose(132, 17, Math.toRadians(340)); // Collect the first of the fourth set of artifacts
    private final Pose collect4bPose = new Pose(132, 10, Math.toRadians(340)); // Collect the second of the fourth set of artifacts
    private final Pose collect4bControlPose = new Pose(128, 16, Math.toRadians(340)); // Control for collecting the 4b artifact
    private final Pose collect4cPose = new Pose(133, 7, Math.toRadians(270)); // Collect the third of the fourth set of artifacts
    private final Pose collect4cControlPose = new Pose(131, 13, Math.toRadians(270)); // Control for collecting the 4c artifact
    private final Pose grabLoadingPose = new Pose(134,12, Math.toRadians(0));
    private Pose parkPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.

    PathChain grabLoadingZone, scoreLoadingZone, scorePreload, scorePreloadSkipScan, grabPickup1, collectPickup1, openGate, scorePickup1, grabPickup2, collectPickup2, scorePickup2, grabPickup3, collectPickup3, scorePickup3, collectPickup4, scorePickup4, park;

    void buildPaths() {

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        if (startingPlace == 1) { // Far
            startPose = new Pose(88, 8, Math.toRadians(0)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO FAR SCAN
            scanPose = new Pose(88, 49, Math.toRadians(0)); // Scan Obelisk
            // TODO: CHANGE THIS POSE TO FAR SCORE
            scorePose = new Pose(92.8, 13.6, Math.toRadians(0)); // Scoring Pose of our robot.
            parkPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.
            scorePreloadPose = scorePose;

            turretScore = 0.36; // TODO: GET ACTUAL TURRET SCORE POSITION
            turretScan = 0.3; // TODO: GET ACTUAL TURRET SCAN POSITION
        } else if (startingPlace == 2) { // Close
            // DONE: CHANGE THIS START POSE TO CLOSE START
            startPose = new Pose(125, 120, Math.toRadians(217)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO CLOSE SCAN
            scanPose = new Pose(88, 100, Math.toRadians(217)); // Scan Obelisk
            scorePose = new Pose(87.5, 91, Math.toRadians(0)); // Scoring Pose of our robot.
            // DONE: CHANGE THIS POSE TO CLOSE PARK
            parkPose = new Pose(120, 12, Math.toRadians(0)); // Park Pose of our robot.
            scorePreloadPose = new Pose(90, 90, Math.toRadians(217));

            turretPreloadScore = 0.5;
            turretScore = 0.35; // TODO: GET ACTUAL TURRET SCORE POSITION
            turretScan = 0.3; // TODO: GET ACTUAL TURRET SCAN POSITION
        }

        if (colorAlliance == 1) {

            scorePreloadSkipScan = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePreloadPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading())
                    .build();
            grabLoadingZone = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, grabLoadingPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), grabLoadingPose.getHeading())
                    .build();
            scoreLoadingZone = follower.pathBuilder()
                    .addPath(new BezierLine(grabLoadingPose, scorePose))
                    .setLinearHeadingInterpolation(grabLoadingPose.getHeading(), scorePose.getHeading())
                    .build();
            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();
            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, collect1Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect1Pose, openGateControlPose, openGatePose))
                    .setLinearHeadingInterpolation(collect1Pose.getHeading(), openGatePose.getHeading())
                    .build();

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            // Avoid artifact 2 if shooting from the far zone
            if(startingPlace == 1) {
                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierCurve(openGatePose, score1ControlPose, scorePose))
                        .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                        .build();
            } else {
                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(openGatePose, scorePose))
                        .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                        .build();
            }

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, collect2Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(collect2Pose, scorePose))
                    .setLinearHeadingInterpolation(collect2Pose.getHeading(), scorePose.getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .build();
            collectPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, collect3Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Pose, scorePose))
                    .setLinearHeadingInterpolation(collect3Pose.getHeading(), scorePose.getHeading())
                    .build();

            collectPickup4 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, collect4aPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), collect4aPose.getHeading())
                    .addPath(new BezierCurve(collect4aPose, collect4bPose, collect4bControlPose))
                    .setLinearHeadingInterpolation(collect4aPose.getHeading(), collect4bPose.getHeading())
                    .addPath(new BezierCurve(collect4bPose, collect4cPose, collect4cControlPose))
                    .setLinearHeadingInterpolation(collect4bPose.getHeading(), collect4cPose.getHeading())
                    .build();

            scorePickup4 = follower.pathBuilder()
                    .addPath(new BezierLine(collect4cPose, scorePose))
                    .setLinearHeadingInterpolation(collect4cPose.getHeading(), scorePose.getHeading())
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, parkPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                    .build();
        } else if (colorAlliance == 2) { // Mirrored poses for blue side
            startPose = startPose.mirror();
            turretScan = 1 - turretScan;
            turretScore = 1 - turretScore;
            turretPreloadScore = 1 - turretPreloadScore;

            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose.mirror(), scorePreloadPose.mirror()))
                    .setLinearHeadingInterpolation(scanPose.mirror().getHeading(), scorePreloadPose.mirror().getHeading())
                    .build();

            scorePreloadSkipScan = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePreloadPose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scorePreloadPose.mirror().getHeading())
                    .build();

            /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup1Pose.mirror().getHeading())
                    .build();
            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose.mirror(), collect1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect1Pose.mirror(), openGateControlPose.mirror(), openGatePose.mirror()))
                    .setLinearHeadingInterpolation(collect1Pose.mirror().getHeading(), openGatePose.mirror().getHeading())
                    .setHeadingConstraint(10)
                    .build();

            // Avoid artifact 2 if shooting from the far zone
            if(startingPlace == 1) {
                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierCurve(openGatePose.mirror(), score1ControlPose.mirror(), scorePose.mirror()))
                        .setLinearHeadingInterpolation(openGatePose.mirror().getHeading(), scorePose.mirror().getHeading())
                        .build();
            } else {
                /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
                scorePickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(openGatePose.mirror(), scorePose.mirror()))
                        .setLinearHeadingInterpolation(openGatePose.mirror().getHeading(), scorePose.mirror().getHeading())
                        .build();
            }

            /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup2Pose.mirror().getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose.mirror(), collect2Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();

            /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(collect2Pose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect2Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup3Pose.mirror().getHeading())
                    .build();
            collectPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose.mirror(), collect3Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();

            /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Pose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect3Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            collectPickup4 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), collect4aPose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), collect4aPose.mirror().getHeading())
                    .addPath(new BezierCurve(collect4aPose.mirror(), collect4bPose.mirror(), collect4bControlPose.mirror()))
                    .setLinearHeadingInterpolation(collect4aPose.mirror().getHeading(), collect4bPose.mirror().getHeading())
                    .addPath(new BezierCurve(collect4bPose.mirror(), collect4cPose.mirror(), collect4cControlPose.mirror()))
                    .setLinearHeadingInterpolation(collect4bPose.mirror().getHeading(), collect4cPose.mirror().getHeading())
                    .build();

            scorePickup4 = follower.pathBuilder()
                    .addPath(new BezierLine(collect4cPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect4cPose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            park = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), parkPose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), parkPose.mirror().getHeading())
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shotCounter = 0;
                intakeMotor.setPower(INTAKING);
                launcherController.setGoal(closeTargetLauncherKineticState);
                turretServo.setPosition(turretScore);
                hoodServo.setPosition(hoodScore);
                follower.followPath(scorePreload);
                sorterServo.setPosition(sorterServoOpenRight);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    /* DONE: SHOOT BALLS */
                    intakeMotor.setPower(INTAKING);
                    if(shotCounter < 3) {
                        launch(true, false);
                    }
                    if (shotCounter >= 3) {
                        follower.followPath(grabLoadingZone, true);
                        intakeMotor.setPower(INTAKING);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    intakeMotor.setPower(INTAKING);
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    intakeMotor.setPower(STOP_SPEED);
                    follower.followPath(scoreLoadingZone, true);
                    setPathState(1);
                }
                break;
            case 4:
                park = follower.pathBuilder()
                        .addPath(new BezierLine(follower::getPose, parkPose))
                        .setLinearHeadingInterpolation(follower.getHeading(), parkPose.getHeading())
                        .build();
                intakeMotor.setPower(STOP_SPEED);
                sorterServo.setPosition(sorterServoOpenRight);
                launcherController.setGoal(stopLauncherKineticState);
                turretServo.setPosition(turretRest);
                follower.followPath(park, true);
                setPathState(5);
            case 5:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
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
        launch(false, false);

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
        leftGateServo = hardwareMap.get(Servo.class, "gateServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .build();

        launcher1.setPower(STOP_SPEED);
        launcher2.setPower(STOP_SPEED);
        launcherController.setGoal(stopLauncherKineticState);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        launcher1.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);

        launcherServo.setPosition(launcherServoDown);
        sorterServo.setPosition(sorterServoOpenRight);
        leftGateServo.setPosition(closeLeftGateServo);
        turretServo.setPosition(turretRest);
        hoodServo.setPosition(hoodRest);

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
        if (lockedIn == false) {
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

            telemetry.addLine("Press X for Far and Y for Also Far");
            if (gamepad1.xWasPressed()) {
                startingPlace = 1; // Far
            } else if (gamepad1.yWasPressed()) {
                startingPlace = 1; // Also Far
            }

            if (startingPlace == 1) {
                telemetry.addData("Starting Position", "Far");
            } else if (startingPlace == 2) {
                telemetry.addData("Starting Position", "How tf did you get here");
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
        if(opmodeTimer.getElapsedTimeSeconds() > 26 && opmodeTimer.getElapsedTimeSeconds() < 26.2) {
            setPathState(4);
        }
        autonomousPathUpdate();

        // Calculate PID for launcher velocity based on goal (set earlier) and current velocity
        currentLauncherKineticState = new KineticState(launcher1.getCurrentPosition(), launcher1.getVelocity());
        launcher1.setPower(launcherController.calculate(currentLauncherKineticState));
        launcher2.setPower(launcherController.calculate(currentLauncherKineticState));

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Detected ID", detectedID);
        telemetry.addData("Detected Motif", motif);
        telemetry.addData("Launcher Velocity", launcher1.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.update();
        autoToTeleop.endAuto = follower.getPose();
    }

    void launch(boolean shotRequested, boolean openGate) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (startingPlace == 2) {
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    if (launcher1.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else if (startingPlace == 1) {
                    launcherController.setGoal(farTargetLauncherKineticState);
                    if (launcher1.getVelocity() > FAR_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < FAR_LAUNCHER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                launcherServo.setPosition(launcherServoUp);
                launchState = LaunchState.LAUNCHING;
                feederTimer.reset();
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > MAX_FEED_TIME) {
                    launchState = LaunchState.WAITING;
                    launcherServo.setPosition(launcherServoDown);
                    feederTimer.reset();

                }
                break;
            case WAITING:
                if (shotCounter == 2) {
                    shotCounter++;
                    feederTimer.reset();
                    launchState = LaunchState.IDLE;
                }  else if (openGate) {
                    if (feederTimer.seconds() > HOLD_MAX_WAITING_TIME) {
                        shotCounter++;
                        feederTimer.reset();
                        launchState = LaunchState.IDLE;
                    }
                }else if (feederTimer.seconds() > MAX_WAITING_TIME) {
                    shotCounter++;
                    feederTimer.reset();
                    launchState = LaunchState.IDLE;
                }
                break;

        }
    }
}
