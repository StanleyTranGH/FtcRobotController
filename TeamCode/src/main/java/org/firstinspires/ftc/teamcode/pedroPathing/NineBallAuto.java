package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Autonomous(name = "Nine Ball Auto", group = "Official")
public class NineBallAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Limelight3A limelight;
    DcMotor intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    final double launcherServoDown = 0.08;
    final double launcherServoUp = 0.47; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1540; // DONE: FIND DESIRED LAUNCHER VELOCITY
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1480;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1560;
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1780; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    final double FAR_LAUNCHER_MIN_VELOCITY = 1740;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1820;
    final double STOP_SPEED = 0.0;
    final double MAX_FEED_TIME = 0.35;
    final double MAX_WAITING_TIME = 0.8;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    int shotCounter = 0;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.015, 0, 0.001);
    ControlSystem launcherController;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        WAITING,
    }
    private LaunchState launchState;
    ElapsedTime feederTimer = new ElapsedTime();
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
    private Pose startPose = new Pose(88, 8, Math.toRadians(90)); // Start Pose of our robot.
    private Pose scanPose = new Pose(88, 75, Math.toRadians(95)); // Scan Obelisk
    private Pose scorePose = new Pose(90, 90, Math.toRadians(45)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(100, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(124, 84, Math.toRadians(0)); // Collect first set of artifacts

    private final Pose pickup2Pose = new Pose(100, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose collect2Pose = new Pose(127, 58, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose pickup3Pose = new Pose(100, 35, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(127, 35, Math.toRadians(0)); // Collect third set of artifacts
    private final Pose collect4aPose = new Pose(132, 17, Math.toRadians(340)); // Collect the first of the fourth set of artifacts
    private final Pose collect4bPose = new Pose(132, 10, Math.toRadians(340)); // Collect the second of the fourth set of artifacts
    private final Pose collect4bControlPose = new Pose(128, 16, Math.toRadians(340)); // Control for collecting the 4b artifact
    private final Pose collect4cPose = new Pose(133, 7, Math.toRadians(270)); // Collect the third of the fourth set of artifacts
    private final Pose collect4cControlPose = new Pose(131, 13, Math.toRadians(270)); // Control for collecting the 4c artifact
    private Pose parkPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.

    PathChain scanObelisk, scorePreload, scorePreloadSkipScan, grabPickup1, collectPickup1, scorePickup1, grabPickup2, collectPickup2, scorePickup2, grabPickup3, collectPickup3, scorePickup3, collectPickup4, scorePickup4, park;
    void buildPaths() {

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        if(startingPlace == 1) { // Far
            startPose = new Pose(88, 8, Math.toRadians(90)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO FAR SCAN
            scanPose = new Pose(88, 49, Math.toRadians(97)); // Scan Obelisk
            // TODO: CHANGE THIS POSE TO FAR SCORE
            scorePose = new Pose(87, 18, Math.toRadians(68)); // Scoring Pose of our robot.
            parkPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.
        } else if (startingPlace == 2) { // Close
            // DONE: CHANGE THIS START POSE TO CLOSE START
            startPose = new Pose(125, 120, Math.toRadians(37)); // Start Pose of our robot.
            // DONE: CHANGE THIS POSE TO CLOSE SCAN
            scanPose = new Pose(88, 100, Math.toRadians(103)); // Scan Obelisk
            scorePose = new Pose(90, 90, Math.toRadians(45)); // Scoring Pose of our robot.
            // DONE: CHANGE THIS POSE TO CLOSE PARK
            parkPose = new Pose(122, 95, Math.toRadians(90)); // Park Pose of our robot.
        }

        if(colorAlliance == 1) {
            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scanPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                    .build();

            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose, scorePose))
                    .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
                    .build();

            scorePreloadSkipScan = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
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

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1Pose, scorePose))
                    .setLinearHeadingInterpolation(collect1Pose.getHeading(), scorePose.getHeading())
                    .build();

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
        } else if(colorAlliance == 2) { // Mirrored poses for blue side
            startPose = startPose.mirror();
            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scanPose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scanPose.mirror().getHeading())
                    .build();

            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(scanPose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            scorePreloadSkipScan = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scorePose.mirror().getHeading())
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

            /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1Pose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect1Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

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
                if(startingPlace == 2) {
                    launcherController.setGoal(closeTargetLauncherKineticState);
                } else {
                    launcherController.setGoal(farTargetLauncherKineticState);
                }
                follower.followPath(scorePreloadSkipScan);
                if(startingPlace == 2) {
                    setPathState(1);
                } else if(startingPlace == 1) {
                    setPathState(101);
                }
                break;

            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* DONE: SHOOT PRELOAD BALLS */

                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        intakeMotor.setPower(INTAKING);
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(INTAKING);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    /* TODO: DECIDE BEST PATH POWER */
                    follower.followPath(collectPickup1, 0.5,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: BALL SORTING */


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    shotCounter = 0;
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* DONE: SHOOT BALLS */
                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                    }

                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(INTAKING);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectPickup2, 0.5,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: BALL SORTING */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    shotCounter = 0;
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* DONE: SHOOT BALLS */
                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        follower.followPath(park, true);
                        intakeMotor.setPower(STOP_SPEED);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    setPathState(-1000);
                }
                break;
            case 101:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* DONE: SHOOT PRELOAD BALLS */

                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        intakeMotor.setPower(INTAKING);
                        follower.followPath(grabPickup3, true);
                        setPathState(102);
                    }
                }
                break;
            case 102:
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(INTAKING);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    /* TODO: DECIDE BEST PATH POWER */
                    follower.followPath(collectPickup3, 0.5,true);
                    setPathState(103);
                }
                break;
            case 103:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: BALL SORTING */


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3,true);
                    launcherController.setGoal(farTargetLauncherKineticState);
                    shotCounter = 0;
                    setPathState(104);
                }
                break;
            case 104:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                if(!follower.isBusy()) {
                    /* DONE: SHOOT BALLS */
                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        intakeMotor.setPower(INTAKING);
                        follower.followPath(collectPickup4, 0.5, true);
                        setPathState(105);
                    }

                }
                break;
            case 105:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: BALL SORTING */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup4, true);
                    launcherController.setGoal(farTargetLauncherKineticState);
                    shotCounter = 0;
                    setPathState(106);
                }
                break;
            case 106:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* DONE: SHOOT BALLS */
                    if(shotCounter < 3) {
                        launch(true);
                    } else {
                        launcherController.setGoal(stopLauncherKineticState);
                        follower.followPath(park, true);
                        intakeMotor.setPower(STOP_SPEED);
                        setPathState(107);
                    }
                }
                break;
            case 107:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    setPathState(-1001);
                }
                break;

            // SKIPPING SCANNING FOR NOW
            /*
            case 12:

                // DONE: SCAN MOTIF

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    // Valid target detected
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        detectedID = fiducial.getFiducialId();
                    }

                    if(detectedID == 21) {
                        obeliskID = detectedID;
                        motif = "GPP (Green Purple Purple)";
                    } else if(detectedID == 22) {
                        obeliskID = detectedID;
                        motif = "PGP (Purple Green Purple)";
                    } else if(detectedID == 23) {
                        obeliskID = detectedID;
                        motif = "PPG (Purple Purple Green)";
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 5 && !follower.isBusy()) {
                    obeliskID = 23;
                    motif = "Couldn't Detect! Guessing PPG";
                }

                // DONE: ADJUST ELAPSED TIME SECONDS OR CHANGE IF NEEDED
                if(obeliskID != 0) {
                    follower.followPath(scorePreload);
                    launcherController.setGoal(targetLauncherKineticState);
                    shotCounter = 0;
                    intakeMotor.setPower(INTAKING);
                    setPathState(1);
                }
                break;

             */
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        launchState = LaunchState.IDLE;
        launch(false);

        follower = Constants.createFollower(hardwareMap);

        /* Limelight Stuff */
        // Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

        launcher.setZeroPowerBehavior(BRAKE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .build();

        launcher.setPower(STOP_SPEED);
        launcherController.setGoal(stopLauncherKineticState);

        launcherServo.setPosition(launcherServoDown);

        // Ensure we're using pipeline 0
        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        if(lockedIn == false) {
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
        }
        else {
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

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if(colorAlliance == 0) {
            colorAlliance = 1;
        }
        if(startingPlace == 0) {
            startingPlace = 2;
        }

        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Calculate PID for launcher velocity based on goal (set earlier) and current velocity
        currentLauncherKineticState = new KineticState(launcher.getCurrentPosition(), launcher.getVelocity());
        launcher.setPower(launcherController.calculate(currentLauncherKineticState));

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Detected ID", detectedID);
        telemetry.addData("Detected Motif", motif);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if(startingPlace == 2) {
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    if (launcher.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else if(startingPlace == 1) {
                    launcherController.setGoal(farTargetLauncherKineticState);
                    if (launcher.getVelocity() > FAR_LAUNCHER_MIN_VELOCITY && launcher.getVelocity() < FAR_LAUNCHER_MAX_VELOCITY) {
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
                if (shotCounter == 2 && feederTimer.seconds() > 0.1) {
                    shotCounter = shotCounter + 1;
                    feederTimer.reset();
                    launchState = LaunchState.IDLE;
                } else if (feederTimer.seconds() > MAX_WAITING_TIME) {
                    shotCounter++;
                    feederTimer.reset();
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
