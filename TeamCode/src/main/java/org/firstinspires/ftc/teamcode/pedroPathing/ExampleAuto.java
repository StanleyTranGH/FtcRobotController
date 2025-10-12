package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
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

import java.util.List;

@Autonomous(name = "12 Ball Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Limelight3A limelight;
    DcMotorEx intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    String motif = "Not Detected Yet";
    int detectedID;
    int obeliskID = 0; // Having separate detectedID and obeliskID prevents goal targeting April Tags from being mistaken as obelisk
    /*
    21: GPP
    22: PGP
    23: PPG
    */
    public static int colorAlliance = 0; // 1: Red 2: Blue (no use yet, need to add mirror)
    final double launcherServoDown = 0.10;
    final double launcherServoUp = 0.45; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double LAUNCHER_TARGET_VELOCITY = 1500; // DONE: FIND DESIRED LAUNDER VELOCITY
    final double LAUNCHER_MIN_VELOCITY = 1440;
    final double LAUNCHER_MAX_VELOCITY = 1560;
    final double STOP_SPEED = 0.0;
    final double MAX_FEED_TIME = 0.3;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private int pathState;
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(88, 75, Math.toRadians(95)); // Scan Obelisk
    private final Pose scorePose = new Pose(86, 84, Math.toRadians(45)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(100, 81, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(120, 81, Math.toRadians(0)); // Collect first set of artifacts

    private final Pose pickup2Pose = new Pose(100, 57, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose collect2Pose = new Pose(120, 57, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose pickup3Pose = new Pose(100, 32, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(120, 32, Math.toRadians(0)); // Collect third set of artifacts
    private final Pose parkPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.

    PathChain scanObelisk, scorePreload, grabPickup1, collectPickup1, scorePickup1, grabPickup2, collectPickup2, scorePickup2, grabPickup3, collectPickup3, scorePickup3, park;
     void buildPaths() {

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

         scanObelisk = follower.pathBuilder()
                 .addPath(new BezierLine(startPose, scanPose))
                 .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                 .build();

         scorePreload = follower.pathBuilder()
                 .addPath(new BezierLine(scanPose, scorePose))
                 .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
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

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scanObelisk);
                setPathState(12);
                break;
            case 12:

                /* DONE: SCAN MOTIF */

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
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                    follower.followPath(scorePreload);
                    setPathState(1);
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
                    /* TODO: SHOOT PRELOAD BALLS */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(obeliskID == 23) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                    else if(obeliskID == 22) {
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                    } else if(obeliskID == 21) {
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
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
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds() > 1) {

                }
                if(!follower.isBusy()) {
                    /* TODO: SHOOT BALLS */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(obeliskID == 23 || obeliskID == 21) {
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                    } else if(obeliskID == 22) {
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(1);

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
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds() > 1) {

                }
                if(!follower.isBusy()) {
                    /* TODO: SHOOT BALLS */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(obeliskID == 23) {
                        follower.followPath(grabPickup3, true);
                        setPathState(8);
                    } else if(obeliskID == 22) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    } else if(obeliskID == 21) {
                        follower.followPath(park, true);
                        setPathState(11);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* DONE: ACTIVATE INTAKE */
                    intakeMotor.setPower(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(collectPickup3, 0.5,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* TODO: BALL SORTING */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTimeSeconds() > 1) {

                }
                if(!follower.isBusy()) {
                    /* TODO: SHOOT BALLS */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(obeliskID == 23 || obeliskID == 22) {
                        follower.followPath(park, true);
                        setPathState(11);
                    } else if(obeliskID == 21) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
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


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        LaunchState launchState = LaunchState.IDLE;

        /* Limelight Stuff */
        // Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        launcherServo.setPosition(launcherServoDown);

        // Ensure we're using pipeline 0 (your AprilTag pipeline)
        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press A for Red and B for Blue");

        if(gamepad1.a) {
            colorAlliance = 1; // Red
        } else if(gamepad1.b) {
            colorAlliance = 2; // Blue
        }

        if (colorAlliance == 1) {
            telemetry.addData("Selected Color", "Red");
        } else if (colorAlliance == 2) {
            telemetry.addData("Selected Color", "Blue");
        } else if (colorAlliance == 0) {
            telemetry.addData("Selected Color", "No Color Selected");
        }

        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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
        telemetry.addData("Detected ID", detectedID);
        telemetry.addData("Detected Motif", motif);
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}