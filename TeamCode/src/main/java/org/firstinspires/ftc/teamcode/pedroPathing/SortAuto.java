package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Autonomous(name = "Sort Auto", group = "Official")
public class SortAuto extends OpMode{
    private Follower follower;
    private Timer pathTimer, ballTimer, opmodeTimer;

    // parts
    private Limelight3A limelight;
    DcMotor intakeMotor;
    DcMotorEx launcher1;
    DcMotorEx launcher2;
    DcMotorEx turretMotor;
    Servo sorterServo;
    Servo leftGateServo;
    Servo hoodServo;
    Servo shooterGateServo;
    final double sorterServoOpenRight = 0.7; // DONE: SET SORTER SERVO POSITIONS
    final double sorterServoOpenLeft = 0.3;
    final double closeLeftGateServo = 0.83; // DONE: SET GATE SERVO POSITIONS
    final double openLeftGateServo = 0.95;
    final double closeShooterGateServo = 0.52; // DONE: SET SHOOTER GATE POSITIONS
    final double openShooterGateServo = 0.97;

    // turret stuff
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1640; // todo: find target velocity
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1580;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1660;
    final double STOP_SPEED = 0.0;
    final double MAX_BALL_TIME = 0.1; // todo: find actual max feed time
    final double MAX_SCAN_TIME = 2.0;
    final double MAX_RAMP_TIME = 3.5; // todo: find actual ramp time
    final double firstBallXRed = 106.3;
    final double secondBallXRed = 109.5;
    final double thirdBallXRed = 113.7;
    final double firstBallXBlue = 38.8;
    final double secondBallXBlue = 33.9;
    final double thirdBallXBlue = 28.5;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    final double TRANSFER_POWER = 0.8; // todo: find the right transfer power
    int shotCounter = 0;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0005);
    public static double launcherFF = 0.0003;
    public static PIDCoefficients turretPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0002); // DONE: TUNE TURRET PID
    ControlSystem launcherController;
    ControlSystem turretController;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);
    double turretTargetPosition = 0.0;
    final double hoodRest = 0;
    double hoodScore = 0.3; // todo: find hood angle
    KineticState turretRestKineticState = new KineticState(0);
    KineticState turretTargetKineticState = turretRestKineticState;
    KineticState turretCurrentKineticState = turretTargetKineticState;

    private enum LaunchState {
        IDLE,
        MOTORS,
        LAUNCHING,
        LAUNCHED
    }
    private LaunchState launchState;
    String motif = "Not Detected Yet";
    int detectedID;
    int obeliskID = 0; // Having separate detectedID and obeliskID prevents goal targeting April Tags from being mistaken as obelisk
    /*
    21: GPP
    22: PGP
    23: PPG
    */
    public static int colorAlliance = 0; // 1: Red 2: Blue
    boolean lockedIn = false;

    // pedropathing
    private int pathState; //DONE: use visualizer find poses
    private final Pose startPose = new Pose(125, 120, Math.toRadians(127)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(105, 105, Math.toRadians(127)); // Scan Obelisk
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(0)); // Scoring Pose of our robot.
    private final Pose parkScorePose = new Pose (93, 110, Math.toRadians(225)); // Parked Scoring Pose of our robot.
    private final Pose pickup2Pose = new Pose(101, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose collect2Pose = new Pose(127, 58, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose score2ControlPose = new Pose(103, 70); // Avoid 1 to score 2
    private final Pose openGateControlPose = new Pose(100, 69); // Avoid 1 to open Gate
    private final Pose openGatePose = new Pose(132, 64, Math.toRadians(0)); // Opens Gate
    private final Pose collectGatePose = new Pose(134.6, 60); // Collects from Gate
    private final Pose scoreGateControlPose = new Pose(102.5, 66.9); // Avoid gate and 1 to score Gate
    private final Pose pickup1Pose = new Pose(101, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(124, 84, Math.toRadians(0)); // Collect first set of artifacts
    private final Pose pickup3Pose = new Pose(101, 37, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(127, 37, Math.toRadians(0)); // Collect third set of artifacts

    int shootGateLoop = 1;
    final double intakePathSpeed = 0.8;
    final double sortPathSpeed = 0.3;

    PathChain scanObelisk, scorePreload, grabPickup2, collectPickup2, scorePickup2, openGate, collectPickupGate, scorePickupGate, grabPickup1, collectPickup1, scorePickup1, grabPickup3, collectPickup3, scorePickup3;

    void buildPaths() {
        if (colorAlliance == 1) {
            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scanPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                    .build();
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose, scorePose))
                    .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, collect2Pose))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose, score2ControlPose, scorePose))
                    .setLinearHeadingInterpolation(collect2Pose.getHeading(), scorePose.getHeading())
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, openGateControlPose, openGatePose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                    .build();
            collectPickupGate = follower.pathBuilder()
                    .addPath(new BezierLine(openGatePose, collectGatePose))
                    .setLinearHeadingInterpolation(openGatePose.getHeading(), collectGatePose.getHeading())
                    .build();
            scorePickupGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collectGatePose, scoreGateControlPose, scorePose))
                    .setLinearHeadingInterpolation(collectGatePose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                    .build();
            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose, collect1Pose))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1Pose, scorePose))
                    .setLinearHeadingInterpolation(collect1Pose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .build();
            collectPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose, collect3Pose))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Pose, parkScorePose))
                    .setLinearHeadingInterpolation(collect3Pose.getHeading(), parkScorePose.getHeading())
                    .build();

        } else if (colorAlliance == 2) {
            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose.mirror(), scanPose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scanPose.mirror().getHeading())
                    .build();
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(scanPose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup2Pose.mirror().getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose.mirror(), collect2Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose.mirror(), score2ControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect2Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose.mirror(), openGateControlPose.mirror(), openGatePose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), openGatePose.mirror().getHeading())
                    .build();
            collectPickupGate = follower.pathBuilder()
                    .addPath(new BezierLine(openGatePose.mirror(), collectGatePose.mirror()))
                    .setLinearHeadingInterpolation(openGatePose.mirror().getHeading(), collectGatePose.mirror().getHeading())
                    .build();
            scorePickupGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collectGatePose.mirror(), scoreGateControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collectGatePose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup1Pose.mirror().getHeading())
                    .build();
            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup1Pose.mirror(), collect1Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1Pose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect1Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            grabPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup3Pose.mirror().getHeading())
                    .build();
            collectPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup3Pose.mirror(), collect3Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Pose.mirror(), parkScorePose.mirror()))
                    .setLinearHeadingInterpolation(collect3Pose.mirror().getHeading(), parkScorePose.mirror().getHeading())
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                shotCounter = 0;
                launcherController.setGoal(closeTargetLauncherKineticState);
                intakeMotor.setPower(INTAKING);
                follower.followPath(scanObelisk);
                hoodServo.setPosition(hoodScore);
                sorterServo.setPosition(sorterServoOpenRight);
                setPathState(1);
                break;
            case 1:
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    // Valid target detected
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        detectedID = fiducial.getFiducialId();
                    }

                    if (detectedID == 21) {
                        obeliskID = detectedID;
                        motif = "GPP (Green Purple Purple)";
                    } else if (detectedID == 22) {
                        obeliskID = detectedID;
                        motif = "PGP (Purple Green Purple)";
                    } else if (detectedID == 23) {
                        obeliskID = detectedID;
                        motif = "PPG (Purple Purple Green)";
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > MAX_SCAN_TIME && !follower.isBusy()) {
                    obeliskID = 23;
                    motif = "Couldn't Detect! Guessing PPG";
                }

                if (obeliskID != 0) {
                    follower.followPath(scorePreload);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    intakeMotor.setPower(STOP_SPEED);
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (shotCounter < 3) {
                        launch(true, false);
                        if(shotCounter == 1) {
                            leftGateServo.setPosition(openLeftGateServo);
                        }
                    } else {
                        intakeMotor.setPower(INTAKING);
                        leftGateServo.setPosition(closeLeftGateServo);
                        shooterGateServo.setPosition(closeShooterGateServo);
                        follower.followPath(grabPickup2, true);
                        aimTurret(false);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(INTAKING);
                    if (detectedID == 22) {
                        follower.followPath(collectPickup2, intakePathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    } else {
                        follower.followPath(collectPickup2, sortPathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    }
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() > firstBallXRed && detectedID == 21 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < firstBallXBlue && detectedID == 21 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() > secondBallXRed && detectedID == 23 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < secondBallXBlue && detectedID == 23 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
                    intakeMotor.setPower(STOP_SPEED);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    shotCounter = 0;
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (shotCounter == 0) {
                        launch(true, false);
                        if (detectedID == 23) {
                            launch(true, true);
                            leftGateServo.setPosition(openLeftGateServo);
                        } else {
                            launch(true, false);
                        }
                    } else if (shotCounter == 1) {
                        if (detectedID == 21) {
                            launch(true, true);
                            leftGateServo.setPosition(openLeftGateServo);
                        } else {
                            launch(true, false);
                        }
                    } else if (shotCounter == 2) {
                        launch(true, true);
                    } else if (shotCounter >= 3){
                        intakeMotor.setPower(INTAKING);
                        leftGateServo.setPosition(closeLeftGateServo);
                        shooterGateServo.setPosition(closeShooterGateServo);
                        follower.followPath(openGate, true);
                        aimTurret(false);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(collectPickupGate, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > MAX_RAMP_TIME) {
                    follower.followPath(scorePickupGate, true);
                    intakeMotor.setPower(STOP_SPEED);
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    shotCounter = 0;
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    if (shootGateLoop <= 1) {
                        shootGateLoop--;
                        setPathState(5);
                    } else if (shotCounter < 3) {
                        launch(true, false);
                    } else {
                        intakeMotor.setPower(INTAKING);
                        leftGateServo.setPosition(closeLeftGateServo);
                        shooterGateServo.setPosition(closeShooterGateServo);
                        follower.followPath(grabPickup1, true);
                        aimTurret(false);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    if (detectedID == 23) {
                        follower.followPath(collectPickup2, intakePathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    } else {
                        follower.followPath(collectPickup2, sortPathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenLeft);
                    }
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getPose().getX() > firstBallXRed && detectedID == 22 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < firstBallXBlue && detectedID == 22 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() > secondBallXRed && detectedID == 21 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < secondBallXBlue && detectedID == 21 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    shotCounter = 0;
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (shotCounter == 0) {
                        launch(true, false);
                        if (detectedID == 21) {
                            launch(true, true);
                            leftGateServo.setPosition(openLeftGateServo);
                        }
                    }
                    if (shotCounter == 1) {
                        if (detectedID == 22) {
                            launch(true, true);
                            leftGateServo.setPosition(openLeftGateServo);
                        }  else {
                            launch(true, false);
                        }
                    } else if (shotCounter == 2) {
                        if (detectedID == 22) {
                            launch(true, true);
                            leftGateServo.setPosition(openLeftGateServo);
                        } else {
                            launch(true, false);
                        }
                    } else if (shotCounter >= 3) {
                        intakeMotor.setPower(INTAKING);
                        leftGateServo.setPosition(closeLeftGateServo);
                        shooterGateServo.setPosition(closeShooterGateServo);
                        follower.followPath(grabPickup3, true);
                        aimTurret(false);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    if (detectedID == 21) {
                        follower.followPath(collectPickup2, intakePathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    } else {
                        follower.followPath(collectPickup2, sortPathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenLeft);
                    }
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getPose().getX() > firstBallXRed && detectedID == 23 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < firstBallXBlue && detectedID == 23 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() > secondBallXRed && detectedID == 22 && colorAlliance == 1) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (follower.getPose().getX() < secondBallXRed && detectedID == 22 && colorAlliance == 2) {
                    sorterServo.setPosition(sorterServoOpenRight);
                }
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    intakeMotor.setPower(STOP_SPEED);
                    shotCounter = 0;
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    if (shotCounter == 0) {
                        intakeMotor.setPower(TRANSFER_POWER);
                        launch(true, false);
                        if (detectedID == 22) {
                            leftGateServo.setPosition(openLeftGateServo);
                        }
                    }
                    if (shotCounter == 1) {
                        launch(true, true);
                        if (detectedID == 23) {
                            leftGateServo.setPosition(openLeftGateServo);
                        }
                    } else if (shotCounter == 2) {
                        launch(true, true);
                        if (detectedID == 23) {
                            leftGateServo.setPosition(openLeftGateServo);
                        }
                    } else if (shotCounter >= 3) {
                        intakeMotor.setPower(STOP_SPEED);
                        sorterServo.setPosition(sorterServoOpenRight);
                        leftGateServo.setPosition(closeLeftGateServo);
                        shooterGateServo.setPosition(closeShooterGateServo);
                        aimTurret(false);
                        setPathState(-123456789);
                    }
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        ballTimer = new Timer();

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
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");
        shooterGateServo = hardwareMap.get(Servo.class, "shooterGateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(STOP_SPEED);

        launcher1.setZeroPowerBehavior(BRAKE);
        launcher2.setZeroPowerBehavior(BRAKE);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .basicFF(launcherFF)
                .build();

        turretController = ControlSystem.builder()
                .posPid(turretPIDCoefficients)
                .build();

        launcher1.setPower(STOP_SPEED);
        launcher2.setPower(STOP_SPEED);
        launcherController.setGoal(stopLauncherKineticState);

        shooterGateServo.setPosition(closeShooterGateServo);
        sorterServo.setPosition(sorterServoOpenRight);
        leftGateServo.setPosition(closeLeftGateServo);
        hoodServo.setPosition(hoodRest);

        // Ensure we're using pipeline 0
        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        if (!lockedIn) {
            telemetry.addLine("yo lock ts in fr");

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

            telemetry.addLine("Use the dpad to control the number of balls in auto");

            if (gamepad1.dpadUpWasPressed()) {
                shootGateLoop++;
            } else if (gamepad1.dpadDownWasPressed()) {
                shootGateLoop--;
            } else if (shootGateLoop <= 0) {
                shootGateLoop = 1;
            }

            if (shootGateLoop == 2) {
                telemetry.addData("Selected Ball Count", "18 ball auto");
            } else if (shootGateLoop == 1) {
                telemetry.addData("Selected Ball Count", "15 ball auto");
            } else if (shootGateLoop == 3) {
                telemetry.addData("Selected Ball Count", "21 ball auto");
            } else if (shootGateLoop == 4) {
                telemetry.addData("Selected Ball Count", "24 ball auto");
            } else if (shootGateLoop >= 5) {
                telemetry.addData("Selected Ball Count", "dawg is ts even possible, turn ts down bruh i'm gonna run out of if statements");
                shootGateLoop = 5;
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
        }

        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();

        if (colorAlliance == 0) {
            colorAlliance = 1;
        }

        autoToTeleop.colorAlliance = colorAlliance;
        autoToTeleop.startingPosition = 1;

        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Calculate PID for launcher velocity based on goal (set earlier) and current velocity
        currentLauncherKineticState = new KineticState(launcher1.getCurrentPosition(), launcher1.getVelocity());
        launcher1.setPower(launcherController.calculate(currentLauncherKineticState));
        launcher2.setPower(launcherController.calculate(currentLauncherKineticState));

        // DONE: turret stuff

        turretController.setGoal(turretTargetKineticState);
        turretCurrentKineticState = new KineticState(turretMotor.getCurrentPosition());
        double turretPower = turretController.calculate(turretCurrentKineticState);
        turretMotor.setPower(turretPower);

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

    void aimTurret(boolean aimAtGoal) {
        if (aimAtGoal) {
            turretTargetPosition = calculateTurretPosition(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            turretTargetKineticState = new KineticState(turretTargetPosition);
        } else {
            turretTargetKineticState = turretRestKineticState;
        }

    }

    void launch(boolean shotRequested, boolean openGate) {
        switch(launchState) {
            case IDLE: // idle per SHOT
                if (shotRequested) {
                    launchState = LaunchState.MOTORS;
                } else {
                    intakeMotor.setPower(STOP_SPEED);
                }
                break;

            case MOTORS:
                launcherController.setGoal(closeTargetLauncherKineticState);
                shooterGateServo.setPosition(openShooterGateServo);
                if (launcher1.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                    launchState = LaunchState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                intakeMotor.setPower(TRANSFER_POWER);
                ballTimer.resetTimer();
                break;

            case LAUNCHED:
                if (shotCounter >= 2) {
                    // penultimate shot, in the process of third
                    shotCounter = 0; // just reset since we're in last ball
                    // go back to no shooting mode
                    intakeMotor.setPower(STOP_SPEED);
                    ballTimer.resetTimer();
                    launchState = LaunchState.IDLE;
                } else if (openGate) {
                    // waiting for ball to intake
                    if (ballTimer.getElapsedTimeSeconds() > MAX_BALL_TIME) {
                        // past the usual time to shoot, assuming shot is completed
                        shotCounter++;
                        ballTimer.resetTimer();
                        launchState = LaunchState.IDLE;
                    }

                } else if (ballTimer.getElapsedTimeSeconds() > MAX_BALL_TIME) {
                    // past the usual time to shoot, assuming shot is completed
                    shotCounter++;
                    ballTimer.resetTimer();
                    launchState = LaunchState.IDLE;

                }
                break;
        }
    }

    double calculateTurretPosition(double currentX, double currentY, double robotHeadingRad) {

        // ===== FIELD GOAL =====
        double goalX = (colorAlliance == 1) ? 142 : 2;
        double goalY = 142;

        // ===== SHOOTER OFFSET (behind robot center) =====
        double shooterOffset = -2.5; // inches

        double shooterX = currentX - shooterOffset * Math.cos(robotHeadingRad);
        double shooterY = currentY - shooterOffset * Math.sin(robotHeadingRad);

        // ===== ANGLE TO GOAL (FIELD FRAME) =====
        double dx = goalX - shooterX;
        double dy = goalY - shooterY;
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // ===== RELATIVE ANGLE (ROBOT FRAME) =====
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);
        double relativeAngleDeg = targetAngleDeg - robotHeadingDeg;

        // Wrap to [-180, 180]
        relativeAngleDeg = wrapTo180(relativeAngleDeg);

        // Invert if motor direction requires it
        relativeAngleDeg *= -1;

        // ===== ANGLE → ENCODER TICKS =====
        final double TICKS_PER_DEGREE = 1538.0 / 360.0; // ≈ 4.27

        int targetTicks = (int) Math.round(relativeAngleDeg * TICKS_PER_DEGREE);

        return targetTicks;
    }

    double wrapTo180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
