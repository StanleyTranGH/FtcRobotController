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

@Autonomous(name = "No Sort Auto", group = "Official")
public class NoSortAuto extends OpMode{
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
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1420; // todo: find target velocity
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1340;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1440;
    final double STOP_SPEED = 0.0;
    final double MAX_BALL_TIME = 0.9; // todo: find actual max feed time
    final double ONE_BALL_TIME = 0.15;
    final double TWO_BALL_TIME = 0.15;
    final double MAX_SCAN_TIME = 2.0;
    final double MAX_RAMP_TIME_A = 1.0; // todo: find actual ramp time
    final double MAX_RAMP_TIME = 2.0; // todo: find actual ramp time
    final double firstBallXRed = 106.3;
    final double secondBallXRed = 109.5;
    final double thirdBallXRed = 113.7;
    final double firstBallXBlue = 38.8;
    final double secondBallXBlue = 33.9;
    final double thirdBallXBlue = 28.5;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    final double TRANSFER_POWER = 1.0; // todo: find the right transfer power
    double parkTurretEncoderPosition, regularTurretEncoderPosition;
    double turretScore = regularTurretEncoderPosition;
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
    double hoodScore = 0.3; // todo: find hood angles
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
    private Pose startPose = new Pose(125, 120, Math.toRadians(127)); // Start Pose of our robot.
    private final Pose scanPose = new Pose(105, 105, Math.toRadians(127)); // Scan Obelisk
    private final Pose scorePose = new Pose(90, 90, Math.toRadians(0)); // Scoring Pose of our robot.
    private final Pose parkScorePose = new Pose (93, 110, Math.toRadians(0)); // Parked Scoring Pose of our robot.
    private final Pose pickup2Pose = new Pose(101, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose collect2Pose = new Pose(134, 58, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose score2ControlPose = new Pose(88, 54); // Avoid 1 to score 2
    private final Pose openGateControlPose = new Pose(112.7, 53); // From 2 to open Gate
    private final Pose openGateControlPoseB = new Pose(115, 53); // From score to open gate, avoid 1
    private final Pose openGatePose = new Pose(128, 66, Math.toRadians(0)); // Opens Gate
    private final Pose collectGatePose = new Pose(135, 60, Math.toRadians(35)); // Collects from Gate
    private final Pose openGatePoseB = new Pose(125, 56, Math.toRadians(27)); // Pre open gate
    private final Pose collectGatePoseB = new Pose(134.81, 60.61, Math.toRadians(30.34)); // Collects from Gate directly with open
    private final Pose scoreGateControlPose = new Pose(86, 55); // Avoid gate and 1 to score Gate
    private final Pose pickup1Pose = new Pose(101, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(128, 84, Math.toRadians(0)); // Collect first set of artifacts
    private final Pose pickup3Pose = new Pose(101, 37, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(134, 37, Math.toRadians(0)); // Collect third set of artifacts

    int shootGateLoop = 1;
    int initShootGateLoop;
    final double intakePathSpeed = 1.0;
    final double extraIntakeTime = 0.2;

    PathChain scanObelisk, scorePreload, directScorePreload, grabPickup2, collectPickup2, collectPickup2Chain, scorePickup2, scorePickup2B, openGate, collectPickupGate, openGateB, collectPickupGateB, collectPickupGateBChain,  scorePickupGateB, scorePickupGate, grabPickup1, collectPickup1, collectPickup1Chain, scorePickup1, grabPickup3, collectPickup3, collectPickup3Chain, scorePickup3;

    void buildPaths() {
        if (colorAlliance == 1) { //RED
            //NOTE: IF YOU CHANGE ANYTHING IN THIS IF STATEMENT, MAKE SURE TO CHANGE IT IN THE OTHER IF STATEMENT SO IT AFFECTS BOTH COLORS
            parkTurretEncoderPosition = 97;
            regularTurretEncoderPosition = 156;
            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scanPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                    .build();
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose, scorePose))
                    .setLinearHeadingInterpolation(scanPose.getHeading(), scorePose.getHeading())
                    .build();
            directScorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose, collect2Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            collectPickup2Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup2Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                    .addPath(new BezierLine(pickup2Pose, collect2Pose))
                    .setTangentHeadingInterpolation()
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose, openGateControlPose, openGatePose))
                    .setLinearHeadingInterpolation(collect2Pose.getHeading(), openGatePose.getHeading())
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(openGatePose, score2ControlPose, scorePose))
                    .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                    .build();

            scorePickup2B = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose, score2ControlPose, scorePose))
                    .setLinearHeadingInterpolation(collect2Pose.getHeading(), scorePose.getHeading())
                    .build();

            openGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, openGateControlPoseB, openGatePoseB))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePoseB.getHeading())
                    .build();
            collectPickupGateB = follower.pathBuilder()
                    .addPath(new BezierLine(openGatePoseB, collectGatePoseB))
                    .setLinearHeadingInterpolation(openGatePoseB.getHeading(), collectGatePoseB.getHeading())
                    .build();
            collectPickupGateBChain = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, openGateControlPoseB, openGatePoseB))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePoseB.getHeading())
                    .addPath(new BezierLine(openGatePoseB, collectGatePoseB))
                    .setLinearHeadingInterpolation(openGatePoseB.getHeading(), collectGatePoseB.getHeading())
                    .build();

            scorePickupGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(collectGatePoseB, scoreGateControlPose, scorePose))
                    .setLinearHeadingInterpolation(collectGatePoseB.getHeading(), scorePose.getHeading())
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
            collectPickup1Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup1Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
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
            collectPickup3Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, pickup3Pose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                    .addPath(new BezierLine(pickup3Pose, collect3Pose))
                    .setTangentHeadingInterpolation()
                    .build();
            scorePickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Pose, parkScorePose))
                    .setLinearHeadingInterpolation(collect3Pose.getHeading(), parkScorePose.getHeading())
                    .build();

        } else if (colorAlliance == 2) { //BLUE
            startPose = startPose.mirror();
            parkTurretEncoderPosition = -110;
            regularTurretEncoderPosition = -170;

            scanObelisk = follower.pathBuilder()
                    .addPath(new BezierLine(startPose.mirror(), scanPose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scanPose.mirror().getHeading())
                    .build();
            scorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(scanPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(scanPose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();
            directScorePreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(startPose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            grabPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup2Pose.mirror().getHeading())
                    .build();
            collectPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(pickup2Pose.mirror(), collect2Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();
            collectPickup2Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup2Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup2Pose.mirror().getHeading())
                    .addPath(new BezierLine(pickup2Pose.mirror(), collect2Pose.mirror()))
                    .setTangentHeadingInterpolation()
                    .build();

            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose.mirror(), openGateControlPose.mirror(), openGatePose.mirror()))
                    .setLinearHeadingInterpolation(collect2Pose.mirror().getHeading(), openGatePose.mirror().getHeading())
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(openGatePose.mirror(), score2ControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(openGatePose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            scorePickup2B = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose.mirror(), score2ControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collect2Pose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            openGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose.mirror(), openGateControlPoseB.mirror(), openGatePoseB.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), openGatePoseB.mirror().getHeading())
                    .build();
            collectPickupGateB = follower.pathBuilder()
                    .addPath(new BezierLine(openGatePoseB.mirror(), collectGatePoseB.mirror()))
                    .setLinearHeadingInterpolation(openGatePoseB.mirror().getHeading(), collectGatePoseB.mirror().getHeading())
                    .build();
            collectPickupGateBChain = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose.mirror(), openGateControlPoseB.mirror(), openGatePoseB.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), openGatePoseB.mirror().getHeading())
                    .addPath(new BezierLine(openGatePoseB.mirror(), collectGatePoseB.mirror()))
                    .setLinearHeadingInterpolation(openGatePoseB.mirror().getHeading(), collectGatePoseB.mirror().getHeading())
                    .build();

            scorePickupGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(collectGatePoseB.mirror(), scoreGateControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(collectGatePoseB.mirror().getHeading(), scorePose.mirror().getHeading())
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
            collectPickup1Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup1Pose.mirror().getHeading())
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
            collectPickup3Chain = follower.pathBuilder()
                    .addPath(new BezierLine(scorePose.mirror(), pickup3Pose.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), pickup3Pose.mirror().getHeading())
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
                turretScore = regularTurretEncoderPosition;
                launcherController.setGoal(closeTargetLauncherKineticState);
                intakeMotor.setPower(INTAKING);
                follower.followPath(directScorePreload, true);
                hoodServo.setPosition(hoodScore);
                sorterServo.setPosition(sorterServoOpenRight);
                setPathState(1);
                break;
            case 1:
                /*

               COMMENTING ALL OF THIS OUT SINCE WE DON'T NEED TO SCAN

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
                    detectedID = 23;
                    motif = "Couldn't Detect! Guessing PPG";
                }
                */
                obeliskID = 23;
                detectedID = 23;
                motif = "this motif doesn't even scan bru";

                if (obeliskID != 0) {
                    launch(0, -1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(collectPickup2Chain, true);
                        intakeMotor.setPower(INTAKING);
                        setPathState(801);
                    }
                }
                break;
            case 3: // not chain
                if (!follower.isBusy()) {
                    intakeMotor.setPower(INTAKING);
                    follower.followPath(collectPickup2, intakePathSpeed, true);
                    setPathState(501);
                }
                break;

            case 801:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2B);
                    launch(0, -1);
                    setPathState(202);
                }
                break;

            case 501:
                if (!follower.isBusy()) {
                    follower.followPath(openGate, true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    setPathState(202);
                }
                break;

            case 202:
                if(pathTimer.getElapsedTimeSeconds() > extraIntakeTime) {
                    intakeMotor.setPower(STOP_SPEED);
                    setPathState(204);
                }
                break;
            case 203:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    launch(0, -1);
                    setPathState(204);
                }
                break;
            case 204:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(INTAKING);
                        if (shootGateLoop == -1) {
                            follower.followPath(collectPickup1Chain, true);
                            setPathState(11);
                        } else {
                            intakeMotor.setPower(INTAKING);
                            follower.followPath(collectPickupGateBChain,true);
                            setPathState(901);
                        }
                    }
                }
            break;
                //TODO: CHECK IF OPENING GATE CODE WORKS
            case 901:
                if (!follower.isBusy()) {
                    setPathState(106);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(collectPickupGateB, true);
                    intakeMotor.setPower(INTAKING);
                    setPathState(106);
                }
                break;
            case 106:
                if (initShootGateLoop == shootGateLoop) {
                    if (pathTimer.getElapsedTimeSeconds() > MAX_RAMP_TIME_A) {
                        follower.followPath(scorePickupGateB, true);
                        launcherController.setGoal(closeTargetLauncherKineticState);
                        setPathState(502);
                    }
                } else if (pathTimer.getElapsedTimeSeconds() > MAX_RAMP_TIME) {
                    follower.followPath(scorePickupGateB, true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    setPathState(502);
                }
                break;
            case 502:
                if (pathTimer.getElapsedTimeSeconds() > extraIntakeTime) {
                    launch(0, -1);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else if (shootGateLoop > 0) {
                        follower.followPath(openGateB, true);
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(INTAKING);
                        shootGateLoop--;
                        setPathState(5);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(INTAKING);
                        follower.followPath(collectPickup1Chain, true);
                        setPathState(12);
                    }
                }
                break;
            case 11: // not chain
                if (!follower.isBusy()) {
                    follower.followPath(collectPickup1, intakePathSpeed, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    setPathState(503);
                }
                break;
            case 503:
                if (pathTimer.getElapsedTimeSeconds() > extraIntakeTime) {
                    launch(0, -1);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(INTAKING);
                        follower.followPath(collectPickup3Chain, true);
                        setPathState(504);
                    }
                }
                break;
            case 15: // not chain
                if (!follower.isBusy()) {
                    follower.followPath(collectPickup3, intakePathSpeed, true);
                    setPathState(504);
                }
                break;
            case 504:
                if (!follower.isBusy()) {
                    turretScore = parkTurretEncoderPosition;
                    follower.followPath(scorePickup3, true);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > extraIntakeTime) {
                    launch(0, -1);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        intakeMotor.setPower(STOP_SPEED);
                        sorterServo.setPosition(sorterServoOpenRight);
                        launch(2, -1);
                        launch(2, -1);
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

        follower = Constants.createFollower(hardwareMap);

        /* Limelight Stuff */
        // Initialize Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        leftGateServo = hardwareMap.get(Servo.class, "gateServo");
        shooterGateServo = hardwareMap.get(Servo.class, "shooterGateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
            } else if (shootGateLoop < -1) {
                shootGateLoop = -1;
            }

            if (shootGateLoop == 1) {
                telemetry.addData("Selected Ball Count", "18 ball auto");
            } else if (shootGateLoop == 0) {
                telemetry.addData("Selected Ball Count", "15 ball auto");
            } else if (shootGateLoop == 2) {
                telemetry.addData("Selected Ball Count", "21 ball auto");
            } else if (shootGateLoop == 3) {
                telemetry.addData("Selected Ball Count", "24 ball auto");
            } else if (shootGateLoop >= 4) {
                telemetry.addData("Selected Ball Count", "dawg is ts even possible, turn ts down bruh i'm gonna run out of if statements");
                shootGateLoop = 3;
            } else if (shootGateLoop == -1) {
                telemetry.addData("Selected Ball Count", "12 ball auto");
            }

            if (gamepad1.rightBumperWasPressed()) {
                buildPaths();
                follower.setStartingPose(startPose);
                lockedIn = true;
            }
        } else {
            telemetry.addLine("---------- LOCKED IN ----------");
            initShootGateLoop = shootGateLoop;
            if (colorAlliance == 1) {
                telemetry.addData("Selected Color", "Red");
            } else if (colorAlliance == 2) {
                telemetry.addData("Selected Color", "Blue");
            } else if (colorAlliance == 0) {
                telemetry.addData("Selected Color", "No Color Selected");
            }

            if (shootGateLoop == 1) {
                telemetry.addData("Selected Ball Count", "18 ball auto");
            } else if (shootGateLoop == 0) {
                telemetry.addData("Selected Ball Count", "15 ball auto");
            } else if (shootGateLoop == 2) {
                telemetry.addData("Selected Ball Count", "21 ball auto");
            } else if (shootGateLoop == 3) {
                telemetry.addData("Selected Ball Count", "24 ball auto");
            } else if (shootGateLoop == -1) {
                telemetry.addData("Selected Ball Count", "12 ball auto");
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
            turretTargetPosition = turretScore;
            turretTargetKineticState = new KineticState(turretTargetPosition);
        } else {
            turretTargetKineticState = turretRestKineticState;
        }

    }

    void launch(int shotRequested, int openGate) {
        switch(launchState) {
            case IDLE: // idle per SHOT
                if (shotRequested == 0) { // pre shooting
                    aimTurret(true);
                    intakeMotor.setPower(STOP_SPEED);
                    shooterGateServo.setPosition(openShooterGateServo);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                } else if (shotRequested == 1) { // shooting
                    if (launcher1.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        intakeMotor.setPower(TRANSFER_POWER);
                        ballTimer.resetTimer();
                        launchState = LaunchState.LAUNCHING;
                    }
                } else if (shotRequested == 2) { // post shooting
                    leftGateServo.setPosition(closeLeftGateServo);
                    shooterGateServo.setPosition(closeShooterGateServo);
                    sorterServo.setPosition(sorterServoOpenRight);
                    aimTurret(false);
                }
                break;

            case LAUNCHING:
                if (openGate == 0 && ballTimer.getElapsedTimeSeconds() > MAX_BALL_TIME) {
                    launchState = LaunchState.LAUNCHED;
                } else if (openGate == 1 && ballTimer.getElapsedTimeSeconds() > ONE_BALL_TIME) {
                    leftGateServo.setPosition(openLeftGateServo);
                    if (ballTimer.getElapsedTimeSeconds() > TWO_BALL_TIME) {
                        launchState = LaunchState.LAUNCHED;
                    }
                } else if (openGate == 2 && ballTimer.getElapsedTimeSeconds() > TWO_BALL_TIME) {
                    leftGateServo.setPosition(openLeftGateServo);
                    if (ballTimer.getElapsedTimeSeconds() > ONE_BALL_TIME) {
                        launchState = LaunchState.LAUNCHED;
                    }
                }
                break;

            case LAUNCHED:
                // just a way to detect if done shooting
                launchState = LaunchState.IDLE;
                break;
        }
    }
}
