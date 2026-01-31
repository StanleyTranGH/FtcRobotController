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
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1380; // todo: find target velocity
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1340;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1430;
    final double STOP_SPEED = 0.0;
    final double MAX_BALL_TIME = 3.0; // todo: find actual max feed time
    final double ONE_BALL_TIME = 0.4;
    final double TWO_BALL_TIME = 0.9;
    final double MAX_SCAN_TIME = 2.0;
    final double MAX_RAMP_TIME = 2.0; // todo: find actual ramp time
    final double firstBallXRed = 109.1 ; //was 105.3
    final double secondBallXRed = 115.1; //was 108.5, then 113.1
    final double thirdBallXRed = 112.7;
    final double firstBallXBlue = 39.8;
    final double secondBallXBlue = 34.9;
    final double thirdBallXBlue = 29.5;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    final double TRANSFER_POWER = 1.0; // todo: find the right transfer power
    double turretScore = 156;
    double parkTurretEncoderPosition, regularTurretEncoderPosition;
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
    private final Pose collect2Pose = new Pose(134, 56, Math.toRadians(0)); // Collect second set of artifacts
    private final Pose score2ControlPose = new Pose(88, 54); // Avoid 1 to score 2
    private final Pose openGateControlPose = new Pose(107.7, 53); // From 2 to open Gate
    private final Pose openGateControlPoseB = new Pose(115, 56); // From score to open gate, avoid 1
    private final Pose openGatePose = new Pose(128, 66, Math.toRadians(0)); // Opens Gate
    private final Pose collectGatePose = new Pose(135, 61, Math.toRadians(35)); // Collects from Gate
    private final Pose openGatePoseB = new Pose(125, 59.77, Math.toRadians(27)); // Pre open gate
    private final Pose collectGatePoseB = new Pose(135.45, 59.7, Math.toRadians(27)); // Collects from Gate directly with open
    private final Pose scoreGateControlPose = new Pose(86, 55); // Avoid gate and 1 to score Gate
    private final Pose pickup1Pose = new Pose(101, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose collect1Pose = new Pose(128, 82, Math.toRadians(0)); // Collect first set of artifacts
    private final Pose pickup3Pose = new Pose(101, 35, Math.toRadians(0) ); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose collect3Pose = new Pose(134, 35, Math.toRadians(0)); // Collect third set of artifacts

    int shootGateLoop = 0;
    final double intakePathSpeed = 0.8;
    final double sortPathSpeed = 0.30; //was 0.35

    PathChain scanObelisk, scorePreload, directScorePreload, grabPickup2, collectPickup2, scorePickup2, openGate, collectPickupGate, openGateB, collectPickupGateB, scorePickupGateB, scorePickupGate, grabPickup1, collectPickup1, scorePickup1, grabPickup3, collectPickup3, scorePickup3;
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
            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose, openGateControlPose, openGatePose))
                    .setLinearHeadingInterpolation(collect2Pose.getHeading(), openGatePose.getHeading())
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(openGatePose, score2ControlPose, scorePose))
                    .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                    .build();

            openGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, openGateControlPoseB, openGatePoseB))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePoseB.getHeading())
                    .build();
            collectPickupGateB = follower.pathBuilder()
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
            openGate = follower.pathBuilder()
                    .addPath(new BezierCurve(collect2Pose.mirror(), openGateControlPose.mirror(), openGatePose.mirror()))
                    .setLinearHeadingInterpolation(collect2Pose.mirror().getHeading(), openGatePose.mirror().getHeading())
                    .build();
            scorePickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(openGatePose.mirror(), score2ControlPose.mirror(), scorePose.mirror()))
                    .setLinearHeadingInterpolation(openGatePose.mirror().getHeading(), scorePose.mirror().getHeading())
                    .build();

            openGateB = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose.mirror(), openGateControlPoseB.mirror(), openGatePoseB.mirror()))
                    .setLinearHeadingInterpolation(scorePose.mirror().getHeading(), openGatePoseB.mirror().getHeading())
                    .build();
            collectPickupGateB = follower.pathBuilder()
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
                turretScore = regularTurretEncoderPosition;
                shooterGateServo.setPosition(closeShooterGateServo);
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
                    detectedID = 23;
                    motif = "Couldn't Detect! Guessing PPG";
                }

                if (obeliskID != 0) {
                    follower.followPath(scorePreload);
                    launch(0, -1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        intakeMotor.setPower(INTAKING);
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(grabPickup2, true);
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
                        sorterServo.setPosition(sorterServoOpenLeft);
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
                    follower.followPath(openGate,true);
                    intakeMotor.setPower(STOP_SPEED);
                    launch(0, -1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(STOP_SPEED);
                    follower.followPath(scorePickup2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if ((detectedID == 23) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 1);
                    } else if ((detectedID == 21) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 2);
                    } else if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(INTAKING);
                        if (shootGateLoop == -1) {
                            follower.followPath(grabPickup1, true);
                            setPathState(11);
                        } else {
                            follower.followPath(openGateB,true);
                            setPathState(7);
                        }
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(collectPickupGateB, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > MAX_RAMP_TIME) {
                    follower.followPath(scorePickupGateB, true);
                    intakeMotor.setPower(STOP_SPEED);
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
                        intakeMotor.setPower(INTAKING);
                        launch(2, -1);
                        launch(2, -1);
                        shootGateLoop--;
                        setPathState(7);
                    } else {
                        intakeMotor.setPower(INTAKING);
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(grabPickup1, true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (detectedID == 23) {
                        follower.followPath(collectPickup1, intakePathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    } else {
                        follower.followPath(collectPickup1, sortPathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenLeft);
                    }
                    setPathState(12);
                }
                break;
            case 12:
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
                    launch(0, -1);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    if ((detectedID == 21) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 1);
                    } else if ((detectedID == 22) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 2);
                    } else if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        intakeMotor.setPower(INTAKING);
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(grabPickup3, true);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    if (detectedID == 21) {
                        follower.followPath(collectPickup3, intakePathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenRight);
                    } else {
                        follower.followPath(collectPickup3, sortPathSpeed, true);
                        sorterServo.setPosition(sorterServoOpenLeft);
                    }
                    setPathState(15);
                }
                break;
            case 15:
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
                    turretScore = parkTurretEncoderPosition;
                    follower.followPath(scorePickup3, true);
                    launch(0, -1);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    if ((detectedID == 22) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 1);
                    } else if ((detectedID == 23) && (launchState != LaunchState.LAUNCHED)) {
                        launch(1, 2);
                    } else if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        intakeMotor.setPower(STOP_SPEED);
                        sorterServo.setPosition(sorterServoOpenRight);
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
            case IDLE:
                if (shotRequested == 0) { // pre shooting
                    aimTurret(true);
                    shooterGateServo.setPosition(openShooterGateServo);
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    intakeMotor.setPower(STOP_SPEED);
                } else if (shotRequested == 1) { // shooting
                    if (launcher1.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        intakeMotor.setPower(TRANSFER_POWER);
                        ballTimer.resetTimer();
                        launchState = LaunchState.LAUNCHING;
                    }
                } else if (shotRequested == 2) { // post shooting
                    leftGateServo.setPosition(closeLeftGateServo);
                    shooterGateServo.setPosition(closeShooterGateServo);
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
