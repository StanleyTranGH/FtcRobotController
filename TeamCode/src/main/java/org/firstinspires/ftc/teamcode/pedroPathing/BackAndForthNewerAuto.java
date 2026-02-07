package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
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

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Autonomous(name = "Back and Forth Newer Auto", group = "Official")
public class BackAndForthNewerAuto extends OpMode {
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
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1800; // todo: find target velocity
    final double FAR_LAUNCHER_MIN_VELOCITY = 1760;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1840;
    final double STOP_SPEED = 0.0;
    final double MAX_BALL_TIME = 0.9; // todo: find actual max feed time
    final double MAX_SCAN_TIME = 2.0;
    final double firstBallXRed = 106.3;
    final double secondBallXRed = 109.5;
    final double thirdBallXRed = 113.7;
    final double firstBallXBlue = 38.8;
    final double secondBallXBlue = 33.9;
    final double thirdBallXBlue = 28.5;
    final double INTAKING = 1.0;
    final double OUTAKING = -1.0;
    final double TRANSFER_POWER = 1.0; // todo: find the right transfer power
    double turretScore;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0005);
    public static double launcherFF = 0.0003;
    public static PIDCoefficients turretPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0002); // DONE: TUNE TURRET PID
    ControlSystem launcherController;
    ControlSystem turretController;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
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
    boolean prioritizePark = false;
    boolean lockedIn = false;

    // pedropathing
    private int pathState; // todo: use visualizer find poses
    private Pose startPose = new Pose(80.7, 9, Math.toRadians(0)); // start pose
    private Pose shootPose = new Pose(87, 16, Math.toRadians(0)); // shoot pose
    private final Pose collectPickup1Pose = new Pose(133, 9.2, Math.toRadians(0)); // collect 1st set of artifacts in loading zone
    private final Pose collectPickup1ReloadPose = new Pose(125, 12, Math.toRadians(0)); // ensure we actually intake them, i saw other teams do ts
    private final Pose collectPickup1BPose = new Pose(133, 14, Math.toRadians(5)); //
    private final Pose collectExtraAPose = new Pose(133, 10.3, Math.toRadians(15)); //
    private final Pose collectExtraReloadPose = new Pose(118, 18, Math.toRadians(20)); //
    private final Pose collectExtraBPose = new Pose(134, 23, Math.toRadians(20)); //
    private Pose parkPose = new Pose(120, 12, Math.toRadians(0)); //

    PathChain shootPreload, collectPickup1, shootPickup1,  collectExtra, shootExtra, park;
    public void buildPaths() {
        if (colorAlliance == 1) { // red (MIRROR ALL CHANGES IN BLUE)
            turretScore = 254; // todo: find real turretScore
            shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collectPickup1Pose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectPickup1Pose.getHeading())
                    .addPath(new BezierLine(collectPickup1Pose, collectPickup1ReloadPose))
                    .setLinearHeadingInterpolation(collectPickup1Pose.getHeading(), collectPickup1ReloadPose.getHeading())
                    .addPath(new BezierLine(collectPickup1ReloadPose, collectPickup1BPose))
                    .setLinearHeadingInterpolation(collectPickup1ReloadPose.getHeading(), collectPickup1BPose.getHeading())
                    .build();
            shootPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collectPickup1BPose, shootPose))
                    .setLinearHeadingInterpolation(collectPickup1BPose.getHeading(), shootPose.getHeading())
                    .build();

            collectExtra = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collectExtraAPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectExtraAPose.getHeading())
                    .addPath(new BezierLine(collectExtraAPose, collectExtraReloadPose))
                    .setLinearHeadingInterpolation(collectExtraAPose.getHeading(), collectExtraReloadPose.getHeading())
                    .addPath(new BezierLine(collectExtraReloadPose, collectExtraBPose))
                    .setLinearHeadingInterpolation(collectExtraReloadPose.getHeading(), collectExtraBPose.getHeading())
                    .build();
            shootExtra = follower.pathBuilder()
                    .addPath(new BezierLine(collectExtraBPose, shootPose))
                    .setLinearHeadingInterpolation(collectExtraBPose.getHeading(), shootPose.getHeading())
                    .build();

            if (!prioritizePark) {
                park = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, parkPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                        .build();
            }

        } else { // blue
            startPose = startPose.mirror();
            parkPose = parkPose.mirror();
            shootPose = shootPose.mirror();

            turretScore = -254; // todo: find real turretScore

            shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            collectPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collectPickup1Pose.mirror()))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectPickup1Pose.mirror().getHeading())
                    .addPath(new BezierLine(collectPickup1Pose.mirror(), collectPickup1ReloadPose.mirror()))
                    .setLinearHeadingInterpolation(collectPickup1Pose.mirror().getHeading(), collectPickup1ReloadPose.mirror().getHeading())
                    .addPath(new BezierLine(collectPickup1ReloadPose.mirror(), collectPickup1BPose.mirror()))
                    .setLinearHeadingInterpolation(collectPickup1ReloadPose.mirror().getHeading(), collectPickup1BPose.mirror().getHeading())
                    .build();
            shootPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(collectPickup1BPose.mirror(), shootPose))
                    .setLinearHeadingInterpolation(collectPickup1BPose.mirror().getHeading(), shootPose.getHeading())
                    .build();

            collectExtra = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collectExtraAPose.mirror()))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collectExtraAPose.mirror().getHeading())
                    .addPath(new BezierLine(collectExtraAPose.mirror(), collectExtraReloadPose.mirror()))
                    .setLinearHeadingInterpolation(collectExtraAPose.mirror().getHeading(), collectExtraReloadPose.mirror().getHeading())
                    .addPath(new BezierLine(collectExtraReloadPose.mirror(), collectExtraBPose.mirror()))
                    .setLinearHeadingInterpolation(collectExtraReloadPose.mirror().getHeading(), collectExtraBPose.mirror().getHeading())
                    .build();
            shootExtra = follower.pathBuilder()
                    .addPath(new BezierLine(collectExtraBPose.mirror(), shootPose))
                    .setLinearHeadingInterpolation(collectExtraBPose.mirror().getHeading(), shootPose.getHeading())
                    .build();

            if (!prioritizePark) {
                park = follower.pathBuilder()
                        .addPath(new BezierLine(shootPose, parkPose))
                        .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                        .build();
            }
        }
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                launcherController.setGoal(farTargetLauncherKineticState);
                hoodServo.setPosition(hoodScore);
                sorterServo.setPosition(sorterServoOpenRight);
                launch(0, -1);
                follower.followPath(shootPreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(collectPickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    launch(0, -1);
                    follower.followPath(shootPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(collectExtra, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    launch(0, -1);
                    follower.followPath(shootExtra);
                    setPathState(3);
                }
            case 1000:
                if (!prioritizePark) {
                    PathChain shootExtra = follower.pathBuilder()
                            .addPath(new BezierLine(follower::getPose, shootPose)) //DONE: GET PARK POSE
                            .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                            .build();
                    launch(0, -1);
                    follower.followPath(shootExtra, true);
                    setPathState(1001);
                } else {
                    park = follower.pathBuilder()
                            .addPath(new BezierLine(follower::getPose, parkPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), parkPose.getHeading())
                            .build();
                    launch(2, -1);
                    launch(2, -1);
                    follower.followPath(park, true);
                    setPathState(-96485);
                }
                break;
            case 1001:
                if (!follower.isBusy()) {
                    if (launchState != LaunchState.LAUNCHED) {
                        launch(1, 0);
                    } else {
                        launch(2, -1);
                        launch(2, -1);
                        follower.followPath(park, true);
                        setPathState(-96485);
                    }
                }
                break;
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
            if (gamepad1.xWasPressed()) {
                prioritizePark = true; // will park
            } else if (gamepad1.yWasPressed()) {
                prioritizePark = false; // shoot then park
            }

            if (colorAlliance == 1) {
                telemetry.addData("Selected Color", "Red");
            } else if (colorAlliance == 2) {
                telemetry.addData("Selected Color", "Blue");
            } else if (colorAlliance == 0) {
                telemetry.addData("Selected Color", "No Color Selected");
            }

            telemetry.addLine("Press X for prioritizing park (will not shoot remaining balls), Y to shoot and then park");

            if (prioritizePark) {
                telemetry.addData("Prioritize Park", "True");
            } else {
                telemetry.addData("Prioritize Park", "False");
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

            if (prioritizePark) {
                telemetry.addData("Prioritize Park", "True");
            } else {
                telemetry.addData("Prioritize Park", "False");
            }
        }
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
        if (colorAlliance == 1) {
            telemetry.addData("Selected Color", "Red");
        } else if (colorAlliance == 2) {
            telemetry.addData("Selected Color", "Blue");
        }

        if (prioritizePark) {
            if (opmodeTimer.getElapsedTimeSeconds() > 26 && opmodeTimer.getElapsedTimeSeconds() < 27) {
                setPathState(1000);
            }
        } else {
            if (opmodeTimer.getElapsedTimeSeconds() > 24 && opmodeTimer.getElapsedTimeSeconds() < 25) {
                setPathState(1000);
            }
        }

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
                    launcherController.setGoal(farTargetLauncherKineticState);
                } else if (shotRequested == 1) { // shooting
                    if (launcher1.getVelocity() > FAR_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < FAR_LAUNCHER_MAX_VELOCITY) {
                        intakeMotor.setPower(TRANSFER_POWER);
                        ballTimer.resetTimer();
                        launchState = LaunchState.LAUNCHING;
                    }
                } else if (shotRequested == 2) { // post shooting
                    leftGateServo.setPosition(closeLeftGateServo);
                    shooterGateServo.setPosition(closeShooterGateServo);
                    sorterServo.setPosition(sorterServoOpenRight);
                    intakeMotor.setPower(INTAKING);
                    aimTurret(false);
                }
                break;

            case LAUNCHING:
                if (openGate == 0 && ballTimer.getElapsedTimeSeconds() > MAX_BALL_TIME) {
                    launchState = LaunchState.LAUNCHED;
                }
                break;

            case LAUNCHED:
                // just a way to detect if done shooting
                launchState = LaunchState.IDLE;
                break;
        }
    }
}
