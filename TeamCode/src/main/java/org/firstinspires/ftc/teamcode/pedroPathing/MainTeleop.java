package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
@TeleOp(name = "Main Teleop", group = "Official")
public class MainTeleop extends OpMode {
    private Follower follower;
    //TODO: CHANGE THIS STARTING POSE AFTER AUTO IS DONE
    public static Pose startingPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    public static Pose parkPose;
    public static Pose scorePose;
    private Supplier<PathChain> parkChain, shootChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Collects the starting place from either the Nine Ball or Twelve Ball autos, depending on which is used
    private int teleopStartingPlace = 0;
    private int teleopColorAlliance = 0;

    // Limelight fields
    private Limelight3A limelight;
    private int aimTagID;
    // 0.019, 0.0006, 0.0006
    // 0.018, 0.0007, 0.0006
    // 0.018, 0.0006, 0.0007
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.015, 0, 0.001);
    ControlSystem launcherController;

    // Mechanisms
    DcMotor intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    ElapsedTime feederTimer = new ElapsedTime();

    final double launcherServoDown = 0.10;
    final double launcherServoUp = 0.47; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    String launcherRange = "CLOSE"; // CLOSE or FAR
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1540; // DONE: FIND DESIRED LAUNCHER VELOCITY
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1480;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1560;
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1780; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    final double FAR_LAUNCHER_MIN_VELOCITY = 1740;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1820;
    final double STOP_SPEED = 0.0;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);;

    final double MAX_FEED_TIME = 0.35;
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        launcher.setZeroPowerBehavior(BRAKE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)

                .build();

        launcher.setPower(0.0);

        launcherServo.setPosition(launcherServoDown);
        launcherController.setGoal(stopLauncherKineticState);

        // Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        if(NineBallAuto.startingPlace == 1 || NineBallAuto.startingPlace == 2) {
            teleopStartingPlace = NineBallAuto.startingPlace;
        } else if (TwelveBallAuto.startingPlace == 1 || TwelveBallAuto.startingPlace == 2) {
            teleopStartingPlace = TwelveBallAuto.startingPlace;
        }

        // TODO: SWAP PARK POSES FOR REAL COMPETITION (mirror this one and un-mirror other one)
        // TODO: FIND REAL PARK & SCORE POSE
        parkPose = new Pose(108.4, 33.8, Math.toRadians(90));
        scorePose = new Pose(90, 90, Math.toRadians(45));

        if(teleopStartingPlace == 1) {
            // startingPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
            startingPose = new Pose(86, 50, Math.toRadians(90));
        } else if (teleopStartingPlace == 2) {
            startingPose = new Pose(122, 95, Math.toRadians(90)); // Park Pose of our robot. // Park Pose of our robot.; //See ExampleAuto to understand how to use this
        }


        if(teleopColorAlliance == 1) {
            // Set lazy curve to red alliance base if on team red
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                    .build();

        } else {
            // Set lazy curve to blue alliance by default
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, (180 - scorePose.getHeading()), 0.8))
                    .build();
            startingPose = startingPose.mirror();
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press A for Red and B for Blue");
        telemetry.addLine("Press X for Far and Y for Close");

        if (gamepad1.aWasPressed()) {
            teleopColorAlliance = 1;
        } else if (gamepad1.bWasPressed()) {
            teleopColorAlliance = 2;
        } else if (gamepad1.xWasPressed()) {
            teleopStartingPlace = 1;
        } else if (gamepad1.yWasPressed()) {
            teleopStartingPlace = 2;
        }

        parkPose = new Pose(108.4, 33.8, Math.toRadians(90));
        scorePose = new Pose(90, 90, Math.toRadians(45));

        if(teleopStartingPlace == 1) {
            // startingPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
            startingPose = new Pose(86, 50, Math.toRadians(90));
        } else if (teleopStartingPlace == 2) {
            startingPose = new Pose(122, 95, Math.toRadians(90)); // Park Pose of our robot. // Park Pose of our robot.; //See ExampleAuto to understand how to use this
        }


        if(teleopColorAlliance == 1) {
            // Set lazy curve to red alliance base if on team red
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                    .build();

        } else {
            // Set lazy curve to blue alliance by default
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, (180 - scorePose.getHeading()), 0.8))
                    .build();
            startingPose = startingPose.mirror();
        }

        telemetry.addData("Current Selected Alliance", teleopColorAlliance);
        telemetry.addData("Current Selected Starting Position", teleopStartingPlace);
    }

    @Override
    public void start() {
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) {
                if(teleopColorAlliance == 1) {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            false // Field Centric
                    );
                } else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            false, // Field Centric
                            Math.toRadians(180)
                    );
                }
            }
                //This is how it looks with slowMode on
            else {
                if(teleopColorAlliance == 1) {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            false // Field Centric
                    );
                } else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            false, // Field Centric
                            Math.toRadians(180)
                    );
                }
            }
        }

        if (gamepad1.bWasPressed()) {
            automatedDrive = !automatedDrive;
            if (automatedDrive) {
                follower.followPath(parkChain.get());
            } else {
                follower.startTeleopDrive(true);
            }
        }

        if(gamepad1.rightBumperWasPressed()) {
            automatedDrive = !automatedDrive;
            if(automatedDrive) {
                follower.followPath(shootChain.get());
            } else {
                follower.startTeleopDrive(true);
            }
        }

        //Slow Mode toggle
        if (gamepad1.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        // Intaking
        if (gamepad1.xWasPressed()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(1);
        } else if (gamepad1.yWasPressed()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPower(1);
        } else if (gamepad1.yWasReleased()) {
            intakeMotor.setPower(0);
        } else if (gamepad1.aWasPressed()) {
            intakeMotor.setPower(0);
        }

        // Launching
        if (gamepad2.yWasPressed()) {
            launcherRange = "CLOSE";
            launcherController.setGoal(closeTargetLauncherKineticState);
            scorePose = new Pose(90, 90, Math.toRadians(45));
        } else if (gamepad2.xWasPressed()) {
            launcherRange = "FAR";
            launcherController.setGoal(farTargetLauncherKineticState);
            scorePose = new Pose(87, 18, Math.toRadians(68)); // Scoring Pose of our robot.
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            launcherController.setGoal(stopLauncherKineticState);
        }

        launch(gamepad2.rightBumperWasPressed());

        currentLauncherKineticState = new KineticState(launcher.getCurrentPosition(), launcher.getVelocity());
        launcher.setPower(launcherController.calculate(currentLauncherKineticState));

        telemetry.addLine("-------- PATHING --------");
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Automated Drive", automatedDrive);

        telemetry.addLine("-------- LAUNCHER --------");
        telemetry.addData("Launch Range", launcherRange);
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity", launcher.getVelocity());

        telemetry.addLine("-------- VARIABLES --------");
        telemetry.addData("Current Alliance", teleopColorAlliance);
        telemetry.addData("Starting Position", teleopStartingPlace);

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if(Objects.equals(launcherRange, "CLOSE")) {
                    launcherController.setGoal(closeTargetLauncherKineticState);
                    if (launcher.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else if(Objects.equals(launcherRange, "FAR")) {
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
                    launchState = LaunchState.IDLE;
                    launcherServo.setPosition(launcherServoDown);
                }
                break;
        }
    }
}
