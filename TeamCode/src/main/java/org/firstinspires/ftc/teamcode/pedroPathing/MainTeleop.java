package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.ExampleAuto.colorAlliance;

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
    public static Pose startingPose = new Pose(88, 8, Math.toRadians(90));
    // public static Pose startingPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    public static Pose parkPose;
    public static Pose scorePose;
    private Supplier<PathChain> parkChain, shootChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Limelight fields
    private Limelight3A limelight;
    private int aimTagID;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.015, 0, 0.001);
    ControlSystem launcherController;

    // Mechanisms
    DcMotor intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    ElapsedTime feederTimer = new ElapsedTime();

    final double launcherServoDown = 0.10;
    final double launcherServoUp = 0.45; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double LAUNCHER_TARGET_VELOCITY = 1500; // DONE: FIND DESIRED LAUNDER VELOCITY
    final double LAUNCHER_MIN_VELOCITY = 1440;
    final double LAUNCHER_MAX_VELOCITY = 1560;
    final double STOP_SPEED = 0.0;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState targetLauncherKineticState = new KineticState(0, LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);;

    final double MAX_FEED_TIME = 0.3;
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
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
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

        // TODO: SWAP PARK POSES FOR REAL COMPETITION (mirror this one and un-mirror other one)
        // TODO: FIND REAL PARK & SCORE POSE
        parkPose = new Pose(108, 33, Math.toRadians(90));
        scorePose = new Pose(90, 118, Math.toRadians(50));

        if(colorAlliance == 1) {
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
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
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
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Field Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Field Centric
            );
        }

        if (gamepad1.aWasPressed()) {
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
        } else if (gamepad1.bWasPressed()) {
            intakeMotor.setPower(0);
        }

        // Launching
        if (gamepad2.yWasPressed()) {
            launcherController.setGoal(targetLauncherKineticState);
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            launcherController.setGoal(stopLauncherKineticState);
        }

        launch(gamepad2.rightBumperWasPressed());

        currentLauncherKineticState = new KineticState(launcher.getCurrentPosition(), launcher.getVelocity());
        launcher.setPower(launcherController.calculate(currentLauncherKineticState));

        telemetry.addData("Current Alliance", colorAlliance);
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Automated Drive", automatedDrive);
        telemetry.addData("gamepad2 right", gamepad2.rightBumperWasPressed());
        telemetry.addData("launch state", launchState);
        telemetry.addData("launcher velocity", launcher.getVelocity());

    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcherController.setGoal(targetLauncherKineticState);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY && launcher.getVelocity() < LAUNCHER_MAX_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
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
