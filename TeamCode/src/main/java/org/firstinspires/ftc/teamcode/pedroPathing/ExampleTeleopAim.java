package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.ExampleAuto.colorAlliance;

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

@Configurable
@TeleOp(name = "Example Teleop Aim", group = "Examples")
public class ExampleTeleopAim extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(86, 50, Math.toRadians(270)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Limelight fields
    private Limelight3A limelight;
    private int aimTagID;
    private PIDFController aimController = new PIDFController(0.03, 0.0, 0.002, 0.0); // kP, kI, kD, kF

    // Mechanisms
    DcMotor intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    ElapsedTime feederTimer = new ElapsedTime();

    final double launcherServoDown = 0.16;
    final double launcherServoUp = 0.45; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double LAUNCHER_TARGET_VELOCITY = 1400; // DONE: FIND DESIRED LAUNDER VELOCITY
    final double LAUNCHER_MIN_VELOCITY = 1350;
    final double STOP_SPEED = 0.0;

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

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        launcherServo.setPosition(launcherServoDown);

        // Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        if(colorAlliance == 1) {
            // TODO: CHANGE BACK TO colorAlliance == 2 FOR COMPETITION
            // Set lazy curve to blue alliance base if on team blue
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(104, 33))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
        } else {
            // Set lazy curve to red alliance by default
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(39, 33))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
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
        follower.update();
        telemetryM.update();

        double turn;

        // Driving
        if (!automatedDrive) {
            // Check for auto-aim9
            if (gamepad1.left_trigger > 0.5) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        aimTagID = fiducial.getFiducialId();
                    }
                    if(aimTagID == 24 && colorAlliance == 1 || aimTagID == 20 && colorAlliance == 2 ) {
                        // Making sure we're tracking the appropriate alliance april tag
                        double tx = result.getTx(); // horizontal offset

                        // Deadband to prevent jitter
                        if (Math.abs(tx) < 1.0) {
                            tx = 0;
                            aimController.reset();
                        }

                        turn = aimController.calculate(tx, 0); // target setpoint = 0 (centered tag)
                    } else {
                        turn = -gamepad1.right_stick_x;
                    }
                } else {
                    turn = -gamepad1.right_stick_x;
                }
            } else {
                turn = -gamepad1.right_stick_x;
            }

            // Normal or slow mode XY control
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    turn,
                    false // Field Centric
            );
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    turn,
                    false // Field Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode toggle
        if (gamepad1.rightBumperWasPressed()) {
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
        if (gamepad2.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        launch(gamepad2.rightBumperWasPressed());

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
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
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


