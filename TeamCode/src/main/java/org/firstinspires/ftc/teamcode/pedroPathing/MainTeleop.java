package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private boolean robotCentricOn = false;

    // Limelight fields
    private Limelight3A limelight;
    private int aimTagID;
    // 0.019, 0.0006, 0.0006
    // 0.018, 0.0007, 0.0006
    // 0.018, 0.0006, 0.0007
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.015, 0, 0.0015);
    ControlSystem launcherController;

    // Mechanisms
    DcMotor intakeMotor;
    DcMotorEx launcher;
    Servo launcherServo;
    Servo sorterServo;
    Servo leftGateServo;
    Servo turretServo;
    ElapsedTime feederTimer = new ElapsedTime();

    final double launcherServoDown = 0.18;
    final double launcherServoUp = 0.49; // TODO: SET THESE VALUES TO PROPER SERVO POSITION
    final double sorterServoOpenLeft = 0.67; //DONE: SET THIS VALUE TO OPEN THE LEFT SIDE
    final double sorterServoOpenRight = 0.36; //DONE: SET THIS VALUE TO OPEN THE RIGHT SIDE
    final double closeLeftGateServo = 0.73; // DONE: GET THE GATE CLOSE VALUE
    final double openLeftGateServo = 0.44; //DONE: GET THE GATE OPEN VALUE
    String launcherRange = "CLOSE"; // CLOSE or FAR
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1640; // DONE: FIND DESIRED LAUNCHER VELOCITY
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1600;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1660;
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1800; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    final double FAR_LAUNCHER_MIN_VELOCITY = 1760;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1820;
    final double turretRestPosition = 0.5;
    double turretTargetPosition = 0.5;
    boolean shootingMode = false;
    boolean pedroMode = true; // True: Pedro Tracking False: Limelight Tracking
    final double STOP_SPEED = 0.0;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);;

    final double MAX_FEED_TIME = 0.35;
    final double MAX_SCAN_TIME = 2.0;
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
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");


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
        sorterServo.setPosition(sorterServoOpenRight);
        leftGateServo.setPosition(closeLeftGateServo);

        // Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        if(NineBallAuto.startingPlace == 1 || NineBallAuto.startingPlace == 2) {
            teleopStartingPlace = NineBallAuto.startingPlace;
        } else if (TwelveBallAutoB.startingPlace == 1 || TwelveBallAutoB.startingPlace == 2) {
            teleopStartingPlace = TwelveBallAutoB.startingPlace;
        }

        if(NineBallAuto.colorAlliance == 1 || NineBallAuto.colorAlliance == 2) {
            teleopColorAlliance = NineBallAuto.colorAlliance;
        } else if (TwelveBallAutoB.startingPlace == 1 || TwelveBallAutoB.startingPlace == 2) {
            teleopColorAlliance = TwelveBallAutoB.colorAlliance;
        }

        // DONE: SWAP PARK POSES FOR REAL COMPETITION (mirror this one and un-mirror other one)
        // TODO: FIND REAL PARK & SCORE POSE
        parkPose = new Pose(108.4, 33.8, Math.toRadians(90));
        scorePose = new Pose(92, 88, Math.toRadians(225));

        if(teleopStartingPlace == 1) {
            // startingPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
            startingPose = new Pose(86, 50, Math.toRadians(270));
        } else if (teleopStartingPlace == 2) {
            startingPose = new Pose(122, 95, Math.toRadians(270)); // Park Pose of our robot. // Park Pose of our robot.; //See ExampleAuto to understand how to use this
        }



        if(teleopColorAlliance == 1) {
            // Set lazy curve to red alliance base if on team red
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.getHeading(), 0.8))
                    .build();

        } else {
            // Set lazy curve to blue alliance by default
            parkChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, parkPose)))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                    .build();
            shootChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, scorePose.mirror())))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, scorePose.mirror().getHeading(), 0.8))
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
        scorePose = new Pose(87.5, 91, Math.toRadians(-141));

        if(teleopStartingPlace == 1) {
            // startingPose = new Pose(86, 50, Math.toRadians(90)); // Park Pose of our robot.; //See ExampleAuto to understand how to use this
            startingPose = new Pose(86, 50, Math.toRadians(270));
        } else if (teleopStartingPlace == 2) {
            startingPose = new Pose(122, 95, Math.toRadians(270)); // Park Pose of our robot. // Park Pose of our robot.; //See ExampleAuto to understand how to use this
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

        telemetry.addLine("------------------------------");

        telemetry.addLine("Press START if robot centric mode needs to be turned on");
        if(gamepad1.startWasPressed()) {
            robotCentricOn = !robotCentricOn;
        }
        telemetry.addData("Is Robot Centric On", robotCentricOn);
    }

    @Override
    public void start() {
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive(true);

        turretServo.setPosition(turretRestPosition);
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
                            robotCentricOn // Field Centric
                    );
                } else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x,
                            robotCentricOn, // Field Centric
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
                            robotCentricOn // Field Centric
                    );
                } else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * slowModeMultiplier,
                            -gamepad1.left_stick_x * slowModeMultiplier,
                            -gamepad1.right_stick_x * slowModeMultiplier,
                            robotCentricOn, // Field Centric
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

        if(gamepad1.startWasPressed()) {
            robotCentricOn = !robotCentricOn;
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

        if(gamepad1.dpadLeftWasPressed()) {
            sorterServo.setPosition(sorterServoOpenRight);
        } else if(gamepad1.dpadRightWasPressed()) {
            sorterServo.setPosition(sorterServoOpenLeft);
        }

        // Launching
        if (gamepad2.yWasPressed()) {
            launcherRange = "CLOSE";
            launcherController.setGoal(closeTargetLauncherKineticState);
            scorePose = new Pose(87.5, 91, Math.toRadians(-141));
        } else if (gamepad2.xWasPressed()) {
            launcherRange = "FAR";
            launcherController.setGoal(farTargetLauncherKineticState);
            scorePose = new Pose(92.8, 13.6, Math.toRadians(-110.3)); // Scoring Pose of our robot.
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            launcherController.setGoal(stopLauncherKineticState);
        }

        if(gamepad2.dpadUpWasPressed()) {
            leftGateServo.setPosition(openLeftGateServo);
        } else if(gamepad2.dpadDownWasPressed()) {
            leftGateServo.setPosition(closeLeftGateServo);
        }

        launch(gamepad2.rightBumperWasPressed());

        // Turret

        if(gamepad2.startWasPressed()) {
            pedroMode = !pedroMode;
        }

        if(shootingMode) {
            if(pedroMode) { // Pedro Tracking
                turretTargetPosition = calculateTurretPositionPedro(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));
            } else { // Limelight Tracking
                turretTargetPosition = calculateTurretPositionLimelight();
            }
            turretServo.setPosition(turretTargetPosition);
        } else {
            turretServo.setPosition(turretRestPosition);
        }

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
    double calculateTurretPositionPedro(double currentX, double currentY, double robotHeading) {

        double goalX;
        double goalY;

        if(teleopColorAlliance == 1) { // Red
            goalX = 133; // TODO: GET GOAL X AND Y VALUES
            goalY = 137;
        } else {
            goalX = 11; // TODO: GET GOAL X AND Y VALUES
            goalY = 137;
        }

        double dx = Math.abs(goalX - currentX); // X and Y offsets from the goal
        double dy = Math.abs(goalY - currentY);

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx)); // Tangent to calculate angle
        double relativeAngle = targetAngle - robotHeading; // Angle relative to where the robot is facing
        relativeAngle = wrapDeg(relativeAngle); // Wrap it within the -180 to 180 degrees

        double servoGearReduction = 4.0;
        double servoDegrees = relativeAngle * servoGearReduction; // Degrees the servo needs to turn
        double servoPositionChange = servoDegrees / 1800.0; // Reduce it within the 5 turn servo
        double targetPosition = 0.5 + servoPositionChange; // Add 0.5 because 0.5 is starting position

        return Range.clip(targetPosition, 0.0, 1.0); // Clip it in case of an error
    }
    double calculateTurretPositionLimelight() {
        double hi = 0;
        double detectedID = 0;
        double tx = 0;
        double ty = 0;
        double ta = 0;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }
        }

        if (detectedID == 20 && teleopColorAlliance == 2 || detectedID == 24 && teleopColorAlliance == 1) {

        }

        return hi;
    }

    double wrapDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
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
