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
    private final double slowModeMultiplier = 0.5;

    // Collects the starting place from either the Nine Ball or Twelve Ball autos, depending on which is used
    private int teleopStartingPlace = 0;
    private int teleopColorAlliance = 0;
    private boolean robotCentricOn = false;

    // Limelight fields
    private Limelight3A limelight;
    private int aimTagID;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.0005, 0, 0.00005); // TODO: GET VALUES
    public static double launcherFF = 0.000359;
    ControlSystem launcherController;

    // Mechanisms
    DcMotor intakeMotor;
    DcMotorEx launcher1;
    DcMotorEx launcher2;
    Servo launcherServo;
    Servo sorterServo;
    Servo gateServo;
    Servo turretServo;
    Servo hoodServo;
    ElapsedTime feederTimer = new ElapsedTime();

    final double launcherServoDown = 0.4;
    final double launcherServoUp = 0.15; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double sorterServoOpenLeft = 0.67; //TODO: SET THIS VALUE TO OPEN THE LEFT SIDE
    final double sorterServoOpenRight = 0.36; //TODO: SET THIS VALUE TO OPEN THE RIGHT SIDE
    final double closeGateServo = 0.5; // TODO: GET GATE SERVO VALUES
    final double openLeftSideGateServo = 0.7;
    final double openRightSideGateServo = 0.3;
    String launcherRange = "CLOSE"; // CLOSE or FAR
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1640; // TODO: FIND DESIRED LAUNCHER VELOCITY
    final double CLOSE_LAUNCHER_MIN_VELOCITY = 1600;
    final double CLOSE_LAUNCHER_MAX_VELOCITY = 1660;
    final double FAR_LAUNCHER_TARGET_VELOCITY = 1800; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    final double FAR_LAUNCHER_MIN_VELOCITY = 1760;
    final double FAR_LAUNCHER_MAX_VELOCITY = 1820;
    final double turretRestPosition = 0.5;
    double turretTargetPosition = 0.5;
    final double hoodMinPosition = 0;
    final double hoodMaxPosition = 0.56; // DONE: GET THE REAL HOOD MAX POSITION
    double hoodTargetPosition = 0;
    double robotDistanceFromGoal = 0;
    boolean shootingMode = false;
    int pedroMode = 1; // 1: Pedro + LL Tracking 2: Pedro Tracking 3: Limelight Tracking 4: No Tracking
    String pedroModeText = "Pedro + LL";
    final double STOP_SPEED = 0.0;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);

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
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");


        follower = Constants.createFollower(hardwareMap);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .basicFF(launcherFF)
                .build();

        launcher1.setPower(STOP_SPEED);
        launcher2.setPower(STOP_SPEED);

        // Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        teleopColorAlliance = autoToTeleop.colorAlliance;
        teleopStartingPlace = autoToTeleop.startingPosition;

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
        startingPose = autoToTeleop.endAuto == null ?  new Pose(72, 72, 0) : autoToTeleop.endAuto;
        follower.setStartingPose(startingPose);
        follower.startTeleopDrive(true);

        /* Start all of the servos once the START button is pressed, not the INIT button */

        turretServo.setPosition(turretRestPosition);
        launcherServo.setPosition(launcherServoDown);
        launcherController.setGoal(stopLauncherKineticState);
        sorterServo.setPosition(sorterServoOpenRight);
        gateServo.setPosition(openRightSideGateServo);
        hoodServo.setPosition(hoodMinPosition);

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
            shootingMode = true;
            launcherRange = "CLOSE";
            launcherController.setGoal(closeTargetLauncherKineticState);
            scorePose = new Pose(87.5, 91, Math.toRadians(-141));
        } else if (gamepad2.xWasPressed()) {
            shootingMode = true;
            launcherRange = "FAR";
            launcherController.setGoal(farTargetLauncherKineticState);
            scorePose = new Pose(92.8, 13.6, Math.toRadians(-110.3)); // Scoring Pose of our robot.
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            shootingMode = false;
            launcherController.setGoal(stopLauncherKineticState);
        }

        /* TODO: ADD THESE COMMANDS BACK LATER AFTER HOOD IS FIXED
        if(gamepad2.dpadLeftWasPressed()) {
            gateServo.setPosition(openLeftSideGateServo);
        } else if(gamepad2.dpadRightWasPressed()) {
            gateServo.setPosition(openRightSideGateServo);
        } else if(gamepad2.dpadDownWasPressed()) {
            gateServo.setPosition(closeGateServo);
        }
         */

        if(gamepad2.dpadDownWasPressed()) {
            hoodTargetPosition -= 0.01;
        } else if(gamepad2.dpadUpWasPressed()) {
            hoodTargetPosition += 0.01;
        }

        launch(gamepad2.rightBumperWasPressed());

        // Turret & Hood

        if(gamepad2.startWasPressed()) {
            pedroMode++;
            if (pedroMode > 4) {
                pedroMode = 1;
            }
            if(pedroMode == 1) {
                pedroModeText = "Pedro + LL";
            } else if (pedroMode == 2) {
                pedroModeText = "Only Pedro";
            } else if (pedroMode == 3) {
                pedroModeText = "Only Limelight";
            } else if (pedroMode == 4) {
                pedroModeText = "No Tracking";
            }
        }

        if(shootingMode) {
            if(pedroMode == 1) {

                turretTargetPosition = calculateTurretPositionPedroLL(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));
                hoodTargetPosition = calculateHoodPositionPedro(follower.getPose().getX(), follower.getPose().getY());

            } else if(pedroMode == 2) { // Pedro Tracking

                turretTargetPosition = calculateTurretPositionPedro(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));
                // hoodTargetPosition = calculateHoodPositionPedro(follower.getPose().getX(), follower.getPose().getY());

            } else if (pedroMode == 3) { // Limelight Tracking
                turretTargetPosition = calculateTurretPositionLimelight();
                // hoodTargetPosition = 0;
            } else {
                turretTargetPosition = turretRestPosition;
                hoodTargetPosition = hoodMinPosition;
            }
            hoodServo.setPosition(hoodTargetPosition);
            turretServo.setPosition(turretTargetPosition);
        } else {
            hoodServo.setPosition(hoodMinPosition);
            turretServo.setPosition(turretRestPosition);
        }

        currentLauncherKineticState = new KineticState(launcher1.getCurrentPosition(), launcher1.getVelocity());
        launcher1.setPower(launcherController.calculate(currentLauncherKineticState));
        launcher2.setPower(launcherController.calculate(currentLauncherKineticState));

        telemetry.addLine("-------- PATHING --------");
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Automated Drive On?", automatedDrive);
        telemetry.addData("Distance", robotDistanceFromGoal);

        telemetry.addLine("-------- LAUNCHER --------");
        telemetry.addData("Launch Range", launcherRange);
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity", launcher1.getVelocity());
        telemetry.addData("Launcher Power", launcher1.getPower());

        telemetry.addLine("-------- VARIABLES --------");
        telemetry.addData("Current Alliance", teleopColorAlliance);
        telemetry.addData("Starting Position", teleopStartingPlace);

        telemetry.addLine("-------- TURRET --------");
        telemetry.addData("Turret Current Position", turretServo.getPosition());
        telemetry.addData("Hood Servo Position", hoodTargetPosition);
        telemetry.addData("Tracking Mode", pedroModeText);
    }

    double calculateTurretPositionPedroLL (double currentX, double currentY, double robotHeadingDeg) {

        double targetPosition = 0.5; // Default position
        LLResult result = limelight.getLatestResult();
        double detectedID = 0;

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }

        }

        if((detectedID == 24 && teleopColorAlliance == 1) || (detectedID == 20 && teleopColorAlliance == 2)) {

            targetPosition = calculateTurretPositionLimelight();

        } else {
            targetPosition = calculateTurretPositionPedro(currentX, currentY, robotHeadingDeg);
        }

        return targetPosition;
    }
    double calculateHoodPositionPedroLL (double currentX, double currentY, double robotHeadingDeg) {

        double targetPosition = 0.5; // Default position
        LLResult result = limelight.getLatestResult();
        double detectedID = 0;

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }

        }

        if((detectedID == 24 && teleopColorAlliance == 1) || (detectedID == 20 && teleopColorAlliance == 2)) {

            targetPosition = calculateHoodPositionPedro(currentX, currentY);

        } else {

            targetPosition = calculateHoodPositionPedro(currentX, currentY);

        }

        return targetPosition;
    }
    double calculateTurretPositionPedro(double currentX, double currentY, double robotHeadingDeg) {

        // TODO: FIND BEST GOAL COORDINATES
        double goalX = (teleopColorAlliance == 1) ? 133 : 11;
        double goalY = 137;

        // Angle to goal
        double dx = goalX - currentX;
        double dy = goalY - currentY;
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Relative angle (-180 to 180)
        double relativeAngle = targetAngleDeg - robotHeadingDeg;
        relativeAngle = wrapTo180(relativeAngle);
        relativeAngle *= -1;

        // Convert angle to servo position
        //  -180° -> 0.06
        //   0°   -> 0.50
        //  +180° -> 0.94
        double servoMin = 0.06;
        double servoMid = 0.50;
        double servoMax = 0.94;

        double servoRange = servoMax - servoMin;

        double servoPos = servoMid + (relativeAngle / 180.0) * (servoRange / 2.0);

        return Range.clip(servoPos, servoMin, servoMax);
    }

    double wrapTo180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    double calculateTurretPositionLimelight() {
        // TODO: FINISH LIMELIGHT TRACKING CODE
        double targetPosition = 0.5; // default position
        double detectedID = 0;
        LLResult result = limelight.getLatestResult();
        double tx = result.getTx(); // max is roughly 27.25 degrees, also i got NO IDEA which direction tx goes so... i'm just gonna convert to servo rotations and pray, no wrap cause it shouldn't be more than 180
        double servoGearReduction = 4.0;

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }
        }

        if (detectedID == 20 && teleopColorAlliance == 2 || detectedID == 24 && teleopColorAlliance == 1) {
            double servoDegrees = tx * servoGearReduction;
            double servoPositionChange = servoDegrees / 1800.0;
            targetPosition = 0.5 + servoPositionChange;
        }

        return Range.clip(targetPosition, 0.0, 1.0);
    }

    double calculateHoodPositionPedro(double currentX, double currentY) {
        // TODO: FIND BEST GOAL COORDINATES
        double goalX = (teleopColorAlliance == 1) ? 133 : 11;
        double goalY = 137;

        // Distance From Goal (just pythagorean theorem)
        double dx = goalX - currentX;
        double dy = goalY - currentY;
        double targetDistance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        // Scale distance to the proper
        double angle = Math.toRadians(Math.atan(2300+Math.sqrt(Math.pow(2300, 2)-(2)(9.8)(45)(2300) - (Math.pow(9.8, 2)(Math.pow(targetDistance, 2)))) / (9.8)(targetDistance))))+40;
        // Relative angle (-180 to 180)

        // Convert angle to servo position
        double servoPosition = (angle-25) * 0.0224;

        return Range.clip(servoPosition);
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
                    if (launcher1.getVelocity() > CLOSE_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < CLOSE_LAUNCHER_MAX_VELOCITY) {
                        launchState = LaunchState.LAUNCH;
                    }
                } else if(Objects.equals(launcherRange, "FAR")) {
                    launcherController.setGoal(farTargetLauncherKineticState);
                    if (launcher1.getVelocity() > FAR_LAUNCHER_MIN_VELOCITY && launcher1.getVelocity() < FAR_LAUNCHER_MAX_VELOCITY) {
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
