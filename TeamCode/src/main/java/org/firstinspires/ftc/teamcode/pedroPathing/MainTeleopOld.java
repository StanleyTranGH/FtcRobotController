package org.firstinspires.ftc.teamcode.pedroPathing;

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
public class MainTeleopOld extends OpMode {
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
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.006, 0, 0.0006); // DONE: GET VALUES
    public static double launcherFF = 0.0003;
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
    final double launcherServoUp = 0.17; // DONE: SET THESE VALUES TO PROPER SERVO POSITION
    final double sorterServoOpenLeft = 0.70; //DONE: SET THIS VALUE TO OPEN THE LEFT SIDE
    final double sorterServoOpenRight = 0.3; //DONE: SET THIS VALUE TO OPEN THE RIGHT SIDE
    final double closeGateServo = 0.5; // DONE: GET GATE SERVO VALUES
    final double openGateServo = 0.64;
    String launcherRange = "CLOSE"; // CLOSE or FAR
    final double CLOSE_LAUNCHER_TARGET_VELOCITY = 1780; // TODO: FIND DESIRED LAUNCHER VELOCITY
    final double FAR_LAUNCHER_TARGET_VELOCITY = 2240; // TODO: FINE DESIRED FAR LAUNCHER VELOCITY
    double MIN_VELOCITY = 0;
    double TARGET_VELOCITY = 0;
    double MAX_VELOCITY = 0;
    final double turretRestPosition = 0.5;
    double turretTargetPosition = 0.5;
    final double hoodMinPosition = 0;
    final double hoodMaxPosition = 0.56; // DONE: GET THE REAL HOOD MAX POSITION
    double hoodTargetPosition = 0;
    double robotDistanceFromGoal = 0;
    boolean shootingMode = false;
    int pedroMode = 1; // 0: Pedro + LL Tracking 1: Pedro Tracking 2: Limelight Tracking 3: No Tracking
    String pedroModeText = "Pedro + LL";
    static double goalXRed = 142;
    final double STOP_SPEED = 0.0;
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState closeTargetLauncherKineticState = new KineticState(0, CLOSE_LAUNCHER_TARGET_VELOCITY);
    KineticState farTargetLauncherKineticState = new KineticState(0, FAR_LAUNCHER_TARGET_VELOCITY);
    KineticState currentLauncherKineticState = new KineticState(0, 0);
    KineticState targetLauncherKineticState = stopLauncherKineticState;

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
        scorePose = new Pose(100, 100, Math.toRadians(180));

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
        gateServo.setPosition(closeGateServo);
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
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPower(1);
        } else if (gamepad1.yWasPressed()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        /* TODO: REMOVE IF THE LINEAR VELOCITY SCALING WORKS
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
       */

        TARGET_VELOCITY = calculateVelocity(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        MIN_VELOCITY = TARGET_VELOCITY - 50;
        MAX_VELOCITY = TARGET_VELOCITY + 50;
        targetLauncherKineticState = new KineticState(0, TARGET_VELOCITY);

        if (gamepad2.yWasPressed()) {
            shootingMode = true;
            launcherController.setGoal(targetLauncherKineticState);
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            shootingMode = false;
            launcherController.setGoal(stopLauncherKineticState);
        }

        if(gamepad2.dpadDownWasPressed()) {
            gateServo.setPosition(closeGateServo);
        } else if(gamepad2.dpadUpWasPressed()) {
            gateServo.setPosition(openGateServo);
        }



        // Turret & Hood

        if(gamepad2.startWasPressed()) {
            pedroMode++;
            if (pedroMode > 3) {
                pedroMode = 1;
            }
            if(pedroMode == 0) {
                pedroModeText = "Pedro + LL Tracking";
            } else if(pedroMode == 1) {
                pedroModeText = "Only Pedro";
            } else if (pedroMode == 2) {
                pedroModeText = "Only Limelight";
            } else if (pedroMode == 3) {
                pedroModeText = "No Tracking";
            }
        }

        if(shootingMode) {
            if(pedroMode == 0) { // Combined Tracking

                turretTargetPosition = calculateTurretPositionPedroLL(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));
                hoodTargetPosition = calculateHoodPositionPedroLL(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));

            } else if(pedroMode == 1) { // Pedro Tracking

                turretTargetPosition = calculateTurretPositionPedro(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()));
                hoodTargetPosition = calculateHoodPositionPedro(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

            } else if (pedroMode == 2) { // Limelight Tracking
                turretTargetPosition = calculateTurretPositionLimelight();
                // hoodTargetPosition = calculateHoodPositionLimelight(); TODO: COMPLETE HOOD POSITION LIMELIGHT AND UNCOMMENT THIS OUT
                hoodTargetPosition = calculateHoodPositionPedro(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            } else { // Default Settings
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

        double launcherPower = launcherController.calculate(currentLauncherKineticState);
        launcher1.setPower(launcherPower);
        launcher2.setPower(launcherPower);

        robotDistanceFromGoal = calculateRobotDistanceFromGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

        launch(gamepad2.rightBumperWasPressed());

        telemetry.addLine("-------- PATHING --------");
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Automated Drive On?", automatedDrive);
        telemetry.addData("Distance", robotDistanceFromGoal);

        telemetry.addLine("-------- LAUNCHER --------");
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Launcher Velocity", launcher1.getVelocity());
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Launcher Power", launcher1.getPower());

        telemetry.addLine("-------- VARIABLES --------");
        telemetry.addData("Current Alliance", teleopColorAlliance);
        telemetry.addData("Starting Position", teleopStartingPlace);

        telemetry.addLine("-------- TURRET --------");
        telemetry.addData("Turret Target Position", turretTargetPosition);
        telemetry.addData("Hood Servo Position", hoodTargetPosition);
        telemetry.addData("Tracking Mode", pedroModeText);
    }
    double calculateRobotDistanceFromGoal(double currentX, double currentY, double headingDeg) {
        // Goal coordinates
        double goalX = (teleopColorAlliance == 1) ? 142 : 2;
        double goalY = 142;

        // Shooter offset relative to robot center
        double shooterOffset = -2.0; // 2 inches BEHIND robot center (negative = behind)

        telemetry.addData("Heading Deg", headingDeg);
        // Convert heading to radians
        double headingRad = headingDeg;

        // Compute shooter position using rotation
        double shooterX = currentX + shooterOffset * Math.cos(headingRad);
        double shooterY = currentY + shooterOffset * Math.sin(headingRad);

        // Compute distance from shooter to goal
        double dx = shooterX - goalX;
        double dy = shooterY - goalY;

        return Math.sqrt(dx * dx + dy * dy);
    }

    double calculateVelocity (double currentX, double currentY, double headingDeg) {
        double goalDistance = calculateRobotDistanceFromGoal(currentX, currentY, headingDeg);

        double targetVelocity = 8.41935 * goalDistance + 1103.3871;

        return Range.clip(targetVelocity, 0, 2420);
    }
    double calculateTurretPositionPedroLL (double currentX, double currentY, double robotHeadingDeg) {

        double turretMin = 0.06;
        double turretMax = 0.94;
        double targetPosition = calculateTurretPositionLimelight(); // Default position

        if(targetPosition == -1) {
            targetPosition = calculateTurretPositionPedro(currentX, currentY, robotHeadingDeg);
        }

        return Range.clip(targetPosition, turretMin, turretMax);

    }
    double calculateHoodPositionPedroLL (double currentX, double currentY, double robotHeadingDeg) {

        double hoodMin = 0.00;
        double hoodMax = 0.56;
        double targetPosition = calculateHoodPositionLimelight(); // Default position

        if(targetPosition == -1) {
            targetPosition = calculateHoodPositionPedro(currentX, currentY, robotHeadingDeg);
        }

        return Range.clip(targetPosition, hoodMin, hoodMax);
    }
    double wrapTo180(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
    double calculateTurretPositionPedro(double currentX, double currentY, double robotHeadingDeg) {

        // DONE: FIND BEST GOAL COORDINATES
        double goalX = (teleopColorAlliance == 1) ? goalXRed : 2;
        double goalY = 142;

        double shooterOffset = -2.5;
        double headingRad = Math.toRadians(robotHeadingDeg);

        double shooterX = currentX - shooterOffset * Math.cos(headingRad);
        double shooterY = currentY - shooterOffset * Math.sin(headingRad);

        // Angle to goal
        double dx = goalX - shooterX;
        double dy = goalY - shooterY;
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
    double calculateTurretPositionLimelight() {
        // TODO: FINISH LIMELIGHT TRACKING CODE
        double detectedID = 0;
        double visionAngle;
        double defaultValue = -1;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }
        }

        if ((detectedID == 20 && teleopColorAlliance == 2) || (detectedID == 24 && teleopColorAlliance == 1)) {
            visionAngle = result.getTx();
            double relativeAngleDeg = -visionAngle;

            double servoMin = 0.06;
            double servoMid = 0.50;
            double servoMax = 0.94;

            double servoRange = servoMax - servoMin;

            double servoPos = servoMid + (relativeAngleDeg / 180.0) * (servoRange / 2.0);
            telemetry.addData("Ta", result.getTa());

            return Range.clip(servoPos, servoMin, servoMax);
        } else {
            return -1;
        }
    } 

    double calculateHoodPositionPedro(double currentX, double currentY, double headingDeg) {

        double targetDistance = calculateRobotDistanceFromGoal(currentX, currentY, headingDeg);

        // Scale distance to the proper
       // double servoPosition = -5.339981 + 0.3090759*targetDistance - 0.00610722*Math.pow(targetDistance, 2) + 0.00004666887*Math.pow(targetDistance, 3) - 1.405935 * Math.pow(10, -8) *Math.pow(targetDistance, 4) - 9.799439 * Math.pow(10, -10) * Math.pow(targetDistance, 5);
        double servoPosition = 0.002 * targetDistance - 0.09;
        // Relative angle (-180 to 180)

        return Range.clip(servoPosition, 0.00, 0.56);
    }
    double calculateHoodPositionLimelight() {
        double detectedID = 0;
        double hoodMin = 0.00;
        double hoodMax = 0.56;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Valid target detected
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                detectedID = fiducial.getFiducialId();

            }
        }

        if ((detectedID == 20 && teleopColorAlliance == 2) || (detectedID == 24 && teleopColorAlliance == 1)) {

            double ta = result.getTa();
            double targetDistance = 1 * ta + 1; // TODO: Regression the equation

            double servoPosition = 0.002 * targetDistance - 0.09;
            // Relative angle (-180 to 180)

            return Range.clip(servoPosition, hoodMin, hoodMax);

        } else {
            return -1;
        }

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
                    if (launcher1.getVelocity() > MIN_VELOCITY && launcher1.getVelocity() < MAX_VELOCITY) {
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
