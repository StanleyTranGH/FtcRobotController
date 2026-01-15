package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.function.Supplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
@TeleOp(name = "New Main Teleop", group = "Official")
public class MainTeleopNew extends OpMode {

    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> parkChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Follower Poses
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    public static Pose blueParkPose = new Pose(108.4, 33.8, Math.toRadians(90));
    public static Pose redParkPose = blueParkPose.mirror();
    public Pose parkPose;
    public static Pose blueShootPose = new Pose(100, 100, Math.toRadians(0));
    public static Pose redShootPose = blueShootPose.mirror();
    public Pose shootPose;

    /* Motors & Servos */
    DcMotor intakeMotor;
    DcMotorEx launcher1;
    DcMotorEx launcher2;
    DcMotorEx turretMotor;
    Servo shooterGateServo;
    Servo sorterServo;
    Servo gateServo;
    Servo liftServoLeft;
    Servo liftServoRight;
    Servo hoodServo;
    Limelight3A limelight;

    /* General Variables */
    private final double STOP_SPEED = 0.0;
    private int teleopColorAlliance = 0;
    private int teleopStartingPlace = 0;
    private boolean robotCentricOn = false;
    double distanceFromGoal;
    final double intakePower = 1.0;
    final double transferPower = 1.0; // DONE: TEST BEST POWER FOR TRANSFER
    final double restPower = 0.0;
    double tempHoodTarget = 0.0;
    static double redGoalX = 142;

    /* Servo Positions */
    final double sorterServoOpenRight = 0.6; // DONE: SET SORTER SERVO POSITIONS
    final double sorterServoOpenLeft = 0.3;
    final double closeGateServo = 0.85; // DONE: SET GATE SERVO POSITIONS
    final double openGateServo = 1.0;
    final double closeShooterGateServo = 0.52; // DONE: SET SHOOTER GATE POSITIONS
    final double openShooterGateServo = 0.97;

    /* Turret Stuff */
    double turretTargetPosition = 0.0;
    double hoodTargetPosition = 0.0;
    final double turretRestPosition = 0.0;
    final double hoodRestPosition = 0.0;
    ControlSystem turretController;
    public static PIDCoefficients turretPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0002); //DONE: TUNE TURRET PID
    public static double turretFF = 0.0;
    KineticState turretRestKineticState = new KineticState(0);
    KineticState turretTargetKineticState = turretRestKineticState;
    KineticState turretCurrentKineticState = turretTargetKineticState;

    /* Launcher Stuff */
    ControlSystem launcherController;
    public static PIDCoefficients launcherPIDCoefficients = new PIDCoefficients(0.005, 0, 0.0005); // DONE: GET VALUES
    public static double launcherFF = 0.0003; // DONE: RE-TUNE LAUNCHER PIDF
    double MIN_VELOCITY, MAX_VELOCITY;
    double TARGET_VELOCITY = 1500; //TODO: REMOVE LATER AFTER TESTING
    KineticState stopLauncherKineticState = new KineticState(0, 0);
    KineticState targetLauncherKineticState = stopLauncherKineticState;
    KineticState currentLauncherKineticState = targetLauncherKineticState;
    private enum ShootState {
        INTAKING,
        REST_INTAKING,
        SHOOTING,
        LIFTING,
    }
    private ShootState shootState;
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
        shootState = ShootState.INTAKING;

        // Initialize Hardware
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        shooterGateServo = hardwareMap.get(Servo.class, "shooterGateServo");
        sorterServo = hardwareMap.get(Servo.class, "sorterServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Set Hardware Modes
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher1.setPower(STOP_SPEED);
        launcher2.setPower(STOP_SPEED);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(STOP_SPEED);

        // Initialize Launcher PID
        launcherController = ControlSystem.builder()
                .velPid(launcherPIDCoefficients)
                .basicFF(launcherFF)
                .build();

        turretController = ControlSystem.builder()
                .posPid(turretPIDCoefficients)
                .basicFF(turretFF)
                .build();

        // Carry over variables from autoToTeleop
        teleopColorAlliance = autoToTeleop.colorAlliance;
        teleopStartingPlace = autoToTeleop.startingPosition;

        // Initialize Follower & Starting Pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(autoToTeleop.endAuto);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Limelight Initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

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

        /* Determine Proper Parking & Block Shooting Poses */
        if (teleopColorAlliance == 1) {
            parkPose = redParkPose;
            shootPose = redShootPose;
        } else {
            parkPose = blueParkPose;
            shootPose = blueShootPose;
        }

        /* Start Teleop Drive */
        follower.startTeleopDrive();

        /* Set all Servo Start Positions */
        sorterServo.setPosition(sorterServoOpenRight);
        shooterGateServo.setPosition(closeShooterGateServo);
        gateServo.setPosition(closeGateServo);
    }
    @Override
    public void loop() {

        /* Pedro Updating */
        follower.update();
        telemetryM.update();

        /* Set Drive */
        if (!automatedDrive) {
            if (!slowMode) {
                /* Slow Mode Off */
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
            else {
                /* Slow Mode On */
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

        /* INTAKING CODE */
        if(shootState == ShootState.INTAKING || shootState == ShootState.REST_INTAKING) {
            if (gamepad1.xWasPressed()) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setPower(intakePower);
            } else if (gamepad1.yWasPressed()) {
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setPower(intakePower);
            } else if (gamepad1.yWasReleased() || gamepad1.aWasPressed()) {
                intakeMotor.setPower(restPower);
            }
        }

        /* SORTER CODE */
        if(gamepad1.dpadLeftWasPressed()) {
            sorterServo.setPosition(sorterServoOpenRight);
        } else if(gamepad1.dpadRightWasPressed()) {
            sorterServo.setPosition(sorterServoOpenLeft);
        }

        /* Set Shooter Velocity */
        // DONE: CHECK IF follower.getHeading() IS IN RADIANS OR DEGREES (its in radians)
        distanceFromGoal = calculateRobotDistanceFromGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        // TODO: REMOVE TEST CODE LATER
        // TARGET_VELOCITY = calculateVelocity(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        if(gamepad2.dpadUpWasPressed()) {
            TARGET_VELOCITY = TARGET_VELOCITY + 10;
        } else if(gamepad2.dpadDownWasPressed()) {
            TARGET_VELOCITY = TARGET_VELOCITY - 10;
        }
        MIN_VELOCITY = TARGET_VELOCITY - 50;
        MAX_VELOCITY = TARGET_VELOCITY + 50;
        targetLauncherKineticState = new KineticState(0, TARGET_VELOCITY);

        /* DRIVER 2 GAMEPAD SHOOTING CODE */
        if (gamepad2.yWasPressed()) {
            // DONE: IMPLEMENT SHOOT MODE
            shootState = ShootState.SHOOTING;
            launcherController.setGoal(targetLauncherKineticState);
        } else if (gamepad2.bWasPressed()) { // stop flywheel
            shootState = ShootState.INTAKING;
            launcherController.setGoal(targetLauncherKineticState);
        } else if (gamepad2.aWasPressed()) {
            shootState = ShootState.REST_INTAKING;
            launcherController.setGoal(stopLauncherKineticState);
        }

        /* DRIVER 2 SORTER CODE */
        if(gamepad2.dpadDownWasPressed()) {
            gateServo.setPosition(closeGateServo);
        } else if(gamepad2.dpadUpWasPressed()) {
            gateServo.setPosition(openGateServo);
        }

        // DONE: IMPLEMENT LIMELIGHT RE_LOCALIZING (DO LATER)
        if(shootState == ShootState.SHOOTING) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            turretTargetPosition = calculateTurretPosition(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
            hoodTargetPosition = calculateHoodPosition(follower.getPose().getX(), follower.getPose().getY());

            turretTargetKineticState = new KineticState(turretTargetPosition, 0);

            // TODO: REMOVE TEMPORARY TESTING CODE LATER
            if(gamepad2.dpadLeftWasPressed()) {
                tempHoodTarget = tempHoodTarget - 0.005;
            } if (gamepad2.dpadRightWasPressed()) {
                tempHoodTarget = tempHoodTarget + 0.005;
            }
            hoodServo.setPosition(tempHoodTarget);
            // hoodServo.setPosition(hoodTargetPosition);

            shooterGateServo.setPosition(openShooterGateServo);
        } else {
            // DONE: ALSO IMPLEMENT TURRET PID AND RUNNING TO POSITION CODE HERE
            turretTargetKineticState = turretRestKineticState;
            hoodServo.setPosition(hoodRestPosition);

            shooterGateServo.setPosition(closeShooterGateServo);
        }
        turretController.setGoal(turretTargetKineticState);
        turretCurrentKineticState = new KineticState(turretMotor.getCurrentPosition(), turretMotor.getVelocity());
        double turretPower = turretController.calculate(turretCurrentKineticState);
        turretMotor.setPower(
                turretController.calculate(
                        new KineticState(turretMotor.getCurrentPosition(), turretMotor.getVelocity())
                )
        );

        /* Set shooter power to calculated power */
        currentLauncherKineticState = new KineticState(launcher1.getCurrentPosition(), launcher1.getVelocity());
        double launcherPower = launcherController.calculate(currentLauncherKineticState);
        if(shootState == ShootState.SHOOTING || shootState == ShootState.INTAKING) {
            launcher1.setPower(launcherPower);
            launcher2.setPower(launcherPower);
        } else {
            launcher1.setPower(STOP_SPEED);
            launcher2.setPower(STOP_SPEED);
        }

        if(shootState == ShootState.SHOOTING) {
            if(gamepad2.right_bumper) {
                intakeMotor.setPower(transferPower);
            } else {
                intakeMotor.setPower(restPower);
            }
        }

        // Lift Mode
        /*
        if (gamepad1.bWasPressed()) {
            // DONE: Set to a different less accessible button
            // DONE: Implement lift-mode code here (DOING LATER)
            follower.followPath(parkChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
         */

        if(gamepad1.startWasPressed()) {
            robotCentricOn = !robotCentricOn;
        }

        telemetry.addLine("-------- PATHING --------");
        telemetry.addData("Position", follower.getPose());
        telemetry.addData("Automated Drive On?", automatedDrive);
        telemetry.addData("Distance", distanceFromGoal);

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
        telemetry.addData("Turret Current Position", turretMotor.getCurrentPosition());
        //        telemetry.addData("Hood Servo Position", hoodTargetPosition);
        telemetry.addData("Hood Servo Position", tempHoodTarget
        );
    }
    double calculateRobotDistanceFromGoal(double currentX, double currentY, double headingRad) {
        // Goal coordinates
        double goalX = (teleopColorAlliance == 1) ? 142 : 2;
        double goalY = 142;

        // Shooter offset relative to robot center
        double shooterOffset = -2.0; // 2 inches BEHIND robot center (negative = behind)

        // Compute shooter position using rotation
        double shooterX = currentX + shooterOffset * Math.cos(headingRad);
        double shooterY = currentY + shooterOffset * Math.sin(headingRad);

        // Compute distance from shooter to goal
        double dx = shooterX - goalX;
        double dy = shooterY - goalY;

        return Math.sqrt(dx * dx + dy * dy);
    }

    double calculateVelocity (double currentX, double currentY, double headingDeg) {
        double targetVelocity = 3.95833 * distanceFromGoal + 1402.08333;

        return Range.clip(targetVelocity, 0, 2420);
    }
    double calculateHoodPosition (double currentX, double currentY) {
        double targetDistance = distanceFromGoal;

        // Scale distance to the proper
        // double servoPosition = -5.339981 + 0.3090759*targetDistance - 0.00610722*Math.pow(targetDistance, 2) + 0.00004666887*Math.pow(targetDistance, 3) - 1.405935 * Math.pow(10, -8) *Math.pow(targetDistance, 4) - 9.799439 * Math.pow(10, -10) * Math.pow(targetDistance, 5);
        // TODO: Get Better Hood Regression
        double servoPosition = 0.002 * targetDistance - 0.09;
        // Relative angle (-180 to 180)

        return Range.clip(servoPosition, 0.00, 0.56);
    }
    double calculateTurretPosition(double currentX, double currentY, double robotHeadingRad) {

        // ===== FIELD GOAL =====
        double goalX = (teleopColorAlliance == 1) ? redGoalX : 2;
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
    public void recalibratePoseLimelight() {

    }

}
