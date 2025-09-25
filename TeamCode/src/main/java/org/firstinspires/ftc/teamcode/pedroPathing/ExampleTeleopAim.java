package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.arcrobotics.ftclib.controller.PIDFController;

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

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Limelight init
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        if(colorAlliance == 2) {
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

        if (!automatedDrive) {
            // Check for auto-aim
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

        //Optional adjust slowMode strength
        if (gamepad1.xWasPressed()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setPower(1);
        } else if (gamepad1.yWasPressed()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPower(1);
        } else if (gamepad1.bWasPressed()) {
            intakeMotor.setPower(0);
        }

        telemetryM.debug("Current Alliance", colorAlliance);
        telemetryM.debug("Position", follower.getPose());
        telemetryM.debug("Velocity", follower.getVelocity());
        telemetryM.debug("Automated Drive", automatedDrive);
    }
}


