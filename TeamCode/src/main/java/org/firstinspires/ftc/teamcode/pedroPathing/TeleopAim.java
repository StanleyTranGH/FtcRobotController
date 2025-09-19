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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.Supplier;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Configurable
@TeleOp(name = "Teleop Aim", group = "Examples")
public class TeleopAim extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    // Limelight fields
    private Limelight3A limelight;
    private double kP_Aim = 0.03; // proportional gain for turning

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Limelight init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
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
                    double tx = result.getTx(); // horizontal offset

                    // Deadband to prevent jitter
                    if (Math.abs(tx) < 1.0) {
                        tx = 0;
                    }

                    turn = kP_Aim * tx;
                } else {
                    turn = -gamepad1.right_stick_x;
                }

                // Normal or slow mode XY control
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        turn,
                        true // Robot Centric
                );
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        turn,
                        true // Robot Centric
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
                slowModeMultiplier += 0.25;
            }
            if (gamepad2.yWasPressed()) {
                slowModeMultiplier -= 0.25;
            }

            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);
        }
    }
}
