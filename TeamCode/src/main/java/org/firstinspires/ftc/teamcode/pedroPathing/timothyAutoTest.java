package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;



import java.util.List;

@Disabled
@Autonomous(name = "new auto test", group = "test")
public class timothyAutoTest extends OpMode {

    private Limelight3A limelight;

    private final Pose startPose = new Pose(126.7, 118.4, Math.toRadians(126));
    private final Pose scanPose = new Pose(72, 120, Math.toRadians(90));
    private final Pose shootPose = new Pose(96.3, 108.1, Math.toRadians(36));
    initShootPath = follower.pathBuilder() 
        .addPath(new BezierLine(startPose, shootPose))
        .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
        .build();
    scanPath = follower.pathBuilder() 
        .addPath(new BezierLine(shootPose, scanPose))
        .setLinearHeadingInterpolation(shootPose.getHeading(), scanPose.getHeading())
        .build();
    
    //path 1, PPG
    private final Pose readyPose1 = new Pose(101, 83.4, Math.toRadians(0));
    private final Pose collectPose1 = new Pose(125.2, 83.4, Math.toRadians(0));
    readyPath1 = follower.pathBuilder() 
        .addPath(new BezierLine(scanPose, readyPose1))
        .setLinearHeadingInterpolation(scanPose.getHeading(), readyPose1.getHeading())
        .build();
    
    collectPath1 = follower.pathBuilder() 
        .addPath(new BezierLine(readyPose1, collectPose1))
        .setLinearHeadingInterpolation(readyPose1.getHeading(), collectPose1.getHeading())
        .build();

    shootPath1 = follower.pathBuilder()
        .addPath(new BezierLine(collectPose1, shootPose))
        .setLinearHeadingInterpolation(collectPose1.getHeading(), shootPose.getHeading())
        .build();

    //path 2, PGP
    private final Pose readyPose2 = new Pose(101, 59.4, Math.toRadians(0));
    private final Pose collectPose2 = new Pose(125.2, 59.4, Math.toRadians(0));
    readyPath2 = follower.pathBuilder() 
        .addPath(new BezierLine(scanPose, readyPose2))
        .setLinearHeadingInterpolation(scanPose.getHeading(), readyPose2.getHeading())
        .build();
    
    collectPath2 = follower.pathBuilder() 
        .addPath(new BezierLine(readyPose2, collectPose2))
        .setLinearHeadingInterpolation(readyPose2.getHeading(), collectPose2.getHeading())
        .build();
    
    shootPath2 = follower.pathBuilder()
        .addPath(new BezierLine(collectPose2, shootPose))
        .setLinearHeadingInterpolation(collectPose2.getHeading(), shootPose.getHeading())
        .build();


    //path 3, GPP
    private final Pose readyPose3 = new Pose(101, 35.4, Math.toRadians(0));
    private final Pose collectPose3 = new Pose(125.2, 35.4, Math.toRadians(0));
    readyPath3 = follower.pathBuilder() 
        .addPath(new BezierLine(scanPose, readyPose3))
        .setLinearHeadingInterpolation(scanPose.getHeading(), readyPose3.getHeading())
        .build();
    
    collectPath3 = follower.pathBuilder() 
        .addPath(new BezierLine(readyPose1, collectPose3))
        .setLinearHeadingInterpolation(readyPose3.getHeading(), collectPose3.getHeading())
        .build();

    shootPath3 = follower.pathBuilder()
        .addPath(new BezierLine(collectPose3, shootPose))
        .setLinearHeadingInterpolation(collectPose3.getHeading(), shootPose.getHeading())
        .build();

    @Override
    public void init() {
        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Ensure we're using pipeline 0 (your AprilTag pipeline)
        limelight.pipelineSwitch(0);

        // Start the Limelight
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //
        follower.update();
    }

    @Override
        public void start() {
        follower.setStartingPose(startPose);
        follower.followpath(initShootPath);
        follower.followpath(scanPath);

        loop();
        @Override
        public void loop() {
            try {
                // Get the latest result from Limelight
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    // Valid target detected
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    int aprilTagID = 0;
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        aprilTagID = fiducial.getFiducialId();
                    }

                    telemetry.addData("AprilTag Detected", "Yes");
                    telemetry.addData("AprilTag ID", aprilTagID);
                    if(aprilTagID == 21) {
                        telemetry.addData("Motif", "GPP");
                    } else if(aprilTagID == 22) {
                        telemetry.addData("Motif", "PGP");
                    } else if(aprilTagID == 23) {
                        telemetry.addData("Motif", "PPG");
                    } else {
                        telemetry.addData("Motif", "Error");
                    }

                } else {
                    // No valid target detected
                    telemetry.addData("AprilTag Detected", "No");
                }

            } catch (Exception e) {
            // Handle any exceptions
            telemetry.addData("Error", "Exception: " + e.getMessage());
            }

        // Update telemetry
        telemetry.update();
        }
        
        switch() {

        }

        if(aprilTagID == 21) {
            // GPP
            follower.followpath(readyPath3);
            follower.followpath(collectPath3);
            follower.followpath(shootPath3);

        } else if(aprilTagID == 22) {
            // PGP
            follower.followpath(readyPath2);
            follower.followpath(collectPath2);
            follower.followpath(shootPath2);
        } else if(aprilTagID == 23) {
            // PPG
            follower.followpath(readyPath1);
            follower.followpath(collectPath1);
            follower.followpath(shootPath1);
        }
    }

    @Override
    public void stop() {
        // Stop the Limelight when the OpMode is stopped
        if (limelight != null) {
            limelight.stop();
        }
    }
}
