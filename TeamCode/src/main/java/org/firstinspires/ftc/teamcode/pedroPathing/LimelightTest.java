package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "AprilTag ID Detector", group = "test")
public class LimelightTest extends OpMode {

    private Limelight3A limelight;

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
    }

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

    @Override
    public void stop() {
        // Stop the Limelight when the OpMode is stopped
        if (limelight != null) {
            limelight.stop();
        }
    }
}