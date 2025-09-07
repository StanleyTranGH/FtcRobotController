package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Limelight Distance Calculation")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private static final double TARGET_WIDTH_MM = 58.0;
    private static final double TARGET_HEIGHT_MM = 58.0;
    private static final double MM_TO_INCHES = 0.0393701;
    private static final int YELLOW_PIPELINE = 9; // Changed to 0, assuming it's the first pipeline

    // Limelight 3A horizontal and vertical FOV in degrees
    private static final double HORIZONTAL_FOV = 54.5;
    private static final double VERTICAL_FOV = 42.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set the pipeline to our yellow detection pipeline
        limelight.pipelineSwitch(YELLOW_PIPELINE);

        // Start the Limelight
        limelight.start();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();

        telemetry.addData("Status", "Running...");
        telemetry.update();

        while (opModeIsActive()) {
            // Get the latest result from Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null) {
                telemetry.addData("Result", "Not null");
                telemetry.addData("Is Valid", result.isValid());

                if (result.isValid()) {
                    // Target detected, calculate distances
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();

                    telemetry.addData("TX", tx);
                    telemetry.addData("TY", ty);
                    telemetry.addData("TA", ta);

                    // Calculate distances
                    double[] distances = calculateDistances(tx, ty, ta);

                    // Display results
                    telemetry.addData("Horizontal Distance", String.format("%.2f inches", distances[0]));
                    telemetry.addData("Vertical Distance", String.format("%.2f inches", distances[1]));
                    telemetry.addData("Forward Distance", String.format("%.2f inches", distances[2]));
                } else {
                    telemetry.addData("Status", "Result not valid");
                }
            } else {
                telemetry.addData("Status", "No result from Limelight");
            }

            telemetry.update();
        }
    }

    private double[] calculateDistances(double tx, double ty, double ta) {
        // Convert target area percentage to actual pixels
        double targetPixelArea = ta * (320.0 * 240.0) / 100.0; // Assuming 320x240 resolution

        // Calculate the apparent width and height in pixels
        double targetPixelWidth = Math.sqrt(targetPixelArea * (TARGET_WIDTH_MM / TARGET_HEIGHT_MM));
        double targetPixelHeight = Math.sqrt(targetPixelArea * (TARGET_HEIGHT_MM / TARGET_WIDTH_MM));

        // Calculate forward distance using both width and height, and average them
        double distanceFromWidthMM = (TARGET_WIDTH_MM * 320) / (2 * targetPixelWidth * Math.tan(Math.toRadians(HORIZONTAL_FOV / 2)));
        double distanceFromHeightMM = (TARGET_HEIGHT_MM * 240) / (2 * targetPixelHeight * Math.tan(Math.toRadians(VERTICAL_FOV / 2)));
        double forwardDistanceMM = (distanceFromWidthMM + distanceFromHeightMM) / 2;

        // Calculate horizontal and vertical distances
        double horizontalDistanceMM = forwardDistanceMM * Math.tan(Math.toRadians(tx));
        double verticalDistanceMM = forwardDistanceMM * Math.tan(Math.toRadians(ty));

        // Convert distances to inches
        double horizontalDistanceInches = horizontalDistanceMM * MM_TO_INCHES;
        double verticalDistanceInches = verticalDistanceMM * MM_TO_INCHES;
        double forwardDistanceInches = forwardDistanceMM * MM_TO_INCHES;

        return new double[]{horizontalDistanceInches, verticalDistanceInches, forwardDistanceInches};
    }
}