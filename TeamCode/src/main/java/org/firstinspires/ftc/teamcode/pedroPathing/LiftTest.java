package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Lift Test ManualHold", group = "test")
public class LiftTest extends LinearOpMode {

    DcMotor leftLiftMotor, rightLiftMotor;

    // Adjust this value to match encoder ticks per inch
    static final double TICKS_PER_INCH = 100; // example, tune for your slide
    static final int MAX_HEIGHT_TICKS = 2000; // max travel (~18 in)
    static final int MIN_HEIGHT_TICKS = 0;

    int targetPosition = 0; // tracked manually

    @Override
    public void runOpMode() {
        leftLiftMotor  = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setTargetPosition(targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start at 0
        leftLiftMotor.setTargetPosition(0);
        rightLiftMotor.setTargetPosition(0);

        leftLiftMotor.setPower(0.2);  // holding power
        rightLiftMotor.setPower(0.2);

        telemetry.addLine("Ready to lift!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual control: use left stick Y (up/down)
            double input = -gamepad1.left_stick_y; // up is positive

            // Deadband
            if (Math.abs(input) > 0.1) {
                // Adjust target based on stick input speed
                targetPosition += (int)(input * 20); // 20 ticks per loop, adjust speed
                targetPosition = Math.max(MIN_HEIGHT_TICKS, Math.min(MAX_HEIGHT_TICKS, targetPosition));
            }

            // Apply target to both motors (leader–follower sync)
            rightLiftMotor.setTargetPosition(targetPosition);
            rightLiftMotor.setPower(1.0);

            leftLiftMotor.setTargetPosition(targetPosition);
            leftLiftMotor.setPower(1.0);

            // Telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Right Pos", rightLiftMotor.getCurrentPosition());
            telemetry.addData("Left Pos", leftLiftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}


