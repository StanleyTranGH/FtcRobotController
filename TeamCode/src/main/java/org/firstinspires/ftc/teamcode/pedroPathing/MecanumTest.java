package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum Test", group = "test")
public class MecanumTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontright");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backright");
        Servo horiSlide = hardwareMap.servo.get("horiSlide");
        CRServo intakeServo = hardwareMap.crservo.get("intakeServo");

        boolean wasA = false;
        boolean slideIsOut = false;

        double slideIn = 0.88;
        double slideOut = 0.86 - 0.29;

        double slidePosition = 0.88;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horiSlide.setPosition(slidePosition);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            boolean isA = gamepad1.a;
            boolean isX = gamepad2.x;
            boolean isY = gamepad2.y;

            if(isA && !wasA) {
                if (!slideIsOut) {
                    slidePosition = slideOut;
                    slideIsOut = true;
                } else {
                    slidePosition = slideIn;
                    slideIsOut = false;
                }
            }

            if (isX) {
                intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeServo.setPower(1);
            } else if (isY) {
                intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeServo.setPower(1);
            } else {
                intakeServo.setPower(0);
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            horiSlide.setPosition(slidePosition);

            wasA = isA;

            telemetry.addData("isA", isA);
            telemetry.addData("wasA", wasA);
            telemetry.addData("Slide Position", slidePosition);
        }
    }
}
