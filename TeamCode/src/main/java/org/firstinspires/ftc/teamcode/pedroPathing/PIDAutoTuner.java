package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Full PID Auto-Tuner", group = "Tuning")
public class PIDAutoTuner extends LinearOpMode {

    DcMotorEx launcher;

    // target velocity (ticks per second)
    static final double TARGET_VELOCITY = 1500;

    // initial PID values
    double kP = 0.001;
    double kI = 0.0;
    double kD = 0.0001;

    // search ranges
    double maxKP = 0.02;
    double maxKI = 0.001;
    double maxKD = 0.001;

    double stepP = 0.001;
    double stepI = 0.0001;
    double stepD = 0.0001;

    double tuningPower = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("🧠 PID Auto-Tuner Ready");
        telemetry.addLine("Press PLAY to test motor and start tuning.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // === Step 1: Motor test ===
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Testing motor...");
        telemetry.update();

        double startPos = launcher.getCurrentPosition();
        launcher.setPower(tuningPower);
        sleep(1000);
        launcher.setPower(0);
        double moved = Math.abs(launcher.getCurrentPosition() - startPos);

        if (moved < 10) {
            telemetry.addLine("⚠️ Motor test failed! Check wiring/config name ('launcher').");
            telemetry.update();
            sleep(4000);
            return;
        }

        telemetry.addLine("✅ Motor OK! Starting full PID tuning...");
        telemetry.update();
        sleep(1000);

        // === Step 2: PID tuning ===
        double bestP = kP, bestI = kI, bestD = kD;
        double bestError = Double.MAX_VALUE;

        // --- Tune P ---
        for (double p = 0.001; p <= maxKP; p += stepP) {
            double avgError = testPID(p, kI, kD);
            if (avgError < bestError) {
                bestError = avgError;
                bestP = p;
            }
        }

        // --- Tune I ---
        bestError = Double.MAX_VALUE;
        for (double i = 0.0; i <= maxKI; i += stepI) {
            double avgError = testPID(bestP, i, kD);
            if (avgError < bestError) {
                bestError = avgError;
                bestI = i;
            }
        }

        // --- Tune D ---
        bestError = Double.MAX_VALUE;
        for (double d = 0.0; d <= maxKD; d += stepD) {
            double avgError = testPID(bestP, bestI, d);
            if (avgError < bestError) {
                bestError = avgError;
                bestD = d;
            }
        }

        // === Step 3: Results ===
        launcher.setPower(0);
        telemetry.addLine("✅ Full PID tuning complete!");
        telemetry.addData("Best kP", bestP);
        telemetry.addData("Best kI", bestI);
        telemetry.addData("Best kD", bestD);
        telemetry.addData("Final Avg Error", bestError);
        telemetry.update();

        sleep(5000);
    }

    private double testPID(double p, double i, double d) {
        double integral = 0, lastError = 0;
        ElapsedTime timer = new ElapsedTime();
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(0);
        sleep(200);

        double totalError = 0;
        int samples = 0;

        timer.reset();
        while (opModeIsActive() && timer.seconds() < 1.0) {
            double velocity = launcher.getVelocity();
            double error = TARGET_VELOCITY - velocity;

            integral += error;
            double derivative = error - lastError;

            double output = p * error + i * integral + d * derivative;
            output = Math.max(-1.0, Math.min(1.0, output));
            launcher.setPower(output);

            lastError = error;
            totalError += Math.abs(error);
            samples++;

            telemetry.addData("kP", p);
            telemetry.addData("kI", i);
            telemetry.addData("kD", d);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        launcher.setPower(0);
        sleep(150);

        return totalError / Math.max(samples, 1);
    }
}
