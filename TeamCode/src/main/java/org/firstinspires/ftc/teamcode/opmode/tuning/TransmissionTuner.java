package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Transmission Tuner", group = "Tuning")
public class TransmissionTuner extends LinearOpMode {
    public static double transUp = 0;
    public static double transDown = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SimpleServo servo = new SimpleServo(hardwareMap, "trans", 0, 180);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                servo.setPosition(transUp);
            } else if (gamepad1.b) {
                servo.setPosition(transDown);
            }
            telemetry.update();
        }
    }
}
