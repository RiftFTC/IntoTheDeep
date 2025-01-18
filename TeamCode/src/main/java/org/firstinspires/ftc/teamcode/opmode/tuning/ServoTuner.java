package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Tuner", group = "tuning")
@Config
public class ServoTuner extends LinearOpMode {
    public static String name = "iClaw";
    public static double position = 0.36;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo crServo = hardwareMap.get(Servo.class, name);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                crServo.setPosition(0.5);
            }
        }
    }
}
