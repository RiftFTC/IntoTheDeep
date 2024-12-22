package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo Tuner", group = "tuning")
@Config
public class ServoTuner extends LinearOpMode {
    public static String name = "";
    public static double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo crServo = hardwareMap.get(CRServo.class, name);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                crServo.setPower(0.5);
            }
        }
    }
}
