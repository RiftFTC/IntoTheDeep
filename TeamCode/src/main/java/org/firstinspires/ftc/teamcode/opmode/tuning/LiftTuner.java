package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Lift Tuner", group = "Tuning")
public class LiftTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        MotorEx lift = new MotorEx(hardwareMap, "lil");
        MotorEx encoder = new MotorEx(hardwareMap, "lir");

        TouchSensor touch = hardwareMap.get(TouchSensor.class, "touch");
        lift.stopAndResetEncoder();
        encoder.stopAndResetEncoder();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Lift Pos",lift.getCurrentPosition());
            telemetry.addData("Encoder POS", encoder.getCurrentPosition());
            telemetry.addData("Pressed", touch.isPressed());
            if (touch.isPressed()) {
                encoder.set(1);
                lift.set(1);
            } else {
                encoder.set(0);
                lift.set(0);
            }

            telemetry.update();
        }
    }
}
