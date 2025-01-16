package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

@TeleOp
public class LiftPIDTuner extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
//        gb2(GamepadKeys.Button.A).whenPressed(liftSys.goTo(LiftSys.NONE));
//        gb2(GamepadKeys.Button.B).whenPressed(liftSys.goTo(LiftSys.HIGH_RUNG));
//        gb2(GamepadKeys.Button.X).whenPressed(liftSys.goTo(LiftSys.LOW_BUCKET));
//        gb2(GamepadKeys.Button.Y).whenPressed(liftSys.goTo(LiftSys.HIGH_BUCKET));
//        gb1(GamepadKeys.Button.A).whenPressed(liftSys.vibrate(1500, 0.2));
        schedule(liftSys.goTo(LiftSys.HIGH_RUNG));
    }
}