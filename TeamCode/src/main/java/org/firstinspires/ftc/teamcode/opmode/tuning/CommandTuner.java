package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.*;

@TeleOp
public class CommandTuner extends BaseOpMode {
    @Override
    public void initialize() {
        super.initialize();
        gb1(GamepadKeys.Button.B).toggleWhenPressed(
                new ParallelCommandGroup(
                        outtakeV4bSys.mid(),
                        extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                        new SequentialCommandGroup(
                                intakeV4bSys.goToPos(POS_SPECIMEN_OUT),
                                intakeV4bSys.goToRoll(ROLL_OUT_SPECIMEN)
                        ),
                        intakeClawSys.release()
                ),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                intakeClawSys.pinch(),
                                outtakeClawSys.release()
                        ),
                        new WaitCommand(1000),
                        intakeV4bSys.goToRoll(ROLL_IN_SPECIMEN),
                        new WaitCommand(150),
                        new ParallelCommandGroup(
                                extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                                intakeV4bSys.goToPos(POS_SPECIMEN_IN),
                                outtakeV4bSys.specimen()
                        )
                )
        );

        gb1(GamepadKeys.Button.START).whenPressed(
                new SequentialCommandGroup(
                        liftSys.goTo(LiftSys.LOW_BUCKET),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                liftSys.vibrate(1500, 0.2),
                                transmissionSys.shiftUp()
                        )
                )
        );

        transmissionSys.setDefaultCommand(transmissionSys.manualControl(gamepadEx1::getLeftY));
    }
}
