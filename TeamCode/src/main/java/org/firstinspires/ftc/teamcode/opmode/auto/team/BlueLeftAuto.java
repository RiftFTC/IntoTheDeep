package org.firstinspires.ftc.teamcode.opmode.auto.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmode.auto.LeftAuto;

@Autonomous(name = "Blue Left Auto")
public class BlueLeftAuto extends LeftAuto {
    @Override
    public void init() {
        super.init();
        pipeline.setTeam(BaseOpMode.TEAM.BLUE);
    }
}
