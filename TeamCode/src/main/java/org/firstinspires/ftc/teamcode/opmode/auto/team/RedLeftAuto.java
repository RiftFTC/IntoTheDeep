package org.firstinspires.ftc.teamcode.opmode.auto.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmode.auto.LeftAuto;

@Autonomous(name = "Red Left Auto")
public class RedLeftAuto extends LeftAuto {
    @Override
    public void init() {
        super.init();
        pipeline.setTeam(BaseOpMode.TEAM.RED);
    }
}
