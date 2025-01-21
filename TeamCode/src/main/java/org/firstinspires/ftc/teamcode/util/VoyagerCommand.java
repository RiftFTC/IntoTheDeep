package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;
import java.util.Collections;
import java.util.List;
import java.util.Set;

public class VoyagerCommand implements Command {
    private List<PointXYZ> points;
    private boolean finished = false;
    private final Voyager voyager;

    public VoyagerCommand(Voyager voyager, List<PointXYZ> points) {
        this.points = points;
        this.voyager = voyager;
    }

    @Override
    public void execute() {
        if (voyager.isActive()) {
            // if pathfinder is active (meaning it's going somewhere)
            // we want to tick/update it
            voyager.tick();
        } else {
            // pathfinder's not active - either...
            // 1. there are more points to visit
            // 2. we've finished
            // if there's more points, we should go to the next point. if
            // there are no more points, we should stop the robot.
            if (!points.isEmpty()) {
                PointXYZ nextPoint = points.get(0);
                voyager.goTo(nextPoint);
                points.remove(nextPoint);
                if (points.size() == 1) {
                    voyager.setAngleTolerance(Angle.fromDeg(3));
                    voyager.setTolerance(.7);
                }
            } else {
                // stop the robot
                voyager.getDrive().setTranslation(Translation.zero());
                finished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }
}
