package org.firstinspires.ftc.teamcode.util.pathfinder;

import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Shape;
import xyz.devmello.voyager.zones.Zone;

public class TestZone extends Zone {


    public TestZone(Shape<?> shape) {
        super(shape);
    }

    @Override
    public void onEnter(Voyager voyager) {
        return;
    }
}
