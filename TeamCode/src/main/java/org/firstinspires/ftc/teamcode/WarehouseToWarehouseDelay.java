package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="WarehouseToWarehouse+15secDelay")
public class WarehouseToWarehouseDelay extends VisionAutonomous {

    public WarehouseToWarehouseDelay() {
        super();
        startDelay = 15000;
    }

    @Override
    protected void setupPoses() {

        // park at storage unit if storage unit is visible
        storageUnitEnd = false;

        forceWarehouseStart = true;

        super.setupPoses();
    }
}
