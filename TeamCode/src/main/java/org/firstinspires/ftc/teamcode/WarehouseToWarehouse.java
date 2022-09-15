package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="WarehouseToWarehouse")
public class WarehouseToWarehouse extends VisionAutonomous {

    @Override
    protected void setupPoses() {

        // park at storage unit if storage unit is visible
        storageUnitEnd = false;

        forceWarehouseStart = true;

        super.setupPoses();
    }
}
