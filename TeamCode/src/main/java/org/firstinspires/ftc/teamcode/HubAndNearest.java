package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hub+ParkNearest")
public class HubAndNearest extends VisionAutonomous {

    @Override
    protected void setupPoses() {

        // park at storage unit if storage unit is visible
        storageUnitEnd = pipelineBarcode.isStorageUnitVisible();

        super.setupPoses();
    }
}
