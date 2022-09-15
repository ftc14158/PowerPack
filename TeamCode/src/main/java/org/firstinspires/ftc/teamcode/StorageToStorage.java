package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="StorageToStorage")
public class StorageToStorage extends VisionAutonomous {

    @Override
    protected void setupPoses() {

        // park at storage unit if storage unit is visible
        storageUnitEnd = true;

        forceSUStart = true;

        super.setupPoses();
    }
}
