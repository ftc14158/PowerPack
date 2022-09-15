package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="StorageToStorage+15secDelay")
public class StorageToStorageDelay extends VisionAutonomous {

    public StorageToStorageDelay() {
        super();
        startDelay = 15000;
    }

    @Override
    protected void setupPoses() {

        // park at storage unit if storage unit is visible
        storageUnitEnd = true;

        forceSUStart = true;

        super.setupPoses();
    }
}
