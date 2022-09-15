package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hub+Duck+StorageUnit")

public class TestHubDuckStorage extends VisionAutonomous {
    public TestHubDuckStorage() {
        super();
        storageUnitEnd = true;
        duckAttempt = true;
    }
}
