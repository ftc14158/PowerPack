package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hub+ParkNearest_Delayed15sec")

public class HubAndNearestDelay extends HubAndNearest {

    public HubAndNearestDelay() {
        super();
        startDelay = 15000;
    }
}
