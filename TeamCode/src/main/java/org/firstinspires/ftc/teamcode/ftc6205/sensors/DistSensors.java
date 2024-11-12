package org.firstinspires.ftc.teamcode.ftc6205.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistSensors {
    HardwareMap hwMap;
    DistanceSensor distFront;// distBack;

    public void initDistSensors(HardwareMap ahwMap) {
        hwMap = ahwMap;
        distFront = hwMap.get(DistanceSensor.class, "distFront");
        //distBack = hwMap.get(DistanceSensor.class, "distBack");
    }

}
