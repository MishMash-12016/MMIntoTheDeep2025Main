package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class MMDSensor {

    DistanceSensor intakeDistanceSensor;

    public MMDSensor(HardwareMap hardwareMap) {
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, Configuration.intakeDistanceSensor);
    }

    public boolean checkDis() {
        return intakeDistanceSensor.getDistance(DistanceUnit.CM) < 5;
    }

    public double getDistance(DistanceUnit unit) {
        return intakeDistanceSensor.getDistance(DistanceUnit.CM);
    }


}
