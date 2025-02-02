package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class MMDistSensor {

    DistanceSensor DS1;

    public MMDistSensor(HardwareMap hardwareMap) {
        DS1 = hardwareMap.get(DistanceSensor.class, Configuration.intakeDistanceSensor);
    }

    public boolean checkDis() {
        return DS1.getDistance(DistanceUnit.CM)<5;

    }

    public double getDistance() {
        return DS1.getDistance(DistanceUnit.CM);
    }

    ;


}
