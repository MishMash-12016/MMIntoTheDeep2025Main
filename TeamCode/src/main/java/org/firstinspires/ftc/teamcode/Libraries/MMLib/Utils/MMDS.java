package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.Configuration;

public class MMDS {

    DistanceSensor DS1;

    public MMDS(HardwareMap hardwareMap) {
        DS1 = hardwareMap.get(DistanceSensor.class, Configuration.DS1);
    }

    public boolean checkDis() {
        return DS1.getDistance(DistanceUnit.CM) < 5;
    }

    public double getDistance(DistanceUnit unit) {
        return DS1.getDistance(DistanceUnit.CM);
    }


}
