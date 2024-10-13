package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.utils.Configuration;

public class MMIMU {
    BHI260IMU imu;

    double yawOffset = 0;

    public MMIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BHI260IMU.class, Configuration.IMU);
        BHI260IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(imuParameters);
    }

    public double getYawInDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + yawOffset;
    }

    public void setYaw(double newYaw) {
        yawOffset = newYaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        setYaw(0);
    }

}
