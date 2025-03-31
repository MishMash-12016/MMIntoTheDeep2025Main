package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;

import java.util.function.BooleanSupplier;


public class limelightGetter {
    static openLinearToSample instance;

    public static Command getOpenLinearToSample() {
        instance = new openLinearToSample();
        return instance;
    }

    public static BooleanSupplier getIsLockedInLinear() {
        return () -> instance.finished;
    }

    public static Command getRotateToSample() {
        return new rotateToSample();
    }

    public static Command strafeToSample() {
        return new strafeToSample();
    }
}
