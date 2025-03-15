package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;


public class limelightGetter {
    public static double correctionDist = 20;
    public static Command getOpenLinearToSample() {
        return new openLinearToSample();
    }

    public static Command getAlignToSample(HardwareMap hardwareMap) {
        return new alignToSample(hardwareMap);
    }

    public static Command getAlignToSampleAuto(HardwareMap hardwareMap, PinpointDrive drive) {
        return new alignToSampleAuto(hardwareMap,drive);
    }

    public static Command strafeToSampleAuto(PinpointDrive drive) {
        return new strafeToSampleAuto(drive);
    }

    public static Command getRotateToSample() {
        return new rotateToSample();
    }

    public static Command strafeToSample() {
        return new strafeToSample();
    }
}
