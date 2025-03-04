package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;


public class limelightGetter {

    public static final double maxOpeningLinearMM = 298.0;//cm
    public static final double heightFromGround = 42; //cm
    public static final double angleFixed = 45; //degrees
    public static final double sampleHeight = 3.9; //cm
    public static final double armLength = 5; //cm


    public static Command getOpenLinearToSample() {
        return new openLinearToSample();
    }

    public static Command getAlignToSample(Limelight3A limelight) {
        return new alignToSample(limelight);
    }

    public static Command getAlignToSampleAuto(Limelight3A limelight, PinpointDrive drive ,TrajectoryActionBuilder previousAction) {
        return new alignToSampleAuto(limelight,drive,previousAction);
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
