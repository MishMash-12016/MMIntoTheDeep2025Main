package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.CommandGroup.limelight.alignToSample;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.openLinearToSample;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.rotateToSample;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;


public class limelightGetter {

    public static final double maxOpeningLinearCM = 34.5;//cm
    public static final double heightFromGround = 43; //cm
    public static final double angleFixed = 45; //degrees
    public static final double sampleHeight = 3.9; //cm
    public static final double armLength = 14; //cm


    public static Command getOpenLinearToSample(Limelight3A limelight) {
        return new openLinearToSample(limelight);
    }

    public static Command getAlignToSample(Limelight3A limelight) {
        return new alignToSample(limelight);
    }

    public static Command getAlignToSampleAuto(Limelight3A limelight, PinpointDrive drive ,TrajectoryActionBuilder previousAction) {
        return new alignToSampleAuto(limelight,drive,previousAction);
    }

    public static Command strafeToSampleAuto(Limelight3A limelight, PinpointDrive drive ,TrajectoryActionBuilder previousAction) {
        return new strafeToSampleAuto(limelight,drive,previousAction);
    }

    public static Command getRotateToSample(Limelight3A limelight) {
        return new rotateToSample(limelight);
    }

    public static Command strafeToSample(Limelight3A limelight, PinpointDrive drive) {
        return new strafeToSample(limelight);
    }
}
