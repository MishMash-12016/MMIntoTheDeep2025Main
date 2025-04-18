package org.firstinspires.ftc.teamcode.CommandGroup.limelight;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandGroup.interruptibleTrajectory;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.function.BooleanSupplier;


public class limelightGetter {

    public static TrajectoryActionBuilder currentTraj = MMRobot.getInstance().mmSystems.driveTrain.actionBuilder(MMRobot.getInstance().mmSystems.currentPose);
    public static Command getRotateToSample() {
        return new rotateToSample();
    }
    public static Command goToSpecimen(Pose2d pose2d, double tangent, VelConstraint velConstraint, AccelConstraint accelConstraint , Double setTangent) {
        return new goToSpecimen(pose2d, tangent, velConstraint, accelConstraint, setTangent);
    }

    public static Command strafeToSample(boolean isSample) {
        return new strafeToSample(isSample);
    }

    public static Command interruptibleTrajectory(BooleanSupplier flag , Vector2d pose2d, double tangent, VelConstraint velConstraint, AccelConstraint accelConstraint , Double setTangent) {
        return new interruptibleTrajectory( flag ,  pose2d,  tangent,  velConstraint,  accelConstraint ,  setTangent);
    }

}
