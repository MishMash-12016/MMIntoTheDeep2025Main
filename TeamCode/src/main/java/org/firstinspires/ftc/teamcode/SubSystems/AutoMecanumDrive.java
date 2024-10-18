package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.MMRobot;

public class AutoMecanumDrive extends SubsystemBase{

    private final SampleMecanumDrive drive;

    public AutoMecanumDrive(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
    }

    public SampleMecanumDrive getDrive(){
        return drive;
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
    }


}
