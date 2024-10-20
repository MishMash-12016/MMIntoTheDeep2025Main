package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.AutoMecanumDrive;

public class TrajectroryFollowerCommend extends CommandBase {

    private final AutoMecanumDrive drive;
    private final TrajectorySequence trajectorySequence;

    public TrajectroryFollowerCommend(TrajectorySequence trajectorySequence, AutoMecanumDrive drive) {
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectorySequence);
    }

    @Override
    public void execute() {
        drive.update();
    }


    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}

