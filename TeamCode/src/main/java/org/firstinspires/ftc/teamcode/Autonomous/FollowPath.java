package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

import java.util.Collections;

@Autonomous
public class FollowPath extends MMOpMode {

    MMRobot robotInstance;

    public FollowPath() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose= new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);


        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose).strafeToConstantHeading(new Vector2d(15,58));
        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample.endTrajectory().fresh().lineToX(8);
        TrajectoryActionBuilder driveToPickUpSecondSample1 = driveToScorePreloadSample.endTrajectory().fresh().lineToY(10);
        TrajectoryActionBuilder DriveToScoreSecondsample = driveToPickUpSecondSample1.endTrajectory().fresh().lineToX(10);
        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondsample.endTrajectory().fresh().lineToX(10);

        new SequentialCommandGroup(

                //drive to score first pre load sample
                new ActionCommand(driveToScorePreloadSample.build(), Collections.emptySet()),
                RobotCommands.PrepareLowSample(),
                RobotCommands.ScoreSample()
                //driving to pick up second sample + pick up sample commands
//                new ActionCommand(driveToPickUpSecondSample.build(), Collections.emptySet()),
//                new ActionCommand(driveToPickUpSecondSample1.build() , Collections.emptySet())
//                RobotCommands.IntakeCommand(()-> 10),
//                new WaitCommand(500),
//                RobotCommands.IntakeDoneCommand(),
                // driving to score second sample + elevator commands
              //  new ActionCommand(DriveToScoreSecondsample.build() , Collections.emptySet()),
//                RobotCommands.PrepareHighSample(),
//                RobotCommands.ScoreSample(),
                //driving to pick up third sample + pickup commands
              //  new ActionCommand(drivetoPickUpthirdsample.build() , Collections.emptySet())
//                RobotCommands.IntakeCommand(()-> 10),
//                new WaitCommand(500),
//                RobotCommands.IntakeDoneCommand()
        ).schedule();

    }
}
