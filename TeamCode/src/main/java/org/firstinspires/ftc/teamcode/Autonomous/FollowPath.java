package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.HeadingPath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.RobotCommands;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
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

        // x11 ,y21 ,heading 2

        //x25 ,y23 ,heading 2

        // x4 y 13 heading2

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose).splineTo(new Vector2d(7, 50) , Math.toRadians(310));
//        TrajectoryActionBuilder driveToScorePreloadSample1 =driveToScorePreloadSample.endTrajectory();
        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample.endTrajectory().fresh().splineToConstantHeading(new Vector2d(25 , 23) , Math.toRadians(50));
        TrajectoryActionBuilder driveToPickUpSecondSample1 = driveToPickUpSecondSample.endTrajectory().fresh().lineToY(11) ;
        TrajectoryActionBuilder DriveToScoreSecondsample = driveToPickUpSecondSample1.endTrajectory().fresh().lineToX(10);
        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondsample.endTrajectory().fresh().lineToX(10);

        new SequentialCommandGroup(

//                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.3),
                //drive to score first pre load sample
                new ActionCommand(driveToScorePreloadSample.build(), Collections.emptySet())
//                RobotCommands.PrepareHighSample(),
//                new WaitCommand(500),
//                RobotCommands.ScoreSample(),
                //driving to pick up second sample + pick up sample commands
//                new ActionCommand(driveToPickUpSecondSample.build(), Collections.emptySet()),
//                new ActionCommand(driveToPickUpSecondSample1.build() , Collections.emptySet())
//                RobotCommands.IntakeCommand(()-> 10),
//                new WaitCommand(500),
//                RobotCommands.IntakeDoneCommand(),
                // driving to score second sample + elevator commands
//                new ActionCommand(DriveToScoreSecondsample.build() , Collections.emptySet()),
//                RobotCommands.PrepareHighSample(),
//                RobotCommands.ScoreSample(),
                //driving to pick up third sample + pickup commands
//                new ActionCommand(drivetoPickUpthirdsample.build() , Collections.emptySet()),
//                RobotCommands.IntakeCommand(()-> 10),
//                new WaitCommand(500),
//                RobotCommands.IntakeDoneCommand()
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target pose:", MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }
}

