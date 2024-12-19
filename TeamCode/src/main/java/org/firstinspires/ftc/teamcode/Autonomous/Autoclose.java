package org.firstinspires.ftc.teamcode.Autonomous;


import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
public class Autoclose extends MMOpMode {

    MMRobot robotInstance;

    public Autoclose() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose= new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose).strafeToLinearHeading(new Vector2d(9, 52) , toRadians(-60));
        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample.endTrajectory().fresh().splineToSplineHeading(new Pose2d(9.45 , 41.74, 0) , Math.toRadians(0));
        TrajectoryActionBuilder driveToPickUpSecondSample1 = driveToPickUpSecondSample.endTrajectory().fresh().lineToY(11) ;
        TrajectoryActionBuilder DriveToScoreSecondsample = driveToPickUpSecondSample1.endTrajectory().fresh().lineToX(10);
        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondsample.endTrajectory().fresh().lineToX(10);

        new SequentialCommandGroup(

                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),

                //drive to score first pre load sample
                new ActionCommand(driveToScorePreloadSample.build(), Collections.emptySet()),
//                RobotCommands.PrepareHighSample(),
                new WaitCommand(300),
//                RobotCommands.ScoreSample()
                //driving to pick up second sample + pick up sample commands
                new ActionCommand(driveToPickUpSecondSample.build(), Collections.emptySet()),
//                new ActionCommand(driveToPickUpSecondSample1.build() , Collections.emptySet())
                RobotCommands.IntakeCommand(()-> 10),
                new WaitCommand(500),
               RobotCommands.IntakeDoneCommand()
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

