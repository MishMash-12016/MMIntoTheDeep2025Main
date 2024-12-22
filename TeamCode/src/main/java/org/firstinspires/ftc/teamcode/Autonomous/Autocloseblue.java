package org.firstinspires.ftc.teamcode.Autonomous;


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
public class Autocloseblue extends MMOpMode {

    MMRobot robotInstance;


    public Autocloseblue() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose= (new Pose2d(16.49, 64.22, Math.toRadians(-90)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose).splineToSplineHeading(new Pose2d(57.52, 60.45, Math.toRadians(-140.00)), Math.toRadians(-54.61));
        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample.endTrajectory().fresh() .setTangent(Math.toRadians(240));
        TrajectoryActionBuilder driveToPickUpSecondSample1 = driveToPickUpSecondSample.endTrajectory().fresh().splineToSplineHeading(new Pose2d(59.34, 36.96, Math.toRadians(270.00)), Math.toRadians(270.00));
        TrajectoryActionBuilder DriveToScoreSecondsample = driveToPickUpSecondSample1.endTrajectory().fresh().splineToSplineHeading(new Pose2d(57.52, 60.45, Math.toRadians(-145.00)), Math.toRadians(117.44));
        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondsample.endTrajectory().fresh();
        new SequentialCommandGroup(

                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),

                //drive to score first pre load sample
                new ActionCommand(driveToScorePreloadSample.build(), Collections.emptySet()),
                RobotCommands.PrepareHighSample(),
                new WaitCommand(300),
                RobotCommands.ScoreSample(),
                //driving to pick up second sample + pick up sample commands
                new ActionCommand(driveToPickUpSecondSample.build(), Collections.emptySet()),
                RobotCommands.IntakeCommand(()-> 3),
                new WaitCommand(300),
                RobotCommands.IntakeDoneCommand(),
                new ActionCommand(driveToPickUpSecondSample1.build(), Collections.emptySet())
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


