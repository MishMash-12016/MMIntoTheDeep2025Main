package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
public class AutofarRed extends MMOpMode {

    MMRobot robotInstance;


    public AutofarRed() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose=(new Pose2d(8.09, -64.52, Math.toRadians(90.00)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose).splineToLinearHeading(new Pose2d(-25.28, -40.65, Math.toRadians(176.01)), Math.toRadians(176.01));
        TrajectoryActionBuilder driveToScorePreloadSample1 = driveToScorePreloadSample.endTrajectory().fresh().splineToLinearHeading(new Pose2d(-59.87, -57.44, Math.toRadians(45.00)), Math.toRadians(111.46));
        TrajectoryActionBuilder driveToPickUpSecondSample = driveToScorePreloadSample1.endTrajectory().fresh().splineToLinearHeading(new Pose2d(-59.87, -39.24, Math.toRadians(90.00)), Math.toRadians(89.10));
        TrajectoryActionBuilder DriveToScoreSecondSample = driveToPickUpSecondSample.endTrajectory().fresh().splineToLinearHeading(new Pose2d(-59.46, -59.06, Math.toRadians(45.00)), Math.toRadians(-47.09));
        TrajectoryActionBuilder drivetoPickUpthirdsample = DriveToScoreSecondSample.endTrajectory().fresh();
        new SequentialCommandGroup(

                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),

                //drive to score first pre load sample
                new ActionCommand(driveToScorePreloadSample.build(), Collections.emptySet()),
                new ActionCommand(driveToScorePreloadSample1.build() , Collections.emptySet()),
                RobotCommands.PrepareHighSample(),
                new WaitCommand(300),
                RobotCommands.ScoreSample(),

                //drive to pickup second sample
                new ActionCommand(driveToPickUpSecondSample.build() , Collections.emptySet())
//                RobotCommands.IntakeCommand(()-> 3),
//                new WaitCommand(300),
//                RobotCommands.IntakeDoneCommand(),

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

