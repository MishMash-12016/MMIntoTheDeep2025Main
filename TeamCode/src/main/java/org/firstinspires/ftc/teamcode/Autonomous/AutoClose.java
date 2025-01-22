package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


@Autonomous
public class AutoClose extends MMOpMode {
    MMRobot robotInstance;
    int waitBeforeOpeningScoringClawTime = 1000;

    public AutoClose() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(-5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-5, -45, Math.toRadians(90)), Math.toRadians(90)) //
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-48.84, -44.84, Math.toRadians(270)), Math.toRadians(180)) //
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-53.8, -53, Math.toRadians(225)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPickUpFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-48.84, -44.84, Math.toRadians(270)), Math.toRadians(80));
        TrajectoryActionBuilder driveToScoreFirstSample = driveToPickUpFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(new Pose2d(-53.8, -53, Math.toRadians(225)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPark = driveToScoreFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-20.65, -8.25, Math.toRadians(180)), Math.toRadians(0.0));


        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(500),
                new ActionCommand(driveToPickUpFirstSample.build()),
                new WaitCommand(200),
                IntakeSampleCommand.prepareSampleIntake(()-> false,()-> false),
                new WaitCommand(200),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreFirstSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(400),
                new ActionCommand(driveToPark.build())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
    }

}





