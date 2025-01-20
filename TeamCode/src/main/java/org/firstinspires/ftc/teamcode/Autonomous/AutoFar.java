package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


@Autonomous
public class AutoFar extends MMOpMode {
    MMRobot robotInstance;

    public AutoFar() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(5, -32, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToScorePreLoadSpecimen2 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .lineToY(-50);
        TrajectoryActionBuilder driveToPickUpFirst = driveToScorePreLoadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(360))//240))
                .splineToLinearHeading(new Pose2d(45.84, -53.84, Math.toRadians(90)), Math.toRadians(360));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToPickUpFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(5, -30, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder driveToPark = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(360))
                .splineToLinearHeading(new Pose2d(45.84, -53.84, Math.toRadians(90)), Math.toRadians(360));


        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()),
                ScoringSpecimanCommand.SpecimanScore(),
                new WaitCommand(500),
                new ActionCommand(driveToScorePreLoadSpecimen2.build())
                        .alongWith(new WaitCommand(400)).andThen(MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()),
                new WaitCommand(200),
                new ActionCommand(driveToPickUpFirst.build()),
               IntakeSpecimansCommand.PrepareSpecimanIntake().withTimeout(500).andThen(
               IntakeSpecimansCommand.SpecimenIntake()),
                new WaitCommand(300),
                new ActionCommand(driveToScoreFirstSpecimen.build()),
                ScoringSpecimanCommand.SpecimanScore(),
                new ActionCommand(driveToPark.build())
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear",MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().addData("height", MMRobot.getInstance().mmSystems.elevator.getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("target pose:", MMRobot.getInstance().mmSystems.elevator.targetPose);
        FtcDashboard.getInstance().getTelemetry().update();
    }

}


