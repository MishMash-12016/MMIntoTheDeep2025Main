package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;


@Autonomous
public class AutoSpecimen extends MMOpMode {
    MMRobot robotInstance;
    int waitBeforeOpeningScoringClawTime = 1000;

    public AutoSpecimen() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-5, -28, Math.toRadians(90)), Math.toRadians(90)); //score specimen

        TrajectoryActionBuilder driveToScorePreLoadSpecimen2 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .lineToY(-45);
        TrajectoryActionBuilder driveToPushSampleToHuman = driveToScorePreLoadSpecimen2.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, -44.84, Math.toRadians(90)), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-15))
                .strafeTo(new Vector2d(46,-15));
        TrajectoryActionBuilder driveToPushSampleToHuman2 = driveToPushSampleToHuman.endTrajectory().fresh()
                .strafeTo(new Vector2d(45,-54)) //- wait 200 sec
                .lineToY(-60);
       TrajectoryActionBuilder driveToPickUpFirst = driveToScorePreLoadSpecimen2.endTrajectory().fresh()
               .setTangent(Math.toRadians(180)) //pick up first
               .splineToLinearHeading(new Pose2d(45.84, -59,Math.toRadians(90)),Math.toRadians(180));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToPickUpFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, -28, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(-50);

//        TrajectoryActionBuilder driveToPickUpFirst2 = driveToPickUpFirst.endTrajectory().fresh()
//                .strafeTo(new Vector2d(45.84, -59));
//        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToPickUpFirst2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-3, -28, Math.toRadians(90)), Math.toRadians(90));
//        TrajectoryActionBuilder driveToScoreFirstSpecimen2 = driveToScoreFirstSpecimen.endTrajectory().fresh()
//                .lineToY(-50);
//        TrajectoryActionBuilder driveToPark = driveToScoreFirstSpecimen2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(360))
//                .splineToLinearHeading(new Pose2d(45.84, -54.84, Math.toRadians(90)), Math.toRadians(360));
//

        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSpecimen.build()),
                ScoringSpecimanCommand.SpecimanScore(),
                new WaitCommand(500),
                new ActionCommand(driveToScorePreLoadSpecimen2.build())
                        .alongWith(new WaitCommand(waitBeforeOpeningScoringClawTime).andThen(MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw())),
                new WaitCommand(200),
                IntakeSpecimansCommand.PrepareSpecimanIntake(),
                new ActionCommand(driveToPickUpFirst.build()),
                new WaitCommand(1000)
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

