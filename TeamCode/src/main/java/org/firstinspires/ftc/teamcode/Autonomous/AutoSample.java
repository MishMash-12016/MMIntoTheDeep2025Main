package org.firstinspires.ftc.teamcode.Autonomous;

//imports

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;



public class AutoSample extends MMOpMode {
    MMRobot robotInstance;
    int waitBeforeOpeningScoringClawTime = 1000;

    public AutoSample() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(-19, -66.5, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        // pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);
        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_SAMPLE_POSE);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-52.8, -59, Math.toRadians(225)), Math.toRadians(250)); //
        TrajectoryActionBuilder driveToPickUpFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-50, -51, Math.toRadians(270)), Math.toRadians(80));
        TrajectoryActionBuilder driveToScoreFirstSample = driveToPickUpFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52.8, -59, Math.toRadians(215)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPickUpSecondSample= driveToScoreFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-60.5, -51, Math.toRadians(270)), Math.toRadians(100));
        TrajectoryActionBuilder driveToScoreSecondSample = driveToPickUpSecondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(-52.8, -59, Math.toRadians(215)), Math.toRadians(300));
        TrajectoryActionBuilder driveToPickUpThirdSample= driveToScoreSecondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-54, -45, Math.toRadians(315)), Math.toRadians(95)) ;
        TrajectoryActionBuilder driveToScoreThirdSample = driveToPickUpThirdSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-51.8, -58, Math.toRadians(215)), Math.toRadians(200));
        TrajectoryActionBuilder driveToTurn = driveToScoreThirdSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-51.8, -50, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(-60);
//        TrajectoryActionBuilder driveToPark = driveToScoreThirdSample.endTrajectory().fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-20.65, -8.25, Math.toRadians(0)), Math.toRadians(180));


        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.build()),
                ScoringSampleCommand.HighSampleAuto(),
                new WaitCommand(500),
                new ActionCommand(driveToPickUpFirstSample.build()),
                new WaitCommand(200),
                IntakeSampleCommand.prepareSampleIntake(() -> false, () -> false).withTimeout(1000),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreFirstSample.build()).alongWith(
                ScoringSampleCommand.PrepareHighSample()),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
               new ActionCommand(driveToPickUpSecondSample.build()),
                new WaitCommand(200),
                IntakeSampleCommand.prepareSampleIntake(() -> false, () -> false).withTimeout(1000),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreSecondSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new ActionCommand(driveToTurn.build())




//                new ActionCommand(driveToPark.build()),
//                new WaitCommand(200),
//                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO)
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("height",MMRobot.getInstance().mmSystems.elevator.getHeight());
        telemetry.addData("target",MMRobot.getInstance().mmSystems.elevator.targetPose);
        telemetry.update();
    }

}





