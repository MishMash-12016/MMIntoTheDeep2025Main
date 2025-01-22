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


@Autonomous
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
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-51.8, -58, Math.toRadians(225)), Math.toRadians(250)); //
        TrajectoryActionBuilder driveToPickUpFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-49, -50.84, Math.toRadians(275)), Math.toRadians(80));
        TrajectoryActionBuilder driveToScoreFirstSample = driveToPickUpFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-51.8, -58, Math.toRadians(225)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPark = driveToScoreFirstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(-20.65, -8.25, Math.toRadians(0)), Math.toRadians(0.0));


        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.build()),
                ScoringSampleCommand.HighSampleAuto(),
                new WaitCommand(500),
                new ActionCommand(driveToPickUpFirstSample.build()),
                new WaitCommand(200),
                IntakeSampleCommand.prepareSampleIntake(() -> false, () -> false).withTimeout(600),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreFirstSample.build()),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),
                new WaitCommand(400),
                new ActionCommand(driveToPark.build()),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO)
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





