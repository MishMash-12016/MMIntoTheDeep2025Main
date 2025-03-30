package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoSample7 extends MMOpMode {
    static MMRobot robotInstance;
    final Pose2d scorePose = new Pose2d(-58, -49, Math.toRadians(-115));
    final Pose2d intakePose = new Pose2d(-24, -8, Math.toRadians(180));
    public AutoSample7() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(-39, -65.5, Math.toRadians(180)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);



        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                 .strafeToLinearHeading(new Vector2d(-48, -65.5), Math.toRadians(180));

        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.9, -46.8), Math.toRadians(250));

        TrajectoryActionBuilder driveToScoreFirstSample = driveToIntakeFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.1, -52), Math.toRadians(247.7));

        TrajectoryActionBuilder driveToSecondSample = driveToScoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.9, -48.5), Math.toRadians(266));

        TrajectoryActionBuilder driveToIntakeThird = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-61, -47.28), Math.toRadians(295));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -49), Math.toRadians(259.33));

        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.4));

        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(),scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.2));

        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.4));

        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(),scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.2));

        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.4));

        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(),scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.2));

        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(intakePose.component1(), Math.toRadians(0)), Math.toRadians(20)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.5));

        new SequentialCommandGroup(
                new InstantCommand(),

                new ActionCommand(driveToScorePreloadSample.build()),
                AutoSpecimensCommand.SpecimenScorePreLoad(),
                new ActionCommand(driveToIntakeFirstSample.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.4),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToScoreFirstSample.build()),
                new ActionCommand(driveToSecondSample.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.35),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToIntakeThird.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.25),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToScoreThird.build()),
                new ActionCommand(driveToIntakeForth.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToScoreForth.build()),
                new ActionCommand(driveToIntakeFifth.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToScoreFifth.build()),
                new ActionCommand(driveToIntakeSixth.build()),
                IntakeSampleCommand.prepareSampleIntakeWithoutButton(),
                new WaitCommand(400),
                IntakeSampleCommand.SampleIntake(),
                new ActionCommand(driveToScoreSixth.build()),
                new ActionCommand(driveToPark.build())
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
