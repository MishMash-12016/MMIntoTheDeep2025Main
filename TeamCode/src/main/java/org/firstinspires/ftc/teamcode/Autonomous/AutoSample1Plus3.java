package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoSample1Plus3 extends MMOpMode {
    MMRobot robotInstance;

    final double halfOpenClaw = 0.7;
    final double rotator = 0;
    final double intakeArmPose = 0.59;

    private Limelight3A limelight;

    public AutoSample1Plus3() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = new Pose2d(-39, -65.5, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-50, -65.5), Math.toRadians(180));

        TrajectoryActionBuilder driveToFirst = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58.49, -47.7), Math.toRadians(246));

        TrajectoryActionBuilder driveToSecond = driveToFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-63.4, -49.3), Math.toRadians(258.9));

        TrajectoryActionBuilder driveToIntakeThird = driveToSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50.8, -45), Math.toRadians(315));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-56, -56), Math.toRadians(225));

        TrajectoryActionBuilder driveToPark = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(65))
                .splineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)), Math.toRadians(0));

        new SequentialCommandGroup(
                new InstantCommand(),
                //pre-load
                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET), //the height of the high basket
                        new WaitCommand(100),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SAMPLE)
                ),
                ScoringSampleCommand.ScoreHighSample(),

                //first
                new ActionCommand(driveToFirst.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake_Lime(limelight,drive)
                ),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //second
                new ActionCommand(driveToSecond.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake_Lime(limelight,drive)
                ),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoringSampleCommand.ScoreHighSample(),

                //third
                new ActionCommand(driveToIntakeThird.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntake_Lime(limelight,drive)
                ),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                ScoringSampleCommand.ScoreHighSample(),

                //park
                new ActionCommand(driveToPark.build()).alongWith(
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO)
                )//,
                //new InstantCommand(() -> robotInstance.mmSystems.scoringArm.CutPower())
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

    public Command setupForPushing(){
        return new ParallelCommandGroup(
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
        );
    }


}