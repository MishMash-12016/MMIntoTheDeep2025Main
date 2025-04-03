package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoSample7 extends MMOpMode {
    static MMRobot robotInstance;
    final Pose2d scorePose = new Pose2d(-58, -49, Math.toRadians(230));
    final Pose2d intakePose = new Pose2d(-24, -8, Math.toRadians(180));

    public AutoSample7() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        MMRobot.getInstance().mmSystems.vision.trackRedDetector();

        Pose2d currentPose = new Pose2d(-39, -65.5, Math.toRadians(180));
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;;

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);


        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-65, -49.7), Math.toRadians(264.5));

        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.7, -48), Math.toRadians(245));

        TrajectoryActionBuilder driveToScoreFirstSample = driveToIntakeFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.1, -52), Math.toRadians(247.7));

        TrajectoryActionBuilder driveToIntakeThird = driveToScoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.7, -48.2), Math.toRadians(307)
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.7)
                        , new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel * 0.7));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -47.5), Math.toRadians(259.33));

        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.4));

        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(), scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.4));

        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(), scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .strafeToLinearHeading(intakePose.component1(), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.4));

        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .strafeToLinearHeading(scorePose.component1(), scorePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20, -8), intakePose.component2()
                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5));

        new SequentialCommandGroup(
                new InstantCommand(),
                //1
                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        scorePreLoadSample()
                ),

                new ParallelCommandGroup(
                        ScoreHighSampleWithoutIntake(),
                        prepareSampleIntakeWithoutButtonAndScoring(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.4)
                ),

                //2
                new WaitCommand(100),//lamlam

                //TODO:can be joined
                IntakeSampleCommand.SampleIntake(),

                ScoringSampleCommand.PrepareHighSample(),

                new WaitCommand(200),

                //3
                new ActionCommand(driveToIntakeFirstSample.build()).alongWith(
                        ScoreHighSampleWithoutIntake(),
                        prepareSampleIntakeWithoutButtonAndScoring(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.35)
                ),

                IntakeSampleCommand.SampleIntake(),

                new ActionCommand(driveToScoreFirstSample.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                new ActionCommand(driveToIntakeThird.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample()
                ),

                IntakeSampleCommand.prepareSampleIntakeWithoutButton().alongWith(
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.25)
                ),
                IntakeSampleCommand.SampleIntake(),

                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                new ActionCommand(driveToIntakeForth.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample()
                ),

                //5
                IntakeSampleCommand.limeLightIntake_Auto(),

                new ActionCommand(driveToScoreForth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                new ActionCommand(driveToIntakeFifth.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample()
                ),

                //6
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.trackRedDetector()),
                IntakeSampleCommand.limeLightIntake_TeleOp(),

                new ActionCommand(driveToScoreFifth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                new ActionCommand(driveToIntakeSixth.build()).alongWith(
                        ScoringSampleCommand.ScoreHighSample()
                ),

                //7
                IntakeSampleCommand.limeLightIntake_Auto(),

                new ActionCommand(driveToScoreSixth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                new ActionCommand(driveToPark.build()).alongWith(
                        //basically scoring
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(200),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE),
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                )
                        )
                )
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear", MMRobot.getInstance().mmSystems.elevator.getElevatorSwitchState());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public Command scorePreLoadSample() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_SCORE)
                ),
                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
                )
        );
    }

    public static Command ScoreHighSampleWithoutIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORING_ARM_SCORE_POSE),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER)
                )
        );
    }

    public static Command prepareSampleIntakeWithoutButtonAndScoring() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(0.51),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                new WaitCommand(70).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ));
    }
}
