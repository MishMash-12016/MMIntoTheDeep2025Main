package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.LazyActionCommand;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class TrialAutoSample extends MMOpMode {

    MMRobot robotInstance;


    public TrialAutoSample() {
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
                .strafeToLinearHeading(new Vector2d(-48, -65.5), Math.toRadians(180));

        // limelight update

        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58.9, -46.8), Math.toRadians(-112.3));

        TrajectoryActionBuilder driveToScoreFirstSample = driveToIntakeFirstSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59.1, -52), Math.toRadians(-112.3));

        TrajectoryActionBuilder driveToSecondSample = driveToScoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.9, -48.5), Math.toRadians(-102.56));

        TrajectoryActionBuilder driveToIntakeThird = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.5, -47.28), Math.toRadians(-75.16));

        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -49), Math.toRadians(-100.67));

        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(67))
                .splineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(180)), Math.toRadians(0), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel));

        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -49), Math.toRadians(-100.67));

        TrajectoryActionBuilder driveToPark = driveToScoreForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(67))
                .splineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel));

        new SequentialCommandGroup(
                new InstantCommand(),
                //pre-load
                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.REST_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE),
                                        MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET)
                                ),
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SAMPLE)
                        )
                ),

                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeFirstSample.build()),
                        new WaitCommand(300).andThen(
                                ScoreHighSample()
                        )
                ),

                limelightGetter.getAlignToSampleAuto(hardwareMap, drive).withTimeout(1500),

                new ParallelCommandGroup(
                        prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.4)
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),


                new ActionCommand(driveToScoreFirstSample.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                ScoreHighSample().alongWith(
                        new WaitCommand(200).andThen(new ActionCommand(driveToSecondSample.build()))
                ),

                //second

                limelightGetter.getAlignToSampleAuto(hardwareMap, drive).withTimeout(1500),

                new ParallelCommandGroup(
                        prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.35)
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),

                new ActionCommand(driveToSecondSample.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                ScoreHighSample().alongWith(
                        new WaitCommand(200).andThen(new ActionCommand(driveToIntakeThird.build())),
                        prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.25)
                ),

                //third
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()),
                new WaitCommand(200),
                ScoreHighSample().alongWith(new ActionCommand(driveToIntakeForth.build())),

                limelightGetter.getAlignToSampleAuto(hardwareMap, drive).withTimeout(1500),

                prepareSampleIntake(),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                new WaitCommand(200),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreForth.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                ScoreHighSample().alongWith(new ActionCommand(driveToPark.build())),
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.REST_POSE)
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    private static Command ScoreHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.REST_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.REST_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SPECIMEN_POSE)
        );
    }
    private static Command prepareSampleIntake() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );
    }
}