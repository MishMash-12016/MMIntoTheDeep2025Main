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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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


public class AutoSampleNoLimelight extends MMOpMode {
    MMRobot robotInstance;

    final double halfOpenClaw = 0.7;
    final double rotator = 0;
    final double intakeArmPose = 0.59;

    public AutoSampleNoLimelight() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = new Pose2d(-39, -65.5, Math.toRadians(180));
        MMRobot.getInstance().mmSystems.localizerCurrentPose = currentPose;
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-48, -65.5), Math.toRadians(180));

        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58.9, -46.8), Math.toRadians(-112.3));
        TrajectoryActionBuilder driveToScoreFirstSample =  driveToIntakeFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.1, -52), Math.toRadians(-112.3));

        TrajectoryActionBuilder driveToSecondSample = driveToScoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.9, -48.5), Math.toRadians(-102.56));

        TrajectoryActionBuilder driveToIntakeThird = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.5, -46.28), Math.toRadians(-75.16));
        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-64.9, -49), Math.toRadians(-100.67));

        TrajectoryActionBuilder driveToPark = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(67))
                .splineToLinearHeading(new Pose2d(-24, 0, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*0.8));

        new SequentialCommandGroup(
                new InstantCommand(),
                //pre-load
                new ActionCommand(driveToScorePreloadSample.build()).alongWith(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.REST_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                ),
                                MMRobot.getInstance().mmSystems.elevator.moveToPose(Elevator.ElevatorState.HIGH_BASKET), //the height of the high basket
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SAMPLE)
                        )
                ),

                new WaitCommand(300),

                //first
                new ActionCommand(driveToIntakeFirstSample.build()).alongWith(
                        ScoreHighSample(),
                        new WaitCommand(500).andThen(
                                prepareSampleIntake(),
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.4)
                        )
                ),
                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),



                new ActionCommand(driveToScoreFirstSample.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()
                ),
                new WaitCommand(200),
                ScoreHighSample().alongWith(
                        new WaitCommand(200).andThen(new ActionCommand(driveToSecondSample.build())),
                        prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.35)
                ),

                //second

                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                ScoringSampleCommand.PrepareHighSample(),
                new WaitCommand(200),
                ScoreHighSample().alongWith(
                        new WaitCommand(200).andThen(new ActionCommand(driveToIntakeThird.build())),
                        prepareSampleIntake(),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.25)
                ),
                //third


                new WaitCommand(100),
                IntakeSampleCommand.SampleIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThird.build()).alongWith(
                        ScoringSampleCommand.PrepareHighSample()),
                new WaitCommand(200),
                ScoreHighSample(),
                new WaitCommand(200),

                //park
                new ActionCommand(driveToPark.build())
                //robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PARK_AUTO)
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }


    public static Command ScoreHighSample(){
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                new WaitCommand(300),
                //MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.MID_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.elevator.ElevatorGetToZeroSensor(),
                //MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_SPECIMEN_POSE)
        );
    }
    public static Command prepareSampleIntake() {
        return new ParallelCommandGroup(
               // MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
        );
    }
}