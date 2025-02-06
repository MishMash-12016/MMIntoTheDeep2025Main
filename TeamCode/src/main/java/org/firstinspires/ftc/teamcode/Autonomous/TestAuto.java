package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class TestAuto extends MMOpMode {
    MMRobot robotInstance;

    public TestAuto() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -65.5, Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);


        //poses for pushing
        final double halfOpenClaw = 0.7;
        final double rotator = 0;
        final double intakeArmPose = 0.59;


        //Score pre-load
        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(5.5, -30), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel + 20));

/*
     -----------------------
        pushing
     -----------------------
*/
        //Push first specimen
        TrajectoryActionBuilder driveToPush1 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToSplineHeading(new Pose2d(29.8, -38, Math.toRadians(235)), Math.toRadians(0));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(32.8, -53, Math.toRadians(120)), Math.toRadians(240), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.8));

        //Push second specimen
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(38.1, -38, Math.toRadians(235)), Math.toRadians(70));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(39.5, -53, Math.toRadians(120)), Math.toRadians(240), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.8));

        //Push third specimen
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(47, -38, Math.toRadians(235)), Math.toRadians(70));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(280))
                .splineToLinearHeading(new Pose2d(47, -53, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.8));

/*
     -----------------------
        intake & scoring
     -----------------------
*/

        //First specimen
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(47, -64, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.6));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-6, -28.5), Math.toRadians(90));

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -61), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-8, -28.5), Math.toRadians(90));

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -56.5), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10, -28.5), Math.toRadians(90));

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -55.3), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-12, -28.5), Math.toRadians(90));

        //park
        TrajectoryActionBuilder driveToPark = driveToScoreForthSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(90));


        new SequentialCommandGroup(
                new InstantCommand(),

                new ActionCommand(driveToScorePreloadSpecimen.build()).alongWith(
                        ScoringSpecimanCommand.SpecimanScore()),

                new ParallelCommandGroup(
                        new ActionCommand(driveToPush1.build()),
                        new WaitCommand(800).andThen(
                                new ParallelCommandGroup(
                                        robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                                        robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                        robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                                        robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
                                )
                        )
                ),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),

                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),

                new ActionCommand(turnRobot.build()),
                new ActionCommand(driveToPush2.build()).alongWith(
                        new ParallelCommandGroup(
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw))),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),

                new WaitCommand(200),
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),

                new ActionCommand(turnRobot2.build()),
                new ActionCommand(driveToPush3.build()).alongWith(
                        new ParallelCommandGroup(
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw))),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),
                new ActionCommand(turnRobot3.build()),
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),

                //First
                new ActionCommand(driveToIntakeFirstSpecimen.build()).alongWith(
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
                                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw())),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        IntakeSpecimansCommand.SpecimenIntake(),
                        new WaitCommand(200).andThen(
                                new ActionCommand(driveToScoreFirstSpecimen.build()))),

                //Second
                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeSecondSpecimen.build()),
                        new WaitCommand(1200).andThen(
                                new SequentialCommandGroup(
                                    MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                    MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                    MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                                    MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                    MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
                                    MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                                )
                        )
                ),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        IntakeSpecimansCommand.SpecimenIntake(),
                        new WaitCommand(200).andThen(
                                new ActionCommand(driveToScoreSecondSpecimen.build()))),

                //Third
                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeThirdSpecimen.build()),
                        new WaitCommand(1200).andThen(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                                )
                        )
                ),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        IntakeSpecimansCommand.SpecimenIntake(),
                        new WaitCommand(200).andThen(
                                new ActionCommand(driveToScoreThirdSpecimen.build()))),


                //Forth
                new ParallelCommandGroup(
                        new ActionCommand(driveToIntakeForthSpecimen.build()),
                        new WaitCommand(1200).andThen(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                                )
                        )
                ),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        IntakeSpecimansCommand.SpecimenIntake(),
                        new WaitCommand(200).andThen(
                                new ActionCommand(driveToScoreForthSpecimen.build()))),


                //park
                new ActionCommand(driveToPark.build()).andThen(
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE)
                )


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

