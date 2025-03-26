package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoOnePlusFiveChangingLamLam extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;

    //parking position
    private static final double tangentsToScoreSpecimen = 135;
    private static final double tangentsToIntakeSpecimen = 310;
    final Pose2d intakePose = new Pose2d(40, -66, Math.toRadians(90));
    static final Pose2d scorePose = new Pose2d(8, -35, Math.toRadians(120));
    final boolean RedSample= false;

    public AutoOnePlusFiveChangingLamLam() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.vision.trackRed();

        Pose2d currentPose = (new Pose2d(5.5, -61.23, Math.toRadians(270)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.5, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToEject = driveToScorePreload.endTrajectory().fresh()
                .setTangent(Math.toRadians(260))
                .splineToSplineHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.5, MecanumDrive.PARAMS.maxProfileAccel));

        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(29, -35, Math.toRadians(230)), Math.toRadians(50), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -51, Math.toRadians(150)), Math.toRadians(250), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5));

        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -51, Math.toRadians(150)), Math.toRadians(260), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5));

        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -53, Math.toRadians(90)), Math.toRadians(270), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5));

        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(51,-66, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140+20))
                .splineToSplineHeading(new Pose2d(9,-34,Math.toRadians(120)), Math.toRadians(tangentsToScoreSpecimen-20), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(scorePose, Math.toRadians(tangentsToScoreSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(intakePose.position.x, intakePose.position.y + 2, intakePose.heading.toDouble()), Math.toRadians(310), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        TrajectoryActionBuilder driveToIntakeYellowSample = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(18, -45, Math.toRadians(130)), Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreYellowSample = driveToIntakeYellowSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(230));
        new SequentialCommandGroup(
                new InstantCommand(),
                new ParallelCommandGroup(
                        new ActionCommand(driveToScorePreload.build()),
                        AutoSpecimensCommand.SpecimenScorePreLoad(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
                ),
                new SequentialCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.AFTER_SCORING_FRONT_SPECIMEN_POSE),
                        new WaitCommand(50),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.AFTER_SCORING_FRONT_SPECIMEN_POSE),
                        new WaitCommand(200),
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                ).alongWith(IntakeSampleCommand.limeLightIntake_Auto(hardwareMap, drive)),
                new ActionCommand(driveToEject.build()).alongWith(
                        new WaitCommand(700).andThen(
                                ThrowSample()
                        )
                ),


                //push first
                new ActionCommand(driveToPush1.build()).alongWith(
                        new WaitCommand(200).andThen(setupForPushing())
                ),
                new ActionCommand(turnRobot.build()).alongWith(
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose)
                ),
                new ActionCommand(driveToPush2.build()).alongWith(
                        setupForPushing()),

                //push second
                new ActionCommand(turnRobot2.build()).alongWith(
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose)
                ),
                new ActionCommand(driveToPush3.build()).alongWith(
                        setupForPushing()),

                //push third
                new ActionCommand(turnRobot3.build()).alongWith(
                        new SequentialCommandGroup(
                                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                                new WaitCommand(550),
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_LEFT_ANGLE),
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.INIT_POSE),
                                new WaitCommand(100),
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                        )
                ),
                //First
                new ActionCommand(driveToIntakeFirstSpecimen.build()),
                IntakeSpecimenCommand.IntakeFromFrontToSide().alongWith(
                        new WaitCommand(50).andThen(
                                new ActionCommand(driveToScoreFirstSpecimen.build()))),


                //Second
                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreFromTheSide(),
                                new WaitCommand(300),
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                        )
                ),

                IntakeSpecimenCommand.IntakeFromFrontToSide().alongWith(
                        new WaitCommand(50).andThen(
                                new ActionCommand(driveToScoreSecondSpecimen.build()))),

                //Third
                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreFromTheSide(),
                                new WaitCommand(200),
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                        )
                ),

                IntakeSpecimenCommand.IntakeFromFrontToSide().alongWith(
                        new WaitCommand(50).andThen(
                                new ActionCommand(driveToScoreThirdSpecimen.build()))),

                //Forth
                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreFromTheSide(),
                                new WaitCommand(200),
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                        )
                ),

                IntakeSpecimenCommand.IntakeFromFrontToSide().alongWith(
                        new WaitCommand(50).andThen(
                                new ActionCommand(driveToScoreForthSpecimen.build()))),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                        new ActionCommand(driveToIntakeFifthSpecimen.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreFromTheSide(),
                                new WaitCommand(200),
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                        )
                ),
                        IntakeSpecimenCommand.IntakeFromFrontToSide().alongWith(
                                new WaitCommand(50).andThen(
                                        new ActionCommand(driveToScoreFifthSpecimen.build()))),

                        //park
                        new ActionCommand(driveToPark.build()).alongWith(
                                new SequentialCommandGroup(
                                        ScoreFromTheSide(),
                                        new WaitCommand(200),
                                        IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                                ))),
                        new SequentialCommandGroup(
                        new ActionCommand(driveToIntakeYellowSample.build()).alongWith(
                                new SequentialCommandGroup(
                                        ScoreFromTheSide(),
                                        new WaitCommand(200),
                                        IntakeSampleCommand.prepareSampleIntake(()-> false,()-> false),
                                        new WaitCommand(200),
                                        IntakeSampleCommand.SampleIntake()
                                )),
                        new ActionCommand(driveToScoreYellowSample.build()).alongWith(
                                new WaitCommand(300).andThen(ScoringSampleCommand.PrepareHighSample())
                        ),
                        ScoringSampleCommand.ScoreHighSample())
                        ,()-> RedSample)

                //fifth


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

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
        );
    }

    private static Command score() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(0.7),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(0.15)
                ),
                new WaitCommand(200),

                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
    }

    public static Command FirstSampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                )
        );
    }

    public static Command ThrowSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE),//be prepared for transfer
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                new WaitCommand(400).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );
    }

    public static Command ScoreFromTheSide() {

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.AFTER_SCORING_FRONT_SPECIMEN_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.AFTER_SCORING_FRONT_SPECIMEN_POSE)),
                new WaitCommand(125),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
        );
    }
}
