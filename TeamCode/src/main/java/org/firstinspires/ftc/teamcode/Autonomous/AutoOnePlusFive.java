package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoOnePlusFive extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;

    //parking position
    private static final Pose2d dragScoredSpecimenToSide = new Pose2d(-1, -30, Math.toRadians(90)); //side
    private static final double tangentsToScoreSpecimen = 170;
    private static final double tangentsToIntakeSpecimen = 300;
    private static final Vector2d intakePose = new Vector2d(43, -60);
    private static final Vector2d scorePose = new Vector2d(4, -32);

    public AutoOnePlusFive() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -61.23, Math.toRadians(270)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5.5, -27), Math.toRadians(90));
        TrajectoryActionBuilder driveToEject = driveToScorePreload.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(22, -45, Math.toRadians(325 + 180)), Math.toRadians(340));

        TrajectoryActionBuilder driveToPush1 = driveToScorePreload.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(29, -35, Math.toRadians(230)), Math.toRadians(50));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(150)), Math.toRadians(240));
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(150)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -53, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = driveToPush3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(51, -60, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7, -30, Math.toRadians(120)), Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)), Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7, -30, Math.toRadians(120)), Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)), Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7, -30, Math.toRadians(120)), Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)), Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7, -30, Math.toRadians(120)), Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)), Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7, -30, Math.toRadians(120)), Math.toRadians(135));

        new SequentialCommandGroup(
                new InstantCommand(),
                AutoSpecimensCommand.SpecimenScorePreLoad().alongWith(
                        new WaitCommand(200).andThen(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                                        new ActionCommand(driveToScorePreload.build())
                                )
                        )),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.AFTER_SCORING_FRONT_SPECIMEN_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.AFTER_SCORING_FRONT_SPECIMEN_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                IntakeSampleCommand.limeLightIntake_Auto(hardwareMap, drive),
                new ActionCommand(driveToEject.build()),
                IntakeSampleCommand.prepareSampleIntake(()->false,()-> false),
                new ActionCommand(driveToPush1.build()).alongWith(
                        setupForPushing()
                        ),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(50),
                new ActionCommand(turnRobot.build()),
                new ActionCommand(driveToPush2.build()).alongWith(
                        setupForPushing()),

                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(50),

                new ActionCommand(turnRobot2.build()),
                new ActionCommand(driveToPush3.build()).alongWith(
                        setupForPushing()),

                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(50),
                new ActionCommand(turnRobot3.build()).alongWith(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_LEFT_ANGLE),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE)
                        )
                ),
                //First
                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake().alongWith(
                        new ActionCommand(driveToIntakeFirstSpecimen.build())
                ),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreFirstSpecimen.build()))),

////
//
//                //Second
                ScoreSpecimenCommand.ScoreSpecimenAuto(),
                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
//
                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreSecondSpecimen.build()))),
//
//                //Third
                ScoreSpecimenCommand.ScoreSpecimenAuto(),
                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
//
                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreThirdSpecimen.build()))),

                //Forth
                ScoreSpecimenCommand.ScoreSpecimenAuto(),
                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreForthSpecimen.build()))),

                new WaitCommand(200),
                new ActionCommand(driveToIntakeFifthSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreFifthSpecimen.build())))
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

}
