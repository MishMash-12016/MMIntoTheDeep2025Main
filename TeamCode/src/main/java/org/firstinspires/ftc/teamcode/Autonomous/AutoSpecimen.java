package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
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

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoSpecimen extends MMOpMode {
    static MMRobot robotInstance;

    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;
    private static final Pose2d collectionSpecimanPos = new Pose2d(42, -67, Math.toRadians(90));

    private static final Vector2d scoreSpecimanPos = new Vector2d(5.5, -27); // when driving to scoring location
    private static final Vector2d scoreSpecimanPos2 = new Vector2d(5.5, -40); // when driving backwards to score
    private static final int scoreSpecimanXConst = 3; // the spacing of the scored specimens
    private static final Pose2d pushSamplePos = new Pose2d(33.24, -38, Math.toRadians(230));
    private static final float pushingSampleXConst = 10; // the spacing between the samples
    /*
     * a adjustive const for each sample might be needed
     */

    //parking position
    private static final Pose2d parkPos = new Pose2d(45, -60, Math.toRadians(90));


    public AutoSpecimen() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -62.73, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);


        //poses for pushing


        //Score pre-load
        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(5.5, -32), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));

/*
     -----------------------
        pushing
     -----------------------
*/
        //Push first specimen
        TrajectoryActionBuilder driveToPush1 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(29, -35), Math.toRadians(50));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        //Push second specimen
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(160)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel ));
        //Push third specimen
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(48, -38, Math.toRadians(235)), Math.toRadians(80), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(48, -47, Math.toRadians(90)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel ));

//        TrajectoryActionBuilder turnToIntake = turnRobot3.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(48, -47.5, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
/*
     -----------------------
        intake & scoring
     -----------------------
*/
        double tangentsToScoreSpecimen = 170;
        double tangentsToIntakeSpecimen = 300;

        //First specimen
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(48, -58, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(90));

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(90));

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(90));

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(90));


        TrajectoryActionBuilder driveToPark = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen));


        new SequentialCommandGroup(
                new InstantCommand(),
                new ActionCommand(driveToScorePreloadSpecimen.build()).alongWith(
                        AutoSpecimensCommand.SpecimenScorePreLoad()),


                new ActionCommand(driveToPush1.build()).alongWith(
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SPECIMEN_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SCORE_POSE),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SCORE_SPECIMEN),
                        robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.COMPLETELYOPEN).andThen(
                                setupForPushing())),
                new ParallelCommandGroup(
                        robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                        robotInstance.mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_POSE),
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose)
                ),
                new WaitCommand(700),


                new ActionCommand(turnRobot.build()),
                new ActionCommand(driveToPush2.build()).alongWith(
                        setupForPushing()),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                robotInstance.mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.TRANSFER_POSE),

                new ActionCommand(turnRobot2.build()),
                new ActionCommand(driveToPush3.build()).alongWith(
                        setupForPushing()),

                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),
                new ActionCommand(turnRobot3.build()).alongWith(
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_LEFT_ANGLE),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE)
                        )
                ),
                //First
                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake().alongWith(
                                new ActionCommand(driveToIntakeFirstSpecimen.build())
                        ),

                new WaitCommand(200).andThen(
                        IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400))),
                        new ActionCommand(driveToScoreFirstSpecimen.build()),

////
//
//                //Second
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
//
                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreSecondSpecimen.build()),
//
//                //Third
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
//
                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreThirdSpecimen.build()
                ),

                //Forth
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake(),
                new WaitCommand(200),
                new ActionCommand(driveToScoreForthSpecimen.build()),

                //park
                new ActionCommand(driveToPark.build())
                        .alongWith(
                                new WaitCommand(800).andThen(
                                        new SequentialCommandGroup(
                                                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.OPEN),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE)
                                        )
                                )
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

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.maxOpening),
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