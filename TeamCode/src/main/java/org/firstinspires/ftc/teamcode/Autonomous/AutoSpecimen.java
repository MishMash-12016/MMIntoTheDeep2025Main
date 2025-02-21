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
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(5.5, -33), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*1.2), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel*1.2));

/*
     -----------------------
        pushing
     -----------------------
*/
        //Push first specimen
        TrajectoryActionBuilder driveToPush1 = driveToScorePreloadSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(5.5, -36), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(31, -38, Math.toRadians(235)), Math.toRadians(0));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240),null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel*0.5));
        //Push second specimen
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(160)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 0.4));
        //Push third specimen
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(50, -38, Math.toRadians(235)), Math.toRadians(80));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(50, -47, Math.toRadians(90)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 0.3));

        TrajectoryActionBuilder turnToIntake = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(50,-47.5,Math.toRadians(90)), Math.toRadians(270));
/*
     -----------------------
        intake & scoring
     -----------------------
*/

        //First specimen
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnToIntake.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(50, -59.2),Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.6));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2, -25), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 0.8))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-2, -25), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-5, -25), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (12.0 - 45.0)))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-5, -25), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        TrajectoryActionBuilder park = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(90)), Math.atan((-42.0 + 57) / (12.0 - 45)))
                .strafeToLinearHeading(new Vector2d(45, -57), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));


        new SequentialCommandGroup(
                new InstantCommand(),
                new ActionCommand(driveToScorePreloadSpecimen.build()).alongWith(
                        AutoSpecimensCommand.SpecimenScorePreLoad()),

                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),

                new ParallelCommandGroup(
                        new ActionCommand(driveToPush1.build()),
                        new WaitCommand(250).andThen(
                                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
                                robotInstance.mmSystems.scoringClawEndUnit.openScoringClaw(),
                                setupForPushing())),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),



                new ActionCommand(turnRobot.build()),
                new ActionCommand(driveToPush2.build()).alongWith(
                        setupForPushing()),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),
                robotInstance.mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),
                robotInstance.mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),

                new ActionCommand(turnRobot2.build()),
                new ActionCommand(driveToPush3.build()).alongWith(
                        robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)),
                        new WaitCommand(50),
                        setupForPushing(),


                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new WaitCommand(200),
                new ActionCommand(turnRobot3.build()).alongWith(
                        new SequentialCommandGroup(
                                new WaitCommand(900),
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                new WaitCommand(200),

                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.ROTATE_LEFT_ANGLE),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE)
                        )
                ),
                new ActionCommand(turnToIntake.build()).alongWith(
                        IntakeSpecimenCommand.PrepareSystemsSpecimenIntake()
                ),
                new WaitCommand(50),
                //First
                new ActionCommand(driveToIntakeFirstSpecimen.build()),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new WaitCommand(500).andThen(
                                new ActionCommand(driveToScoreFirstSpecimen.build())
                        )),
                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),
                //Second

                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                        new WaitCommand(1000).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new WaitCommand(500).andThen(
                                new ActionCommand(driveToScoreSecondSpecimen.build())
                        )),

                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),
                //Third

                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new WaitCommand(1000).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new WaitCommand(500).andThen(
                                new ActionCommand(driveToScoreThirdSpecimen.build())
                        )),

                //Forth
                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),

                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new WaitCommand(1000).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                        new WaitCommand(500).andThen(
                                new ActionCommand(driveToScoreForthSpecimen.build())
                        )),

                robotInstance.mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),

                new ActionCommand(park.build())
                        .alongWith(
                        new WaitCommand(1000).andThen(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_POSE),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
                                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE)
                                )
                        ))

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


}