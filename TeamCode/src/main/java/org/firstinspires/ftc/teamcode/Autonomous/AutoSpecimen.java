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
import org.firstinspires.ftc.teamcode.CommandGroup.ScoreSpecimenCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit.ScoringClawState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow.ScoringElbowState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoSpecimen extends MMOpMode {
    static MMRobot robotInstance;

    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;

    //parking position
    private static final Pose2d dragScoredSpecimenToSide = new Pose2d(-1,-30,Math.toRadians(90)); //side
    private static final double tangentsToScoreSpecimen = 170;
    private static final double tangentsToIntakeSpecimen = 300;
    private static final  Vector2d intakePose = new Vector2d(43, -60);
    private static final  Vector2d scorePose = new Vector2d(4, -32);



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
                .splineToConstantHeading(new Vector2d(0, -27), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));

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
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(150)), Math.toRadians(240), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        //Push second specimen
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(150)), Math.toRadians(250), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5));
        //Push third specimen
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel ));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -47, Math.toRadians(90)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel ));
/*
     -----------------------
        intake & scoring
     -----------------------
*/

        //First specimen
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(48, intakePose.y, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel ));

        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(scorePose,Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide,Math.toRadians(180));//side

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose,Math.toRadians(tangentsToIntakeSpecimen));

        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(scorePose,Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide,Math.toRadians(180)); //side

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose,Math.toRadians(tangentsToIntakeSpecimen));

        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(scorePose,Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide,Math.toRadians(180)); //side

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose,Math.toRadians(tangentsToIntakeSpecimen));

        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(scorePose.x, -28),Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide,Math.toRadians(180)); //side


        TrajectoryActionBuilder driveToPark = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(intakePose.x, -58),Math.toRadians(tangentsToIntakeSpecimen),new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel));


        //Executing the paths
        new SequentialCommandGroup(
                new InstantCommand(),
                AutoSpecimensCommand.SpecimenScorePreLoad().alongWith(
                        new WaitCommand(200).andThen(
                                new ActionCommand(driveToScorePreloadSpecimen.build())
                        )
                ),

                new ActionCommand(driveToPush1.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreSpecimenCommand.ScoreSpecimen(),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose)
                                ),
                                setupForPushing()
                        )
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
                                robotInstance.mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.ROTATE_LEFT_ANGLE),
                                robotInstance.mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE)
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
                ScoreSpecimenCommand.ScoreSpecimen(),
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
                ScoreSpecimenCommand.ScoreSpecimen(),
                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),
//
                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                        new WaitCommand(400).andThen(
                        new ActionCommand(driveToScoreThirdSpecimen.build()))),

                //Forth
                ScoreSpecimenCommand.ScoreSpecimen(),
                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                        new WaitCommand(800).andThen(
                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())),

                new WaitCommand(200),
                IntakeSpecimenCommand.SpecimenIntake().alongWith(
                    new WaitCommand(400).andThen(
                        new ActionCommand(driveToScoreForthSpecimen.build()))),

                //park
                new ActionCommand(driveToPark.build())
                        .alongWith(
                                new WaitCommand(100).andThen(ScoreSpecimenCommand.ScoreSpecimen()).andThen(
                                    new WaitCommand(300).andThen(
                                            new ParallelCommandGroup(
                                                    MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                                                    MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                                    MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                                                    MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.TRANSFER_SPECIMEN_POSE),
                                                    MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeRotatorState.DEFAULT_POSE),
                                                    MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringRotatorState.SPECIMEN_TRANSFER_POSE)
                                        )
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
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
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