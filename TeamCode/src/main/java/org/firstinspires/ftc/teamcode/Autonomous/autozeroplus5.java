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
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator.IntakeRotatorState;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit.ScoringClawState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow.ScoringElbowState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator.ScoringRotatorState;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class autozeroplus5 extends MMOpMode {
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


    public autozeroplus5() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, -62.73, Math.toRadians(270)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);


/*
 -----------------------
    pushing
 -----------------------
*/
        //Push first specimen
        TrajectoryActionBuilder driveToPush1 =  drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(29, -35,Math.toRadians(260)), Math.toRadians(70));
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240));
        //Push second specimen
        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(150)), Math.toRadians(250), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5));
        //Push third specimen
        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -47, Math.toRadians(90)), Math.toRadians(240), new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel));
/*
 -----------------------
    intake & scoring
 -----------------------
*/

        //First specimen
        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(48, intakePose.y, Math.toRadians(90)), Math.toRadians(270), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(scorePose, Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide, Math.toRadians(180));//side

        //Second specimen
        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(scorePose, Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide, Math.toRadians(180)); //side

        //Third specimen
        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(scorePose, Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide, Math.toRadians(180)); //side

        //Forth specimen
        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(scorePose.x, -28), Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide, Math.toRadians(180)); //side
        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(intakePose, Math.toRadians(tangentsToIntakeSpecimen));
        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                .splineToConstantHeading(new Vector2d(scorePose.x, -28), Math.toRadians(90))
                .splineToLinearHeading(dragScoredSpecimenToSide, Math.toRadians(180)); //side


        TrajectoryActionBuilder driveToPark = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                .splineToConstantHeading(new Vector2d(intakePose.x, -58), Math.toRadians(tangentsToIntakeSpecimen), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel));


        new SequentialCommandGroup(
                new InstantCommand(),

                new ActionCommand(driveToPush1.build()).alongWith(
                        new SequentialCommandGroup(
                                ScoreSpecimenCommand.ScoreSpecimen(),
                                new WaitCommand(100),
                                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
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
                IntakeSpecimenCommand.PrepareSpecimenIntakeFront().alongWith(
                        new ActionCommand(driveToIntakeFirstSpecimen.build())
                ),
                IntakeSpecimenCommand.IntakeFromFront().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreFirstSpecimen.build()))),


                //Second
                ScoreSpecimenCommand.ScoreSpecimen(),
                new ActionCommand(driveToIntakeSecondSpecimen.build()).alongWith(
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                ),

                new WaitCommand(200),
                IntakeSpecimenCommand.IntakeFromFront().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreSecondSpecimen.build()))),

                //Third
                new ActionCommand(driveToIntakeThirdSpecimen.build()).alongWith(
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                ),

                new WaitCommand(200),
                IntakeSpecimenCommand.IntakeFromFront().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreThirdSpecimen.build()))),

                //Forth
                new ActionCommand(driveToIntakeForthSpecimen.build()).alongWith(
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                ),

                new WaitCommand(200),
                IntakeSpecimenCommand.IntakeFromFront().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreForthSpecimen.build()))),

                //Fifth
                new ActionCommand(driveToIntakeFifthSpecimen.build()).alongWith(
                                IntakeSpecimenCommand.PrepareSpecimenIntakeFront()
                ),

                new WaitCommand(200),
                IntakeSpecimenCommand.IntakeFromFront().alongWith(
                        new WaitCommand(400).andThen(
                                new ActionCommand(driveToScoreFifthSpecimen.build()))),

                //park
                new ActionCommand(driveToPark.build())
                        .alongWith(
                                new WaitCommand(100).andThen(ScoreSpecimenCommand.ScoreSpecimen()).andThen(
                                        new WaitCommand(300).andThen(
                                                new ParallelCommandGroup(
                                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE),
                                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.TRANSFER_SPECIMEN_POSE),
                                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                                                        MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.SPECIMEN_TRANSFER_POSE)
                                                )
                                        )
                                ))

        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
    }

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                robotInstance.mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)

        );
    }

}
