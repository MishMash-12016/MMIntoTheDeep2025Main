package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHere;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.driveToScoreFirstSpecimen;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToIntake;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToPark;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToScore;
import org.firstinspires.ftc.teamcode.CommandGroup.touchSensors;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
import org.firstinspires.ftc.teamcode.utils.ParallelCommandGroupNoCheck;

@Config
@Autonomous
public class Blue_Right_6 extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.65;
    static final double rotator = 0;
    final double intakeArmPose = 0.61;

    //parking position
    public static final double tangentsToIntakeSpecimen = 310;
    public static final Pose2d intakePose = new Pose2d(40, -71, Math.toRadians(90));
    public static final Pose2d scorePose = new Pose2d(2, -28, Math.toRadians(90));


    public Blue_Right_6() {
        super(OpModeType.Competition.AUTO);
    }

    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems(this);
        MMRobot.getInstance().mmSystems.vision.trackBlue();

        Pose2d currentPose = new Pose2d(5.5, -61.23, Math.toRadians(PinpointDrive._autoStartAngle = 270));
//        PinpointDrive._autoStartAngle -= 90;
        MMSystems.localizer = null;
        MMRobot.getInstance().mmSystems.initLocalize(currentPose);
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;
        drive.pinpoint.setPosition(currentPose);
        drive.pinpoint.update();

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);


        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(4, -28), Math.toRadians(90),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.4));


        //there isn't driveToEject its all conspiracy
        TrajectoryActionBuilder driveToEject = drive.actionBuilder(new Pose2d(5.5, -29, Math.toRadians(270)))
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(26.5, -46, Math.toRadians(150)), Math.toRadians(0),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));


        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(28.5, -37, Math.toRadians(225)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2)
                );
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(31, -47), Math.toRadians(140),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.5, MecanumDrive.PARAMS.maxProfileAccel * 1.6));

        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38.3, -37, Math.toRadians(225)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38.7, -45), Math.toRadians(130),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.5, MecanumDrive.PARAMS.maxProfileAccel * 1.6));

        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(47, -34, Math.toRadians(210)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(47, -47, Math.toRadians(90)), Math.toRadians(270),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2))
                .splineToLinearHeading(new Pose2d(47, -75, Math.toRadians(90)), Math.toRadians(270),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel*0.9));

//        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
//
//                        null,
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .strafeToConstantHeading(intakePose.component1(),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.8, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .strafeToConstantHeading(intakePose.component1(),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.8, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .strafeToConstantHeading(intakePose.component1(),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.8, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .strafeToConstantHeading(intakePose.component1(),
                        null,
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.8, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(28, -50, Math.toRadians(140)), Math.toRadians(140 + 180),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel *1.5, MecanumDrive.PARAMS.maxProfileAccel * 1.5));

        FtcDashboard.getInstance().getTelemetry().addData("rotatorAngle", 0);
        FtcDashboard.getInstance().getTelemetry().addData("robotAngle", 0);
        FtcDashboard.getInstance().getTelemetry().update();

        new SequentialCommandGroup(
                new InstantCommand(() -> drive.pinpoint.setPosition(currentPose)),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new ParallelCommandGroup(
                        new ActionCommand(driveToScorePreload.build()) {
                            @Override
                            public void end(boolean interrupted) {
                                super.end(interrupted);
                                if (interrupted) {
                                    FtcDashboard.getInstance().getTelemetry().addLine("interrupted the preload");
                                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                                }
                            }
                        }.interruptOn(touchSensors::getStateIntake).andThen(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                        ),
                        AutoSpecimensCommand.PrepareSpecimenScorePreLoad(),
                        new WaitCommand(300).andThen(
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
                        ),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ),

                IntakeSampleCommand.limeLightIntake_Auto_for_specimen().withTimeout(4000),

                new ParallelCommandGroupNoCheck(
                        new runFromHere(),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.34),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                        new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().heading.toDouble() < Math.toRadians(-135)).andThen(
                                ThrowSample()
                        )
                ),
                //push first
                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                new ActionCommand(driveToPush1.build()).alongWith(
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE).andThen(
                                new WaitUntilCommand(() -> getAng() >= 190).andThen(
                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING)
                                )
                        ),
                        new SequentialCommandGroup(

                                new WaitCommand(200).andThen(setupForPushing())
                        )
                ),
                new ActionCommand(turnRobot.build()).alongWith(
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                        new WaitUntilCommand(() -> getAng() <= 160).andThen(
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.15)
                        )
                ),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                new ActionCommand(driveToPush2.build()).alongWith(
                        new WaitUntilCommand(() -> getAng() >= 190).andThen(
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING)
                        ),
                        setupForPushing()
                ),

                //push second
                new ActionCommand(turnRobot2.build()).alongWith(
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                        new WaitUntilCommand(() -> getAng() <= 160).andThen(
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.08)
                        )
                ),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                new ActionCommand(driveToPush3.build()).alongWith(
                        new WaitUntilCommand(() -> getAng() >= 180).andThen(
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING)
                        ),
                        setupForPushing()
                ),
                //push third
                new ParallelCommandGroup(
                        new ActionCommand(turnRobot3.build()) {
                            public void end(boolean interrupted) {
                                super.end(interrupted);
                                if (interrupted) {
                                    FtcDashboard.getInstance().getTelemetry().addLine("interrupted the intake first");
                                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                                }
                            }
                        },
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose).andThen(
                                new WaitUntilCommand(() -> getAng() <= 190).andThen(
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.15),
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.SCORE_SAMPLE_POSE)
                                        )
                                ),
                                new WaitUntilCommand(() -> getAng() <= 175).andThen(
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoringArmPrepareSampleTransferPose),
                                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE),
                                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.INIT_POSE)
                                        ),
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                        )
                                ))
                ).interruptOn(
                        () -> (touchSensors.getStateIntake() && MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().heading.toDouble() > Math.toRadians(45))),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(75),
                new driveToScoreFirstSpecimen() {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the score first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateScoring).alongWith(
                                IntakeSpecimenCommand.SpecimenIntakeAuto()
                        ),

                //Second
                new runFromHereToIntake() {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the intake second");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateIntake)
                        .alongWith(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                                        ),
                                        new WaitCommand(200),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        ),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                new runFromHereToScore(scorePose.plus(new Twist2d(new Vector2d(0.5,0),0))) {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the score second");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateScoring).alongWith(
                                IntakeSpecimenCommand.SpecimenIntakeAuto()
                        ),

                //Third
                new runFromHereToIntake() {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the intake third");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateIntake)
                        .alongWith(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                                        ),
                                        new WaitCommand(200),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        ),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                new runFromHereToScore(scorePose.plus(new Twist2d(new Vector2d(1,0),0))) {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the score third");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateScoring).alongWith(
                                IntakeSpecimenCommand.SpecimenIntakeAuto()
                        ),

                //Forth
                new runFromHereToIntake() {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the intake forth");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateIntake)
                        .alongWith(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                                        ),
                                        new WaitCommand(200),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        ),

                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                new runFromHereToScore(scorePose.plus(new Twist2d(new Vector2d(1.5,0),0))) {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the score forth");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateScoring).alongWith(
                                IntakeSpecimenCommand.SpecimenIntakeAuto()
                        ),

                //fifth
                new runFromHereToIntake() {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the intake fifth");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateIntake)
                        .alongWith(
                                new SequentialCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                        new WaitCommand(100),
                                        new ParallelCommandGroup(
                                                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw()
                                        ),
                                        new WaitCommand(200),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        ),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw(),
                new WaitCommand(100),
                new runFromHereToScore(scorePose.plus(new Twist2d(new Vector2d(2,0),0))) {
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if (interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the score fifth");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                        }
                    }
                }
                        .interruptOn(touchSensors::getStateScoring).alongWith(
                                IntakeSpecimenCommand.SpecimenIntakeAuto()
                        ),

                //park
                new runFromHereToPark().alongWith(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.4)
                )
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMSystems.AutoPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor back", touchSensors.getStateScoring());
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor front", touchSensors.getStateIntake());
        FtcDashboard.getInstance().getTelemetry().addData("ang of the big robot for ori", getAng());
        FtcDashboard.getInstance().getTelemetry().addData("a", getAng() >= Math.toRadians(200));

        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void reset() {
        super.reset();

    }

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                //MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                robotInstance.mmSystems.intakeArm.setPosition(0.66),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
        );
    }

    public static Command ThrowSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE),//be prepared for transfer
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE)
        );
    }

    public static double getAng() {
        double ang = Math.toDegrees(MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().heading.toDouble());
        ang = ang < 0 ? ang + 360 : ang;
        return ang;
    }
}
