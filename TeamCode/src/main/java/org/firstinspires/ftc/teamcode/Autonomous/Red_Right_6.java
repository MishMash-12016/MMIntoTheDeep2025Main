package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHere;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.driveToScoreFirstSpecimen;
import org.firstinspires.ftc.teamcode.CommandGroup.touchSensors;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
import org.firstinspires.ftc.teamcode.utils.ParallelCommandGroupNoCheck;

@Config
@Autonomous
public class Red_Right_6 extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.59;

    //parking position
    private static final double tangentsToScoreSpecimen = 135;
    public static final double tangentsToIntakeSpecimen = 310;
    final Pose2d intakePose = new Pose2d(40, -65.5, Math.toRadians(90));
    public static final Pose2d scorePose = new Pose2d(5, -30, Math.toRadians(90));


    public Red_Right_6() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        MMRobot.getInstance().mmSystems.vision.trackRed();
        MMRobot.getInstance().mmSystems.vision.switchToDetector();

        Pose2d currentPose = (new Pose2d(5.5, -61.23, Math.toRadians(270)));
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);





        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.4));


        //there isn't driveToEject its all conspiracy
        TrajectoryActionBuilder driveToEject = drive.actionBuilder(new Pose2d(5.5, -29, Math.toRadians(270)))
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));


        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(28, -37, Math.toRadians(225)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2)
                );
        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(28, -45), Math.toRadians(140),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.5, MecanumDrive.PARAMS.maxProfileAccel*1.6));

        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38, -37, Math.toRadians(225)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(38, -45), Math.toRadians(140),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.5, MecanumDrive.PARAMS.maxProfileAccel*1.6));

        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(46.5, -34, Math.toRadians(210)), Math.toRadians(50),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(46.5, -51, Math.toRadians(90)), Math.toRadians(270),
                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.2, MecanumDrive.PARAMS.maxProfileAccel ));

        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(48.5, -66.6, Math.toRadians(90)), Math.toRadians(270),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(scorePose, Math.toRadians(100));

        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(28, -50, Math.toRadians(140)), Math.toRadians(140+180),
                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

        FtcDashboard.getInstance().getTelemetry().addData("rotatorAngle", 0);
        FtcDashboard.getInstance().getTelemetry().addData("robotAngle", 0);
        FtcDashboard.getInstance().getTelemetry().update();

        new SequentialCommandGroup(
                new InstantCommand(),
                new ParallelCommandGroup(
                        new ActionCommand(driveToScorePreload.build()),
                        AutoSpecimensCommand.PrepareSpecimenScorePreLoad(),
                        new WaitCommand(300).andThen(
                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE)
                        )
                ),


                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                        IntakeSampleCommand.limeLightIntake_Auto_for_specimen().withTimeout(5000)
                ),


                new ParallelCommandGroupNoCheck(
                        new runFromHere(),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(0.4),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                                )
                        ),
                        new WaitCommand(700).andThen(
                                ThrowSample()
                        )
                ),
                //push first
                new ActionCommand(driveToPush1.build())
                        .alongWith(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                                ),
                                new WaitCommand(200).andThen(setupForPushing())
                        )
                ),
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new ActionCommand(turnRobot.build()).alongWith(
                        new WaitCommand(300).andThen(
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.08)
                        )
                ),
                new ActionCommand(driveToPush2.build()).alongWith(
                        setupForPushing()
                        ),

                //push second
                robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                new ActionCommand(turnRobot2.build()).alongWith(
                        new WaitCommand(300).andThen(
                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(0.08)
                        )
                ),
                new ActionCommand(driveToPush3.build()).alongWith(
                        setupForPushing()
                ),
                //push third
                new ParallelCommandGroupNoCheck(
                        new ActionCommand(turnRobot3.build()),
                        robotInstance.mmSystems.intakeArm.setPosition(intakeArmPose),
                        new WaitCommand(750).andThen(
                                new ParallelCommandGroup(

                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.scoringArmPrepareSampleTransferPose),
                                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INIT_POSE),
                                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.INIT_POSE)
                                )
                        )
                )
                ,
                //First
                new ActionCommand(driveToIntakeFirstSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateArm())
                        .alongWith(IntakeSpecimenCommand.PrepareSpecimenIntakeFront()),

                new driveToScoreFirstSpecimen(){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateBumper()).alongWith(
                        IntakeSpecimenCommand.SpecimenIntake()
                ),
                //Second
                new ActionCommand(driveToIntakeSecondSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateArm())
                        .alongWith(
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(300),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        )
                ),

                new ActionCommand(driveToScoreSecondSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateBumper()).alongWith(
                        IntakeSpecimenCommand.SpecimenIntake()
                ),

                //Third
                new ActionCommand(driveToIntakeThirdSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateArm())
                        .alongWith(
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        )
                ),

                new ActionCommand(driveToScoreThirdSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateBumper()).alongWith(
                        IntakeSpecimenCommand.SpecimenIntake()
                ),

                //Forth
                new ActionCommand(driveToIntakeForthSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateArm())
                        .alongWith(
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(300),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        )
                ),


                new ActionCommand(driveToScoreForthSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateBumper()).alongWith(
                        IntakeSpecimenCommand.SpecimenIntake()
                ),

                //fifth
                new ActionCommand(driveToIntakeFifthSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateArm())
                        .alongWith(
                        new SequentialCommandGroup(
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                                new WaitCommand(300),
                                new ParallelCommandGroup(
                                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                                        MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw()
                                )
                        )
                ),

                new ActionCommand(driveToScoreFifthSpecimen.build()){
                    @Override
                    public void end(boolean interrupted) {
                        super.end(interrupted);
                        if(interrupted) {
                            FtcDashboard.getInstance().getTelemetry().addLine("interrupted the first");
                            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                        }
                    }
                }
                        .interruptOn(()-> touchSensors.getStateBumper()).alongWith(
                        IntakeSpecimenCommand.SpecimenIntake()
                ),

                //park
                new ActionCommand(driveToPark.build()).alongWith(
                        IntakeSampleCommand.prepareSampleIntakeWithoutButton()
                )
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("----------------------------", "");
        FtcDashboard.getInstance().getTelemetry().addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor bumber", touchSensors.getStateBumper());
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor arm", touchSensors.getStateArm());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                robotInstance.mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                robotInstance.mmSystems.intakeArm.setPosition(0.54),
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


    public static Command PrepareSpecimenIntakeFront() {
        return new SequentialCommandGroup(
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.INTAKE_FROM_FRONT_POSE),
                        MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.INTAKE_FROM_FRONT_POSE)
                )
        );
    }
}
