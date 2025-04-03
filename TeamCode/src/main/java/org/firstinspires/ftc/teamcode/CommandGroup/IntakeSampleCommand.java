package org.firstinspires.ftc.teamcode.CommandGroup;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.limelightvision.LLResult;

//import org.firstinspires.ftc.teamcode.Autonomous.AutoOnePlusFiveRightRed;
import org.firstinspires.ftc.teamcode.CommandGroup.limelight.limelightGetter;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm.IntakeArmState;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake.LinearIntakeState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm.ScoringArmState;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utils.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utils.geometry.Translation2d;

import java.util.function.BooleanSupplier;

public class IntakeSampleCommand {
    public static Command prepareSampleIntake(BooleanSupplier rotateRightButton, BooleanSupplier rotateLeftButton) {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),//be prepared for transfer
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.rotateByButton(rotateLeftButton, rotateRightButton),
                new WaitCommand(70).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                )
        );
    }

    public static Command prepareSampleIntakeWithoutButton() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),

                new WaitCommand(70).andThen(
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw()
                ));
    }

    public static Command SampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SPECIMEN_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command SampleIntakeLowExit() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.SAMPLE_INTAKE_POSE),
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArmState.ARM_PREPARE_SAMPLE_TRANSFER_POSE),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command limeLightIntake_TeleOp() {
        return new FixedSequentialCommandGroup(
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd", "enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd2", "not enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd3", "not enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd4", "not enderd")),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.trackRedDetector()),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd2", "enderd")),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == 0),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd3", "enderd")),

                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),

                //Lamlam side:
                new FixedSequentialCommandGroup(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                        MMRobot.getInstance().mmSystems.vision.onlyAngleChange(),
                        limelightGetter.getRotateToSample(),

                        limelightGetter.strafeToSample().alongWith(
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING))

//                        limelightGetter.getOpenLinearToSample()
                ).interruptOn(
                        () -> MMRobot.getInstance().mmSystems.driveTrain.joystickMoved()),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                new WaitCommand(500),
                FirstSampleIntake(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command limeLightIntake_Auto() {
        return new FixedSequentialCommandGroup(
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd", "enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd2", "not enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd3", "not enderd")),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd4", "not enderd")),
                new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.trackRedDetector()),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd2", "enderd")),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == 0),
                new InstantCommand(() -> FtcDashboard.getInstance().getTelemetry().addData("enderd3", "enderd")),

                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),

                //Lamlam side:
                new FixedSequentialCommandGroup(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                        MMRobot.getInstance().mmSystems.vision.onlyAngleChange(),

                        limelightGetter.strafeToSample().alongWith(
                                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING))

//                        limelightGetter.getOpenLinearToSample()
                ).interruptOn(
                        () -> MMRobot.getInstance().mmSystems.driveTrain.joystickMoved()),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArmState.PREPARE_SAMPLE_INTAKE),
                new WaitCommand(500),
                FirstSampleIntake(),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.CLOSED_POSE)
        );
    }

    public static Command doLimelightIntake_Auto() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                        MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw(),
                        limelightGetter.strafeToSampleAuto(strafeTrajectory),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(angle),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntakeState.MAX_OPENING)
                ),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                new WaitCommand(150),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                )

        );
    }

    private static MecanumDrive.CancelableFollowTrajectoryAction strafeTrajectory;
    private static Double angle;

    public Command prepareForLimelight_Auto() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.trackRedDetector()),
                new WaitUntilCommand(() -> MMRobot.getInstance().mmSystems.vision.getPipelineIndex() == 0),
                //Lamlam side:
                new SequentialCommandGroup(
                        new InstantCommand(() -> MMRobot.getInstance().mmSystems.vision.setPreviousResult()),
                        MMRobot.getInstance().mmSystems.vision.prepareAngle(),
                        new InstantCommand(this::prepareStrafeTrajectory),
                        new InstantCommand(this::calculateAngle)
                )
        );
    }

    public void calculateAngle(){
        angle = MMRobot.getInstance().mmSystems.vision.getTurnServoDegree();
        if (angle != null) {
            if (angle >= 0 && angle <= 90) {
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() - angle;
            } else {
                angle = 180 - angle;
                angle /= 270;
                angle = IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE.position.get() + angle;
            }
        }
    }

    public void prepareStrafeTrajectory() {

        double maxDistanceY = 470;
        double plusDistanceX = 1.5;


        LLResult lastResult = MMRobot.getInstance().mmSystems.vision.getPreviousResult();
        double distanceX = MMRobot.getInstance().mmSystems.vision.getStrafeOffset(lastResult) + plusDistanceX;
        double distanceY = (maxDistanceY - MMRobot.getInstance().mmSystems.vision.getDistance(lastResult)) / 25.4;

        if (distanceX != 0) {
            Pose2d currentPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

            Translation2d distanceXVector = new Translation2d(distanceX, new Rotation2d(currentPose.heading.toDouble() + Math.toRadians(90)));
            Translation2d distanceYVector = new Translation2d(distanceY, new Rotation2d(currentPose.heading.toDouble()));
            Translation2d endPoint = new Translation2d(currentPose.position.x, currentPose.position.y)
                    .plus(distanceXVector)
                    .plus(distanceYVector);

            TrajectoryBuilder strafe = MMRobot.getInstance().mmSystems.driveTrain.trajectoryBuilder(currentPose)
                    .strafeTo(new Vector2d(endPoint.getX(), endPoint.getY()),
                            new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel), new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.65, MecanumDrive.PARAMS.maxProfileAccel * 0.65));

            strafeTrajectory = MMRobot.getInstance().mmSystems.driveTrain.getCancelableFollowTrajectoryAction(strafe.build().get(0));
        }
    }

    private static Command FirstSampleIntake() {
        return new SequentialCommandGroup(
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SAMPLE_INTAKE_POSE),
                new WaitCommand(300),
                MMRobot.getInstance().mmSystems.intakEndUnit.closeIntakeClaw(),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
                        MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE),
                        MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE)
                )
        );
    }
}