package org.firstinspires.ftc.teamcode.CommandGroup;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Autonomous.ActionCommand;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringClawEndUnit;

public class TeleOPAutonomous {
    public static Command IntakeAndScore()
    {
        Pose2d currentPose = (new Pose2d(5.5, -62.73, Math.toRadians(90.00)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);


        TrajectoryActionBuilder driveToIntakeSpecimen = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(10, -40, Math.toRadians(90)), Math.atan((-40 + 59.2) / (10.0 - 45.0)))
                .strafeToLinearHeading(new Vector2d(45, -59.2), Math.toRadians(90));
        TrajectoryActionBuilder driveToScoreSpecimen = driveToIntakeSpecimen.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-5, -25), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2));

        return new SequentialCommandGroup(
                new RunCommand(() ->
                        new SequentialCommandGroup(
                                //intake and drive to score
                                new ParallelCommandGroup(
                                        AutoSpecimensCommand.SpecimenIntakeAuto(),
                                        new WaitCommand(500).andThen(
                                                new ActionCommand(driveToScoreSpecimen.build()))
                                ),
                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.setPosition(ScoringClawEndUnit.ScoringClawState.BARELY_OPEN),

                                //score and drive to intake
                                new ActionCommand(driveToIntakeSpecimen.build()).alongWith(
                                        new WaitCommand(1000).andThen(
                                                IntakeSpecimenCommand.PrepareSystemsSpecimenIntake())
                                ),
                                new WaitCommand(200)
                        )
                ).interruptOn(() -> (
                        MMRobot.getInstance().mmSystems.gamepadEx1.getLeftX() > 0.1 ||
                                MMRobot.getInstance().mmSystems.gamepadEx1.getLeftY() > 0.1 ||
                                MMRobot.getInstance().mmSystems.gamepadEx1.getRightX() > 0.1)),

                new InstantCommand(() -> MMRobot.getInstance().mmSystems.driveTrain.stop(),
                        MMRobot.getInstance().mmSystems.driveTrain)
        );
    }
}