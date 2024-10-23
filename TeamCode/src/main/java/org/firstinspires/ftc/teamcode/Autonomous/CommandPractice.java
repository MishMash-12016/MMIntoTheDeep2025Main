package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoIntake;
import org.firstinspires.ftc.teamcode.CommandGroup.ElevatorBackTo_0;
import org.firstinspires.ftc.teamcode.CommandGroup.Scoring;
import org.firstinspires.ftc.teamcode.Commands.SetLinearPosition;
import org.firstinspires.ftc.teamcode.Commands.TrajectroryFollowerCommend;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class CommandPractice extends MMOpMode {

    public CommandPractice() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }

    @Override
    public void onInit() {

        MMRobot.getInstance().mmSystems.initElevator();
        MMRobot.getInstance().mmSystems.initLinearIntake();
        MMRobot.getInstance().mmSystems.initClaw();
        MMRobot.getInstance().mmSystems.initIntakeArm();
        MMRobot.getInstance().mmSystems.initScoringArm();
        MMRobot.getInstance().mmSystems.initIntakeRoller();


        addRunnableOnInit(
                ()->MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.CLOSE)
        );

        final AutoMecanumDrive drive = new AutoMecanumDrive(new SampleMecanumDrive(hardwareMap));


        Pose2d startPose = new Pose2d(-60, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence START_POINT_TO_BASCET = drive.trajectorySequenceBuilder(startPose)

                .splineToSplineHeading(
                        new Pose2d(-54, 54,Math.toRadians(-45)),
                        Math.toRadians(90)  //tangent
                ).build();

        TrajectorySequence FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE = drive.trajectorySequenceBuilder(START_POINT_TO_BASCET.end())
                .setTangent(Math.toRadians(-70))
                .splineToSplineHeading(
                        new Pose2d(-36,29,Math.toRadians(60)),
                        Math.toRadians(-80)  //tangent
                ).build();

        TrajectorySequence FROM_FIRST_INTAKE_TO_SCORING = drive.trajectorySequenceBuilder(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE.end())
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(
                        new Pose2d(-54,54,Math.toRadians(-45)),
                        Math.toRadians(97))//tangent
                .build();

        TrajectorySequence FROM_SCORING_TO_PARKING = drive.trajectorySequenceBuilder(FROM_FIRST_INTAKE_TO_SCORING.end())
                .splineToSplineHeading(
                        new Pose2d(-60,-55,Math.toRadians(-90)),
                        Math.toRadians(-95))//tangent
                .build();


        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(),
                        new TrajectroryFollowerCommend(START_POINT_TO_BASCET, drive),
                        new WaitCommand(1000),
                        new Scoring(Elevator.HIGH_BASKET),
                        new WaitCommand(1000),
                        new ElevatorBackTo_0(),
                        new TrajectroryFollowerCommend(FROM_THE_FIRST_SCORING_TO_THE_FIRST_INTAKE, drive),
                        new WaitCommand(1000),
                        new AutoIntake(),
                        new WaitCommand(1000),
                        new TrajectroryFollowerCommend(FROM_FIRST_INTAKE_TO_SCORING,drive),
                        new WaitCommand(1000),
                        new Scoring(Elevator.HIGH_BASKET),
                        new WaitCommand(1000),
                        new ElevatorBackTo_0(),
                        new WaitCommand(500),
                        new TrajectroryFollowerCommend(FROM_SCORING_TO_PARKING,drive)
                )
        );
    }

    @Override
    public void run() {
        super.run();
        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        telemetry.addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
        telemetry.update();
    }
}


