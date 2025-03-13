package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class AutoOnePlusFive extends MMOpMode {
    static MMRobot robotInstance;

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

        TrajectoryActionBuilder driveToPreload = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5.5, -27), Math.toRadians(90));
        TrajectoryActionBuilder driveToPush1 = driveToPreload.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(22, -45,Math.toRadians(325+180)), Math.toRadians(340))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(29, -35,Math.toRadians(230)), Math.toRadians(50))
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(150)),Math.toRadians(240));
        TrajectoryActionBuilder driveToPush2 = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70))
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(150)), Math.toRadians(250));
        TrajectoryActionBuilder driveToPush3= driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, -53, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToIntakeFirstSpecimen= driveToPush3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(51, -60, Math.toRadians(90)), Math.toRadians(270));
        TrajectoryActionBuilder driveToScoreFirstSpecimen= driveToIntakeFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeSecondSpecimen= driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreSecondSpecimen= driveToIntakeSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeThirdSpecimen= driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreThirdSpecimen= driveToIntakeThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeForthSpecimen= driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreForthSpecimen= driveToIntakeForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135));
        TrajectoryActionBuilder driveToIntakeFifthSpecimen= driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310));
        TrajectoryActionBuilder driveToScoreFifthSpecimen= driveToIntakeFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135));
        new SequentialCommandGroup(
                new InstantCommand(),
                new ActionCommand(driveToPreload.build())




    }
