package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimansCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.ScoringSpecimanCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitRotator;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
@Autonomous
public class TrajCancellationTest extends MMOpMode {
    MMRobot robotInstance;

    public TrajCancellationTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    PinpointDrive drive;

    @Override
    public void onInit() {

        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();

        Pose2d currentPose = (new Pose2d(5.5, 65.5, Math.toRadians(90)));
        drive = new PinpointDrive(hardwareMap, currentPose);

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);




        //poses for pushing
        final double halfOpenClaw = 0.7;
        final double rotator = 0;
        final double intakeArmPose = 0.59;


        //Score pre-load
        TrajectoryActionBuilder driveToScorePreloadSpecimen = drive.actionBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(5.5, 0), Math.toRadians(90), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel*0.2));


//
//        new SequentialCommandGroup(
//
//                //arrive with strafe & PrepareSpecimen command
//                new ActionCommand(driveToIntakeFirstSpecimen.build()).alongWith(
//                        new SequentialCommandGroup(
//                                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.PREPARE_TRANSFER),//be prepared for transfer
//                                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
//                                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.SPECIMEN_INTAKE),
//                                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.INTAKE_SPECIMEN_POSE),
//                                MMRobot.getInstance().mmSystems.scoringEndUnitRotator.setPosition(ScoringEndUnitRotator.ScoringRotatorState.TRANSFER_POSE),
//                                MMRobot.getInstance().mmSystems.intakEndUnit.openIntakeClaw())),
//                new WaitCommand(200),
//
//                //--------------
//                //new RunCommand()
//                new ConditionalCommand(IntakeSpecimansCommand.SpecimenIntake(), goBackThanForwordAnd-ChackAgain , () -> MMRobot.getInstance().mmSystems.intakeDistSensor.getDistance() < 4)
//
//                ),
//                new ParallelCommandGroup(
//                        IntakeSpecimansCommand.SpecimenIntake(),
//                        new WaitCommand(200).andThen(
//                                new ActionCommand(driveToScoreFirstSpecimen.build())))
//        ).schedule();
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
//        telemetry.addData("linear", MMRobot.getInstance().mmSystems.linearIntake.getPosition());
//        telemetry.update();
//        FtcDashboard.getInstance().getTelemetry().update();
  }
//
}

