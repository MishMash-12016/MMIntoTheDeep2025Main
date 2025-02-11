package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.utils.OpModeType;

@Autonomous
public class MMTest extends MMOpMode {
    public MMTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }
    MMRobot robotInstance;


    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems();
        Pose2d currentPose = (new Pose2d(0, 0, Math.toRadians(90)));
        PinpointDrive drive = new PinpointDrive(hardwareMap, currentPose);

        waitForStart();

        TrajectoryActionBuilder driveleft = drive.actionBuilder(currentPose)
                .setTangent(Math.toRadians(180))
                .lineToX(-30);
        new ActionCommand(driveleft.build()).schedule();
}}
