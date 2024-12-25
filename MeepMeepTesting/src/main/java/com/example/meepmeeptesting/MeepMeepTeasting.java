package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTeasting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions( 44 / 2.54,39 / 2.54)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.59)
                .followTrajectorySequence(drive ->
                         drive.trajectorySequenceBuilder(new Pose2d(8.09, -64.52, Math.toRadians(90.00)))
                        .splineToLinearHeading(new Pose2d(-25.28, -40.65, Math.toRadians(176.01)), Math.toRadians(176.01))
                        .splineToLinearHeading(new Pose2d(-59.87, -57.44, Math.toRadians(45.00)), Math.toRadians(111.46))
                        .splineToLinearHeading(new Pose2d(-59.87, -39.24, Math.toRadians(90.00)), Math.toRadians(89.10))
                        .splineToLinearHeading(new Pose2d(-59.46, -59.06, Math.toRadians(45.00)), Math.toRadians(-47.09))

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
