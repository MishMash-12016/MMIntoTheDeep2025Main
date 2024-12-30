package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class autoclassred {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions( 44 / 2.54,39 / 2.54)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.59)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-16.18, -68.52, Math.toRadians(90.00)))
                        .splineToLinearHeading(new Pose2d(-57.50, -58.00, Math.toRadians(50.00)), Math.toRadians(191.51))
                        .splineToLinearHeading(new Pose2d(-48.74, -37.62, Math.toRadians(90.00)), Math.toRadians(78.16))
                                .setTangent(Math.toRadians(240))
                        .splineToSplineHeading(new Pose2d(-54.00, -55.21, Math.toRadians(50.00)), Math.toRadians(78.60))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}