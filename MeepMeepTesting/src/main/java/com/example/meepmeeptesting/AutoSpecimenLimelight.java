package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.nio.channels.MembershipKey;

public class AutoSpecimenLimelight {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.02,14.5)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -32, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(330))
                        .splineToLinearHeading(new Pose2d(38, -40, Math.toRadians(-45)), Math.toRadians(0))

//
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
