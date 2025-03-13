package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class OneandFive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double maxWheelVel = 70;
        double maxProfileAccel = 70;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//+(150/25.4)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(270)))

                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90))

/*
     -----------------------
        pushing
     -----------------------
*/
                        //Push first specimen
                        .setTangent(Math.toRadians(260))
                        .splineToLinearHeading(new Pose2d(22, -45,Math.toRadians(325+180)), Math.toRadians(340))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(29, -35,Math.toRadians(230)), Math.toRadians(50))
                        .setTangent(Math.toRadians(290))
                        .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(150)),Math.toRadians(240))
                        //Push second specimen
                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(new Pose2d(40, -38, Math.toRadians(235)), Math.toRadians(70))
                        .setTangent(Math.toRadians(300))
                        .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(150)), Math.toRadians(250))
                        //Push third specimen
                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(51, -53, Math.toRadians(90)), Math.toRadians(270))


//intake & score

//first
                        .splineToLinearHeading(new Pose2d(51, -60, Math.toRadians(90)), Math.toRadians(270))

                        .setTangent(Math.toRadians(140))
                        .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135))
//second
                        .setTangent(Math.toRadians(310))
                        .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310))

                        .setTangent(Math.toRadians(140))
                        .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135))
//third
                        .setTangent(Math.toRadians(310))
                        .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310))

                        .setTangent(Math.toRadians(140))
                        .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135))
//forth
                        .setTangent(Math.toRadians(310))
                        .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310))


                        .setTangent(Math.toRadians(140))
                        .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135))
//fifth
                        .setTangent(Math.toRadians(310))
                        .splineToLinearHeading(new Pose2d(38, -58, Math.toRadians(90)),Math.toRadians(310))

                        .setTangent(Math.toRadians(140))
                        .splineToSplineHeading(new Pose2d(7,-30,Math.toRadians(120)),Math.toRadians(135))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


