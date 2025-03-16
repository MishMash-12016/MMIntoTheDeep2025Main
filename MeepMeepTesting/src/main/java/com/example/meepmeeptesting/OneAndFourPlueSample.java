package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class OneAndFourPlueSample {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        double tangentsToScoreSpecimen = 130;
        double tangentsToIntakeSpecimen = 300;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(11.02,14.5)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(720), Math.toRadians(720), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90)))


                        .setTangent(Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(5.5, -32, Math.toRadians(90)))

/*
     -----------------------
        pushing
     -----------------------
*/
                        //Push first specimen
                        .setTangent(Math.toRadians(270))
                        .splineTo(new Vector2d(29, -35), Math.toRadians(50))
                        .setTangent(Math.toRadians(290))
                        .splineToLinearHeading(new Pose2d(35.8, -47, Math.toRadians(160)), Math.toRadians(240))
                        //Push second specimen
                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(new Pose2d(40, -35, Math.toRadians(230)), Math.toRadians(70))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(45.8, -47, Math.toRadians(160)), Math.toRadians(270))

                        .setTangent(Math.toRadians(80))
                        .splineToLinearHeading(new Pose2d(51, -38, Math.toRadians(235)), Math.toRadians(80))

                        .setTangent(Math.toRadians(240))
                        .splineToLinearHeading(new Pose2d(48, -52, Math.toRadians(90)), Math.toRadians(270))


                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(48, -58,Math.toRadians(90)), Math.toRadians(270)) //driveToIntakeFirstSpecimen

                        .setTangent(Math.toRadians(130))
                        .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(120))

                        .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                        .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen))

                        .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                        .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(120))


                        .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                        .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen))

                        .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                        .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(120))
//
//
                        .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                        .splineToConstantHeading(new Vector2d(43, -58),Math.toRadians(tangentsToIntakeSpecimen))

                        .setTangent(Math.toRadians(tangentsToScoreSpecimen))
                        .splineToConstantHeading(new Vector2d(4, -30),Math.toRadians(120))

                        //replace the tested code with the code above

                        .setTangent(Math.toRadians(tangentsToIntakeSpecimen))
                        .splineToLinearHeading(new Pose2d(58, -55, Math.toRadians(180)),Math.toRadians(0))

                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-54, -60,Math.toRadians(180)), Math.toRadians(180))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
