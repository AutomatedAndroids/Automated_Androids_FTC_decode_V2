package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Vector2d shootBlue = new Vector2d(-16, 0);
        Vector2d pickUpBlueFar = new Vector2d(-18, 0);
        Vector2d pickUpBlueMid = new Vector2d(-18, 0);
        Vector2d pickUpBlueNear = new Vector2d(-18, 0);

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, -15, Math.toRadians(180)))
                        .lineToXSplineHeading(-16,5*Math.PI/4)
                        .turn(Math.PI/4)
                        .strafeTo(new Vector2d(-12, -20))
                        .strafeTo(new Vector2d(12, -20))
                        .strafeTo(new Vector2d(36, -20))

                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}