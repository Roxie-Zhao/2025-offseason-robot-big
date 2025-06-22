package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotConstants;
import frc.robot.utils.TunableNumber;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNav questNav;


    // Transform from robot center to Quest headset
    private final TunableNumber robotToQuestX = new TunableNumber("QuestNav/RobotToQuestX", 0.0);
    private final TunableNumber robotToQuestY = new TunableNumber("QuestNav/RobotToQuestY", 0.0);
    private final TunableNumber robotToQuestRotDeg = new TunableNumber("QuestNav/RobotToQuestRotDeg", 0.0);

    public QuestNavIOReal() {
        questNav = new QuestNav(); 
        System.out.println("QuestNavIOReal initialized successfully");
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.connected = questNav.isConnected();
        inputs.tracking = questNav.isTracking();
        inputs.pose = questNav.getPose();
        inputs.timestamp = questNav.getDataTimestamp();
    }

    @Override
    public void setPose(Pose2d robotPose) {
        // Transform robot pose to Quest pose
        Transform2d robotToQuest = new Transform2d(
            robotToQuestX.get(),
            robotToQuestY.get(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(robotToQuestRotDeg.get())
        );
        
        Pose2d questPose = robotPose.transformBy(robotToQuest);
        questNav.setPose(questPose);
        
        System.out.println("QuestNavIOReal: setPose called with robot pose: " + robotPose + ", quest pose: " + questPose);
    }

    @Override
    public void commandPeriodic() {
        questNav.commandPeriodic();
        // This is critical for QuestNav v2025-1.0.0+ to function properly
    }
} 