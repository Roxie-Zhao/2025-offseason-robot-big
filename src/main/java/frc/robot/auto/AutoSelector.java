package frc.robot.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.ironpulse.utils.Logging;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class AutoSelector {
  private static AutoSelector instance;

  private final SendableChooser<AutoConfig.AutoType> typeSelector;
  private final SendableChooser<AutoConfig.ScoringLocation> loc1Selector;
  private final SendableChooser<AutoConfig.ScoringLocation> loc2Selector;
  private final SendableChooser<AutoConfig.ScoringLocation> loc3Selector;
  private final SendableChooser<AutoConfig.ScoringLocation> loc4Selector;
  private final SendableChooser<AutoConfig.ScoringLevel> level1Selector;
  private final SendableChooser<AutoConfig.ScoringLevel> level2Selector;
  private final SendableChooser<AutoConfig.ScoringLevel> level3Selector;
  private final SendableChooser<AutoConfig.ScoringLevel> level4Selector;

  private static final List<AutoConfig.ScoringLocation> ALL_LOCS =
      List.of(AutoConfig.ScoringLocation.values());
  private static final List<AutoConfig.ScoringLevel> ALL_LVLS =
      List.of(AutoConfig.ScoringLevel.values());

  // Alert for invalid configurations
  private final Alert doNothingAlert;
  private final Alert invalidConfigAlert;
  private final Alert finalL4FlatNotMatchAlert;
  private boolean hasError = false;

  private AutoSelector() {
    // Initialize alerts
    doNothingAlert = new Alert("Auto is set to Do Nothing", Alert.AlertType.kWarning);
    invalidConfigAlert = new Alert("Auto configuration is invalid", Alert.AlertType.kError);
    finalL4FlatNotMatchAlert = new Alert("Auto Final L4 is on Flat, but does not match auto type.", Alert.AlertType.kWarning);

    // auto type chooser
    typeSelector = new SendableChooser<>();
    typeSelector.setDefaultOption("Left Routine", AutoConfig.AutoType.LeftRoutine);
    typeSelector.addOption("Do Nothing", AutoConfig.AutoType.DoNothing);
    typeSelector.addOption("Right Routine", AutoConfig.AutoType.RightRoutine);
    SmartDashboard.putData("Auto Type", typeSelector);

    // location choosers with defaults
    loc1Selector = makeLocationChooser("Location 1", AutoConfig.ScoringLocation.FarLeft);
    loc2Selector = makeLocationChooser("Location 2", AutoConfig.ScoringLocation.NearLeft);
    loc3Selector = makeLocationChooser("Location 3", AutoConfig.ScoringLocation.NearRight);
    loc4Selector = makeLocationChooser("Location 4", AutoConfig.ScoringLocation.FlatLeft);

    // level choosers with defaults
    level1Selector = makeLevelChooser("Level 1", AutoConfig.ScoringLevel.L4);
    level2Selector = makeLevelChooser("Level 2", AutoConfig.ScoringLevel.L4);
    level3Selector = makeLevelChooser("Level 3", AutoConfig.ScoringLevel.L4);
    level4Selector = makeLevelChooser("Level 4", AutoConfig.ScoringLevel.L4);
  }

  public static AutoSelector getInstance() {
    if (instance == null) {
      instance = new AutoSelector();
    }
    return instance;
  }

  /**
   * call this once per loop (e.g. in robotPeriodic) to update alerts
   */
  public void updateAlerts() {
    var type = typeSelector.getSelected();

    // Check for Do Nothing alert
    boolean isDoNothing = (type == AutoConfig.AutoType.DoNothing);
    doNothingAlert.set(isDoNothing);

    // Check for invalid configuration
    boolean isInvalid = false;
    String invalidReason = "";

    if (type == AutoConfig.AutoType.LeftRoutine || type == AutoConfig.AutoType.RightRoutine) {
      // Check if there are any valid targets
      var targets = getValidTargets();
      if (targets.isEmpty()) {
        isInvalid = true;
        invalidReason = "No valid targets selected for " + type.name();
      } else {
        // Check for duplicate targets
        Set<Pair<AutoConfig.ScoringLocation, AutoConfig.ScoringLevel>> uniqueTargets = new HashSet<>(targets);
        if (uniqueTargets.size() != targets.size()) {
          isInvalid = true;
          invalidReason = "Duplicate targets selected";
        }
      }
    }

    if (isInvalid) {
      invalidConfigAlert.setText("Auto configuration is invalid: " + invalidReason);
    }
    invalidConfigAlert.set(isInvalid);
    hasError = isInvalid;

    finalL4FlatNotMatchAlert.set(
        (type == AutoConfig.AutoType.LeftRoutine && loc4Selector.getSelected() == AutoConfig.ScoringLocation.FlatRight)
        || (type == AutoConfig.AutoType.RightRoutine && loc4Selector.getSelected() == AutoConfig.ScoringLocation.FlatLeft)
    );
  }

  /** reads the dashboard and builds an AutoConfig in index order, skipping "None" slots */
  public AutoConfig getAutoConfig() {
    // If there's an error, return a DoNothing config
    if (hasError) {
      Logging.warn("AutoSelector", "Returning DoNothing config due to invalid configuration");
      return new AutoConfig().withAutoType(AutoConfig.AutoType.DoNothing);
    }

    var cfg = new AutoConfig().withAutoType(typeSelector.getSelected());

    appendIfPresent(cfg, loc1Selector, level1Selector);
    appendIfPresent(cfg, loc2Selector, level2Selector);
    appendIfPresent(cfg, loc3Selector, level3Selector);
    appendIfPresent(cfg, loc4Selector, level4Selector);

    Logging.info("AutoSelector", "Built config with %d targets", cfg.getCoralCount());
    return cfg;
  }

  /** Get list of valid targets for validation */
  private List<Pair<AutoConfig.ScoringLocation, AutoConfig.ScoringLevel>> getValidTargets() {
    var targets = new ArrayList<Pair<AutoConfig.ScoringLocation, AutoConfig.ScoringLevel>>();

    addTargetIfPresent(targets, loc1Selector, level1Selector);
    addTargetIfPresent(targets, loc2Selector, level2Selector);
    addTargetIfPresent(targets, loc3Selector, level3Selector);
    addTargetIfPresent(targets, loc4Selector, level4Selector);

    return targets;
  }

  private void appendIfPresent(
      AutoConfig cfg,
      SendableChooser<AutoConfig.ScoringLocation> loc,
      SendableChooser<AutoConfig.ScoringLevel> lvl) {
    var l = loc.getSelected();
    var v = lvl.getSelected();
    if (l != null && v != null) {
      cfg.appendScoringTarget(l, v);
    }
  }

  private void addTargetIfPresent(
      List<Pair<AutoConfig.ScoringLocation, AutoConfig.ScoringLevel>> targets,
      SendableChooser<AutoConfig.ScoringLocation> loc,
      SendableChooser<AutoConfig.ScoringLevel> lvl) {
    var l = loc.getSelected();
    var v = lvl.getSelected();
    if (l != null && v != null) {
      targets.add(Pair.of(l, v));
    }
  }

  private SendableChooser<AutoConfig.ScoringLocation> makeLocationChooser(String label, AutoConfig.ScoringLocation defaultOption) {
    var c = new SendableChooser<AutoConfig.ScoringLocation>();
    c.setDefaultOption(defaultOption.name(), defaultOption);
    for (var o : ALL_LOCS) {
      if (!o.equals(defaultOption)) {
        c.addOption(o.name(), o);
      }
    }
    c.addOption("None", null);
    SmartDashboard.putData(label, c);
    return c;
  }

  private SendableChooser<AutoConfig.ScoringLevel> makeLevelChooser(String label, AutoConfig.ScoringLevel defaultOption) {
    var c = new SendableChooser<AutoConfig.ScoringLevel>();
    c.setDefaultOption(defaultOption.name(), defaultOption);
    for (var o : ALL_LVLS) {
      if (!o.equals(defaultOption)) {
        c.addOption(o.name(), o);
      }
    }
    c.addOption("None", null);
    SmartDashboard.putData(label, c);
    return c;
  }
}