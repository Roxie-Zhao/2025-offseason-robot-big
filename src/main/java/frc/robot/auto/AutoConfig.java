package frc.robot.auto;

import edu.wpi.first.math.Pair;
import lombok.Getter;

import java.util.ArrayList;
import java.util.Optional;

public class AutoConfig {
  public enum AutoType {
    DoNothing, LeftRoutine, RightRoutine
  }

  public enum ScoringLocation {
    FarLeft, FarRight, NearLeft, NearRight, FlatLeft, FlatRight
  }

  public enum ScoringLevel {
    L4, L3, L2
  }

  @Getter
  private AutoType autoType = AutoType.DoNothing;
  private ArrayList<Pair<ScoringLocation, ScoringLevel>> scoringTargets = new ArrayList<>();

  public AutoConfig withAutoType(AutoType autoType) {
    this.autoType = autoType;
    return this;
  }

  public AutoConfig appendScoringTarget(ScoringLocation location, ScoringLevel level) {
    scoringTargets.add(Pair.of(location, level));
    return this;
  }

  public Optional<Pair<ScoringLocation, ScoringLevel>> getScoringTarget(int idx) {
    return idx < scoringTargets.size() ? Optional.of(scoringTargets.get(idx)) : Optional.empty();
  }

  public int getCoralCount() {
    return scoringTargets.size();
  }
}
