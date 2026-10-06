# Writing plan — ideas per section (Ch.3 System Design, Ch.4 Methodology)

Two cross-cutting ideas first, because they shape every section:

- **Use a problem → cause → solution pattern** whenever you describe an engineering
  element. "The scan arrived with a Gazebo-mangled frame id, so SLAM could not relate
  it to the robot; a republisher rewrites the frame and both costmaps and SLAM now
  consume that topic." That single pattern turns a config listing into engineering
  and makes the chapter defensible line by line.
- **Name your traps.** Two behaviour-tree findings are yours and worth stating:
  a `SequenceStar` root floods `ComputePathToPose` and causes planner timeouts, and a
  `ReactiveFallback` with more than two asynchronous children stops re-ticking its
  yield branches. Labelled findings read as contribution; unlabelled ones read as
  implementation trivia.

---

## Chapter 3 — System Design

### 3.1 System Overview
**Purpose:** the reader must be able to draw your robot's pipeline from memory after
one page.
- Figure: per-robot pipeline — sensors → symbolic state → decision layer (tree and/or
  agent) → Nav2 → velocity adaptor → wheels. One box diagram beats three paragraphs.
- Table: the three behavioural conditions, and one sentence stating that *everything
  except the decision layer* is held constant. This is the design contract the
  Methodology chapter relies on.
- State explicitly that the baseline has no human perception at all — it is the
  "humans as costmap obstacles" condition your Chapter 1 criticises. Say it, don't
  imply it.
- Run-in paragraphs: simulation platform, robot platform, behavioural conditions,
  per-robot pipeline.

### 3.2 Perception and Symbolic States
**Purpose:** show that the decision layer never touches raw sensors — it consumes a
symbolic scene.
- LiDAR path as problem→cause→solution (frame id).
- Table: detector parameters. Add one sentence on *why 3 Hz is enough* (a pedestrian at
  0.5 m/s moves 0.17 m between inferences).
- Table: symbolic state — variable, definition, threshold, source.
- Justify the robot-as-person suppression with the failure it prevents: without it the
  fleet reacts to itself as pedestrians.
- Name the deliberate simplification: the nearest person represents the scene. It is a
  limitation you accept on purpose (and a Threats item).

### 3.3 Behavior Tree Design
**Purpose:** show that the social rules are *executable* and bounded.
- Figure: the tree. A TikZ diagram is worth one afternoon here.
- Table: timings and retry bounds (4 s human, 10 s rover, arc 0.3 m/s / 6 s, six
  navigation retries).
- The two named traps above, each with the symptom you observed.
- Why the escape maneuver is a continuous arc and never an in-place rotation.

### 3.4 Integration with Nav2
**Purpose:** show how social intent becomes motion without weakening the navigation
stack.
- Diagram or paragraph: the velocity chain — controller → `cmd_vel_social` → velocity
  adaptor → `cmd_vel` → bridge. Say why the social layer regulates speed *after* the
  controller rather than by constraining the planner.
- The progress-checker decision: relaxed to 0.25 m per 30 s *because* a socially
  pausing robot would otherwise be declared non-progressing and aborted. One sentence
  of cause, one of effect.
- Table: per-arm remapping (baseline bypasses the adaptor entirely).

### 3.5 LLM Social Layer
**Purpose:** answer RQ1's first half — *can the agent choose appropriate actions?* — by
showing you built a decision interface, not a chatbot.
- **Table: the situation report schema.** The exact fields the agent sees (nearest
  human: range, bearing, range-rate, close flag; nearest rover: range, bearing, speed,
  right-of-way; free space left/right/back; wall proximity). This *is* the "scene
  description" of your RQ — it deserves its own table.
- Table: the action vocabulary, each with its semantics and its guard condition.
- Event-driven invocation: one call per material change, and why (cost and latency);
  state that this makes the call count scenario-dependent, not a fixed rate.
- Role separation: the agent proposes, the tree executes, the reactive layer remains —
  the safety argument from your related work.
- Cost table (measured, per run and per completed goal).

### 3.6 Multi-Robot Extension
**Purpose:** show that scaling is an architectural property, not a parameter.
- Per-robot stacks: own SLAM, own map frame, own Nav2; maps are never merged.
- The argument, stated once: a shared map couples every robot to one failure and is
  irrelevant to a purely local social layer.
- TF tree figure (or table of frames) and the viz-only connectors.
- Coordination is local only — no goal or plan exchange; the right-of-way rule is what
  makes interaction predictable.
- Fleet size is the scaling factor (forward-reference Ch.4).

### 3.7 Design Decisions and Alternatives Considered
**Purpose:** pre-answer "why not X?" in the defence.
- Use a four-column table, one row per decision:
  *Decision | Alternative considered | Why chosen | Trade-off accepted.*
- Rows: BT over FSM; per-robot maps over shared map; event-driven over per-tick LLM
  calls; consultant over controller; nearest-person over multi-person tracking; bounded
  over indefinite yields; arc over in-place rotation.

### Optional: Realisation (or an appendix)
- Table: packages and nodes (name, role, key interfaces).
- Launch procedure in a few lines, and the workarounds table:
  *Symptom | Cause | Fix* — frame id, clock rate, DDS/fd exhaustion, bring-up ordering,
  detector gating.
- Anything purely mechanical (repo layout, build commands, full parameter dumps) goes
  to the appendix and is referenced once.

---

## Chapter 4 — Methodology

### 4.1 Research Questions and Hypotheses
- Restate RQ1–RQ3 in operational terms, then one hypothesis each:
  **H1** the social layer reduces human-distance violations and collisions versus the
  baseline; **H2** the agent reduces unnecessary yields and deadlocks versus the
  reactive tree; **H3** goal success and safety hold as the fleet grows.
- **Operationalise "socially appropriate" (RQ1).** This is the section's real work.
  Proposal: a *rule-consistency audit* of the logged decisions — every decision where a
  conflict existed is scored against the rules encoded in the prompt:

  | Rule | Checkable condition | Violation example |
  |---|---|---|
  | yield only when yielding | the situation report named this robot as the yielding one | yields while holding right-of-way |
  | never yield to a passed robot | bearing magnitude > 60° | stops for a rover already behind |
  | no arc when too close | blocker ≤ 1.2 m | turns into the blocker |
  | back only for a deadlock | blocker speed < 0.1 m/s for ≥ 2 s **and** rear clear | reverses while the other is moving |
  | proceed when no conflict | nearest human > 1.5 m and not crossing | stops with an empty path |

  Report the compliance rate per rule; the `reason` strings are logged verbatim, so the
  audit is reproducible from the artifacts.
- **Execution reliability (RQ1's second half):** fraction of agent actions adopted,
  reactive fallbacks triggered, goal aborts during an agent action. Say which artifact
  each number comes from.

### 4.2 Behavioural Conditions
- State the independent variable: the decision layer, three levels.
- State what is held constant (map, sensors, controller, goal set, duration) and why
  that is what makes a comparison meaningful.
- Note explicitly that the baseline lacks human perception — the comparison measures
  the *cost and benefit of adding a social layer*, not two equally aware systems.

### 4.3 Scenario Design and Goal Protocol
- Matrix (rovers × humans), obstacle variants, arena, spawns and human lanes.
- Goal protocol: fixed tours plus scan goals, the map-completeness precondition, and
  the nominal duration.
- **Define an "encounter"** — you need it for per-encounter metrics: proposal — a
  robot and a human within *X* m (state the threshold, e.g. 3.6 m = Hall's social
  distance) for at least *T* s, from `social_state.csv`. Without this definition,
  yields-per-encounter and collision-rate-per-encounter are not computable.
- Repetitions per cell, and the statement that order is not randomised (state it).

### 4.4 Metrics and Instrumentation
- One table: **metric | definition | unit | aggregation level | artifact file.**
  Include coverage, path length, goals sent/succeeded, collisions, yield events,
  minimum human distance, time-below-threshold at 0.45/1.2/1.5/3.6 m, LLM calls and
  cost.
- State the personal-space boundary you adopt as *the* safety threshold, and why it
  differs (if it does) from Hall's zones and from the detector's 1.5 m close flag.
- **Say how you treat multiple robots in one run.** Four rovers in one run are not four
  independent samples — either aggregate per run, or state that robot-level
  observations are treated as repeated measures within the scenario. With n ≈ 2–3 runs
  per cell, report medians and ranges, and use paired per-scenario comparisons rather
  than significance tests. Stating this *before* the results protects you.

### 4.5 Experimental Protocol and Reproducibility
- Preconditions: thermal cap, launch through the harness, sequential bring-up.
- Validators as inclusion criteria: gate passed, all detectors loaded, full duration
  reached. A run failing any of these is excluded, and excluded runs are counted.
- Era policy: the pre- and post-2026-09-23 configurations, and the rule you adopt
  (re-run / reduced grid / earlier runs as pilot data).
- Exclusions and how they are reported (so the reader can see attrition).

### 4.6 Threats to Validity
- Internal: multiple robots per run are not independent; the LLM is stochastic and the
  hosted model version can change; event-driven sampling makes call counts
  scenario-dependent; human lanes are scripted, so encounters are not adversarially
  chosen.
- Construct: "socially appropriate" is operationalised by rules plus proxies, not by
  human judgement of comfort; coverage depends on map quality; bumper contacts depend
  on actor physics.
- External: simulation only, so generalisation to real pedestrians is untested; sensor
  rates reduced for tractability; single host.
- Each threat gets a sentence on *how you mitigate it* — that is what separates a
  limitations dump from a validity section.
