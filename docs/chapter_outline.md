# Thesis chapter outline — System Design (Ch.3) + Methodology (Ch.4)

## Do you need a System Design chapter? — verdict

**Yes, but keep it lean.** It is the only place your four contributions are
documented (LLM pipeline, social BT, YOLO+depth detector, simulation suite), and it
is the chapter you can finish *today* because nothing in it depends on results.

Keep it lean because the failure mode is a manual, not an argument:
- if a subsection reduces to a table without losing meaning, make it a table
  (platform, perception parameters, Nav2 config, metrics);
- spend prose only on **decisions and why**: BT over FSM, per-robot maps with no
  merging, event-driven LLM calls, agent-as-consultant not controller, bounded
  yields (4 s / 10 s).

Drop the separate chapter only if your department mandates a single "Methodology"
chapter — then use the combined variant (Option D, at the end of this file) and say
in the chapter's first paragraph that it has two halves.

Budget: Ch.3 ≈ 8–12 pages (mostly tables + short rationale paragraphs);
        Ch.4 ≈ 10–14 pages (design of experiment, metrics, protocol, validity).

## What goes where — Design vs Implementation vs Methodology

| The chapter answers | Chapter | Practical test for an item |
|---|---|---|
| What is the system, and why is it shaped this way? | System Design | answerable from a diagram or a config file |
| How was it made to run, and how would someone rebuild it? | Implementation (section or appendix) | needs file names, commands, or a workaround story |
| How were the claims established, and how should they be read? | Methodology | exists only because something was measured |

**Your own content, mapped:**

| Content | Design | Impl. | Method. |
|---|---|---|---|
| architecture, node graph, TF tree, per-robot stacks | X | | |
| design decisions and rejected alternatives | X | | |
| interfaces: topics, actions, QoS | X | | |
| detector parameters, symbolic state definitions | X | | |
| tree structure and timing bounds | X | | |
| decision prompt (verbatim) | X (or appendix) | | |
| packages/nodes table, launch procedure, build steps | | X | |
| frame rewriting, clock relay, fd/DDS settings, sequential bring-up, detector gating | | X | |
| three behavioural conditions | | | X |
| scenario matrix, goal protocol, duration, repetitions | | | X |
| metric definitions and their artifacts | | | X |
| run protocol, validators, inclusion/exclusion, era policy | | | X |
| token/cost accounting method | | | X |
| threats to validity | | | X |

Note the split *inside* the engineering work: how the system is brought up and
patched = Implementation; the acceptance criteria for a valid run (gate passed,
detectors loaded) = Methodology.

## Writing order

1. Ch.3 §3.2 Platform — pure facts, no decisions pending.
2. Ch.3 §3.3–3.4 Perception + symbolic state — facts + tables.
3. Ch.3 §3.5–3.6 BT + LLM layer — the contribution core; prompt verbatim → appendix.
4. Ch.3 §3.7–3.9 Nav2 integration, multi-robot extension, design rationale.
5. Ch.4 — blocked on: (a) RQ1 operationalisation, (b) personal-space definition,
   (c) the era policy.
6. Ch.5 Results — organise **by RQ**, not by metric: RQ1 (action appropriateness +
   execution reliability), RQ2 (three-arm comparison), RQ3 (scaling 1→N).
7. Ch.6 Discussion — answer each RQ, then map back to Ch.2 (freezing-robot problem,
   proxemics, ORCA/CADRL) and state the limitations from 4.6.

Cut principle: **Ch.3 = what was built** (still true if no experiment were ever run);
**Ch.4 = how the claims are established** (exists only to justify measurements).

Mapping to your own RQs and contributions:

- **RQ1** (LLM chooses actions a BT reliably executes) → design in Ch.3, *measurement* in Ch.4
- **RQ2** (simulation suite gives metrics for a 3-way comparison) → nearly all Ch.4
- **RQ3** (per-robot performance/safety as the fleet scales) → Ch.4
- Contributions 1–3 → Ch.3; contribution 4 (the simulation suite) → Ch.3 §2 + Ch.4 §5

---

## Chapter 3 — System Design (lean, agreed)

| § | Section | Covers | Contributes to | ~pages |
|---|---|---|---|---|
| 3.1 | System Overview | platform facts, robot model, then the three behaviour conditions (Nav2 / BT / BT+LLM) and a pipeline diagram | all | 2–2.5 |
| 3.2 | Perception and Symbolic State Extraction | LiDAR path + `lidar_republish`; YOLOv8n detector (conf 0.4, 3 Hz, 87° FOV, depth ranging, robot-as-person suppression); the state variables and thresholds that feed tree and agent | C3 | 2 |
| 3.3 | Social Behaviour Tree Design | tree structure, bounded yields (4 s / 10 s), nested fallbacks, arc maneuver, custom BT nodes, blackboard keys, progress-checker relaxation | C2 | 2.5 |
| 3.4 | LLM Social Decision Layer | action vocabulary, situation report, prompt rules (right-of-way, adaptive passing, stall escape), event-driven invocation, agent-as-consultant, cost accounting; prompt verbatim → Appendix | C1 | 2 |
| 3.5 | Multi-Robot Extension and Scaling | per-robot SLAM/Nav2 stacks, per-robot map frames, TF, proximity layer, local-only coordination, the fleet-size factor | C4/RQ3 | 1–1.5 |
| 3.6 | Design Decisions and Alternatives | one paragraph per decision: BT over FSM, per-robot maps, event-driven LLM, bounded yields, nearest-person representation | all | 1 |

*(3.6 is optional in the user's current plan. If dropped: fold one rationale
paragraph into the end of each of 3.1–3.5, and delete the `sec:meth_rationale`
label wherever it is referenced. Do not delete the reasoning itself — the
"why not X" answers are what the defence depends on.)*

**Do not drop 3.5** — RQ3 asks about performance as the fleet scales; without this
section the scaling study has no designed object. It may be short, not absent.

**Where the dropped "Nav2 integration" content goes** (so nothing is lost):

| Item | New home |
|---|---|
| Velocity chain `cmd_vel_social` → `velocity_adaptor` → `cmd_vel` | §3.3 (it is the social speed mechanism) |
| Planner/controller choice (NavFn, DWB, `allow_unknown`), costmap resolution 0.05 m, robot radius 0.30 m, inflation 0.25 / 0.7 m | §3.1 config table |
| Progress checker relaxed to 0.25 m / 30 s — and why | §3.3 as a design decision (the social layer pauses deliberately) |
| Goal tolerances, lifecycle on wall clock, bond 60 s / service 15 s | Appendix A (runtime configuration), referenced once from §3.1 |
| Per-arm `cmd_vel` remapping (no / reactive / ai) | §3.1 where the three conditions are introduced |

Facts for every table: `docs/methodology_facts.md`.

---

## Chapter 4 — Methodology

| § | Section | Must specify |
|---|---|---|
| 4.1 | Research Questions and Hypotheses | RQ1–RQ3 restated operationally + H1 (social layer vs Nav2), H2 (LLM vs reactive BT), H3 (scaling 1→N) |
| 4.2 | Behavioural Conditions | the three arms as the factor: Nav2 (`--bt stock`), BT (`--bt reactive`), BT+LLM (`--bt ai`); everything else held constant |
| 4.3 | Scenario Design and Goal Protocol | matrix (rovers × humans), obstacle variants, human lanes (walking only — see gaps), goal tours, scan tour, duration |
| 4.4 | Metrics and Instrumentation | every metric defined + its artifact (table below) |
| 4.5 | Experimental Protocol and Reproducibility | preconditions (thermal), gates/validators as inclusion criteria, artifact set per run, exclusions |
| 4.6 | Threats to Validity | simulation-only; kinematic non-reactive humans; perception limits; hosted LLM version; single host; metric dependence |

### RQ → condition → metric → artifact

| RQ | Metric | Definition | Artifact |
|---|---|---|---|
| RQ1 | Action appropriateness | rule-consistency audit of logged decisions vs the encoded rules (see gap 1) | `llm_usage.csv` |
| RQ1 | Execution reliability | fraction of agent actions adopted by the tree; fallback/abort counts | `llm_usage.csv`, `social_events.csv` |
| RQ2 | Goal success rate | goals succeeded / sent per condition | `goal_stats.csv` |
| RQ2 | Coverage | visited cells / 196 (1 m grid) | `coverage_final.csv`, `coverage_map.png` |
| RQ2 | Path length, mean speed while moving | integrated odometry | `trajectories.csv`, `summary.csv` |
| RQ2 | Collision count | contact episodes (human / rover / wall) | `events.csv`, `social_events.csv` |
| RQ2 | Social acts | yields (`BACKWARD_*`), `ROBOT_NEAR`, `OBSTACLE_NEAR` | `social_events.csv` |
| RQ2 | Cost | tokens, USD per run and per completed goal | `llm_usage.csv`, `llm_summary.csv` |
| RQ3 | Goal success rate vs fleet size | as above, grouped by robot count | `goal_stats.csv` |
| RQ3 | Collision rate per encounter·km | contacts normalised by distance travelled | `events.csv`, `trajectories.csv` |
| RQ3 | Personal-space distance | minimum and percentiles of human range while moving (see gap 2) | `social_state.csv` |

---

## Gaps to close before drafting Ch.4

1. **"Socially appropriate" has no operational definition (RQ1).** This is the one thing
   that currently makes RQ1 unanswerable. Practical options, cheapest first:
   - rule-consistency audit: sample the logged decisions (the `reason` field is
     recorded verbatim) and score each against the encoded rules — did the yielding
     robot yield, did anyone arc into an occupied side, was `back` used only for a
     genuine deadlock;
   - outcome proxies: personal-space violations, deadlock incidence, yields per encounter;
   - expert annotation of a decision sample (strongest, most expensive).
   Recommend: audit + proxies, with the audit criteria stated explicitly as the rubric.
2. **Personal-space distance needs one definition.** Related work cites Hall
   (personal 0.45–1.2 m, social 1.2–3.6 m), the detector's `human_close` uses 1.5 m,
   and RQ3 says "personal-space distance". Recommend reporting the minimum recorded
   human range plus time-below-threshold at 0.45 / 1.2 / 1.5 / 3.6 m, and stating
   once in 4.4 which one you treat as the safety boundary.
3. **Contribution 4 overclaims static humans.** The scenario generator scripts
   walking lanes only; static elements are boxes/cylinders. Either implement standing
   humans, or reword to "human-populated scenarios with scripted walking pedestrians
   and static obstacle layouts".
4. **Era policy (still open).** Runs before 2026-09-23 used 10 Hz LiDAR / 5 Hz camera
   @ 640×480 and an unthrottled 383 Hz clock; from 2026-09-23: 5/3 Hz @ 320×240 with a
   50 Hz clock relay and a relaxed progress checker. Decide: re-run, re-run a reduced
   grid, or declare the earlier runs pilot data — and state the decision in 4.5.
